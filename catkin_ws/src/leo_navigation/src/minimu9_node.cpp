// minimu9_node.cpp — ROS1 driver for the Pololu MinIMU-9 v5 (LSM6DS33
// accel+gyro, LIS3MDL magnetometer) over the Raspberry Pi's own I2C bus.
//
// Written 2026-09-08, BEFORE the physical sensor arrived (Hector ordered it
// today). Every register address, bit encoding, and I2C address in this
// file was checked directly against ST's official datasheets (LSM6DS33
// DocID027423 Rev 4, LIS3MDL DocID024204 Rev 4) -- not recalled from
// memory or copied from a secondary source -- specifically because a wrong
// register constant here would silently corrupt MINS's inertial input the
// same way a sign error nearly did in the tag-correction geometry. What
// this file CANNOT do yet is prove itself against real silicon: treat it
// as a carefully-derived template, verify against the datasheets once more
// when the hardware is in hand, and run the checklist at the bottom of the
// architecture note before trusting a single sample of it.
//
// TWO REAL GOTCHAS this project would otherwise have hit at first power-on,
// found by reading the actual register tables rather than assuming a
// "sensible" default:
//   1. LIS3MDL::CTRL_REG3 defaults to MD[1:0] = 11 = POWER-DOWN. The
//      magnetometer will not produce a single sample until this is
//      explicitly written to continuous-conversion (00).
//   2. LIS3MDL's Z axis has ITS OWN operating-mode bits, in a DIFFERENT
//      register (CTRL_REG4::OMZ) from X/Y (CTRL_REG1::OM). Setting only
//      CTRL_REG1 leaves Z in low-power mode while X/Y run in whatever was
//      requested -- an asymmetric noise floor across axes that would be
//      very easy to miss.
//
// TIMESTAMP POLICY (as discussed with Hector): stamped with ros::Time::now()
// immediately after the I2C burst read returns, before any further work.
// Neither chip carries a hardware timestamp over plain I2C the way the
// CORE2 firmware's own IMU message does (see firmware_message_converter.cpp,
// which forwards a firmware-origin stamp) -- this is deliberately the best
// a userspace I2C read on non-RT Linux can honestly offer, not a claim of
// hardware-grade timing. At MINS's propagation rate that jitter is not
// expected to matter; it would if this fed a much faster control loop.

// leo_msgs/Imu, NOT sensor_msgs/Imu, and this is an integration decision
// rather than a style preference. The chain that actually feeds MINS is
//   CORE2 -> /serial_node -> /firmware/imu (leo_msgs/Imu)
//         -> imu_sanitizer (subscribes with the leo_msgs type)
//         -> /imu/data_clean  <- MINS reads THIS
// Publishing sensor_msgs on /imu/data_raw would have done two wrong things
// at once: collided with firmware_message_converter, which already
// publishes there, and never reached MINS at all, since config_imu.yaml
// says in as many words "/imu/data_clean, NOT /imu/data_raw. Never point
// MINS at the raw firmware IMU."
//
// Emitting leo_msgs on a dedicated topic instead makes this a drop-in
// replacement at the SOURCE of the chain: imu_sanitizer only needs its
// ~in_topic repointed, and every downstream guard it provides -- spike
// rejection, the timestamp-unlock guard, ZUPT detection, the accel rescale
// -- keeps working untouched.
//
// The magnetometer stays sensor_msgs/MagneticField: leo_msgs has no
// magnetic type, and nothing in this pipeline consumes one today anyway.
#include <leo_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <cmath>
#include <memory>
#include <string>

#include "leo_navigation/i2c_bus.h"
#include "ros/ros.h"

namespace leo_navigation {

// ---------------------------------------------------------------------
// LSM6DS33 -- register addresses and bit fields, DocID027423 Rev 4.
// ---------------------------------------------------------------------
namespace lsm6ds33 {
constexpr uint8_t kAddrSA0Low = 0x6A;   // SDO/SA0 pin tied to GND  (p.29)
constexpr uint8_t kAddrSA0High = 0x6B;  // SDO/SA0 pin tied to VDD  (p.29)

constexpr uint8_t kRegWhoAmI = 0x0F;
constexpr uint8_t kWhoAmIValue = 0x69;  // fixed value, Table 41

constexpr uint8_t kRegCtrl1Xl = 0x10;  // accel: ODR_XL[3:0] FS_XL[1:0] BW_XL[1:0]
constexpr uint8_t kRegCtrl2G = 0x11;   // gyro:  ODR_G[3:0]  FS_G[1:0]  FS_125 0
constexpr uint8_t kRegCtrl3C = 0x12;   // BOOT BDU H_LACTIVE PP_OD SIM IF_INC BLE SW_RESET
// STATUS_REG (Table 76, section 9.24): bit layout is
//   [ - - - - EV_BOOT TDA GDA XLDA ]
// so bit 0 = accelerometer data available, bit 1 = gyroscope, bit 2 =
// temperature. Read before every publish so the node never re-publishes a
// sample the chip has not refreshed (see the loop below for why that
// matters more than a missed sample would).
constexpr uint8_t kRegStatus = 0x1E;
constexpr uint8_t kStatusXlda = 0x01;  // accel new data
constexpr uint8_t kStatusGda = 0x02;   // gyro new data
constexpr uint8_t kStatusTda = 0x04;   // temperature new data

// OUT_TEMP_L (0x20) / OUT_TEMP_H (0x21), then OUT_G (0x22-0x27), then
// OUT_XL (0x28-0x2D). All fourteen bytes are CONTIGUOUS, so a single burst
// from 0x20 returns temperature, gyro and accel in ONE I2C transaction --
// verified against the register address map (Table 16) and section 9.25,
// not assumed from a typical chip layout.
constexpr uint8_t kRegOutTempStart = 0x20;
constexpr uint8_t kRegOutGStart = 0x22;   // OUTX_L_G .. OUTZ_H_G, 6 bytes
constexpr uint8_t kRegOutXlStart = 0x28;  // OUTX_L_XL .. OUTZ_H_XL, 6 bytes
constexpr uint8_t kBurstLen = 14;         // 0x20..0x2D inclusive

// Temperature: 16-bit two's complement (Tables 78-80), sensitivity
// TSen = 16 LSB/degC, and the datasheet's own footnote states the output is
// "0 LSB (typ.) at 25 degC" -- hence T = raw/16 + 25. Both constants come
// from the datasheet, neither is a remembered convention.
constexpr double kTempLsbPerDegC = 16.0;
constexpr double kTempZeroDegC = 25.0;
// The temperature channel refreshes at 52 Hz, HALF the 104 Hz ODR, so TDA
// is set on roughly every other cycle. The last good value is therefore
// held between refreshes rather than publishing a stale-looking zero.

// CTRL1_XL: FS_XL[1:0] is NOT in ascending order -- 00=+-2g, 01=+-16g,
// 10=+-4g, 11=+-8g (Table 43). Getting this backwards silently scales
// every accel sample wrong. +-4g (FS_XL=10) is a reasonable default for a
// ground rover: our peak measured |a| this project has ever logged is
// nowhere near 2g of dynamic acceleration, and +-4g keeps more of the
// 16-bit range applied to the signal than +-8/+-16g would.
constexpr uint8_t kFsXl4g = 0b10;
// ODR_XL: 0100 = 104 Hz high-performance (Table 44) -- close to the
// ~85-89 Hz this project already runs the CORE2 IMU at, so MINS's existing
// tuning (clone rates, window_size) does not need rethinking for rate
// alone. Re-measure once real hardware confirms the achieved rate.
constexpr uint8_t kOdrXl104Hz = 0b0100;

// CTRL2_G: FS_G[1:0] IS ascending -- 00=245dps, 01=500dps, 10=1000dps,
// 11=2000dps (Table 47). 500 dps is a generous margin over anything this
// rover has produced (its yaw rate is wheel-limited, see
// leo-roues-rigides-patinage in project notes) while keeping resolution.
constexpr uint8_t kFsG500Dps = 0b01;
constexpr uint8_t kOdrG104Hz = 0b0100;  // same encoding as ODR_XL, Table 48

// Accelerometer sensitivity at +-4g, mg per LSB (Table in "Sensitivity"
// terminology section; ST's standard progression is 0.061/0.122/0.244
// mg/LSB for 2/4/8g -- 4g uses 0.122).
constexpr double kAccelSensitivityMgPerLsb = 0.122;
// Gyro sensitivity at 500 dps, mdps per LSB (ST's standard progression:
// 8.75/17.50/35/70 mdps/LSB for 245/500/1000/2000 dps -- 500dps uses 17.50).
constexpr double kGyroSensitivityMdpsPerLsb = 17.50;
}  // namespace lsm6ds33

// ---------------------------------------------------------------------
// LIS3MDL -- register addresses and bit fields, DocID024204 Rev 4.
// ---------------------------------------------------------------------
namespace lis3mdl {
constexpr uint8_t kAddrSA1Low = 0x1C;   // SDO/SA1 pin tied to GND  (p.17)
constexpr uint8_t kAddrSA1High = 0x1E;  // SDO/SA1 pin tied to VDD  (p.17)

constexpr uint8_t kRegWhoAmI = 0x0F;
constexpr uint8_t kWhoAmIValue = 0x3D;  // Table 17

constexpr uint8_t kRegCtrl1 = 0x20;  // TEMP_EN OM[1:0] DO[2:0] FAST_ODR ST
constexpr uint8_t kRegCtrl2 = 0x21;  // FS[1:0] full-scale
constexpr uint8_t kRegCtrl3 = 0x22;  // MD[1:0] operating mode -- SEE GOTCHA 1
constexpr uint8_t kRegCtrl4 = 0x23;  // OMZ[1:0] Z-axis mode -- SEE GOTCHA 2
constexpr uint8_t kRegOutStart = 0x28;  // OUT_X_L .. OUT_Z_H, 6 bytes
constexpr uint8_t kBurstLen = 6;

// CTRL_REG1: OM[1:0] = 11 = ultra-high-performance for X/Y (Table 21).
constexpr uint8_t kOmUltraHighPerf = 0b11;
// DO[2:0] = 100 = 10 Hz (Table 22). The magnetometer isn't in MINS's own
// filter today (config_imu.yaml carries no magnetometer input on this
// platform) -- this driver publishes it anyway, at a modest rate, so it
// exists on the bus for whatever consumes it later without competing for
// I2C bandwidth against the much more time-critical accel/gyro reads.
constexpr uint8_t kDo10Hz = 0b100;

// CTRL_REG2: FS[1:0] = 00 = +-4 gauss (Table 25) -- Earth's field is
// ~0.25-0.65 gauss; +-4 gauss keeps full resolution while leaving margin
// for nearby ferrous/motor fields on the rover chassis.
constexpr uint8_t kFs4Gauss = 0b00;
constexpr double kSensitivityLsbPerGauss = 6842.0;  // FS=+-4G, from datasheet

// CTRL_REG3: MD[1:0] = 00 = continuous-conversion. GOTCHA 1: the chip's
// OWN default here is 11 (power-down) -- without writing this explicitly
// the magnetometer would sit silent forever, looking exactly like a wiring
// fault rather than a missed register write.
constexpr uint8_t kModeContinuous = 0b00;

// CTRL_REG4: OMZ[1:0] = 11 = ultra-high-performance for Z. GOTCHA 2: this
// is a SEPARATE field from CTRL_REG1's OM -- omitting this write leaves Z
// in low-power mode (default 00) while X/Y run at whatever CTRL_REG1
// requested, an asymmetric noise floor across axes that gives no error,
// just quietly worse Z data.
constexpr uint8_t kOmzUltraHighPerf = 0b11;
}  // namespace lis3mdl

constexpr double kGravityMps2 = 9.80665;  // standard gravity, for mg->m/s^2
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kGaussToTesla = 1e-4;

/// Tries both possible I2C addresses for a chip (SA0/SA1 pin strapping is
/// a hardware fact this driver cannot know in advance without the board in
/// hand) and confirms via WHO_AM_I rather than assuming either address is
/// actually the right chip -- a device that ACKs its address but isn't the
/// chip we think it is would otherwise corrupt every subsequent register
/// write silently.
std::unique_ptr<I2CDevice> probe(const std::string &bus_path, uint8_t addr_a,
                                 uint8_t addr_b, uint8_t who_am_i_reg,
                                 uint8_t who_am_i_expected,
                                 const std::string &chip_name) {
  for (uint8_t addr : {addr_a, addr_b}) {
    try {
      auto dev = std::make_unique<I2CDevice>(bus_path, addr);
      uint8_t who = 0;
      if (dev->read_reg(who_am_i_reg, &who) && who == who_am_i_expected) {
        ROS_INFO("[minimu9] %s found at 0x%02X (WHO_AM_I=0x%02X)",
                 chip_name.c_str(), addr, who);
        return dev;
      }
    } catch (const std::exception &e) {
      ROS_WARN("[minimu9] probing %s at 0x%02X: %s", chip_name.c_str(), addr,
               e.what());
    }
  }
  return nullptr;
}

/// Sign-extend a little-endian 16-bit two's-complement pair as the LSM6DS33
/// and LIS3MDL both use for every output register (confirmed: both
/// datasheets default BLE=0, "data LSb @ lower address").
int16_t le16(uint8_t lo, uint8_t hi) {
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

}  // namespace leo_navigation

int main(int argc, char **argv) {
  ros::init(argc, argv, "minimu9_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string bus_path;
  pnh.param<std::string>("i2c_bus", bus_path, "/dev/i2c-1");
  std::string frame_id;
  pnh.param<std::string>("frame_id", frame_id, "imu_link");
  // POLL rate, not publish rate -- the distinction matters now that the
  // loop is gated on STATUS_REG. The published rate is set by the sensor's
  // ODR (104 Hz); this is only how often we ASK whether a new sample
  // exists. It must therefore be comfortably FASTER than the ODR: polling
  // at 100 Hz against a 104 Hz source would fall behind by ~4 samples a
  // second and the queue would drift. 250 Hz gives roughly 2.4 polls per
  // sample, so each one is picked up within ~4 ms of becoming ready, at a
  // cost of a single one-byte I2C read per empty poll.
  //
  // For reference, the chain this replaces delivers 85.7 Hz measured on
  // /imu/data_clean, so 104 Hz is a modest improvement rather than a
  // change MINS has to be retuned for.
  double rate_hz;
  pnh.param<double>("rate_hz", rate_hz, 250.0);

  using namespace leo_navigation;

  auto accel_gyro =
      probe(bus_path, lsm6ds33::kAddrSA0Low, lsm6ds33::kAddrSA0High,
            lsm6ds33::kRegWhoAmI, lsm6ds33::kWhoAmIValue, "LSM6DS33");
  if (!accel_gyro) {
    ROS_FATAL(
        "[minimu9] LSM6DS33 not found on %s at either 0x%02X or 0x%02X. "
        "Check wiring, that I2C is enabled, and that nothing else is "
        "holding the bus.",
        bus_path.c_str(), lsm6ds33::kAddrSA0Low, lsm6ds33::kAddrSA0High);
    return 1;
  }

  auto mag = probe(bus_path, lis3mdl::kAddrSA1Low, lis3mdl::kAddrSA1High,
                   lis3mdl::kRegWhoAmI, lis3mdl::kWhoAmIValue, "LIS3MDL");
  if (!mag) {
    // Non-fatal: MINS doesn't consume the magnetometer on this platform
    // (see config_imu.yaml). The node keeps running IMU-only rather than
    // refusing to publish accel/gyro data over a chip nothing critical
    // depends on.
    ROS_WARN(
        "[minimu9] LIS3MDL not found -- continuing without magnetometer "
        "(MINS does not use one on this platform, so this is not fatal).");
  }

  // ---- Configure LSM6DS33 ----
  {
    uint8_t ctrl1_xl = (lsm6ds33::kOdrXl104Hz << 4) | (lsm6ds33::kFsXl4g << 2);
    uint8_t ctrl2_g = (lsm6ds33::kOdrG104Hz << 4) | (lsm6ds33::kFsG500Dps << 2);
    // CTRL3_C: BDU=1 (bit 6) so a burst read can never straddle the sensor
    // updating registers mid-transaction; IF_INC=1 (bit 2) for the
    // auto-increment burst read -- IF_INC already defaults to 1 (Table 50)
    // but is set explicitly rather than relied upon, matching this
    // project's standing rule of not trusting silent defaults.
    uint8_t ctrl3_c = (1 << 6) | (1 << 2);
    bool ok = accel_gyro->write_reg(lsm6ds33::kRegCtrl3C, ctrl3_c) &&
              accel_gyro->write_reg(lsm6ds33::kRegCtrl1Xl, ctrl1_xl) &&
              accel_gyro->write_reg(lsm6ds33::kRegCtrl2G, ctrl2_g);
    if (!ok) {
      ROS_FATAL("[minimu9] failed to configure LSM6DS33 control registers");
      return 1;
    }
  }

  // ---- Configure LIS3MDL (gotchas 1 and 2, both handled explicitly) ----
  if (mag) {
    uint8_t ctrl1 = (lis3mdl::kOmUltraHighPerf << 5) | (lis3mdl::kDo10Hz << 2);
    uint8_t ctrl2 = lis3mdl::kFs4Gauss << 5;
    bool ok = mag->write_reg(lis3mdl::kRegCtrl1, ctrl1) &&
              mag->write_reg(lis3mdl::kRegCtrl2, ctrl2) &&
              // Gotcha 2: Z axis mode is separate from X/Y.
              mag->write_reg(lis3mdl::kRegCtrl4,
                             lis3mdl::kOmzUltraHighPerf << 2) &&
              // Gotcha 1: must leave power-down explicitly, or nothing
              // ever gets sampled. Written LAST and deliberately: every
              // other config register should already hold its final
              // value before the chip starts converting.
              mag->write_reg(lis3mdl::kRegCtrl3, lis3mdl::kModeContinuous);
    if (!ok) {
      ROS_WARN(
          "[minimu9] failed to configure LIS3MDL -- continuing without "
          "magnetometer");
      mag.reset();
    }
  }

  // Dedicated topic. NOT imu/data_raw: firmware_message_converter already
  // publishes there, and two publishers on one topic interleave silently.
  std::string imu_topic;
  pnh.param<std::string>("imu_topic", imu_topic, "/minimu9/imu");
  ros::Publisher pub_imu = nh.advertise<leo_msgs::Imu>(imu_topic, 10);
  ros::Publisher pub_mag;
  if (mag) {
    pub_mag = nh.advertise<sensor_msgs::MagneticField>("/minimu9/mag", 10);
  }

  // leo_msgs/Imu carries no covariance and no frame_id -- it is the raw
  // firmware-shaped message. That is deliberate here: the covariances this
  // project trusts are the Allan-variance densities in MINS's own
  // config_imu.yaml, characterized from the installed unit, and inventing
  // placeholder ones on the wire would only invite someone to believe them.
  // ~frame_id is kept as a parameter because it still labels the
  // magnetometer message, which IS a sensor_msgs type.
  leo_msgs::Imu imu_msg;

  ROS_INFO(
      "[minimu9] configured: accel +-4g @104Hz, gyro +-500dps @104Hz%s. "
      "Publishing leo_msgs/Imu on %s, gated on the data-ready bits, "
      "loop %.1f Hz.",
      mag ? ", mag +-4G @10Hz" : " (no magnetometer)", imu_topic.c_str(),
      rate_hz);

  ros::Rate loop(rate_hz);
  uint8_t buf[lsm6ds33::kBurstLen];
  uint8_t mag_buf[lis3mdl::kBurstLen];

  // Held across iterations: the temperature channel refreshes at 52 Hz,
  // half the inertial ODR, so on roughly every other pass TDA is clear and
  // there is simply no new value to read. Publishing 0 on those cycles
  // would look like a sensor reading 0 degC rather than "unchanged".
  float derniere_temp = 0.0f;
  bool temp_valide = false;

  while (ros::ok()) {
    // ---- data-ready gate -------------------------------------------
    // A free-running poll against a 104 Hz ODR does two bad things: it
    // re-reads a sample the chip has not refreshed, and it occasionally
    // skips one. For an estimator that INTEGRATES, the duplicate is the
    // worse of the two -- it counts the same motion twice, biasing the
    // propagated velocity, whereas a dropped sample only widens the
    // interval. Gating on STATUS_REG removes the duplicates outright.
    uint8_t status = 0;
    if (!accel_gyro->read_reg(lsm6ds33::kRegStatus, &status)) {
      ROS_WARN_THROTTLE(5.0, "[minimu9] STATUS_REG read failed");
      loop.sleep();
      continue;
    }
    const bool inertiel_pret =
        (status & lsm6ds33::kStatusXlda) && (status & lsm6ds33::kStatusGda);
    if (!inertiel_pret) {
      // Nothing new yet. Poll again rather than publish a stale copy.
      loop.sleep();
      continue;
    }

    // One 14-byte burst from OUT_TEMP_L covers temperature, gyro and
    // accel: 0x20..0x2D are contiguous, so this stays a single I2C
    // transaction even though it now carries three quantities.
    if (accel_gyro->read_burst(lsm6ds33::kRegOutTempStart, buf,
                               lsm6ds33::kBurstLen)) {
      // Timestamp taken HERE, immediately after the I2C transaction
      // returns and before any conversion math -- see the file header
      // note on timestamp policy.
      ros::Time stamp = ros::Time::now();

      int16_t traw = le16(buf[0], buf[1]);   // 0x20-0x21
      int16_t gx = le16(buf[2], buf[3]);     // 0x22-0x23
      int16_t gy = le16(buf[4], buf[5]);
      int16_t gz = le16(buf[6], buf[7]);
      int16_t ax = le16(buf[8], buf[9]);     // 0x28-0x29
      int16_t ay = le16(buf[10], buf[11]);
      int16_t az = le16(buf[12], buf[13]);

      if (status & lsm6ds33::kStatusTda) {
        derniere_temp = static_cast<float>(
            traw / lsm6ds33::kTempLsbPerDegC + lsm6ds33::kTempZeroDegC);
        temp_valide = true;
      }

      // leo_msgs/Imu carries its own `stamp` field rather than a
      // std_msgs/Header -- the same shape firmware_message_converter
      // forwards from the CORE2, which is precisely what makes this a
      // drop-in at the head of the chain.
      imu_msg.stamp = stamp;
      imu_msg.temperature = temp_valide ? derniere_temp : 0.0f;
      imu_msg.gyro_x =
          gx * lsm6ds33::kGyroSensitivityMdpsPerLsb * 1e-3 * kDegToRad;
      imu_msg.gyro_y =
          gy * lsm6ds33::kGyroSensitivityMdpsPerLsb * 1e-3 * kDegToRad;
      imu_msg.gyro_z =
          gz * lsm6ds33::kGyroSensitivityMdpsPerLsb * 1e-3 * kDegToRad;
      imu_msg.accel_x =
          ax * lsm6ds33::kAccelSensitivityMgPerLsb * 1e-3 * kGravityMps2;
      imu_msg.accel_y =
          ay * lsm6ds33::kAccelSensitivityMgPerLsb * 1e-3 * kGravityMps2;
      imu_msg.accel_z =
          az * lsm6ds33::kAccelSensitivityMgPerLsb * 1e-3 * kGravityMps2;
      pub_imu.publish(imu_msg);
    } else {
      ROS_WARN_THROTTLE(5.0, "[minimu9] I2C read from LSM6DS33 failed");
    }

    if (mag && mag->read_burst(lis3mdl::kRegOutStart, mag_buf,
                               lis3mdl::kBurstLen)) {
      ros::Time stamp = ros::Time::now();
      int16_t mx = le16(mag_buf[0], mag_buf[1]);
      int16_t my = le16(mag_buf[2], mag_buf[3]);
      int16_t mz = le16(mag_buf[4], mag_buf[5]);

      sensor_msgs::MagneticField mag_msg;
      mag_msg.header.stamp = stamp;
      mag_msg.header.frame_id = frame_id;
      mag_msg.magnetic_field.x =
          (mx / lis3mdl::kSensitivityLsbPerGauss) * kGaussToTesla;
      mag_msg.magnetic_field.y =
          (my / lis3mdl::kSensitivityLsbPerGauss) * kGaussToTesla;
      mag_msg.magnetic_field.z =
          (mz / lis3mdl::kSensitivityLsbPerGauss) * kGaussToTesla;
      pub_mag.publish(mag_msg);
    }

    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
