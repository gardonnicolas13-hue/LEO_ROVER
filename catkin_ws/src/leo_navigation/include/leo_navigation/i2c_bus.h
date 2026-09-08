// i2c_bus.h — thin, safe wrapper over Linux's i2c-dev userspace interface.
//
// Deliberately NOT wiringPi: wiringPi is unmaintained for the Pi's current
// kernel/GPIO stack (its author stopped supporting it after the Pi 3/4
// GPIO changes, and it does not support the Pi 5 at all) and is a
// dependency this project would need to build and pin for no real benefit
// -- i2c-dev.h + ioctl() is the standard Linux kernel interface, ships with
// every Raspberry Pi OS image, and needs no extra library at all.
//
// Every call here is a blocking syscall on the SAME bus /dev/i2c-N used by
// every other I2C client on the Pi -- this class does not own the bus,
// it just talks to one address on it. Two devices sharing a bus (as the
// LSM6DS33 and LIS3MDL do on the MinIMU-9 v5, at different addresses) is
// completely normal and requires nothing special here: each I2CDevice
// instance re-selects its own slave address before every transaction via
// I2C_SLAVE, so interleaving reads from two devices on the same node is
// safe.
#ifndef LEO_NAVIGATION_I2C_BUS_H
#define LEO_NAVIGATION_I2C_BUS_H

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace leo_navigation {

/// One I2C slave device on one bus. Construction opens the bus node; every
/// read/write re-asserts the slave address first, so this is safe to use
/// even if something else on the process also touches the same bus node
/// through a different I2CDevice instance.
class I2CDevice {
 public:
  I2CDevice(const std::string &bus_path, uint8_t address)
      : address_(address) {
    fd_ = open(bus_path.c_str(), O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error(
          "I2CDevice: could not open " + bus_path + " (" +
          std::string(strerror(errno)) +
          "). Is I2C enabled (raspi-config) and is this user in the i2c "
          "group?");
    }
  }

  ~I2CDevice() {
    if (fd_ >= 0) close(fd_);
  }

  I2CDevice(const I2CDevice &) = delete;
  I2CDevice &operator=(const I2CDevice &) = delete;

  /// Write one byte to a register.
  bool write_reg(uint8_t reg, uint8_t value) {
    if (!select()) return false;
    uint8_t buf[2] = {reg, value};
    return write(fd_, buf, 2) == 2;
  }

  /// Read one byte from a register.
  bool read_reg(uint8_t reg, uint8_t *out) {
    if (!select()) return false;
    if (write(fd_, &reg, 1) != 1) return false;
    return read(fd_, out, 1) == 1;
  }

  /// Burst-read `len` contiguous bytes starting at `reg`. Both the
  /// LSM6DS33 and LIS3MDL auto-increment the register pointer on a
  /// multi-byte read by default (LSM6DS33: IF_INC in CTRL3_C, default 1;
  /// LIS3MDL: the MSb of the sub-address byte requests auto-increment,
  /// set unconditionally below) -- this is what makes a single burst read
  /// of gyro+accel (12 bytes from 0x22) a single I2C transaction instead
  /// of twelve.
  bool read_burst(uint8_t start_reg, uint8_t *out, size_t len) {
    if (!select()) return false;
    // LIS3MDL requires the auto-increment bit (MSb of SUB) set explicitly;
    // LSM6DS33 ignores this bit on its own address byte (it uses IF_INC in
    // a control register instead), so setting it unconditionally is safe
    // for both chips sharing this helper class.
    uint8_t reg = start_reg | 0x80;
    if (write(fd_, &reg, 1) != 1) return false;
    return read(fd_, out, len) == static_cast<ssize_t>(len);
  }

 private:
  bool select() { return ioctl(fd_, I2C_SLAVE, address_) >= 0; }

  int fd_ = -1;
  uint8_t address_;
};

}  // namespace leo_navigation

#endif  // LEO_NAVIGATION_I2C_BUS_H
