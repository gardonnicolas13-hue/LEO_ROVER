# Patchs MINS — LEO Rover / Florida Tech, 2026-09-06

Contre [rpng/MINS](https://github.com/rpng/MINS). Répertoire séparé de
`/patches/` (qui vise `rpng/sqrtVINS`) et de `/patches_openvins/` — même
mécanisme, projet amont différent.

```bash
cd <racine du dépôt MINS>
git apply --check patches_mins/01_ros1_entry_point_20260906.patch
git apply         patches_mins/01_ros1_entry_point_20260906.patch
```

**Vérifié** : appliqué sur le `mins/src/run_subscribe.cpp` amont, ce patch
reproduit au octet près le fichier en service sur cette plateforme.

---

## 01 — Le point d'entrée ROS1 ne s'abonnait à rien

**Le défaut.** `mins/src/run_subscribe.cpp` implémente la voie ROS2 en entier
— `rclcpp::init`, création du nœud, `get_parameter("config_path")`,
`ROS2Publisher`/`ROS2Subscriber`, exécuteur multi-thread — mais la branche
`#elif ROS_AVAILABLE == 1` **n'existait pas**. Le fichier incluait pourtant
`core/ROSPublisher.h` et `core/ROSSubscriber.h` sans jamais les instancier.

Conséquence : sous ROS1, `rosrun mins subscribe` chargeait la configuration,
imprimait le résumé de `op->load_print()`, puis **sortait immédiatement sans
s'abonner à quoi que ce soit**. Aucun message d'erreur, aucun crash, code de
retour 0 — le mode de défaillance le plus coûteux à diagnostiquer, parce que
tout ce qui est visible se comporte normalement.

L'écart date du portage vers ROS2 : la branche ROS1 n'a jamais été réécrite
après. Il a été trouvé et corrigé indépendamment ici (2026-08-31) et par
Mike (2026-09-04) — c'est **un seul et même bug amont**, pas deux versions
divergentes de MINS.

**Le remède**, en deux parties.

*Partie 1 — instancier la voie ROS1*, en miroir de la voie ROS2 déjà
présente : `ros::init`, `ros::NodeHandle`, `ROSPublisher`/`ROSSubscriber`,
`ros::spin()`.

*Partie 2 — résoudre le chemin de configuration comme le fait openVINS.* La
voie ROS2 lit un paramètre `config_path` ; la voie ROS1 n'avait aucun
équivalent et ne pouvait donc lire que `argv[1]`. Reprise de la forme
canonique de `run_subscribe_msckf.cpp` d'openVINS, dont MINS dérive :

```cpp
auto nh = std::make_shared<ros::NodeHandle>("~");
nh->param<std::string>("config_path", config_path, config_path);
```

Deux points méritent d'être explicités, parce qu'ils se trompent en silence.

**Le NodeHandle doit être privé (`"~"`).** roslaunch place un `<param>`
déclaré à l'intérieur d'une balise `<node>` dans l'espace de noms **privé** de
ce nœud. Un handle global chercherait `/config_path`, ne le trouverait jamais,
et retomberait indéfiniment sur `argv[1]` : un correctif qui a l'air juste et
ne fait rien. C'est sans danger pour les topics **et cela a été vérifié, pas
supposé** — `ROSPublisher` annonce des noms absolus (`/mins/imu/odom`, …) et
tous les topics de `config/leo/*.yaml` le sont aussi ; un `/` initial rend un
nom insensible à l'espace de noms du handle. Si un nom **relatif** est un jour
ajouté à cette configuration, il sera remappé sous `/mins_subscribe/`.

**Le défaut passé à `param()` est `config_path`, pas `argv[1]`.** Les deux
expriment la même intention — paramètre ROS d'abord, `argv[1]` en repli — car
`config_path` contient déjà `argv[1]` à ce point du code. Mais passer
littéralement `argv[1]` est un comportement indéfini quand `argc == 1` : la
norme C garantit `argv[argc] == NULL`, donc `argv[1]` y est un pointeur nul,
pas une chaîne.

**Effet de bord utile pour cette plateforme.** `catkin_ws/src/leo_navigation/
launch/mins.launch` passait la configuration par `args="…"` précisément parce
que `<param name="config_path">` échouait en silence, et son commentaire le
documentait. Les deux voies fonctionnent désormais ; `args=` reste en place,
la bascule vers `<param>` n'ayant aucune urgence.

**Non couvert par ce patch.** La voie ROS1 n'appelle pas
`parser->set_node_handler(nh)`, alors que la voie ROS2 appelle
`parser->set_node(node)` et qu'openVINS appelle bien l'équivalent ROS1. Les
valeurs du YAML ne peuvent donc pas être surchargées par des paramètres ROS
sous ROS1. C'est un écart réel avec la voie ROS2, laissé de côté à dessein :
le corriger changerait le comportement en service (des paramètres ROS
prendraient soudain le pas sur le YAML), ce qui dépasse la portée d'un
correctif de point d'entrée.
