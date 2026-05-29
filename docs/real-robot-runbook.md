# Runbook de prueba en robot real

Esta guia empieza **despues de tener hechos los bringups del robot y de la camara**. La camara oficial del proyecto es la TurtleBot3 Raspberry Pi Camera (`turtlebot3_rpicamera.launch`) y el topic esperado por defecto es `/raspicam_node/image`. Es decir, asume que el PC ya tiene `roscore` y que el robot ya publica odometria, `/scan` y el topic de rpicamera correspondiente.

## Terminal 1 - Navegacion + sistema guia

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

rosparam set /use_sim_time false
rosparam get /use_sim_time

roslaunch event_guide_robot navigation_with_guide.launch
```

Debe devolver `false` en `rosparam get /use_sim_time`.

Este launch usa por defecto:

```text
$(find event_guide_robot)/maps/map.yaml
```

y tambien relaja el yaw final de `move_base` para que el robot no tenga que recolocarse mirando a una orientacion concreta al llegar a la zona.

Si quieres lanzar con otro mapa:

```bash
roslaunch event_guide_robot navigation_with_guide.launch map_file:=/ruta/a/otro/map.yaml
```

## Terminal 2 - Comprobaciones basicas

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic echo -n 1 /map_metadata
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rosrun tf tf_echo odom base_footprint
```

Si `tf_echo odom base_footprint` falla, prueba:

```bash
rosrun tf tf_echo odom base_link
```

Si `/map_metadata` no devuelve nada, revisa primero:

```bash
rosparam get /use_sim_time
```

En robot real debe ser `false`.

## Terminal 3 - Estados del sistema guia

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic echo /guide/state
```

Estados esperados durante una prueba normal:

```text
PARSE_TARGET
TARGET_RESOLVED
NAVIGATE_TO_ZONE
NAVIGATION_SUCCEEDED
LOCAL_VISUAL_SEARCH
APPROACH_TARGET
FOUND_TARGET
```

## Terminal 4 - Plan y resultado

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic echo /guide/plan
```

En otra pestana o terminal adicional puedes dejar:

```bash
rostopic echo /guide/result
```

## Terminal 5 - Enviar comando de usuario

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic pub -1 /guide/command std_msgs/String "data: 'qualcomm'"
```

Otros comandos que deberian resolver:

```bash
rostopic pub -1 /guide/command std_msgs/String "data: 'samsung'"
rostopic pub -1 /guide/command std_msgs/String "data: 'telefonica'"
rostopic pub -1 /guide/command std_msgs/String "data: 'nokia'"
rostopic pub -1 /guide/command std_msgs/String "data: 'ericsson'"
rostopic pub -1 /guide/command std_msgs/String "data: 'gsma'"
rostopic pub -1 /guide/command std_msgs/String "data: 'meta'"
rostopic pub -1 /guide/command std_msgs/String "data: 'nvidia'"
```

## Terminal 6 - Simular deteccion ArUco si la camara no esta validada

Cuando `/guide/state` llegue a:

```text
LOCAL_VISUAL_SEARCH
```

puedes simular una deteccion lejana de Qualcomm (`marker_id: 11`) para probar la fase de aproximacion:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rostopic pub -1 /vision/detections std_msgs/String "data: '{\"marker_id\":11,\"label_id\":\"qualcomm_ai_hub\",\"confidence\":1.0,\"stable\":true,\"distance_m\":0.8,\"center_offset_x\":0.0}'"
```

El estado deberia pasar a `APPROACH_TARGET` y el robot avanzara despacio. Para simular que ya ha llegado a unos 20 cm, publica despues:

```bash
rostopic pub -1 /vision/detections std_msgs/String "data: '{\"marker_id\":11,\"label_id\":\"qualcomm_ai_hub\",\"confidence\":1.0,\"stable\":true,\"distance_m\":0.2,\"center_offset_x\":0.0}'"
```

Entonces debe pasar a `FOUND_TARGET`, publicar velocidad cero y detenerse.

## Ver topics de camara

La camara real es `rpicamera`. Para validar que publica el topic esperado:

```bash
rostopic list | grep image
```

Si por configuracion local no es `/raspicam_node/image`, lanza el sistema indicando el topic correcto:

```bash
roslaunch event_guide_robot navigation_with_guide.launch image_topic:=/topic/de/la/camara
```

## Diagnostico rapido

```bash
rosparam get /use_sim_time
rostopic list | grep guide
rostopic info /guide/command
rostopic info /guide/plan
rostopic list | grep move_base
rosnode list | grep -E 'semantic|navigation|local_search|vision|move_base|map_server|amcl'
```

Puntos clave:

- En robot real, `/use_sim_time` debe ser `false`.
- Si `/guide/command` recibe mensajes pero `/guide/plan` no sale, mira `/guide/result`: puede ser `PARSE_FAILED` por alias mal escrito.
- `qualcomm` lleva doble `m`.
- Si el robot no navega con `2D Nav Goal` manual en RViz, el problema esta en navegacion/AMCL/mapa, no en el sistema guia.
