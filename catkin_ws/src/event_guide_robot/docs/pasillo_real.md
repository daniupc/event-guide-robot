# Lanzamiento en pasillo real

Esta rama usa `maps/mapa_passadis.yaml` como mapa por defecto y conserva la
misma logica de zonas, stands y marcadores ArUco de la demo anterior.

## Preparar la rama

```bash
cd ~/event-guide-robot
git fetch origin
git switch pasillo
git pull
```

## Compilar el workspace

```bash
cd ~/event-guide-robot/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Si `source devel/setup.bash` falla, normalmente es porque aun no se ha
ejecutado `catkin_make` desde `~/event-guide-robot/catkin_ws`.

## Lanzar navegacion y guia

En el robot o en el PC que controle la base, deja arrancado el bringup real del
TurtleBot3 segun la configuracion del laboratorio.

```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch event_guide_robot navigation_with_guide.launch
```

El launch anterior carga `mapa_passadis.yaml`, AMCL, move_base y los nodos del
guia. En RViz marca primero la pose inicial del robot con `2D Pose Estimate`.

## Enviar una orden de prueba

```bash
rostopic pub /guide/command std_msgs/String "data: 'qualcomm'"
```

Otros aliases siguen funcionando igual, por ejemplo `samsung`, `telefonica`,
`nokia`, `ericsson`, `gsma`, `meta` o `nvidia`.

## Comprobar estado

```bash
rostopic echo /guide/state
rostopic echo /guide/result
rostopic echo /vision/detections
```

## Ver o reajustar puntos en RViz

```bash
rostopic echo /clicked_point
```

En RViz usa `Publish Point` sobre el mapa. Los puntos que interesen se copian en
`config/semantic_map.yaml`, manteniendo el `frame_id` `map`.
