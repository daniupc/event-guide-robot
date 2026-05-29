# Guia para crear etiquetas semanticas sobre un mapa ROS 1

Esta nota resume como crear una capa semantica para el proyecto `Event Guide Robot` una vez ya existe un mapa generado con TurtleBot3 Waffle en ROS 1.

## Idea principal

No se deben modificar directamente los archivos del mapa (`map.pgm` o `map.yaml`) para anadir informacion semantica. El mapa de navegacion sigue siendo geometrico y se usa para `map_server`, `amcl` y `move_base`.

Las etiquetas semanticas se guardan en un archivo YAML aparte, por ejemplo:

```text
catkin_ws/src/event_guide_robot/config/semantic_map.yaml
```

Ese archivo relaciona nombres humanos como `zona_startups`, `hall_robotica` o `stand_qualcomm` con coordenadas reales del frame `map`.

## Estado actual del mapa semantico

El proyecto ya tiene una primera version de:

```text
catkin_ws/src/event_guide_robot/config/semantic_map.yaml
```

Se genero a partir de los puntos medidos en `puntos.txt`:

| Zona | x | y | Stands actuales |
| --- | ---: | ---: | --- |
| `zona_arriba` | -0.529138445854187 | -0.044567957520484924 | Qualcomm AI Hub, Samsung Galaxy Experience |
| `zona_izquierda` | 0.2924858331680298 | -0.3563343286514282 | Telefonica Open Gateway, Nokia Networks Lab |
| `zona_abajo` | 0.853394627571106 | 0.3277086019515991 | Ericsson 5G Arena, GSMA Innovation City |
| `zona_derecha` | -0.07007773220539093 | 0.84162437915802 | Meta XR Showcase, NVIDIA Edge AI |

Los `yaw` actuales se calcularon inicialmente para mirar hacia el centroide de los cuatro puntos, pero en la demo real no se quiere forzar esa orientacion porque los espacios son estrechos. El launch recomendado relaja `yaw_goal_tolerance` para que el robot acepte la orientacion natural de llegada. Solo revisa `yaw` en RViz si necesitas una orientacion exacta para una prueba concreta.

Los campos `marker_id` son IDs ArUco definitivos para el MVP. Cada valor debe corresponder al marcador impreso y colocado junto al stand correspondiente.

## Flujo esperado

Ejemplo:

```text
Usuario: "busca Qualcomm"

1. El sistema busca "qualcomm" en semantic_map.yaml.
2. Encuentra que pertenece a la zona "zona_startups".
3. El robot navega con move_base hasta el nav_goal de zona_startups.
4. Al llegar, ejecuta busqueda local en search_waypoints.
5. La camara busca el marker_id, texto o cartel asociado a Qualcomm.
6. Si lo detecta de forma estable, el robot se para y reporta exito.
```

Con la implementacion actual, el flujo entre nodos es:

```text
/guide/command
  -> semantic_planner_node
  -> /guide/plan JSON
  -> navigation_manager_node
  -> move_base
  -> local_search_manager_node
  -> /cmd_vel
  -> vision_detector_node
  -> /vision/detections JSON
```

La busqueda local solo acepta detecciones con `stable: true`.

## Como obtener coordenadas de una zona

### Opcion A: usar 2D Nav Goal en RViz

1. Lanza la navegacion con el mapa:

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/ruta/a/tu/map.yaml
```

2. En RViz pulsa `2D Nav Goal`.
3. Haz click en el punto al que quieres que vaya el robot.
4. Orienta la flecha hacia donde quieres que mire.
5. Lee el goal publicado:

```bash
rostopic echo /move_base_simple/goal
```

Salida tipica:

```yaml
pose:
  position:
    x: 1.42
    y: 3.18
    z: 0.0
  orientation:
    z: 0.70
    w: 0.71
```

Guarda `x`, `y` y convierte la orientacion a `yaw` si necesitas una orientacion exacta.

### Opcion B: mover el robot y leer AMCL

1. Teleopera el robot hasta la zona:

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

2. Lee su pose localizada:

```bash
rostopic echo /amcl_pose
```

3. Cuando el robot este bien colocado y AMCL estable, guarda esa pose como `nav_goal` o como `search_waypoint`.

## Diferencia entre zona y etiqueta concreta

### Zona semantica

Una zona es una region general del mapa. Por simplicidad, se representa con:

- un `nav_goal` principal;
- varios `search_waypoints` dentro o alrededor de la zona;
- aliases para reconocer ordenes humanas.

Ejemplo:

```yaml
zona_startups:
  name: "Zona Startups"
  aliases:
    - "startups"
    - "zona startups"
    - "emprendedores"
  nav_goal:
    x: 2.10
    y: 1.80
    yaw: 1.57
```

### Etiqueta especifica

Una etiqueta concreta es lo que el robot debe buscar con la camara dentro de esa zona.

Ejemplo:

```yaml
labels:
  qualcomm:
    aliases:
      - "qualcomm"
      - "qcom"
    marker_id: 17
```

Flujo:

```text
qualcomm -> zona_startups -> move_base(nav_goal) -> busqueda local -> detectar marker_id 17
```

## Estructura recomendada de semantic_map.yaml

Ejemplo completo:

```yaml
zones:
  entrada:
    name: "Entrada principal"
    aliases:
      - "entrada"
      - "inicio"
      - "recepcion"
    nav_goal:
      x: 0.50
      y: 0.20
      yaw: 0.00
    search_waypoints:
      - {x: 0.50, y: 0.20, yaw: 0.00}
      - {x: 0.80, y: 0.40, yaw: 1.57}
    labels: {}

  zona_startups:
    name: "Zona Startups"
    aliases:
      - "startups"
      - "zona startups"
      - "emprendedores"
    nav_goal:
      x: 2.10
      y: 1.80
      yaw: 1.57
    search_waypoints:
      - {x: 2.10, y: 1.80, yaw: 0.00}
      - {x: 2.50, y: 1.80, yaw: 1.57}
      - {x: 2.10, y: 2.20, yaw: 3.14}
    labels:
      qualcomm:
        aliases:
          - "qualcomm"
          - "qcom"
        marker_id: 17
      samsung:
        aliases:
          - "samsung"
        marker_id: 18

  hall_robotica:
    name: "Hall de Robotica"
    aliases:
      - "robotica"
      - "robots"
      - "demos robots"
    nav_goal:
      x: -1.20
      y: 3.40
      yaw: 0.00
    search_waypoints:
      - {x: -1.20, y: 3.40, yaw: 0.00}
      - {x: -1.60, y: 3.60, yaw: 1.57}
    labels:
      turtlebot:
        aliases:
          - "turtlebot"
          - "robot movil"
        marker_id: 21
```

## Como representar una zona sin geometria compleja

Para este proyecto no hace falta definir poligonos ni regiones matematicas. Es suficiente con:

1. Un punto principal de llegada: `nav_goal`.
2. Dos o tres puntos internos de busqueda: `search_waypoints`.

Ejemplo:

```yaml
search_waypoints:
  - {x: 2.10, y: 1.80, yaw: 0.00}
  - {x: 2.50, y: 1.80, yaw: 1.57}
  - {x: 2.50, y: 2.30, yaw: 3.14}
  - {x: 2.10, y: 2.30, yaw: -1.57}
```

El robot puede ir a cada waypoint y girar lentamente mientras consulta las detecciones de la camara.

## Conversion de quaternion a yaw

Si RViz o `/amcl_pose` te da orientacion como quaternion simplificado en 2D:

```yaml
orientation:
  z: 0.707
  w: 0.707
```

Puedes convertirlo a yaw con Python:

```python
import math

z = 0.707
w = 0.707

yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
print(yaw)
```

Valores utiles para empezar:

```text
0.00   -> mirando hacia +X
1.57   -> 90 grados
3.14   -> 180 grados
-1.57  -> -90 grados
```

## Recomendacion practica para la primera version

Primera version minima:

1. Crear 3 zonas.
2. Cada zona con 1 `nav_goal`.
3. Cada zona con 2 o 3 `search_waypoints`.
4. Cada etiqueta con un `marker_id` ArUco/AprilTag.
5. Probar una orden tipo `busca qualcomm`.

Secuencia esperada:

```text
busca qualcomm
  -> semantic_planner encuentra label qualcomm
  -> label pertenece a zona_startups
  -> navigation_manager manda nav_goal a move_base
  -> local_search_manager recorre waypoints y gira
  -> vision_detector detecta marker_id 17
  -> robot publica FOUND_TARGET y se detiene
```

## Criterios de validacion

Antes de dar por buena una etiqueta semantica:

- El `nav_goal` debe estar en zona libre del costmap.
- El robot debe poder llegar a ese goal desde RViz.
- Los `search_waypoints` no deben estar pegados a paredes u obstaculos.
- La orientacion `yaw` no debe ser estricta en pasillos estrechos; usa la tolerancia amplia de `navigation_with_guide.launch` salvo que una prueba concreta requiera mirar a un cartel.
- La etiqueta debe tener aliases naturales.
- Si usa marcador visual, el `marker_id` debe ser unico.
- La deteccion debe confirmarse durante varios frames antes de declarar exito.
