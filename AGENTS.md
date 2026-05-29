# AGENTS.md - Event Guide Robot

## Rol del agente

Eres un agente de desarrollo para el proyecto `Event Guide Robot`, un proyecto ROS 1 con TurtleBot3 Waffle/Waffle Pi. Tu objetivo es ayudar a implementar, documentar, probar y depurar un robot guia de evento que navega hasta zonas semanticas del mapa y busca senales concretas con la camara.

Trabaja de forma autonoma cuando la tarea sea clara. No pidas confirmacion para pasos reversibles como leer archivos, crear codigo, ejecutar pruebas locales o mejorar documentacion. Pregunta solo si falta informacion que cambie de forma importante la arquitectura, si hay riesgo destructivo o si se requiere acceso a hardware que no esta disponible.

## Contexto del proyecto

El enunciado pide usar TurtleBot3 Waffle para tareas de navegacion y control en Robotica Movil. Los topics minimos son:

- `/cmd_vel`
- `/images` o topic equivalente de camara
- `/scan`

La idea elegida es un guia de evento tipo MWC:

1. El usuario indica una zona, stand o etiqueta.
2. El robot usa un mapa semantico para ir a la zona general.
3. Dentro de esa zona, usa la camara para buscar una senal especifica.
4. Cuando la encuentra, se detiene y reporta exito.

## Stack esperado

- ROS 1, preferiblemente Noetic si el entorno lo permite.
- TurtleBot3 Waffle/Waffle Pi.
- `turtlebot3_bringup`.
- `turtlebot3_navigation`.
- `map_server`, `amcl`, `move_base`.
- Python para nodos ROS.
- OpenCV para vision.
- ArUco/AprilTag como deteccion MVP recomendada.
- OCR como mejora opcional, no como dependencia critica de la demo.

## Principios de implementacion

- Prioriza una demo robusta sobre una arquitectura demasiado ambiciosa.
- Mantén la navegacion global separada de la busqueda visual local.
- No modifiques directamente el mapa `.pgm` para informacion semantica; usa YAML aparte.
- Implementa primero una version minima fiable y luego mejora.
- Evita dependencias pesadas en la Raspberry Pi si el procesamiento puede correr en el PC remoto.
- Publica estados claros para poder depurar desde `rostopic echo` y RViz.
- Cada nodo debe tener una responsabilidad concreta.

## Arquitectura objetivo

```text
/guide/command
      |
      v
semantic_planner_node
      |
      v
navigation_manager_node ---> move_base
      |                       |
      |                       v
      |                    /cmd_vel
      v
local_search_manager_node
      |
      v
vision_detector_node <--- /camera/image
```

## Paquete recomendado

Si el codigo aun no existe, crea un paquete llamado:

```text
event_guide_robot
```

Estructura recomendada:

```text
catkin_ws/src/event_guide_robot/
  CMakeLists.txt
  package.xml
  launch/
    guide_system.launch
    navigation_with_guide.launch
  config/
    semantic_map.yaml
    search_params.yaml
  scripts/
    semantic_planner_node.py
    navigation_manager_node.py
    local_search_manager_node.py
    vision_detector_node.py
  msg/
    Detection.msg
  srv/
    FindTarget.srv
  test/
    test_semantic_map.py
    test_command_parser.py
```

## Responsabilidades por nodo

### `semantic_planner_node.py`

- Carga `config/semantic_map.yaml`.
- Recibe comandos de usuario.
- Hace matching por aliases.
- Devuelve zona, goal principal, waypoints de busqueda y etiqueta objetivo.
- No debe llamar directamente a OpenCV ni publicar `/cmd_vel`.

### `navigation_manager_node.py`

- Usa `actionlib` con `/move_base`.
- Envia goals a zonas y waypoints.
- Publica estado en `/guide/state`.
- Maneja success, timeout, abort y cancelacion.
- No debe procesar imagenes.

### `local_search_manager_node.py`

- Coordina la busqueda local dentro de la zona.
- Gira el robot lentamente con `/cmd_vel` cuando proceda.
- Puede pedir a `navigation_manager_node` moverse entre waypoints.
- Se detiene inmediatamente si `vision_detector_node` confirma el objetivo.
- Debe tener timeout para evitar bucles infinitos.

### `vision_detector_node.py`

- Se suscribe a imagenes de camara.
- Detecta marcadores o texto.
- Publica detecciones con identificador, confianza y timestamp.
- Para MVP, prefiere ArUco/AprilTag.
- Requiere varias detecciones consecutivas antes de declarar objetivo encontrado.

## Mapa semantico

Usa `config/semantic_map.yaml` como fuente de verdad para zonas, aliases y waypoints. Debe incluir como minimo:

```yaml
zones:
  zona_id:
    name: "Nombre visible"
    aliases: ["alias 1", "alias 2"]
    nav_goal:
      x: 0.0
      y: 0.0
      yaw: 0.0
    search_waypoints:
      - {x: 0.0, y: 0.0, yaw: 0.0}
    labels:
      etiqueta_id:
        aliases: ["texto usuario"]
        marker_id: 1
```

No uses valores ficticios en codigo final. Los valores de posicion deben salir de RViz o de mediciones reales del mapa.

## Topics recomendados

```text
/guide/command        std_msgs/String
/guide/state          std_msgs/String
/guide/result         std_msgs/String
/vision/detections    mensaje propio o std_msgs/String para MVP
/camera/image         sensor_msgs/Image
/scan                 sensor_msgs/LaserScan
/cmd_vel              geometry_msgs/Twist
/amcl_pose            geometry_msgs/PoseWithCovarianceStamped
/move_base            move_base_msgs/MoveBaseAction
```

## Estrategia de desarrollo

1. Verifica primero que el robot navega con `move_base` a goals manuales desde RViz.
2. Crea el YAML semantico con pocas zonas.
3. Implementa parser por aliases.
4. Integra `move_base` con `actionlib`.
5. Implementa busqueda local con giro controlado.
6. Implementa deteccion visual robusta con ArUco/AprilTag.
7. Integra el flujo completo.
8. Anade OCR solo si la demo MVP ya funciona.

## Testing y validacion

Antes de declarar una tarea completada, ejecuta las comprobaciones aplicables:

- `catkin_make` o `catkin build`.
- `roslaunch` de los launch files principales.
- `rosrun` de cada nodo individual cuando sea posible.
- `rostopic echo /guide/state` para verificar estados.
- `rostopic echo /vision/detections` para verificar detecciones.
- Goal manual en RViz para validar navegacion.
- Prueba con al menos una zona encontrada y una zona no encontrada.

Si no hay robot disponible, valida en simulacion o con pruebas unitarias de parser/YAML y deja claro que falta validacion en hardware.

## Convenciones de codigo

- Python ejecutable en `scripts/` con shebang `#!/usr/bin/env python3` si se usa Noetic.
- Nodos pequenos y con responsabilidades separadas.
- Parametros configurables por YAML o ROS params, no hardcodeados.
- Logs con `rospy.loginfo`, `rospy.logwarn`, `rospy.logerr`.
- Timeouts explicitos para navegacion y busqueda.
- Manejo seguro de parada: publicar velocidad cero antes de terminar una busqueda.

## Criterios de aceptacion

El proyecto debe poder demostrar:

- recepcion de una orden de usuario;
- seleccion correcta de zona semantica;
- navegacion autonoma hasta la zona;
- busqueda local usando camara;
- deteccion de una senal concreta;
- parada del robot al encontrarla;
- comunicacion de exito o fallo;
- uso real de `/cmd_vel`, `/scan` e imagenes de camara.

## Riesgos conocidos

- OCR puede fallar por iluminacion, desenfoque o movimiento.
- AMCL necesita una pose inicial razonable para demos fiables.
- Los waypoints deben estar en zonas libres del costmap.
- La Raspberry Pi puede no ser suficiente para vision pesada.
- Las detecciones deben confirmarse durante varios frames para reducir falsos positivos.

## Recomendacion de demo

La demo final debe preparar un entorno pequeno y controlado con 2 o 3 zonas, carteles visibles y marcadores robustos. El flujo recomendado para la presentacion es:

1. Mostrar el mapa en RViz.
2. Enviar comando de usuario.
3. Ver el estado `NAVIGATE_TO_ZONE`.
4. Ver al robot llegar a la zona.
5. Ver el estado `LOCAL_VISUAL_SEARCH`.
6. Mostrar deteccion visual.
7. Ver al robot detenerse y reportar exito.
