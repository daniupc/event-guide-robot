# Event Guide Robot - TurtleBot3 Waffle en ROS 1

Proyecto final de Robotica Movil para un TurtleBot3 Waffle/Waffle Pi usando ROS 1. La idea del proyecto es convertir el robot en un guia autonomo de un evento tipo MWC: el usuario indica un destino o stand, el robot navega hasta la zona general del mapa y despues busca visualmente una senal concreta dentro de esa zona.

## Resumen del enunciado

El enunciado del proyecto pide desarrollar e implementar algoritmos de navegacion y control para plataformas roboticas moviles, usando TurtleBot3 Waffle. Las tareas sugeridas incluyen exploracion de zonas desconocidas, seguimiento con camara, busqueda de objetos y mapas semanticos. Los topics minimos indicados son:

- `/cmd_vel`
- `/images` o topic equivalente de camara
- `/scan`

La entrega debe incluir descripcion del proyecto, solucion desarrollada, videos del funcionamiento y demostracion en vivo, en simulacion o robot real.

## Concepto del proyecto

El robot se comporta como guia de un evento:

1. El usuario pide un destino, por ejemplo: `llevame al stand de Qualcomm`.
2. El sistema interpreta la peticion y la asocia a una zona semantica del mapa, por ejemplo `zona_startups` o `hall_robotica`.
3. El robot navega con `move_base` hasta una pose segura dentro de esa zona.
4. Al llegar, entra en modo de busqueda local.
5. Mediante la camara, busca una etiqueta, cartel, marcador o senal concreta indicada por el usuario.
6. Cuando la deteccion visual confirma el objetivo, el robot se detiene e informa de que lo ha encontrado.

## Objetivo tecnico

Construir una arquitectura ROS 1 modular que combine:

- navegacion global sobre mapa ya generado;
- localizacion con AMCL;
- planificacion y evitacion de obstaculos con `move_base` y `/scan`;
- capa semantica del mapa mediante YAML;
- busqueda visual local con camara;
- deteccion de senales mediante marcadores ArUco como decision fija del MVP;
- maquina de estados para coordinar el flujo completo.

## Estado actual de implementacion

El paquete ROS ya esta creado en:

```text
catkin_ws/src/event_guide_robot/
```

Componentes implementados:

- `semantic_planner_node.py`: recibe `/guide/command`, resuelve aliases de zonas/stands desde `semantic_map.yaml` y publica un plan JSON en `/guide/plan`.
- `navigation_manager_node.py`: consume `/guide/plan`, construye un `MoveBaseGoal` en frame `map`, lo envia a `/move_base` y publica estados/resultados.
- `local_search_manager_node.py`: recibe `/guide/plan`, espera a `/guide/state = NAVIGATION_SUCCEEDED`, entonces gira con `/cmd_vel`, escucha `/vision/detections`, exige deteccion estable y publica parada segura.
- `vision_detector_node.py`: detector ArUco que publica JSON en `/vision/detections`; usa OpenCV `cv2.aruco` y asocia cada `marker_id` al stand definido en el mapa semantico.
- `guide_system.launch`: lanza los cuatro nodos del sistema guia.
- `navigation_with_guide.launch`: integra `turtlebot3_navigation` con el sistema guia.

Validacion local disponible:

```text
26 tests unitarios pasan con pytest.
py_compile OK para los nodos Python.
XML/YAML validos.
```

Validacion en robot real:

```text
Probado en TurtleBot3 real con ROS 1 usando el catkin_ws original.
El mapa se carga y RViz lo muestra correctamente cuando /use_sim_time=false.
La navegacion con move_base funciona con el mapa real y los goals semanticos.
El parser resuelve aliases como qualcomm y publica /guide/plan.
La camara/vision ArUco queda pendiente de probar.
```

Nota importante para robot real: si antes se uso Gazebo, puede quedar `/use_sim_time=true`. En robot real debe ser `false`; si no, `/map`, `/map_metadata` o RViz pueden parecer bloqueados aunque `map_server` haya cargado el mapa.

## Arquitectura implementada

```text
/guide/command
     |
     v
semantic_planner_node
     |
     | /guide/plan JSON
     v
navigation_manager_node ---> move_base ---> /cmd_vel
     |                       ^
     |                       |
     |               map_server + amcl + /scan
     v
local_search_manager_node
     |
     v
vision_detector_node <--- /raspicam_node/image
     |
     v
Resultado: objetivo encontrado / no encontrado
```

### Componentes

#### 1. `semantic_planner_node`

Responsabilidad: traducir una orden del usuario a una zona general y a un objetivo visual concreto.

Entrada esperada:

```text
/guide/command: "busca el stand de qualcomm"
```

Salida actual:

```text
/guide/plan: JSON con status, zone_id, label_id, display_name, marker_id,
             nav_goal y search_waypoints
```

Este nodo no debe navegar ni procesar imagenes. Solo decide que zona corresponde a la peticion.

#### 2. `navigation_manager_node`

Responsabilidad: mandar objetivos a `move_base` y supervisar si el robot llega o falla.

Funciones actuales:

- enviar goal principal de la zona;
- esperar resultado de `move_base`;
- cancelar goal si hay timeout;
- reportar errores si una zona es inalcanzable;
- publicar `NAVIGATE_TO_ZONE`, `NAVIGATION_SUCCEEDED` o `NAVIGATION_FAILED`.

#### 3. `local_search_manager_node`

Responsabilidad: buscar dentro de una zona tras llegar al goal principal.

Estrategia implementada actualmente:

1. Recibir el plan del stand objetivo y dejarlo en espera.
2. Esperar a que `navigation_manager_node` publique `NAVIGATION_SUCCEEDED`.
3. Girar lentamente publicando `geometry_msgs/Twist` en `/cmd_vel`.
4. Consultar continuamente `/vision/detections`.
5. Aceptar solo detecciones con `stable: true`.
6. Cuando encuentra el ArUco correcto, entrar en `APPROACH_TARGET`.
7. Centrar el marcador en la imagen y avanzar lentamente hacia el hasta quedar a unos 20 cm.
8. Detener el robot publicando velocidad cero y publicar `FOUND_TARGET`.
9. Detener el robot y publicar fallo si se agota el timeout.

La navegacion entre varios waypoints de busqueda queda como siguiente mejora.

#### 4. `vision_detector_node`

Responsabilidad: analizar imagenes de la camara y publicar detecciones.

Estado actual:

- publica detecciones genericas JSON con `marker_id`, `label_id`, `confidence`, `stable` y `stamp`;
- usa ArUco si `cv2.aruco` esta disponible;
- si no hay dependencias de vision, avisa con `logwarn` y no publica falsas detecciones;
- mantiene un formato JSON simple para desacoplar vision de la logica de busqueda.

Decision de vision:

- **MVP final:** ArUco en cada cartel/stand.
- Cada stand usa el `marker_id` definido en `semantic_map.yaml`.
- OCR, AprilTag o detectores de logos quedan solo como mejoras futuras, no como dependencia de la demo.

Para una demo estable se recomienda imprimir marcadores ArUco grandes y de alto contraste, ejecutando vision en el PC remoto si la Raspberry Pi no aguanta el coste de computo.

## Mapa semantico actual

No se recomienda modificar directamente la imagen del mapa (`.pgm`). La capa semantica debe vivir en un archivo separado, por ejemplo:

```text
catkin_ws/src/event_guide_robot/config/semantic_map.yaml
```

El mapa semantico actual contiene cuatro zonas medidas desde `puntos.txt`, con dos stands por zona:

| Zona | Stands |
| --- | --- |
| `zona_arriba` | Qualcomm AI Hub, Samsung Galaxy Experience |
| `zona_izquierda` | Telefonica Open Gateway, Nokia Networks Lab |
| `zona_abajo` | Ericsson 5G Arena, GSMA Innovation City |
| `zona_derecha` | Meta XR Showcase, NVIDIA Edge AI |

Los `marker_id` actuales son los IDs ArUco que deben imprimirse y colocarse fisicamente junto a cada cartel/stand.

Ejemplo real del YAML:

```yaml
zones:
  zona_arriba:
    name: "Zona arriba"
    aliases:
      - "arriba"
      - "zona arriba"
    nav_goal:
      x: -0.529138445854187
      y: -0.044567957520484924
      yaw: 0.3415423349961051
    search_waypoints:
      - {x: -0.529138445854187, y: -0.044567957520484924, yaw: 0.3415423349961051}
    labels:
      qualcomm_ai_hub:
        display_name: "Qualcomm AI Hub"
        aliases: ["qualcomm", "qualcomm ai", "qualcomm ai hub", "stand qualcomm"]
        marker_id: 11
```

## Maquina de estados

La coordinacion principal debe seguir una maquina de estados simple:

```text
IDLE
  -> WAIT_COMMAND
  -> PARSE_TARGET
  -> NAVIGATE_TO_ZONE
  -> LOCAL_VISUAL_SEARCH
  -> APPROACH_TARGET
  -> FOUND_TARGET
  -> IDLE
```

Estados de error:

```text
PARSE_FAILED       si el comando no corresponde a ninguna zona/etiqueta
NAVIGATION_FAILED  si move_base no puede llegar a la zona
SEARCH_FAILED      si no se detecta la senal dentro del tiempo o waypoints definidos
```

## Topics y acciones ROS

Topics de entrada/salida recomendados:

```text
/guide/command              std_msgs/String
/guide/state                std_msgs/String
/guide/result               std_msgs/String
/guide/plan                 std_msgs/String con JSON
/vision/detections          std_msgs/String con JSON
/cmd_vel                    geometry_msgs/Twist
/scan                       sensor_msgs/LaserScan
/raspicam_node/image               sensor_msgs/Image
/amcl_pose                  geometry_msgs/PoseWithCovarianceStamped
```

Accion principal:

```text
/move_base                  move_base_msgs/MoveBaseAction
```

## Estructura de paquete actual

```text
catkin_ws/src/event_guide_robot/
  CMakeLists.txt
  package.xml
  maps/
    map.yaml
    map.pgm
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
  test/
    test_semantic_map.py
    test_command_parser.py
    test_navigation_manager.py
    test_local_search_manager.py
    test_vision_detector.py
    test_launch_files.py
```

## Plan de implementacion

### Fase 0 - Preparacion

- Confirmar si se trabajara en simulacion, robot real o ambos.
- Crear o cargar un mapa del entorno.
- Verificar que `turtlebot3_bringup`, camara, `/scan` y teleoperacion funcionan.
- Guardar poses reales de zonas usando RViz.

### Fase 1 - Navegacion base

- Lanzar `map_server`, `amcl` y `move_base`.
- Comprobar que el robot llega a goals manuales desde RViz.
- Ajustar inflation radius, footprint y parametros de costmap si el robot roza obstaculos.
- Grabar video corto de navegacion simple para evidencia.

### Fase 2 - Mapa semantico

- **Hecho:** crear `semantic_map.yaml`.
- **Hecho:** definir cuatro zonas iniciales a partir de `puntos.txt`.
- **Hecho:** anadir aliases naturales para cada zona y stand.
- **Hecho:** anadir una pose principal y un waypoint inicial por zona.
- **Hecho:** crear pruebas unitarias para validar que el YAML contiene campos obligatorios.

### Fase 3 - Parser de comandos

- **Hecho:** implementar `semantic_planner_node.py`.
- **Hecho:** matching simple por aliases.
- **Hecho:** si una frase contiene un alias de zona o etiqueta, devuelve la zona correcta.
- **Hecho:** publicar errores claros si no hay match.

### Fase 4 - Integracion con `move_base`

- **Hecho en codigo:** implementar `navigation_manager_node.py` usando `actionlib`.
- **Hecho en codigo:** enviar el goal principal de la zona.
- **Hecho en codigo:** esperar resultado.
- **Hecho en codigo:** publicar estado en `/guide/state`.
- **Hecho en codigo:** manejar `SUCCEEDED`, fallo de action server y timeout.
- **Hecho en launch:** relajar `yaw_goal_tolerance` para que no tenga que apuntar al centro del mapa al llegar.
- **Validado en robot real:** `move_base` navega con el mapa real cuando `/use_sim_time=false`.

### Fase 5 - Busqueda local

- **Hecho en codigo:** implementar `local_search_manager_node.py`.
- **Hecho en codigo:** esperar a `NAVIGATION_SUCCEEDED` antes de girar con `/cmd_vel`.
- **Hecho en codigo:** revisar detecciones mientras gira.
- **Hecho en codigo:** al detectar el ArUco correcto, aproximarse visualmente hasta unos 20 cm si la distancia estimada esta disponible.
- **Hecho en codigo:** parar el robot al alcanzar el objetivo o al agotar timeout.
- **Pendiente:** visitar waypoints adicionales si el primer giro no encuentra el objetivo.

### Fase 6 - Vision MVP

- **Hecho en codigo:** detector generico con soporte ArUco si `cv2.aruco` esta disponible.
- **Hecho en codigo:** asociar `marker_id` a `label_id` usando `semantic_map.yaml`.
- **Hecho en codigo:** requerir deteccion estable durante varios frames.
- **Hecho en codigo:** publicar detecciones JSON en `/vision/detections`.
- **Decision cerrada:** ArUco es la solucion final de vision para el MVP.

### Fase 7 - Vision mejorada opcional

- No forma parte del MVP.
- Posibles mejoras futuras: OCR, AprilTag o detector ligero de logos.
- Mantener siempre ArUco como fallback robusto de demo.

### Fase 8 - Demo final

- Preparar entorno con 2 o 3 zonas.
- Colocar senales visibles.
- Ejecutar una demo exitosa y una demo de fallo controlado.
- Grabar video mostrando RViz, camara y robot real/simulado.
- Preparar presentacion con arquitectura, decisiones, limitaciones y mejoras futuras.

## Estrategia de vision establecida

La vision del MVP queda cerrada con ArUco:

1. imprimir un marcador ArUco por stand;
2. usar los `marker_id` numericos de `semantic_map.yaml` como IDs reales de los marcadores;
3. exigir varias detecciones consecutivas antes de aceptar el objetivo;
4. publicar `center_offset_x` y `distance_m` estimados para la aproximacion final;
5. publicar siempre JSON en `/vision/detections` para que la busqueda local no dependa de detalles de OpenCV.


### Orientacion final de los goals

En las primeras pruebas reales, los `yaw` calculados hacia el centro del mapa hacian que el robot tardase mucho en recolocarse al final del trayecto, especialmente en espacios estrechos. La solucion elegida es no exigir una orientacion final estricta para los goals semanticos: `navigation_with_guide.launch` configura una tolerancia de yaw de vuelta completa (`6.283185307` rad).

El `MoveBaseGoal` sigue llevando quaternion porque ROS lo requiere, pero `move_base` debe considerar valido llegar al `x/y` aunque el robot mantenga la orientacion natural de llegada.

## Guia rapida de ejecucion en robot real

Los comandos por terminal para probar despues de los bringups estan separados en:

```text
docs/real-robot-runbook.md
```


### Aproximacion final al ArUco

Tras detectar de forma estable el marcador correcto, el robot ya no termina inmediatamente: entra en `APPROACH_TARGET`. En ese estado usa la posicion horizontal del ArUco en la imagen (`center_offset_x`) para girar suavemente hacia la etiqueta y la distancia estimada (`distance_m`) para avanzar despacio hasta quedar a unos 20 cm.

Parametros principales en `config/search_params.yaml`:

```yaml
approach_enabled: true
approach_target_distance_m: 0.20
approach_forward_speed_m_s: 0.08
approach_turn_gain: 0.6
approach_max_turn_speed_rad_s: 0.25
approach_center_deadband: 0.15
approach_timeout_sec: 20.0
```

La distancia se estima a partir del tamano aparente del marcador. El launch asume marcadores ArUco impresos de `0.16 m` de lado. Si imprimes otro tamano, ajusta `marker_size_m` en `guide_system.launch`.

## Comandos base del robot

### Nota critica: tiempo real vs simulacion

En robot real asegura siempre:

```bash
rosparam set /use_sim_time false
rosparam get /use_sim_time
```

Debe devolver `false`. Si estaba en `true` por haber usado Gazebo, `rostopic echo -n 1 /map` o `/map_metadata` puede no devolver datos y RViz puede no mostrar bien el mapa. Tras cambiarlo, relanza `map_server`/navegacion.

En el PC:

```bash
export TURTLEBOT3_MODEL=waffle_pi
roscore
```

En el robot:

```bash
ssh ubuntu@IP_ROBOT
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Camara oficial del proyecto: TurtleBot3 Raspberry Pi Camera (`rpicamera`).

```bash
ssh ubuntu@IP_ROBOT
roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
```

El topic de imagen cruda esperado por defecto es:

```text
/raspicam_node/image
```

El topic comprimido `/raspicam_node/image/compressed` puede existir, pero `vision_detector_node.py` usa `sensor_msgs/Image`, por lo que debe recibir la imagen cruda.

Remote en PC:

```bash
roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

Navegacion + sistema guia recomendado:

```bash
roslaunch event_guide_robot navigation_with_guide.launch
```

El launch usa por defecto el mapa empaquetado en `$(find event_guide_robot)/maps/map.yaml`. Si quieres probar otro mapa, puedes sobrescribirlo con:

```bash
roslaunch event_guide_robot navigation_with_guide.launch \
  map_file:=/ruta/a/otro/map.yaml
```

Este launch incluye `turtlebot3_navigation` y nuestros nodos guia. Ademas relaja la orientacion final del goal con:

```text
/move_base/DWAPlannerROS/yaw_goal_tolerance = 6.283185307
/move_base/TrajectoryPlannerROS/yaw_goal_tolerance = 6.283185307
```

Asi el robot no pierde tiempo recolocandose para mirar a un yaw concreto al llegar a una zona estrecha; acepta la orientacion con la que llegue y pasa a la busqueda visual.

Si por algun motivo lanzas navegacion por separado, usa despues solo el sistema guia:

```bash
roslaunch event_guide_robot guide_system.launch
```

En ese caso la tolerancia de yaw depende de la configuracion de `move_base` que hayas cargado.

Prueba de comando:

```bash
rostopic pub /guide/command std_msgs/String "data: 'quiero ir al stand de Qualcomm'"
rostopic echo /guide/state
rostopic echo /guide/plan
rostopic echo /guide/result
rostopic echo /vision/detections
```

Validacion local sin ROS:

```bash
python3 -m pytest catkin_ws/src/event_guide_robot/test -q
python3 -m py_compile catkin_ws/src/event_guide_robot/scripts/*.py
```

## Criterios de exito

El proyecto se considera completo si demuestra:

- el robot localiza su posicion en un mapa;
- recibe una orden de destino;
- escoge una zona semantica correcta;
- navega autonomamente hasta esa zona;
- ejecuta una busqueda local;
- detecta una senal concreta con la camara;
- se detiene al encontrar el objetivo;
- informa claramente del resultado;
- usa `/cmd_vel`, `/scan` e imagenes de camara.

## Riesgos y mitigaciones

| Riesgo | Mitigacion |
| --- | --- |
| Marcador ArUco no se detecta por tamano, angulo o luz | Imprimir marcadores grandes, alto contraste, probar distancia y exigir N frames |
| `move_base` no llega al goal | Definir poses seguras y waypoints alternativos |
| Camara lenta en Raspberry Pi | Procesar imagen en PC remoto |
| Falsos positivos visuales | Requerir N detecciones consecutivas |
| Robot se pierde en AMCL | Empezar cada demo con pose inicial bien fijada en RViz |
| Obstaculos dinamicos bloquean zona | Timeout y mensaje de fallo controlado |

## Mejoras futuras

- Interfaz de usuario opcional para la demo: primero una CLI Python interactiva que liste stands, publique en `/guide/command` y muestre `/guide/state` + `/guide/result`; despues, si sobra tiempo, una UI sencilla con Tkinter.
- Interfaz por voz.
- Visualizacion de zonas semanticas en RViz.
- OCR o AprilTag como extension futura, sin afectar al MVP ArUco.
- Replanificacion si una zona queda bloqueada.
- Dialogo multi-turno con el usuario.
- Deteccion de personas para acercarse al usuario antes de recibir ordenes.
