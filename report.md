# Reporte de decisiones tras la obtencion del mapa

## Contexto

Despues de obtener el mapa geometrico del entorno (`map.pgm` y `map.yaml`), se decidio construir una capa semantica separada para el proyecto **Event Guide Robot**. Esta capa no modifica el mapa de navegacion: el mapa sigue siendo usado por `map_server`, `amcl` y `move_base`, mientras que la informacion de zonas, stands y aliases se guarda en un YAML propio del paquete.

El archivo creado para esta capa es:

```text
catkin_ws/src/event_guide_robot/config/semantic_map.yaml
```

## Datos de partida

Las coordenadas iniciales salen del archivo:

```text
puntos.txt
```

Los puntos fueron capturados en el frame `map` y representan cuatro posiciones utiles del entorno:

| Punto medido | x | y |
| --- | ---: | ---: |
| Arriba | -0.529138445854187 | -0.044567957520484924 |
| Izquierda | 0.2924858331680298 | -0.3563343286514282 |
| Abajo | 0.853394627571106 | 0.3277086019515991 |
| Derecha | -0.07007773220539093 | 0.84162437915802 |

## Decision 1: crear un mapa semantico separado

Se decidio no editar directamente `map.pgm` ni `map.yaml` para anadir significado semantico. En su lugar, se creo `semantic_map.yaml`.

Motivos:

- mantiene separado el mapa geometrico del mapa semantico;
- evita romper la compatibilidad con `map_server`, `amcl` y `move_base`;
- permite cambiar nombres de zonas, stands o etiquetas sin regenerar el mapa;
- facilita probar el parser semantico con tests unitarios.

## Decision 2: usar cuatro zonas semanticas iniciales

Cada punto medido se transformo en una zona semantica:

| Zona semantica | Punto origen | Uso previsto |
| --- | --- | --- |
| `zona_arriba` | Arriba | Zona de navegacion y busqueda local |
| `zona_izquierda` | Izquierda | Zona de navegacion y busqueda local |
| `zona_abajo` | Abajo | Zona de navegacion y busqueda local |
| `zona_derecha` | Derecha | Zona de navegacion y busqueda local |

Cada zona incluye:

- `name`: nombre legible;
- `aliases`: formas en que el usuario puede pedir esa zona;
- `nav_goal`: pose principal a la que debe navegar el robot;
- `search_waypoints`: puntos de busqueda local;
- `labels`: stands o senales visuales que se pueden buscar dentro de la zona.

## Decision 3: calcular orientaciones iniciales hacia el centro

Los puntos medidos solo daban posicion `x/y`, no una orientacion final clara. Para tener una primera version funcional, se calculo el centroide de los cuatro puntos:

```text
centroide x = 0.13666607066988945
centroide y = 0.1921076737344265
```

A partir de ese centroide, se calculo un `yaw` para que el robot mire aproximadamente hacia el centro del area desde cada zona.

Esta decision es provisional. Si durante la demo se ve que el robot debe mirar hacia una pared, cartel o pasillo concreto, los `yaw` deberan sustituirse por orientaciones medidas desde RViz o `/amcl_pose`.

## Decision 4: usar al menos dos stands por zona

Para que el escenario parezca un evento tipo **Mobile World Congress**, se decidio que cada zona tendra al menos dos stands con nombres inspirados en empresas o areas tecnologicas reales.

Distribucion actual:

| Zona | Stands |
| --- | --- |
| `zona_arriba` | Qualcomm AI Hub, Samsung Galaxy Experience |
| `zona_izquierda` | Telefonica Open Gateway, Nokia Networks Lab |
| `zona_abajo` | Ericsson 5G Arena, GSMA Innovation City |
| `zona_derecha` | Meta XR Showcase, NVIDIA Edge AI |

Estos nombres permiten que el usuario pueda pedir objetivos de forma natural, por ejemplo:

```text
busca el stand de Qualcomm
ve al stand de Ericsson
quiero ir a NVIDIA
```

## Decision 5: guardar aliases por zona y por stand

Cada zona y cada stand tienen varios aliases. Esto permite que el parser semantico no dependa de una unica frase exacta.

Ejemplo:

```yaml
qualcomm_ai_hub:
  display_name: "Qualcomm AI Hub"
  aliases:
    - "qualcomm"
    - "qualcomm ai"
    - "qualcomm ai hub"
    - "stand qualcomm"
```

Esto sera util para `semantic_planner_node`, que debera resolver comandos de usuario contra estos aliases.

## Decision 6: implementar una interfaz de vision generica

Aunque la estrategia visual final sigue abierta, se decidio que el resto del sistema no debe depender directamente de si se usa ArUco, AprilTag o un detector de numeros.

Por eso `vision_detector_node.py` publica detecciones genericas en JSON:

```json
{
  "marker_id": 11,
  "label_id": "qualcomm_ai_hub",
  "confidence": 1.0,
  "stable": true,
  "stamp": 1779870000.0
}
```

Esta interfaz permite cambiar el detector interno sin modificar el planificador, la navegacion ni la busqueda local.

## Decision 7: exigir detecciones estables antes de parar

Se decidio que `local_search_manager_node.py` solo aceptara detecciones con:

```text
stable: true
```

Motivo:

- evita parar el robot por falsos positivos de un solo frame;
- encaja tanto con ArUco/AprilTag como con un detector de numeros;
- permite configurar el numero de frames consecutivos necesarios desde el nodo de vision.

## Decision pendiente: estrategia de vision final

Todavia no se ha cerrado si la deteccion visual final usara:

1. marcadores ArUco o AprilTag;
2. un modelo visual pequeno entrenado para detectar numeros;
3. una estrategia hibrida.

### Opcion A: ArUco o AprilTag

Ventajas:

- muy robusto para demo;
- facil de implementar con OpenCV;
- no requiere entrenar un modelo;
- devuelve directamente un ID numerico;
- funciona bien con pocos datos y en tiempo real.

Inconvenientes:

- puede parecer una solucion menos ambiciosa;
- visualmente es menos natural que reconocer carteles reales;
- la evaluacion puede valorarla como una deteccion artificial.

### Opcion B: modelo pequeno para detectar numeros

La idea seria poner numeros visibles en los stands y entrenar o adaptar un modelo ligero para reconocerlos con la camara.

Ventajas:

- es mas complejo tecnicamente;
- puede ser mejor valorado si se explica como parte de vision artificial;
- permite una demo mas natural que un marcador cuadrado;
- obliga a trabajar con dataset, entrenamiento, validacion y metricas.

Inconvenientes:

- requiere mas tiempo;
- puede fallar por iluminacion, distancia, movimiento o desenfoque;
- necesita recopilar imagenes del entorno real o simulado;
- puede ser pesado para la Raspberry Pi si no se optimiza;
- aumenta el riesgo de que la demo falle.

### Opcion C: estrategia hibrida recomendada

La opcion mas segura es preparar dos niveles:

1. **Principal demostrable:** ArUco/AprilTag como fallback robusto.
2. **Extension avanzada:** detector de numeros como mejora de vision.

Asi se puede presentar el sistema como:

- navegacion y mapa semantico funcionando siempre;
- deteccion robusta con marcador para asegurar la demo;
- investigacion adicional con detector de numeros si hay tiempo.

Esta estrategia reduce el riesgo de fallo en la presentacion sin renunciar a una parte mas ambiciosa.

## Decision provisional sobre `marker_id`

En el YAML actual se han dejado campos `marker_id` numericos para cada stand:

| Stand | ID provisional |
| --- | ---: |
| Qualcomm AI Hub | 11 |
| Samsung Galaxy Experience | 12 |
| Telefonica Open Gateway | 21 |
| Nokia Networks Lab | 22 |
| Ericsson 5G Arena | 31 |
| GSMA Innovation City | 32 |
| Meta XR Showcase | 41 |
| NVIDIA Edge AI | 42 |

Estos IDs son compatibles con ArUco/AprilTag, pero tambien pueden reutilizarse como identificadores logicos si finalmente se detectan numeros con un modelo visual. Por tanto, mantenerlos en el YAML no obliga todavia a elegir una tecnologia concreta.

## Validacion realizada

Se crearon tests para comprobar la estructura del mapa semantico y el comportamiento de los nodos principales:

```text
catkin_ws/src/event_guide_robot/test/test_semantic_map.py
catkin_ws/src/event_guide_robot/test/test_command_parser.py
catkin_ws/src/event_guide_robot/test/test_navigation_manager.py
catkin_ws/src/event_guide_robot/test/test_local_search_manager.py
catkin_ws/src/event_guide_robot/test/test_vision_detector.py
catkin_ws/src/event_guide_robot/test/test_launch_files.py
```

La validacion comprueba, entre otras cosas:

- que existe la seccion `metadata`;
- que existen zonas semanticas;
- que cada zona tiene `name`, `aliases`, `nav_goal`, `search_waypoints` y `labels`;
- que cada `nav_goal` y waypoint tiene `x`, `y`, `yaw` numericos;
- que existen las cuatro zonas medidas;
- que cada zona tiene al menos dos stands;
- que cada stand tiene aliases e ID numerico.
- que el parser resuelve comandos como `quiero ir al stand de Qualcomm`;
- que comandos desconocidos devuelven `NOT_FOUND`;
- que `navigation_manager_node.py` convierte `x/y/yaw` a un `MoveBaseGoal`;
- que `local_search_manager_node.py` solo acepta detecciones estables;
- que `vision_detector_node.py` construye el indice `marker_id -> label_id`;
- que los launch files son XML validos y lanzan los nodos esperados.

Resultado:

```text
25 passed
```

Tambien se valido:

```text
python3 -m py_compile catkin_ws/src/event_guide_robot/scripts/*.py
XML valido para package.xml y launch files
YAML valido para semantic_map.yaml y search_params.yaml
```

No se pudo validar con `catkin_make` ni `catkin build` porque el entorno actual no tiene instaladas esas herramientas:

```text
catkin_make: command not found
catkin: command not found
```

## Proximos pasos

1. Revisar en RViz si las poses de cada zona son navegables y seguras.
2. Confirmar o ajustar los `yaw` usando `2D Nav Goal` o `/amcl_pose`.
3. Ejecutar `catkin_make` en una maquina con ROS instalado.
4. Lanzar:
   ```bash
   roslaunch event_guide_robot guide_system.launch
   ```
5. Probar comandos de usuario:
   ```bash
   rostopic pub /guide/command std_msgs/String "data: 'quiero ir al stand de Qualcomm'"
   ```
6. Decidir la estrategia de vision:
   - ArUco/AprilTag;
   - detector de numeros;
   - enfoque hibrido.
7. Si se elige detector de numeros, definir:
   - formato de los carteles;
   - numeros por stand;
   - dataset necesario;
   - metrica minima de acierto;
   - fallback para la demo.
8. Validar con robot/simulacion que se usan realmente `/cmd_vel`, `/scan` e imagenes de camara.
