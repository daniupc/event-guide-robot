# Explicacion detallada de los nodos

Este documento explica explicitamente que hace cada nodo del paquete `event_guide_robot` y como se comunican entre ellos.

## 1. `semantic_planner_node.py`

Ruta:

```text
catkin_ws/src/event_guide_robot/scripts/semantic_planner_node.py
```

### Objetivo

Es el nodo que interpreta lo que pide el usuario y lo convierte en un destino semantico navegable.

Ejemplo de entrada:

```text
quiero ir al stand de Qualcomm
```

Este nodo busca en:

```text
config/semantic_map.yaml
```

y decide:

- a que zona pertenece el stand;
- que stand concreto se esta pidiendo;
- cual es el objetivo de navegacion;
- que puntos de busqueda local se usaran despues.

### Topics

Se suscribe a:

```text
/guide/command
```

Tipo:

```text
std_msgs/String
```

Publica en:

```text
/guide/plan
/guide/state
/guide/result
```

Todos como `std_msgs/String`.

### Que hace paso a paso

1. Recibe un comando del usuario por `/guide/command`.

   Ejemplo:

   ```text
   "quiero ir al stand de Qualcomm"
   ```

2. Normaliza el texto:

   - pasa a minusculas;
   - elimina espacios repetidos;
   - recorta espacios al principio y final.

   Por ejemplo:

   ```text
   "  LLEVAME   A   nViDiA  "
   ```

   pasa a:

   ```text
   "llevame a nvidia"
   ```

3. Busca primero entre los aliases de los stands.

   Por ejemplo, para Qualcomm:

   ```yaml
   qualcomm_ai_hub:
     display_name: "Qualcomm AI Hub"
     aliases:
       - "qualcomm"
       - "qualcomm ai"
       - "qualcomm ai hub"
       - "stand qualcomm"
     marker_id: 11
   ```

4. Si encuentra un stand, genera un plan completo.

   Ejemplo de resultado:

   ```json
   {
     "status": "FOUND",
     "zone_id": "zona_arriba",
     "zone_name": "Zona arriba",
     "label_id": "qualcomm_ai_hub",
     "display_name": "Qualcomm AI Hub",
     "marker_id": 11,
     "nav_goal": {
       "x": -0.529138445854187,
       "y": -0.044567957520484924,
       "yaw": 0.3415423349961051
     },
     "search_waypoints": [
       {
         "x": -0.529138445854187,
         "y": -0.044567957520484924,
         "yaw": 0.3415423349961051
       }
     ]
   }
   ```

5. Publica ese plan en:

   ```text
   /guide/plan
   ```

6. Publica el estado:

   ```text
   TARGET_RESOLVED
   ```

7. Publica un resultado legible:

   ```text
   Destino seleccionado: Qualcomm AI Hub
   ```

8. Si no encuentra ninguna zona ni stand, publica:

   ```text
   PARSE_FAILED
   ```

   y un mensaje de error.

### Que NO hace

Este nodo no mueve el robot, no usa `move_base`, no procesa imagenes y no publica `/cmd_vel`.

---

## 2. `navigation_manager_node.py`

Ruta:

```text
catkin_ws/src/event_guide_robot/scripts/navigation_manager_node.py
```

### Objetivo

Es el nodo encargado de transformar el plan semantico en un goal real para `move_base`.

Recibe el plan publicado por `semantic_planner_node.py` y manda el robot hacia el `nav_goal` de la zona.

### Topics y acciones

Se suscribe a:

```text
/guide/plan
```

Publica en:

```text
/guide/state
/guide/result
```

Usa la accion:

```text
/move_base
```

Tipo:

```text
move_base_msgs/MoveBaseAction
```

### Que hace paso a paso

1. Espera a que exista el servidor de accion de `move_base`.

   Es decir, necesita que este lanzado el stack de navegacion:

   ```text
   map_server + amcl + move_base
   ```

2. Recibe un plan JSON por `/guide/plan`.

3. Comprueba que el plan tenga:

   ```json
   "status": "FOUND"
   ```

4. Extrae el objetivo de navegacion:

   ```json
   "nav_goal": {
     "x": "...",
     "y": "...",
     "yaw": "..."
   }
   ```

5. Convierte el `yaw` en quaternion.

   ROS no manda orientacion como `yaw` directamente en `MoveBaseGoal`, sino como quaternion. En la demo real, `navigation_with_guide.launch` relaja `yaw_goal_tolerance` para que esta orientacion no obligue al robot a recolocarse al final del trayecto:

   ```text
   orientation.x
   orientation.y
   orientation.z
   orientation.w
   ```

6. Construye un `MoveBaseGoal` en frame:

   ```text
   map
   ```

7. Publica estado:

   ```text
   NAVIGATE_TO_ZONE
   ```

8. Envia el goal a:

   ```text
   /move_base
   ```

9. Espera el resultado durante un tiempo configurable:

   ```text
   ~navigation_timeout_sec
   ```

   Por defecto:

   ```text
   90.0 segundos
   ```

10. Si `move_base` llega correctamente, publica:

    ```text
    NAVIGATION_SUCCEEDED
    ```

    y en `/guide/result`:

    ```text
    Navegacion completada: Qualcomm AI Hub
    ```

11. Si hay timeout, cancela el goal y publica:

    ```text
    NAVIGATION_FAILED
    ```

12. Si `move_base` devuelve otro estado de fallo, tambien publica:

    ```text
    NAVIGATION_FAILED
    ```

### Que NO hace

Este nodo no interpreta comandos de usuario, no procesa camara y no publica directamente `/cmd_vel`.

---

## 3. `local_search_manager_node.py`

Ruta:

```text
catkin_ws/src/event_guide_robot/scripts/local_search_manager_node.py
```

### Objetivo

Es el nodo que realiza la busqueda visual local cuando el robot ya esta en la zona general.

Su funcion es:

- girar lentamente el robot;
- escuchar detecciones de vision;
- comprobar si la deteccion corresponde al stand buscado;
- detener el robot si encuentra el objetivo;
- detenerlo tambien si se acaba el tiempo.

### Topics

Se suscribe a:

```text
/guide/plan
/vision/detections
```

Publica en:

```text
/cmd_vel
/guide/state
/guide/result
```

### Configuracion

Carga parametros desde:

```text
config/search_params.yaml
```

Actualmente:

```yaml
rotation_speed_rad_s: 0.25
scan_rotation_duration_sec: 8.0
search_timeout_sec: 45.0
detection_stale_sec: 1.0
control_rate_hz: 10.0
```

### Que hace paso a paso

1. Recibe el mismo plan JSON publicado en `/guide/plan`.

2. Comprueba que el plan tenga un objetivo visual:

   - `label_id`, por ejemplo:

     ```text
     qualcomm_ai_hub
     ```

   - o `marker_id`, por ejemplo:

     ```text
     11
     ```

3. Guarda el plan como objetivo pendiente y espera a que `/guide/state` indique:

   ```text
   NAVIGATION_SUCCEEDED
   ```

4. Entonces entra en estado:

   ```text
   LOCAL_VISUAL_SEARCH
   ```

5. Empieza a publicar velocidades en `/cmd_vel`.

   Concretamente publica un `Twist` con:

   ```text
   angular.z = 0.25
   ```

   Esto hace que el robot gire lentamente sobre si mismo.

5. Mientras gira, escucha detecciones en:

   ```text
   /vision/detections
   ```

6. Cada deteccion llega como JSON.

   Ejemplo:

   ```json
   {
     "marker_id": 11,
     "label_id": "qualcomm_ai_hub",
     "confidence": 1.0,
     "stable": true,
     "stamp": 1779870000.0
   }
   ```

7. Comprueba varias cosas:

   - que la deteccion no sea antigua;
   - que tenga `stable: true`;
   - que el `marker_id` coincida con el objetivo;
   - o que el `label_id` coincida si no hay marker.

8. Si coincide, publica velocidad cero:

   ```text
   linear.x = 0
   angular.z = 0
   ```

9. Cambia el estado a:

   ```text
   FOUND_TARGET
   ```

10. Publica un resultado JSON en `/guide/result`.

11. Si pasa demasiado tiempo sin encontrar el objetivo, publica velocidad cero y cambia a:

    ```text
    SEARCH_FAILED
    ```

### Detalle importante

Este nodo **solo acepta detecciones estables**.

Es decir, no basta con que la camara vea algo una vez. El detector de vision tiene que haber confirmado el mismo objetivo durante varios frames y publicar:

```json
"stable": true
```

Esto reduce falsos positivos.

### Que NO hace

Este nodo no procesa imagenes, no usa OpenCV y no decide que stand quiere el usuario. Solo coordina el movimiento local, compara detecciones contra el plan y, tras encontrar el ArUco correcto, usa `center_offset_x` + `distance_m` para acercarse lentamente hasta unos 20 cm.

---

## 4. `vision_detector_node.py`

Ruta:

```text
catkin_ws/src/event_guide_robot/scripts/vision_detector_node.py
```

### Objetivo

Es el nodo encargado de analizar la imagen de la camara y publicar detecciones visuales.

La decision de vision del MVP queda cerrada: este nodo detecta marcadores ArUco con `cv2.aruco` y publica el `marker_id` asociado al stand. OCR, AprilTag o detectores de logos quedan solo como mejoras futuras, no como dependencia de la demo.

### Topics

Se suscribe a:

```text
/raspicam_node/image
```

aunque el topic es configurable con:

```text
~image_topic
```

Publica en:

```text
/vision/detections
```

Tipo:

```text
std_msgs/String
```

con contenido JSON.

### Que hace paso a paso

1. Carga el mapa semantico:

   ```text
   config/semantic_map.yaml
   ```

2. Construye un indice interno:

   ```text
   marker_id -> label_id
   ```

   Ejemplo:

   ```text
   11 -> qualcomm_ai_hub
   42 -> nvidia_edge_ai
   ```

3. Se suscribe al topic de camara.

4. Convierte la imagen ROS a imagen OpenCV usando `cv_bridge`.

5. Si `cv2.aruco` esta disponible:

   - detecta marcadores ArUco;
   - extrae sus IDs;
   - busca que stand corresponde a cada ID;
   - calcula si la deteccion es estable.

6. Para estabilidad mantiene un historial de detecciones consecutivas.

   Por defecto se requieren:

   ```text
   3 frames consecutivos
   ```

7. Publica una deteccion en JSON.

   Ejemplo:

   ```json
   {
     "marker_id": 11,
     "label_id": "qualcomm_ai_hub",
     "confidence": 1.0,
     "stable": true,
     "stamp": 1779870000.0
   }
   ```

8. Si no estan disponibles `cv2`, `cv_bridge` o `cv2.aruco`, no publica detecciones falsas. Solo hace `logwarn`.

### Que NO hace

Este nodo no mueve el robot, no decide si se ha terminado la busqueda y no publica `/cmd_vel`.

Solo informa de lo que ve.

---

## Flujo completo entre nodos

Ejemplo con Qualcomm:

### 1. Usuario envia comando

```bash
rostopic pub /guide/command std_msgs/String "data: 'quiero ir al stand de Qualcomm'"
```

### 2. `semantic_planner_node.py`

Detecta:

```text
qualcomm -> qualcomm_ai_hub
```

y publica:

```text
/guide/state = TARGET_RESOLVED
/guide/plan = JSON con zona_arriba, marker_id 11, nav_goal...
/guide/result = Destino seleccionado: Qualcomm AI Hub
```

### 3. `navigation_manager_node.py`

Recibe `/guide/plan`.

Manda a `move_base`:

```text
x = -0.529138445854187
y = -0.044567957520484924
yaw = 0.3415423349961051
```

Publica:

```text
/guide/state = NAVIGATE_TO_ZONE
```

Si llega:

```text
/guide/state = NAVIGATION_SUCCEEDED
```

### 4. `local_search_manager_node.py`

Tambien recibe `/guide/plan`, pero no empieza a girar hasta observar `NAVIGATION_SUCCEEDED` en `/guide/state`.

Cuando esta activo, gira el robot:

```text
/cmd_vel angular.z = 0.25
```

Publica:

```text
/guide/state = LOCAL_VISUAL_SEARCH
```

### 5. `vision_detector_node.py`

Analiza camara y publica `marker_id`, estabilidad, offset horizontal y distancia estimada al ArUco.

Si ve el marcador o numero asociado al stand, publica:

```json
{
  "marker_id": 11,
  "label_id": "qualcomm_ai_hub",
  "confidence": 1.0,
  "stable": true,
  "stamp": 1779870000.0
}
```

### 6. `local_search_manager_node.py`

Comprueba que coincide con el objetivo.

Si coincide:

```text
/cmd_vel = 0
/guide/state = FOUND_TARGET
/guide/result = objetivo encontrado
```

---

## Resumen por responsabilidades

| Nodo | Responsabilidad principal | Puede mover robot | Usa camara | Usa move_base |
| --- | --- | ---: | ---: | ---: |
| `semantic_planner_node.py` | Convertir comando en plan | No | No | No |
| `navigation_manager_node.py` | Navegar hasta la zona | Indirectamente mediante `move_base` | No | Si |
| `local_search_manager_node.py` | Girar y buscar localmente | Si, con `/cmd_vel` | No | No |
| `vision_detector_node.py` | Detectar senales visuales | No | Si | No |
