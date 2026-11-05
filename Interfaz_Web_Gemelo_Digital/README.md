#  Interfaz Web y Gemelo Digital (Servidor ESP8266)

Este documento describe la arquitectura de software de alto nivel del Proyecto Semillero. Detalla cómo se diseñó una interfaz gráfica HMI (Human-Machine Interface) capaz de renderizar modelos 3D y enviar comandos en tiempo real, utilizando un módulo ESP8266 como servidor local.

El protocolo exacto de los comandos que esta interfaz envía (Modo Manual F/B/L/R/1/2/S hacia el chasis, más el protocolo del brazo) y cómo conviven con el resto del sistema está documentado en Interconexion_del_Sistema/. Este documento se enfoca en cómo se construyó la interfaz y el servidor, no en el protocolo de comandos en sí.

## 1. Arquitectura de Red (el servidor ESP8266)

Para que el proyecto fuera 100% autónomo y no dependiera de redes de internet externas, el microcontrolador ESP8266 NodeMCU fue configurado como Access Point (Punto de Acceso SoftAP).

El ESP8266 crea su propia red Wi-Fi llamada Robot_Mecanum. Al conectarse a esta red, el usuario interactúa con la página web almacenada y servida a través del protocolo WebSocket (puerto 81).

1.1 Filtro anti-ráfaga (firmware del ESP8266)

Para asegurar que los comandos lleguen intactos al PIC del chasis sin saturar la red (colisión de paquetes), se implementó un filtro de “ráfagas” en el ESP8266:



```c
// Fragmento del filtro en ESP8266
if (c != ultimoComando) {
    Serial.write(c);           // Envía la orden nueva
    ultimoComando = c;
    contadorRafa = 1;
}
else if (contadorRafa < 5) {
    Serial.write(c);           // Deja pasar hasta 5 letras idénticas (ráfaga de seguridad)
    contadorRafa++;
}
// Las letras subsecuentes iguales se descartan para evitar saturación UART
```




La lógica: la primera vez que llega una letra nueva, se reenvía de inmediato. Si la misma letra se repite (típico de un botón mantenido presionado), se deja pasar hasta 5 repeticiones consecutivas como margen de seguridad (para no depender de un único paquete que pudiera perderse), pero las repeticiones adicionales se descartan — evitando saturar la UART hacia el PIC con el mismo carácter cientos de veces por segundo mientras el usuario mantiene el botón.

En sentido inverso, el ESP8266 reenvía la telemetría recibida por UART del PIC hacia la interfaz web mediante broadcastTXT().

2. La Interfaz Web (HMI)

El archivo index.html es una aplicación de una sola página (SPA) que consolida los controles físicos y la visualización virtual. Su diseño se inspira en interfaces de control industrial (p. ej. Epson RC+, TIA Portal), vistas en las materias de Automatización Industrial y Robótica Industrial.

2.1 Control tipo “hombre muerto” (Pointer Events)

Para garantizar una experiencia de teleoperación segura — el robot solo se mueve mientras el botón está físicamente activado — se descartó el uso de los eventos tradicionales mousedown/touchstart. En su lugar se usó la API Pointer Events, que unifica clics de ratón y toques de pantalla táctil capturando el “puntero”: si el dedo resbala fuera del botón, el sistema envía de inmediato una ráfaga de frenado.



// Captura de eventos para máxima fiabilidad
const onEnd = (e) => {
    e.preventDefault();
    btn.releasePointerCapture(e.pointerId);
    if (activeButtons[id]) {
        activeButtons[id] = false;
        sendCommand(releaseCmd.repeat(5)); // Ráfaga de seguridad de PARADA
    }
};
btn.addEventListener('pointerup', onEnd);
btn.addEventListener('pointercancel', onEnd);
btn.addEventListener('pointerleave', onEnd);




3. El Gemelo Digital (URDF + Three.js)

La funcionalidad más avanzada de la interfaz es la representación virtual del chasis sincronizada en tiempo real, lograda importando una librería gráfica 3D y un parseador de modelos robóticos.

3.1 Renderizado del URDF

Se importó Three.js (motor 3D sobre WebGL) junto con urdf-loader. El archivo urdf2.urdf (Unified Robot Description Format) contiene la cinemática, los pesos y las mallas visuales de los enlaces (chasis, brazo, rodillos), diseñadas originalmente en SolidWorks y exportadas con el plugin URDF Exporter (ver Diseno_CAD_SolidWorks/ y Exportacion_De_URDF/).



// Carga del modelo en la escena de Three.js
const loader = new URDFLoader();
loader.load('carrosemillero2.urdf', result => {
    robot = result;
    scene.add(robot);
    robot.rotation.x = -Math.PI / 2;   // Ajuste de ejes Z-up a Y-up
});





3.2 Sincronización físico-virtual

Cuando el usuario acciona un control, el PIC del chasis envía un JSON al ESP8266 confirmando que las llantas están girando (ej. {"v":0.1}). La función animate() de Three.js intercepta ese valor y lo aplica a las articulaciones (joints) definidas en el URDF, haciendo que el modelo 3D gire sincronizado con el chasis físico:




// Bucle maestro de renderizado
function animate() {
    requestAnimationFrame(animate);
    if (robot) {
        // Identifica los enlaces de las 4 llantas y les suma la velocidad recibida
        const joints = { fl: 'rdi', fr: 'rdd', rl: 'rti', rr: 'rtd' };
        for (const key in joints) {
            if (robot.joints[joints[key]]) {
                robot.joints[joints[key]].setJointValue(
                    robot.joints[joints[key]].angle + wheelVelocity
                );
            }
        }
        renderer.render(scene, camera);
    }
}







3.3 Escalabilidad: cambiar el URDF a futuro

El sistema es modular — si se rediseña el chasis o se cambia la pinza del brazo, no es necesario reescribir la página web:

Exportar el nuevo diseño desde SolidWorks con el plugin URDF Exporter.
Guardar el nuevo archivo como nuevo_modelo.urdf en la raíz del servidor.
Cambiar la línea loader.load('urdf2.urdf', ...) al nuevo nombre de archivo.
(Opcional) Si los nombres de los motores/articulaciones cambiaron en SolidWorks, actualizar el diccionario const joints en la función de animación.
4. Visualización Sensorial (Radar en Canvas)

Se integró un lienzo HTML5 (<canvas>) superpuesto a la vista 3D para renderizar el radar ultrasónico. La función drawRadar() procesa los paquetes JSON (ej. {"a":40,"d":25}), convirtiendo coordenadas polares (ángulo y distancia) a coordenadas cartesianas mediante trigonometría (Math.cos/Math.sin), dibujando la posición de obstáculos en tiempo real y cambiando su color a rojo si infringen el umbral de seguridad de 7 cm.

Anexo — Despliegue local y análisis del motor gráfico
A.1 El problema de CORS y el servidor local

Abrir index.html con doble clic lo carga bajo el protocolo file:///, lo que impide que el Gemelo Digital (el URDF) se cargue — los navegadores modernos aplican CORS (Cross-Origin Resource Sharing), que prohíbe que urdf-loader/Three.js lean archivos locales del disco por seguridad.

Solución: levantar un servidor local de desarrollo:




python -m http.server 8000



Al acceder a http://localhost:8000, la petición viaja por HTTP estándar y el navegador confía en ese origen, permitiendo cargar urdf2.urdf y sus texturas sin bloqueos.

A.2 Matemática del radar ultrasónico

El radar no es una animación pregrabada: es una representación calculada en tiempo real a partir de la telemetría del sensor.

Memoria de barrido: se usa un arreglo radarData[181], una posición por cada grado posible del servo (0°–180°). Al llegar un paquete {"a":40,"d":25}, se almacena la distancia 25 en el índice 40. Al dibujar, se recorren los 181 grados pintando los ecos almacenados, generando un efecto de persistencia (“rastro de fósforo”).

Transformación de coordenadas: el área de Canvas es de 300×150 px, con origen en el centro-inferior. La conversión de polar (R, θ) a cartesiano (X, Y):

X = Cx + R · cos(180° − θ)
Y = Cy − R · sin(180° − θ)

donde Cx, Cy son el origen del radar en el Canvas, R es la distancia escalada (50 cm reales del HC-SR04 = 100% de la altura visual), y θ es el ángulo del servo convertido a radianes.

Umbral crítico: si la distancia almacenada es ≤ 7 cm, el punto cambia de verde neón (
#00ff00) a rojo alerta (
#ff0000) — trasladando a la interfaz visual la misma lógica de exclusión de seguridad que existe en el hardware físico del chasis.