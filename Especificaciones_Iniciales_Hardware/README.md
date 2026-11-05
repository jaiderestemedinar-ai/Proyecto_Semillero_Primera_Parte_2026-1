# Especificaciones Iniciales de Hardware

Este documento presenta el inventario completo de componentes electromecánicos del proyecto, sus características de fábrica (datasheet) y el modo específico en que cada uno fue integrado y utilizado en este sistema. Para la deducción matemática de la resolución espacial del encoder, ver `Especificaciones_del_Motor_y_Resolucion_Espacial/`.

---

## 1. Plataforma Omnidireccional Mecanum

### 1.1 Motores — GM37-520

| Parámetro | Valor |
|---|---|
| Voltaje nominal | 12V DC |
| Relación de reducción | 1:90 |
| Velocidad nominal (sin carga, 12V) | ~110 RPM |
| Encoder integrado | Cuadratura, 12 PPR (eje del motor, previo a la reducción) |
| Cantidad | 4 (configuración Mecanum, patrón de rodillos en X) |

**Validación realizada:** a duty cycle = 90% se midió experimentalmente ~108 RPM, consistente con la especificación nominal del fabricante (ver `Especificaciones_del_Motor_y_Resolucion_Espacial/` para la metodología completa).

### 1.2 Encoders — Especificaciones

Encoder de cuadratura de 2 canales (A/B), lo que permite determinar tanto la velocidad como el **sentido real de giro** por hardware, en vez de inferirlo del comando enviado. Ver la deducción de la constante de pulsos por vuelta en la carpeta dedicada.

### 1.3 Batería de potencia

CNHL LiPo 3S (11.1V nominal), 5200mAh, descarga 90C.

---

## 2. Brazo Robótico — xArm1S (HiWonder)

### 2.1 Especificaciones de grados de libertad

5 DOF: base (rotación), hombro, codo, inclinación de muñeca, rotación de muñeca, más pinza (apertura/cierre) — 6 servos en total.

### 2.2 Servomotores

| Servo | Ubicación | Rango de posición |
|---|---|---|
| LX-225 | Base (mayor torque requerido) | 0–1000 (equivalente a 0°–240°, 0.24°/unidad) |
| LX-15D | Hombro, codo, muñeca (inclinación/rotación), pinza | 0–1000 |

Batería dedicada: CNHL LiPo 2S, 8000mAh, descarga 120C.

### 2.3 Implementación sin placa de desarrollo

A diferencia del uso estándar de estos servos (con la placa controladora propia del fabricante), en este proyecto se prescindió de dicha placa: el PIC18F45K22 del brazo se comunica **directamente** con el bus de servos mediante un driver propio del protocolo serie LewanSoul/HiWonder (tramas de 10 bytes con cabecera `0x55 0x55`, ID, longitud, comando y checksum), incluyendo la rutina de lectura de posición que gestiona la conmutación half-duplex por software (ver `Acondicionamiento_Xbee/` — nota: el nombre de esa carpeta se ajustará, el contenido corresponde al bus de comunicación con los servos, no a XBee). Esta decisión de diseño elimina un componente de hardware adicional, a cambio de mayor complejidad de firmware.

---

## 3. Ensamblaje Plataforma y Brazo

**Finalidad:** integrar un manipulador de 5 DOF sobre una base móvil omnidireccional, para obtener un robot móvil manipulador capaz de desplazarse y operar sobre un espacio de trabajo plano.

**Resultado:** ensamblaje funcional validado; se identificó y corrigió un problema de contacto irregular de una rueda con el piso, originado por una tensión mecánica introducida en el punto de anclaje del brazo sobre el chasis (ver detalle de diagnóstico en la documentación de caracterización PID). El hallazgo subraya la importancia de validar la rigidez del ensamblaje mecánico antes de atribuir desviaciones de movimiento al sistema de control.

---

## 4. PIC18F45K22

### 4.1 Especificaciones generales

Microcontrolador de 8 bits, oscilador interno con PLL activo (hasta 64MHz), 2 módulos EUSART (UART), 2 módulos MSSP (I2C/SPI), múltiples módulos CCP, timers de 8 y 16 bits.

### 4.2 Modo de uso en este proyecto

Se utilizan **dos unidades independientes**, cada una dedicada a un subsistema:

| Unidad | Reloj | Función |
|---|---|---|
| PIC — Chasis | 64MHz (PLL) | Control de los 4 motores (PWM por software, captura de cuadratura), comunicación I2C con la MPU9250, UART hacia XBee |
| PIC — Brazo | 16MHz (interno) | Control de los 6 servos (UART2, bus half-duplex), UART1 hacia XBee |

La frecuencia más alta del PIC del chasis se justifica por la necesidad de mayor resolución en el PWM generado por software y en la base de tiempo del lazo de control (ciclos de 10-50ms).

---

## 5. Puentes H — BTS7960

### 5.1 Especificaciones generales

Módulo de puente H dual, corriente continua soportada hasta 43A, entradas independientes RPWM/LPWM por cada sentido de giro, pines de habilitación R_EN/L_EN.

### 5.2 Modo de uso en este proyecto

- **PWM generado por software** (no por los módulos PWM de hardware del PIC), mediante interrupción periódica de Timer2, dejando libres los módulos CCP para la lectura de encoders.
- **R_EN y L_EN de los 4 módulos unidos en un solo nodo eléctrico**, controlado por un único pin del PIC (`EN_GLOBAL`): se mantiene en bajo durante el arranque y solo se habilita una vez que el PWM está generándose de forma estable, como medida de protección contra transitorios indeseados al energizar el sistema.
- RPWM y LPWM de cada motor permanecen siempre independientes entre sí y respecto a los demás motores (nunca se unen), ya que cada rueda requiere control de velocidad y sentido individual para la cinemática Mecanum.

---

## 6. ESP8266 (NodeMCU)

### 6.1 Modo de uso

Actúa como punto de acceso WiFi y servidor WebSocket (puerto 81), haciendo de puente bidireccional entre la interfaz web y la red XBee: reenvía por UART (hacia la XBee maestra) los comandos recibidos por WebSocket, y transmite por WebSocket (`broadcastTXT`) la telemetría JSON recibida por UART desde los PIC. La trama de telemetría se delimita por el carácter `\n`, convención adoptada tras corregir un error inicial de doble disparo al usar `}` como delimitador adicional.

---

## 7. XBee

### 7.1 Modo de uso

Red de radiofrecuencia en topología maestro-esclavo: un módulo XBee maestro (conectado al ESP8266) y dos módulos XBee esclavos, uno por cada PIC, a 115200 baudios.

> **Nota de verificación pendiente:** debe confirmarse que ambas XBee esclavas operan en canales o PAN ID diferenciados (o que la trama de comando incluye un identificador de destino), para garantizar que los comandos dirigidos al chasis no sean también interpretados por el PIC del brazo, y viceversa.