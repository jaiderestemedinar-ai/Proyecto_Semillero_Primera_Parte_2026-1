Mantenimiento de Rumbo (MPU9250)
1. Objetivo

Añadir al chasis la capacidad de corregir activamente desviaciones de rumbo durante el movimiento, usando el eje Z de un giroscopio MPU9250 como referencia, en vez de depender únicamente de que las 4 ruedas tengan exactamente la misma velocidad real (lo cual, como demostró el hallazgo de la sección de ensamblaje, no puede garantizarse solo por software si hay una asimetría mecánica).

2. Hardware e interconexión

MPU9250 conectado por I2C: SCL a RC3 (pin 18), SDA a RC4 (pin 23), AD0 a GND (fija la dirección del dispositivo en 0x68). Requiere resistencias de pull-up externas en ambas líneas (4.7 kΩ) — su ausencia fue, de hecho, el primer problema diagnosticado (sección 4).

3. Diagnóstico de bus: el escáner I2C

Antes de escribir cualquier lógica específica de la MPU9250, se construyó un escáner I2C mínimo y genérico (paso1_escaner_i2c.c), que recorre las direcciones 0x08–0x77 buscando una respuesta ACK. Este paso, aunque parezca trivial, es la metodología correcta para diagnosticar un bus I2C desde cero: aísla el problema de “¿el bus funciona y hay algo respondiendo?” del problema de “¿el dispositivo específico responde correctamente a sus propios registros?” — son dos preguntas distintas y deben responderse en ese orden.

3.1 Hallazgo: 56 dispositivos falsos

La primera ejecución del escáner reportó una respuesta positiva en 56 direcciones distintas — un resultado físicamente imposible para un solo sensor conectado. La causa fue la ausencia de resistencias de pull-up: sin ellas, las líneas SDA/SCL flotan y producen patrones de ACK falsos y aleatorios en el escaneo. Al añadir las resistencias de 4.7 kΩ, el escáner reportó correctamente un único dispositivo en 0x68, confirmando tanto el cableado como la dirección esperada.

4. Driver I2C maestro, escrito desde cero

No se usó ninguna librería de terceros para el protocolo I2C: el driver maestro se escribió directamente sobre los registros del periférico MSSP1 del PIC18F45K22 (SSP1STAT, SSP1CON1, SSP1CON2, SSP1ADD, SSP1BUF).

Decisión de diseño crítica — timeout en cada espera bloqueante: toda espera de un bit de estado del bus (inicio, ACK, fin de transmisión) está protegida con un contador de timeout, de modo que un fallo del bus (un dispositivo colgado, un pull-up defectuoso) nunca puede dejar al firmware del chasis congelado indefinidamente esperando una condición que nunca llega. Esto es especialmente importante porque el I2C es bloqueante dentro del lazo de control principal — un bloqueo indefinido en el I2C habría significado perder control de las 4 ruedas también.

Adicionalmente se implementó I2C_Recover(), una función de reinicio de bus: si una transacción falla por timeout, se fuerza una secuencia de reinicio del periférico MSSP1 antes del siguiente intento, evitando que el bus quede en un estado intermedio inválido tras una falla.

5. Inicialización y calibración
5.1 Verificación de identidad

Antes de confiar en cualquier lectura, se verifica el registro WHO_AM_I (dirección 0x75), que debe devolver uno de los valores esperados para la familia MPU9250/6500 (0x71, 0x70, 0x73 o 0x68, según la revisión de silicio). Si no coincide, el firmware marca la IMU como no disponible (imuOK = 0) y el chasis sigue operando exactamente como sin IMU (Capa 3), en vez de fallar o comportarse de forma indefinida.

5.2 Secuencia de arranque

Se escribe PWR_MGMT_1 (0x6B) para sacar al sensor de su modo de reposo (sleep) por defecto, y GYRO_CONFIG (0x1B) para fijar el rango del giroscopio en ±250 °/s, lo que da una sensibilidad de 131 LSB por °/s — constante usada en toda conversión posterior de la lectura cruda a grados/segundo.

5.3 Calibración de sesgo (bias)

El giroscopio, en reposo absoluto, no reporta exactamente cero — tiene un sesgo (bias) propio de cada unidad física. Se calibra promediando 300 muestras consecutivas del eje Z (GYRO_ZOUT_H/GYRO_ZOUT_L, 0x47/0x48) con el chasis físicamente detenido, y se incluye una verificación de estabilidad: si durante el muestreo se detecta una variación anormal entre lecturas (indicio de que el chasis se movió durante la calibración), la calibración se descarta y se le pide al usuario repetirla (comando C), en vez de aceptar silenciosamente un sesgo contaminado por movimiento.

6. Integración de rumbo (heading)

La lectura cruda del giroscopio es una velocidad angular, no un ángulo. El rumbo acumulado (heading) se obtiene integrando esa velocidad angular en el tiempo:

heading += (velocidad_angular_dps) * dt_real

Igual que en el cálculo de RPM de las ruedas (ver Caracterizacion_de_las_Constantes_PID_Kp_Ki_Kd/, sección 5), dt_real se mide con un contador de hardware dedicado (gyroTicks, incrementado cada 10 ms por Timer0) — nunca se asume un intervalo fijo, por el mismo principio ya aplicado y ya validado en el resto del firmware: cualquier operación bloqueante en el lazo (como la propia lectura I2C de la IMU) introduce variabilidad real en el tiempo entre muestras, y asumir un valor fijo produce un error de integración silencioso y acumulativo.

7. PID de mantenimiento de rumbo, con anti-windup
7.1 Por qué PID completo y no solo PD

El error residual de rumbo, en pruebas, ya era “casi imperceptible” con un controlador puramente proporcional-derivativo. Se implementó, sin embargo, el término integral completo (Kih) con su anti-windup correspondiente, por rigor matemático y de completitud para la documentación de tesis — un sistema de control de rumbo formalmente correcto no debe depender de que el error residual “sea pequeño por casualidad”; debe tener la estructura completa que garantice converger a cero ante cualquier perturbación persistente, aunque en la práctica actual esa perturbación sea pequeña.

7.2 Estructura
Wz_correccion = Kh · error_rumbo + Khd · velocidad_angular_medida + Kih · ∫error_rumbo·dt

con las ganancias finales:

Kh = 0.80    Khd = 0.05    Kih = 0.03

El término derivativo usa directamente la velocidad angular ya medida por el giroscopio (en vez de derivar el error por diferencias finitas), lo cual es más preciso y evita el ruido típico de diferenciar una señal medida.

7.3 Anti-windup

Mismo criterio que el PID de velocidad por rueda (congelamiento condicional: el integral deja de acumularse solo cuando la corrección ya está saturada en RUMBO_CORR_MAX y el error empuja en la misma dirección de esa saturación), con límite adicional INTEGRAL_HEADING_MAX = 60 (grado·segundo) como segunda capa de protección contra acumulación excesiva.

7.4 Condiciones de seguridad para activar la corrección

La corrección de rumbo no se aplica incondicionalmente:

RUMBO_VEL_MIN = 10: solo corrige si el chasis se está moviendo de forma apreciable (|Vx| + |Vy| ≥ 10 RPM) — evita que el sistema “persiga” rumbo mientras está físicamente detenido.
RUMBO_CORR_MAX = 8: la corrección de Wz nunca excede 8 RPM, para que el mantenimiento de rumbo no pueda dominar ni desestabilizar el movimiento comandado por el usuario.
RUMBO_ERR_MAX = 30: si el error de rumbo supera los 30°, el hold se desactiva automáticamente y se emite una advertencia por UART (avisoHold) sugiriendo revisar el signo del giroscopio (comando N) — un error de esa magnitud indica con alta probabilidad un problema de signo o calibración, no una perturbación real a corregir.
7.5 Validación

Se validó mediante pruebas de rotación controlada (90° y 360°) contra el firmware de diagnóstico independiente (capa4a_mpu9250_diagnostico.c) antes de integrar la IMU al control del chasis. Tras la integración completa, se capturó telemetría de rumbo durante una prueba extendida (8–10 segundos) que muestra una única transición amortiguada convergiendo a un error cercano a 0°, sin oscilación sostenida — diferenciándose explícitamente de una oscilación que se repite, lo cual se verificó mediante la duración extendida de la prueba a sugerencia propia del proceso de validación.

8. Nota de alcance: filtro complementario

El sistema actual integra únicamente el giroscopio (sin fusión con acelerómetro ni magnetómetro). Esto es válido y suficiente para sesiones de navegación de duración acotada con referencias de rumbo reseteadas periódicamente (comando Z), pero acumula deriva lenta si se usa en sesiones largas sin resets. La decisión de si se requiere un filtro complementario (fusión giroscopio+acelerómetro) queda explícitamente abierta como trabajo futuro, a evaluar según la duración real de las sesiones de navegación autónoma que se planeen una vez se implemente control de posición.