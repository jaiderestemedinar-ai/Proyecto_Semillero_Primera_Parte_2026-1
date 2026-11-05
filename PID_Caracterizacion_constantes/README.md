# Caracterización de las Constantes PID (Kp, Ki, Kd)

## 1. Objetivo

Documentar de forma completa y honesta el proceso que llevó a las ganancias finales del controlador de velocidad por rueda, incluyendo el abandono deliberado del enfoque teórico, la metodología empírica adoptada, el diseño matemático del feedforward y del PID con anti-windup, y un caso de estudio de depuración (el bug de timing) que es, en sí mismo, una lección de ingeniería de control documentable en una tesis.

## 2. Por qué se abandonó el enfoque teórico

El primer intento de obtener la constante de conversión pulsos→RPM fue derivarla matemáticamente a partir de los registros del Timer1 del PIC (T1CON, prescaler, frecuencia del oscilador). Este camino produjo dos resultados incorrectos consecutivos por errores de interpretación de los bits de configuración del datasheet. Se decidió conjuntamente abandonar la derivación teórica y adoptar un método exclusivamente empírico: medir directamente, con el motor corriendo a un duty conocido durante un tiempo conocido, cuántos pulsos reales se producen. Esta decisión queda documentada como metodología deliberada, no como un atajo — de hecho, implicó más trabajo (decenas de mediciones repetidas por motor, por dirección y por duty) precisamente para no depender de una fórmula que ya había fallado dos veces.

## 3. Feedforward: por qué y cómo

### 3.1 Motivación

Un PID puro, sin ningún término de anticipación, parte de salida cero y debe “descubrir” mediante el error acumulado qué duty corresponde a la RPM deseada. Esto es lento y, en un sistema con 4 motores con comportamiento ligeramente distinto entre sí (confirmado empíricamente — ver más abajo), el PID puro tiende a sobrecorregir mientras encuentra el punto de operación. El feedforward resuelve esto: aporta una estimación inicial razonable del duty necesario, calculada fuera del lazo de retroalimentación, de modo que el PID solo tiene que corregir el error residual, no construir la señal desde cero.

### 3.2 Metodología de medición

Para cada uno de los 4 motores y en cada una de las dos direcciones (adelante/reversa), se aplicó un duty fijo conocido (barrido entre 30 y 100, en incrementos de 10) durante intervalos de tiempo fijos (10, 20 y 30 segundos), registrando el conteo de pulsos del encoder en cada caso. Estas mediciones se repitieron íntegramente más de una vez ante inconsistencias observadas entre sesiones (ver sección 6), para descartar que fueran ruido de medición antes de aceptarlas como comportamiento real del motor.

### 3.3 Resultado: relación lineal

Al graficar RPM resultante contra duty aplicado, la relación es consistentemente lineal dentro del rango útil de operación, de la forma:

RPM = m · duty + c

Se ajustó esta recta por regresión lineal, de forma independiente por motor y por dirección (8 ajustes en total: 4 motores × 2 direcciones), porque los 4 motores no son idénticos entre sí — una diferencia medible y reproducible, no un error de medición, confirmada al repetir las mediciones.

### 3.4 Coeficientes finales adoptados
Motor	m (adelante)	c (adelante)	m (reversa)	c (reversa)
M1	1.226	-2.54	1.140	-1.00
M2	1.208	-1.12	1.220	2.05
M3	1.290	-4.10	1.279	-4.20
M4	1.209	-1.61	1.284	-3.50

Estos coeficientes se usan para invertir la relación: dado un setpoint de RPM, el feedforward estima el duty de arranque como duty = (RPM - c) / m, sembrando la salida del PID antes de que el término integral tenga que acumular nada.

## 4. Diseño del PID de velocidad por rueda

### 4.1 Estructura

Control PID estándar sobre el error de RPM (setpoint - rpmMedido), con el feedforward sumado a la salida del PID (no reemplazándola):

duty = feedforward(setpoint) + Kp·error + Ki·∫error·dt + Kd·(derror/dt)

### 4.2 Anti-windup

Dado que la salida (duty) está acotada físicamente a 0–100%, el término integral puede “sobrecargarse” (windup) si el error persiste mientras la salida ya está saturada — produciendo una sobrecorrección lenta de recuperar cuando el error finalmente cambia de signo. Se implementó el criterio clásico de congelamiento condicional del integrador:

El término integral deja de acumularse únicamente cuando la salida ya está saturada y el error tiene el mismo signo que empujaría aún más hacia la saturación.

Esto permite que el integrador siga corrigiendo libremente en cualquier otra condición (incluyendo cuando la salida está saturada pero el error ya cambió de signo, lo cual debe permitirse para que el sistema pueda “salir” de la saturación).

### 4.3 Ganancias finales adoptadas

Tras un proceso iterativo de sintonización manual (observando el comportamiento real del chasis en pruebas de movimiento, no solo en banco), las ganancias finales son:

Kp = 0.20   Ki = 0.40   Kd = 0.02

Este proceso de sintonización partió de valores iniciales (Kp=0.10–0.12, Ki=0.30, Kd=0.02) y se ajustó específicamente para eliminar oscilación observable durante el movimiento lateral puro, que fue el caso de prueba más exigente para el controlador (ver sección 6 de Especificaciones_Iniciales_de_Hardware.md sobre el hallazgo mecánico asociado).

## 5. Caso de estudio: el bug del tiempo de ciclo fijo

### 5.1 El síntoma

Al integrar la IMU al firmware del chasis, aparecieron sacudidas sincronizadas (“tirones feos”) en las 4 ruedas simultáneamente, un patrón muy distinto a un problema de sintonización de ganancias (que normalmente afecta ruedas de forma desigual o depende de la dirección de movimiento).

### 5.2 Diagnóstico

El cálculo de RPM asumía un período de control fijo (DT_CONTROL = 0.05f, 50 ms) para convertir pulsos en velocidad. Esta suposición era válida mientras el ciclo principal tardaba, en la práctica, siempre lo mismo. Al añadir la comunicación I2C con la IMU (bloqueante por diseño, con reintentos y timeouts) y al alargar la línea de reporte UART, el tiempo real entre ciclos de control pasó a variar de forma impredecible — pero el cálculo seguía dividiendo por los mismos 50 ms fijos de siempre. El resultado es una RPM calculada incorrecta en cada ciclo cuya variación no tiene relación con el movimiento real del motor, sino con cuánto tardó esa iteración del lazo en particular — y esa RPM errónea alimentaba directamente al PID.

### 5.3 Principio de ingeniería aplicado

Este es el mismo principio que ya se había aplicado para la integración del heading de la IMU (nunca asumir un dt fijo cuando hay operaciones bloqueantes en el lazo): el tiempo real transcurrido debe medirse, nunca asumirse, en cualquier cálculo que dependa de él.

### 5.4 Solución

Se introdujo un contador de hardware dedicado (ctrlTicks), incrementado cada 10 ms por la interrupción de Timer0, leído y puesto a cero de forma atómica (con interrupciones deshabilitadas momentáneamente) al inicio de cada ciclo de control:

```c
INTCONbits.GIE = 0;
uint8_t ticks = ctrlTicks; ctrlTicks = 0;
INTCONbits.GIE = 1;
float dt_real = ticks * DT_TICK;   // DT_TICK = 0.01f
```

Este dt_real reemplazó a DT_CONTROL en todos los cálculos de RPM. Adicionalmente, se acortó la línea de reporte UART (sección 4 de Captura_de_Encoders_y_Telemetria/) para reducir el tiempo bloqueante de transmisión y así la variabilidad del ciclo en primer lugar.

### 5.5 Validación

Tras el cambio, pruebas de movimiento lateral y en reversa mostraron valores de RPM estables, sin el patrón de sacudida sincronizada previamente observado — confirmado visualmente por telemetría en vivo antes de continuar con cualquier otro ajuste.

## 6. Nota sobre variabilidad entre sesiones de medición

Durante la caracterización del feedforward se observaron inconsistencias puntuales entre sesiones (por ejemplo, un motor que aparecía como “el más rápido” en una sesión y no en otra). Se investigó como posible causa térmica o de conexión antes de aceptar los datos; en al menos un caso se confirmó un conector flojo como causa de una asimetría extrema adelante/reversa en un motor, que se resolvió al revisar el cableado. Esta observación se documenta por honestidad metodológica: los datos finales adoptados son los de las mediciones repetidas y consistentes, no los de la primera sesión.