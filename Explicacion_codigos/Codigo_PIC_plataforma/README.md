# Explicación Completa del Firmware Final (Capa 4): Arquitectura, Cinemática, PID y Mantenimiento de Rumbo con MPU9250

## 1. Objetivo de este documento

Explicar, de forma exhaustiva y sin omitir nada, cómo funciona internamente el firmware final del PIC del chasis (capa4_rumbo_mpu9250.c): su configuración de hardware, el driver I2C construido desde cero, la integración de la MPU9250 y por qué se hizo sin filtro de Kalman ni complementario, la cinemática inversa Mecanum aplicada en código, el PID de velocidad de cada rueda con su anti-windup, el ciclo de control completo, el manejo de interrupciones, y el programa principal. Es el complemento técnico de bajo nivel de Cinematicas_Brazo_y_Plataforma/, PID_Caracterizacion_constantes/, MPU_9250/ e Interconexion_del_sistema/ — aquellos documentos explican el diseño y la metodología; este explica el código real, línea por línea.

## 2. Encabezado y configuración de hardware

```c
#pragma config FOSC = INTIO67, WDTEN = OFF, LVP = OFF, PBADEN = OFF
```

Configura el PIC para usar su oscilador interno (INTIO67, sin cristal externo), apaga el Watchdog Timer (WDTEN = OFF — si el firmware se cuelga, no se reinicia solo; es una decisión consciente, porque durante el desarrollo un reinicio automático silencioso habría ocultado bugs reales en vez de exponerlos), apaga la programación en bajo voltaje (LVP = OFF, libera el pin que esa función usaría) y desactiva el modo “threshold” de PORTB (PBADEN = OFF), haciendo que esos pines funcionen como entradas/salidas digitales normales.

```c
#define _XTAL_FREQ 64000000
```

Indica al compilador la frecuencia real del reloj del micro (64 MHz, usando el PLL interno) — es el valor que usan internamente las funciones __delay_ms()/__delay_us() para calcular cuántos ciclos de reloj esperar.

### 2.1 Pines fijos

```c
#define EN_GLOBAL LATEbits.LATE0
#define ENCB_M1 PORTAbits.RA0
#define ENCB_M2 PORTAbits.RA1
#define ENCB_M3 PORTAbits.RA2
#define ENCB_M4 PORTAbits.RA3
#define INVERT_M2 1
#define INVERT_M4 1
const uint8_t INVERTIDO[4] = {0, INVERT_M2, 0, INVERT_M4};
#define MPU_ADDR 0x68
```

EN_GLOBAL: un solo pin que habilita los 4 puentes H BTS7960 simultáneamente (R_EN+L_EN de los 4 módulos unidos, documentado en Especificaciones_Iniciales_Hardware/).

ENCB_M1..M4: los 4 pines que leen el canal B de cada encoder, usados únicamente para determinar el sentido de giro (el conteo en sí lo hace la interrupción del canal A).

INVERT_M2/INVERT_M4: dos motores giran al revés por el montaje físico. En vez de recablear, el arreglo INVERTIDO[4] se usa más adelante para intercambiar a qué canal de PWM (RPWM o LPWM) se envía la señal en esos dos motores.

MPU_ADDR 0x68: dirección I2C fija de la IMU, resultado de tener AD0 conectado a GND.


### 2.2 Parámetros ajustables

```c
const float FF_M_FWD[4] = {1.226f, 1.208f, 1.290f, 1.209f};
const float FF_C_FWD[4] = {-2.54f, -1.12f, -4.10f, -1.61f};
const float FF_M_REV[4] = {1.140f, 1.220f, 1.279f, 1.284f};
const float FF_C_REV[4] = {-1.00f,  2.05f, -4.20f, -3.50f};

#define PULSOS_POR_VUELTA 1090.0f
#define DT_CONTROL        0.05f
#define MAX_RPM_CMD       120.0f
#define RPM_MANUAL        40.0f

float Kp = 0.15f, Ki = 0.25f, Kd = 0.02f;

#define GYRO_LSB_POR_DPS 131.0f
#define DT_TICK          0.01f
#define RUMBO_VEL_MIN    10.0f
#define RUMBO_CORR_MAX   8.0f
#define RUMBO_ERR_MAX    30.0f
float Kh  = 0.80f;
float Khd = 0.05f;
float Kih = 0.03f;
#define INTEGRAL_HEADING_MAX 60.0f
float integralHeading = 0.0f;
```

Las 8 constantes de feedforward (pendiente m y corte c, por motor y por dirección) ya están documentadas con su metodología de obtención en PID_Caracterizacion_constantes/. PULSOS_POR_VUELTA = 1090 es la constante empírica de resolución del encoder (Especificaciones_del_Motor_y_Resolucion_Espacial/).

Detalle importante: DT_CONTROL = 0.05f ya no se usa para calcular nada en este firmware — es precisamente la constante que causaba el bug del tiempo de ciclo fijo (documentado en PID_Caracterizacion_constantes/, sección 5). Se dejó como constante de referencia/documentación en el código, pero todos los cálculos reales usan el tiempo medido por hardware (ctrlTicks).

Kp, Ki, Kd son las ganancias del PID de velocidad, compartidas entre las 4 ruedas. GYRO_LSB_POR_DPS = 131.0 es la sensibilidad de fábrica del giroscopio en el rango ±250°/s. DT_TICK = 0.01f es cuánto vale un solo tick del contador de hardware (10 ms). RUMBO_VEL_MIN, RUMBO_CORR_MAX, RUMBO_ERR_MAX son los tres límites de seguridad del mantenimiento de rumbo, detallados en la sección 7 de este documento.


## 3. Variables globales y el papel de volatile

```c
volatile int32_t pulsos[4] = {0, 0, 0, 0};
int32_t pulsosAnt[4]   = {0, 0, 0, 0};
float   integral[4]    = {0, 0, 0, 0};
float   errorPrev[4]   = {0, 0, 0, 0};
float   rpmMedido[4]   = {0, 0, 0, 0};
uint8_t dutyActual[4]  = {0, 0, 0, 0};
char    direccion[4]   = {'S', 'S', 'S', 'S'};
float   spRpm[4]       = {0, 0, 0, 0};
```
volatile le indica al compilador que una variable puede cambiar en cualquier momento por fuera del flujo normal del código — en este caso, porque la modifica la interrupción de encoders, no el programa principal. Sin volatile, el compilador podría optimizar una lectura asumiendo que la variable no cambió entre dos líneas seguidas, y esa suposición sería falsa.

Cada rueda tiene su propio conjunto de variables de estado: pulsosAnt[i] (valor de pulsos[i] en el ciclo de control anterior, para calcular la diferencia), integral[i] y errorPrev[i] (memoria del PID), rpmMedido[i] (velocidad real calculada), dutyActual[i] (lo que se envía al puente H en este instante), direccion[i] ('F'/'B'/'S') y spRpm[i] (consigna, siempre positiva — el signo vive exclusivamente en direccion).

```c
volatile uint8_t gyroTicks = 0;
volatile uint8_t ctrlTicks = 0;
```

Estos dos contadores de hardware son la pieza central de todo el firmware: ambos son incrementados por la misma interrupción de 10 ms, pero se leen y ponen en cero en momentos distintos (uno cuando se lee la IMU, otro cuando corre el ciclo de control). Esta separación es, literalmente, la solución al bug del tiempo de ciclo fijo.

## 4. El driver I2C, construido registro por registro

Esto no usa ninguna librería de terceros: está escrito directamente sobre los registros del periférico MSSP1 del PIC18F45K22.

### 4.1 ¿Qué hace el periférico de hardware por ti?

I2C es un bus de 2 cables (SDA = datos, SCL = reloj) donde un maestro (el PIC) se comunica con uno o más esclavos (la MPU9250), cada uno identificado por una dirección de 7 bits. El PIC18F45K22 tiene un periférico de hardware llamado MSSP1 (Master Synchronous Serial Port) que genera los pulsos de reloj y detecta ACK/NACK automáticamente — pero hay que manejarlo escribiendo bits en sus registros de control. Eso es exactamente lo que hace este driver.


```c
void I2C_Init(void){
    TRISCbits.TRISC3 = 1;   // SCL como entrada (el periférico lo controla internamente)
    TRISCbits.TRISC4 = 1;   // SDA como entrada
    SSP1STAT = 0x80;        // SMP=1: sin control de pendiente de flanco, válido a 100 kHz
    SSP1ADD  = 159;         // divisor de frecuencia del reloj I2C
    SSP1CON1 = 0x28;        // enciende el periférico (SSPEN=1) en modo maestro I2C
    SSP1CON2 = 0x00;
}
```

SSP1ADD = 159 fija la velocidad del bus en 100 kHz (modo estándar), según la fórmula del datasheet: SSP1ADD = (Fosc / (4 × Fi2c)) − 1. Con Fosc = 64 MHz y Fi2c = 100 kHz: 64 000 000 / (4×100 000) − 1 = 160 − 1 = 159.

### 4.2 El patrón repetido: espera con timeout

```c
uint8_t I2C_WaitIdle(void){
    uint16_t t = I2C_TIMEOUT;
    while((SSP1CON2 & 0x1F) || (SSP1STAT & I2C_RW_BIT)){
        if(--t == 0) return 0;
    }
    return 1;
}
```

SSP1CON2 & 0x1F revisa si hay cualquier operación I2C en curso (start, stop, repeated-start, recepción, ACK — todos esos bits viven en los 5 bits bajos de SSP1CON2). SSP1STAT & I2C_RW_BIT revisa si hay una transmisión en curso. Si cualquiera está activo, el bus está ocupado.

Punto crítico de diseño: el while no es infinito. El contador t baja en cada vuelta, y si llega a 0, la función devuelve 0 (fallo) en vez de esperar indefinidamente. Este mismo patrón se repite en I2C_Start, I2C_RepStart, I2C_Stop, I2C_Write y I2C_ReadByte — ninguna espera de este driver puede bloquear el firmware para siempre. Si el bus I2C se cuelga (un cable suelto, un ACK que nunca llega), el chasis sigue controlando las 4 ruedas con normalidad; la IMU simplemente se marca como fallida.

```c
void I2C_Recover(void){
    SSP1CON1bits.SSPEN = 0;
    __delay_us(20);
    SSP1CON1bits.SSPEN = 1;
    SSP1CON2 = 0x00;
}
```

Si cualquier operación falla por timeout, se llama esta función: apaga y reenciende el periférico MSSP1 para sacarlo de cualquier estado interno inválido, antes de intentar la siguiente transacción.

### 4.3 Cómo se arma una transacción completa

```c
uint8_t I2C_Write(uint8_t data){
    ...
    SSP1BUF = data;
    while(SSP1STATbits.BF){ ... }
    ...
    return !SSP1CON2bits.ACKSTAT;
}
```
ACKSTAT lo pone el hardware automáticamente después de cada byte transmitido: 0 si el esclavo confirmó (ACK), 1 si no respondió (NACK). La función devuelve éxito solo si hubo ACK real. Por eso, en las funciones de más alto nivel, aparece el patrón ok = ok && I2C_Write(...): si cualquier paso falla, los siguientes ni siquiera se ejecutan (cortocircuito lógico de &&), y al final se llama I2C_Recover().

```c
uint8_t I2C_ReadByte(uint8_t *data, uint8_t enviarAck){
    ...
    SSP1CON2bits.RCEN = 1;
    while(!SSP1STATbits.BF){ ... }
    *data = SSP1BUF;
    ...
    SSP1CON2bits.ACKDT = enviarAck ? 0 : 1;
    SSP1CON2bits.ACKEN = 1;
    ...
}
```

Después de recibir cada byte, el maestro debe decidir si va a pedir otro más (ACK, ACKDT=0) o si ese fue el último (NACK, ACKDT=1) — esto es parte del protocolo I2C, no una decisión arbitraria de este proyecto.

## 5. El driver de la MPU9250

```c
uint8_t MPU_WriteReg(uint8_t reg, uint8_t val){
    uint8_t ok = I2C_Start();
    ok = ok && I2C_Write((uint8_t)(MPU_ADDR << 1));
    ok = ok && I2C_Write(reg);
    ok = ok && I2C_Write(val);
    I2C_Stop();
    if(!ok) I2C_Recover();
    return ok;
}
```
En I2C, la dirección de 7 bits se envía desplazada un bit a la izquierda, y el bit 0 que queda libre indica escritura (0) o lectura (1) — protocolo estándar, no una elección de este proyecto.


```c
uint8_t MPU_ReadReg(uint8_t reg, uint8_t *val){
    ok = I2C_Start();
    ok = ok && I2C_Write(MPU_ADDR << 1);
    ok = ok && I2C_Write(reg);
    ok = ok && I2C_RepStart();
    ok = ok && I2C_Write((MPU_ADDR << 1) | 1);
    ok = ok && I2C_ReadByte(val, 0);
    I2C_Stop();
}
```

Leer un registro requiere dos fases dentro de la misma transacción: primero se le dice al dispositivo qué registro se quiere (modo escritura), luego se cambia a modo lectura sin soltar el bus (RepStart, repeated start — evita que otro dispositivo tome el bus a mitad de la operación), y se lee el dato.

```c
uint8_t MPU_ReadGyroZ(int16_t *out){
    ok = ok && I2C_Write(0x47);                // GYRO_ZOUT_H
    ok = ok && I2C_RepStart();
    ok = ok && I2C_Write((MPU_ADDR << 1) | 1);
    ok = ok && I2C_ReadByte(&h, 1);             // primer byte: pide más (ACK)
    ok = ok && I2C_ReadByte(&l, 0);             // segundo byte: no pide más (NACK)
    *out = (int16_t)(((uint16_t)h << 8) | l);
}
```

El valor del giroscopio son 16 bits repartidos en 2 registros consecutivos (0x47 byte alto, 0x48 byte bajo — se autoincrementan en lecturas consecutivas). Por eso el primer byte se pide con ACK y el segundo con NACK. Al final se arman en un entero de 16 bits con signo mediante desplazamiento de bits.

### 5.1 Secuencia de inicialización

```c
uint8_t MPU_Init(void){
    __delay_ms(100);
    if(!MPU_WriteReg(0x6B, 0x80)) return 0;    // PWR_MGMT_1: RESET de fábrica
    __delay_ms(100);
    MPU_WriteReg(0x6B, 0x01);                  // despierta, reloj PLL del giroscopio
    __delay_ms(50);
    MPU_WriteReg(0x1A, 0x03);                  // filtro digital pasa-bajos a 41 Hz
    MPU_WriteReg(0x1B, 0x00);                  // rango ±250°/s → 131 LSB por °/s
    if(!MPU_ReadReg(0x75, &who)) return 0;
    imuWhoAmI = who;
    return (who == 0x71 || who == 0x73 || who == 0x70 || who == 0x68);
}
```

Secuencia: reset de fábrica → sacar del modo sleep (el sensor arranca dormido por defecto) → configurar filtro y rango → verificar identidad. Los 4 valores de WHO_AM_I aceptados cubren distintas revisiones de silicio de la familia MPU9250/6500. Si no coincide con ninguno, imuOK queda en 0 y el chasis opera como si no tuviera IMU — degradación segura, no un fallo silencioso.

### 5.2 Calibración del sesgo

Un giroscopio en reposo absoluto no da exactamente cero: tiene un sesgo propio de fabricación. Sin restarlo, el rumbo “deriva” aunque el chasis esté quieto.

```c
uint8_t calibrarGiroscopio(void){
    for(intento = 0; intento < 3; intento++){
        suma = 0; mn = 32767; mx = -32768;
        for(i = 0; i < 300; i++){
            if(!MPU_ReadGyroZ(&raw)) return 0;
            suma += raw;
            if(raw < mn) mn = raw;
            if(raw > mx) mx = raw;
            __delay_ms(5);
        }
        if(((int32_t)mx - (int32_t)mn) <= 400){
            gyroBiasRaw = (float)suma / 300.0f;
            return 1;
        }
    }
    return 0;
}
```

Se toman 300 muestras (a 5 ms de separación, ~1.5 s en total) con el chasis quieto, se promedian, y ese promedio es el sesgo (gyroBiasRaw). Se guarda además el mínimo y el máximo del muestreo: si la diferencia entre el más alto y el más bajo supera 400 unidades crudas (~3°/s pico a pico), significa que el chasis se movió durante la calibración, o hay ruido eléctrico anormal, y la calibración se descarta — se reintenta hasta 3 veces antes de rendirse, evitando aceptar silenciosamente un sesgo contaminado.

### 5.3 Integración del rumbo — y por qué no hay Kalman ni filtro complementario

```c
void actualizarGiroscopio(void){
    if(gyroTicks == 0) return;
    INTCONbits.GIE = 0; ticks = gyroTicks; gyroTicks = 0; INTCONbits.GIE = 1;
    if(!imuOK) return;

    if(!MPU_ReadGyroZ(&raw)){
        if(++imuFallos >= 5) imuOK = 0;
        return;
    }
    imuFallos = 0;
    gyroRaw = raw;
    gyroRate = (((float)raw - gyroBiasRaw) / GYRO_LSB_POR_DPS) * gyroSign;
    heading += gyroRate * ((float)ticks * DT_TICK);
}
```

Paso a paso:

gyroTicks == 0: si no ha pasado ni un tick de 10 ms desde la última llamada, no hay nada que hacer — esta función se invoca constantemente desde el loop principal, no desde un temporizador fijo propio.
Lectura atómica del contador: se desactivan las interrupciones un instante para leer gyroTicks y ponerlo en cero sin riesgo de que la interrupción de 10 ms lo modifique a mitad de la operación — una condición de carrera clásica si no se protegiera.
Tolerancia a fallos: si la lectura I2C falla 5 veces seguidas, se da por muerta la IMU (imuOK = 0). El control del chasis sigue funcionando normal, solo sin corrección de rumbo.
gyroRate = (raw - bias) / 131.0 * gyroSign: convierte el valor crudo a grados/segundo, restando el sesgo calibrado y aplicando gyroSign (-1 o +1, según la orientación física de montaje respecto a la convención de signo elegida; el comando N lo invierte en caliente).
heading += gyroRate * (ticks * DT_TICK): integración numérica por el método de Euler: ángulo_nuevo = ángulo_viejo + velocidad_angular × tiempo_transcurrido. ticks * DT_TICK es el tiempo real en segundos desde la última lectura — otra vez, medido con hardware, nunca asumido.

## 6. Cinemática inversa, aplicada en código

```c
void recalcularCinematica(void){
    float wz = Wz + wzCorr;

    tgt[0] = Vy + Vx + wz;      // M1 delantera izquierda
    tgt[1] = Vy - Vx - wz;      // M2 delantera derecha
    tgt[2] = Vy - Vx + wz;      // M3 trasera izquierda
    tgt[3] = Vy + Vx - wz;      // M4 trasera derecha
```

Esta es exactamente la matriz deducida en Cinematicas_Brazo_y_Plataforma/Cinematica_Inversa_mecanum/, con un detalle central: wz no es Wz directamente, es Wz + wzCorr, donde wzCorr es la corrección que calcula el PID de mantenimiento de rumbo (sección 7). Esto significa que la corrección de rumbo se incorpora dentro de la misma matriz de cinemática, no como un ajuste posterior independiente: el chasis gira físicamente un poco de más o de menos en cada rueda para corregir el rumbo, usando exactamente el mismo mecanismo matemático con el que rota a propósito por comando del usuario.

```c
    for(i = 0; i < 4; i++){
        if(tgt[i] > 0.01f)       { nuevaDir = 'F'; sp =  tgt[i]; }
        else if(tgt[i] < -0.01f) { nuevaDir = 'B'; sp = -tgt[i]; }
        else                     { nuevaDir = 'S'; sp = 0.0f; }

        if(nuevaDir != direccion[i]){ integral[i] = 0.0f; errorPrev[i] = 0.0f; }
        direccion[i] = nuevaDir;
        spRpm[i] = (sp > MAX_RPM_CMD) ? MAX_RPM_CMD : sp;
    }
}
```

El resultado con signo se convierte en dirección más magnitud positiva. 0.01f es el umbral para considerar “efectivamente cero”, evitando que ruido numérico de punto flotante dispare una dirección falsa. Detalle de control importante: si la dirección de una rueda cambia (de adelante a atrás, por ejemplo), se resetea su memoria de PID (integral[i] y errorPrev[i] a 0). Sin este reset, el término integral acumulado en un sentido contaminaría el arranque del control en el sentido contrario, produciendo una respuesta incorrecta justo en el instante más delicado: el cambio de dirección.

## 7. El PID de mantenimiento de rumbo

```c
void actualizarCorreccionRumbo(float dt){
    float vel = absf(Vx) + absf(Vy);
    uint8_t traslacion = (absf(Wz) < 0.5f) && (vel >= RUMBO_VEL_MIN);

    if(!holdActivo || !imuOK || !traslacion){
        headingRef = heading;
        wzCorr = 0.0f;
        integralHeading = 0.0f;
        return;
    }
```

El hold de rumbo solo actúa cuando se cumplen tres condiciones a la vez: está activado (holdActivo), la IMU funciona (imuOK), y el chasis se está trasladando de forma pura sin rotación intencional (|Wz| < 0.5 y velocidad de traslación ≥ RUMBO_VEL_MIN). En cualquier otro caso, la referencia de rumbo simplemente sigue al rumbo actual (headingRef = heading) y no se aplica corrección — esto evita que el sistema “pelee” contra una rotación que el propio usuario pidió.

```c
    err = headingRef - heading;
    if(absf(err) > RUMBO_ERR_MAX){
        holdActivo = 0; avisoHold = 1;
        headingRef = heading; wzCorr = 0.0f; integralHeading = 0.0f;
        return;
    }
```

Si el error supera 30°, se asume que algo está mal (típicamente un signo de giroscopio invertido) y el hold se desactiva automáticamente, emitiendo un aviso — en vez de insistir en corregir un error que probablemente es un síntoma de mala calibración y no una perturbación real.

```c
    limite = 0.5f * vel;
    if(limite > RUMBO_CORR_MAX) limite = RUMBO_CORR_MAX;
```

El límite de corrección es proporcional a la velocidad de traslación (con tope absoluto de 8 RPM) — esto garantiza matemáticamente que la corrección de rumbo nunca pueda invertir el sentido de giro de una rueda, porque nunca excede la mitad de la velocidad de traslación que ya se le está pidiendo.

```c
    corrSinLimite = (Kh * err) + (Kih * integralHeading) - (Khd * gyroRate);
    saturado = (corrSinLimite > limite) || (corrSinLimite < -limite);
    mismoSigno = ((corrSinLimite > 0.0f) && (err > 0.0f)) ||
                 ((corrSinLimite < 0.0f) && (err < 0.0f));
    if(!(saturado && mismoSigno)){
        integralHeading += err * dt;
    }
    if(integralHeading >  INTEGRAL_HEADING_MAX) integralHeading =  INTEGRAL_HEADING_MAX;
    if(integralHeading < -INTEGRAL_HEADING_MAX) integralHeading = -INTEGRAL_HEADING_MAX;

    corr = (Kh * err) + (Kih * integralHeading) - (Khd * gyroRate);
    wzCorr = limitarF(corr, limite);
}
```
Estructura PID completa:

Proporcional: Kh * err — corrige en proporción directa al error de rumbo actual.
Integral: Kih * integralHeading — corrige la acumulación de error persistente en el tiempo, con el mismo criterio de anti-windup que el PID de rueda (congelar la acumulación solo cuando la salida ya está saturada y el error empuja en el mismo sentido de esa saturación), más un límite absoluto adicional de ±60 grado·segundo.
Derivativo: -Khd * gyroRate — a diferencia del PID de rueda, aquí el término derivativo no deriva el error por diferencias finitas; usa directamente la velocidad angular ya medida por el sensor (gyroRate), que es una medición más limpia y evita el ruido típico de diferenciar numéricamente una señal.

El resultado final se recorta al límite proporcional a la velocidad (limitarF(corr, limite)) y se guarda en wzCorr, que es justo la variable que recalcularCinematica() suma a Wz en la sección 6.

## 8. El PID de velocidad de cada rueda
```c
void calcularControlRueda(uint8_t i, int32_t actual, float dt){
    int32_t delta = actual - pulsosAnt[i];
    pulsosAnt[i] = actual;


```
actual es una “foto” del contador de pulsos de esa rueda, tomada de forma atómica en cicloControl() (sección 9). delta es cuántos pulsos ocurrieron desde la última vez que se llamó esta función.


```c
    if(direccion[i] == 'S' || spRpm[i] <= 0.0f){
        integral[i] = 0.0f; errorPrev[i] = 0.0f;
        dutyActual[i] = 0; rpmMedido[i] = 0.0f;
        return;
    }

```
Si la rueda no tiene consigna activa, se limpia toda su memoria de control y se sale — no tiene sentido mantener estado de control para una rueda que debe estar detenida.


```c
    if(delta < 0) delta = -delta;
    rpmMedido[i] = ((float)delta * 60.0f) / (dt * PULSOS_POR_VUELTA);

```

Fórmula de conversión pulsos→RPM: pulsos / pulsos_por_vuelta da vueltas; dividido entre dt (segundos) da vueltas/segundo; multiplicado por 60 da RPM. dt es el tiempo real medido, nunca DT_CONTROL — este es el punto exacto donde se aplicó la corrección del bug del tiempo de ciclo fijo documentado en PID_Caracterizacion_constantes/.
```c
    if(direccion[i] == 'F'){ m = FF_M_FWD[i]; c = FF_C_FWD[i]; }
    else                    { m = FF_M_REV[i]; c = FF_C_REV[i]; }

    duty_ff = (spRpm[i] - c) / m;
    if(duty_ff < 0.0f)   duty_ff = 0.0f;
    if(duty_ff > 100.0f) duty_ff = 100.0f;

```

Feedforward: dado que RPM = m·duty + c (la recta medida empíricamente por motor y dirección), se despeja duty = (RPM − c) / m para estimar de entrada qué duty necesita esa rueda específica para alcanzar la consigna, antes de que el PID tenga que descubrirlo por acumulación de error.

```c

    error = spRpm[i] - rpmMedido[i];

    if(duty_ff < 100.0f || error < 0.0f){
        integral[i] += error * dt;
    }
    if(integral[i] >  30.0f) integral[i] =  30.0f;
    if(integral[i] < -30.0f) integral[i] = -30.0f;



```
Anti-windup, con precisión: el término integral deja de acumularse únicamente cuando duty_ff ya está en el tope (100%) y el error sigue siendo positivo (se querría ir más rápido pero ya no se puede). En cualquier otro caso —incluido cuando está saturado pero el error ya es negativo— se permite acumular, porque en ese caso el integrador necesita poder reaccionar para traer la salida de vuelta. Se añade además un límite absoluto de ±30 como segunda capa de protección.
```c

    derivada = (error - errorPrev[i]) / dt;
    errorPrev[i] = error;

    trim = (Kp * error) + (Ki * integral[i]) + (Kd * derivada);

    salida = duty_ff + trim;
    if(salida < 0.0f)   salida = 0.0f;
    if(salida > 100.0f) salida = 100.0f;
    dutyActual[i] = (uint8_t)salida;


```


A diferencia del PID de rumbo, aquí la derivada sí se calcula por diferencia finita del error ((error_actual − error_anterior) / dt), porque no existe un sensor independiente de “tasa de cambio de RPM” como sí existe para el rumbo (el propio giroscopio). salida = duty_ff + trim: el feedforward pone la base, el PID corrige el error residual; el resultado final se recorta a 0–100% (rango físico válido de duty).