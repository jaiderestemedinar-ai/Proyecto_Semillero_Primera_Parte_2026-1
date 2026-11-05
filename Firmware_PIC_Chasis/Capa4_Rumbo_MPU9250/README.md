# Capa 4 — Firmware Final del Chasis (Rumbo con MPU9250)

**Este es el firmware que corre actualmente en el PIC del chasis.** Es la Capa 3 (`capa3_ruedas_individuales.c`) con la integración completa de la MPU9250 para mantenimiento activo de rumbo, y con compatibilidad dual de protocolo de comandos.

## Qué añade sobre la Capa 3

- Lectura del giroscopio Z (MPU9250 por I2C) y mantenimiento de rumbo: PID completo con anti-windup (`Kh`, `Khd`, `Kih`) — diseño matemático completo en `PID_Caracterizacion_constantes/` y `MPU_9250/`.
- El hold de rumbo arranca **apagado** por defecto (`J0`); se activa con `J1`. Si la IMU no responde en el arranque, el chasis se comporta exactamente igual que la Capa 3 (degradación segura, no un fallo silencioso).
- Corrección del bug de tiempo de ciclo fijo que apareció al integrar la IMU (ver `PID_Caracterizacion_constantes/`, sección 5): el cálculo de RPM pasó de asumir un `dt` fijo de 50 ms a medir el tiempo real transcurrido con un contador de hardware.
- Compatibilidad con el **Modo Manual** heredado de la interfaz web original (`F`/`B`/`L`/`R`/`1`/`2`/`S`), coexistiendo con el protocolo numérico nuevo — ver la resolución completa de colisiones de letra y de ambigüedad de dígitos en `Interconexion_del_sistema/`.

## Protocolo de comandos completo

Documentado íntegramente en el encabezado del propio archivo `capa4_rumbo_mpu9250.c`. Resumen:

| Comando | Función |
|---|---|
| `Y<rpm>` / `X<rpm>` / `W<rpm>` | Vy / Vx / Wz, protocolo nuevo |
| `DI`/`DD`/`TI`/`TD<rpm>` | Rueda individual (diagnóstico) |
| `P`/`I`/`D<n>` | Ganancias PID de rueda |
| `G`/`K`/`Q<n>` | Ganancias PID de rumbo (P/D/I) |
| `J1`/`J0` | Mantenimiento de rumbo on/off |
| `C` / `Z` / `N` | Calibrar IMU / zero rumbo / invertir signo |
| `F`/`B`/`L`/`R`/`1`/`2`/`S` | Modo Manual (compatibilidad interfaz web original) |

## Hardware

MPU9250: `SCL` → RC3 (pin 18), `SDA` → RC4 (pin 23), `AD0` → GND. Resto del hardware sin cambios respecto a la Capa 3 — ver `Especificaciones_Iniciales_Hardware/`.



```c
// ==============================================================================
// CAPA 4 - CHASIS MECANUM: PID + FEEDFORWARD POR RUEDA + RUMBO CON MPU9250
//
// Es la Capa 3 estable (ganancias Kp=0.11, Ki=0.30, Kd=0.02) mas:
//   - comandos de rueda individual (diagnostico de contacto con el piso)
//   - lectura del giroscopio Z (MPU9250 por I2C) y mantenimiento de rumbo
//   - el mantenimiento de rumbo arranca APAGADO (H0). Se activa con H1.
// Si la MPU no responde, el chasis se comporta exactamente como la Capa 3.
//
// HARDWARE (sin cambios) + MPU9250: SCL = RC3 (pin 18), SDA = RC4 (pin 23), AD0 a GND.
//
// ------------------------------- COMANDOS (UART 115200) ------------------------
// MOVIMIENTO (terminan con Enter)
//   Y<rpm>     Vy adelante(+)/atras(-)            ej  Y40   Y-40
//   X<rpm>     Vx lateral derecha(+)/izquierda(-) ej  X40
//   W<rpm>     Wz giro horario(+)/antihorario(-)
//   S          detiene todo (sin Enter)
// RUEDA INDIVIDUAL (detiene las demas; negativo = atras)
//   DI<rpm> M1 delantera izq | DD<rpm> o DR<rpm> M2 delantera der
//   TI<rpm> M3 trasera izq   | TD<rpm> o TR<rpm> M4 trasera der   ej TD50
// GANANCIAS DE RUEDA (valor x100)
//   P<n> Kp | I<n> Ki | D<n> Kd           ej  I30 = Ki 0.30
// RUMBO / MPU
//   J1 / J0    activa / desactiva el mantenimiento de rumbo (antes H1/H0; se libera
//              la letra H porque el PIC del brazo usa 'H' para HOME y ambos XBee
//              esclavos pueden estar en el mismo canal RF: ver Interconexion_del_Sistema)
//   G<n>       Kh  (termino P: RPM por grado de error) x100    ej G80 = 0.80
//   K<n>       Khd (termino D: amortiguacion con la velocidad angular) x100
//   Q<n>       Kih (termino I: RPM por grado*segundo acumulado) x100, ej Q3 = 0.03
//              (antes F<n>; se libera la letra F porque el MODO MANUAL de la interfaz
//              web original usa 'F' como comando instantaneo de "avanzar")
//   C          recalibra el sesgo del giroscopio (solo con el chasis detenido)
//   Z          pone el rumbo en 0
//   N          invierte el signo del rumbo
//
// MODO MANUAL (compatibilidad con la interfaz web original, sin Enter, un solo caracter)
//   F  avanza (+Vy)      | B  retrocede (-Vy)
//   L  lateral izq (-Vx) | R  lateral der (+Vx)
//   1  gira antihorario (-Wz) | 2  gira horario (+Wz)   [solo si no hay una captura
//      numerica en curso de otro comando, p.ej. "Y1" sigue siendo Vy=1]
//   S  detiene todo (ya existia)
//   Todos usan una magnitud fija RPM_MANUAL; se mantienen mientras no llegue otro
//   comando (igual que el resto del protocolo: no hay "temporizador" de boton).
//
// Convencion: rumbo POSITIVO = giro HORARIO visto desde arriba (mismo sentido que Wz+).
// ==============================================================================

#pragma config FOSC = INTIO67, WDTEN = OFF, LVP = OFF, PBADEN = OFF

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 64000000

// ==============================================================================
// 1. PINES Y CONFIGURACION FIJA
// ==============================================================================
#define EN_GLOBAL LATEbits.LATE0            // habilita los 4 BTS7960 (R_EN + L_EN)

#define ENCB_M1 PORTAbits.RA0               // canal B de cada encoder (signo)
#define ENCB_M2 PORTAbits.RA1
#define ENCB_M3 PORTAbits.RA2
#define ENCB_M4 PORTAbits.RA3

#define INVERT_M2 1                         // motores que giran al reves por montaje
#define INVERT_M4 1
const uint8_t INVERTIDO[4] = {0, INVERT_M2, 0, INVERT_M4};

#define MPU_ADDR 0x68                       // AD0 a GND

// ==============================================================================
// 2. PARAMETROS AJUSTABLES
// ==============================================================================
// --- Feedforward medido: RPM = m * duty + c   (indice 0..3 = M1..M4) ---
const float FF_M_FWD[4] = {1.226f, 1.208f, 1.290f, 1.209f};
const float FF_C_FWD[4] = {-2.54f, -1.12f, -4.10f, -1.61f};
const float FF_M_REV[4] = {1.140f, 1.220f, 1.279f, 1.284f};
const float FF_C_REV[4] = {-1.00f,  2.05f, -4.20f, -3.50f};

#define PULSOS_POR_VUELTA 1090.0f
#define DT_CONTROL        0.05f             // ciclo de control: 50 ms
#define MAX_RPM_CMD       120.0f            // tope de cualquier comando
#define RPM_MANUAL        40.0f             // magnitud fija para el Modo Manual (F/B/L/R/1/2)

// --- Ganancias del PID de rueda (compartidas) - confirmadas en pruebas reales ---
float Kp = 0.15f, Ki = 0.25f, Kd = 0.02f;

// --- Rumbo (MPU9250) ---
#define GYRO_LSB_POR_DPS 131.0f             // rango +-250 dps
#define DT_TICK          0.01f              // TMR0 cada 10 ms
#define RUMBO_VEL_MIN    10.0f              // solo corrige si |Vx|+|Vy| >= esto (RPM)
#define RUMBO_CORR_MAX   8.0f               // maxima correccion de Wz (RPM)
#define RUMBO_ERR_MAX    30.0f              // si el error supera esto, se apaga el hold
float Kh  = 0.80f;                          // RPM de Wz por grado de error (P)
float Khd = 0.05f;                          // RPM de Wz por grado/seg de giro (D, usa la tasa medida)
float Kih = 0.03f;                          // RPM de Wz por (grado * segundo) acumulado (I)
#define INTEGRAL_HEADING_MAX 60.0f          // limite del termino integral (grado * segundo)
float integralHeading = 0.0f;

// ==============================================================================
// 3. VARIABLES GLOBALES
// ==============================================================================
// --- Estado de las 4 ruedas (0..3 = M1..M4) ---
volatile int32_t pulsos[4] = {0, 0, 0, 0};  // acumulado con signo (cuadratura)
int32_t pulsosAnt[4]   = {0, 0, 0, 0};
float   integral[4]    = {0, 0, 0, 0};
float   errorPrev[4]   = {0, 0, 0, 0};
float   rpmMedido[4]   = {0, 0, 0, 0};
uint8_t dutyActual[4]  = {0, 0, 0, 0};
char    direccion[4]   = {'S', 'S', 'S', 'S'};
float   spRpm[4]       = {0, 0, 0, 0};      // magnitud (siempre positiva)

// --- Comandos de movimiento ---
float   Vx = 0.0f, Vy = 0.0f, Wz = 0.0f;
float   wzCorr = 0.0f;                      // correccion de rumbo sumada a Wz
uint8_t modoRueda = 0;                      // 1 = una sola rueda en prueba

// --- PWM por software (leido por la ISR) ---
volatile uint8_t th_M1A = 0, th_M1B = 0, th_M2A = 0, th_M2B = 0;
volatile uint8_t th_M3A = 0, th_M3B = 0, th_M4A = 0, th_M4B = 0;

uint8_t puerto_b_ant = 0;

// --- UART RX (FIFO circular) ---
#define RX_BUFFER_SIZE 32
volatile char rx_fifo[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0, rx_tail = 0;

// --- Banderas de tiempo (las pone la ISR) ---
volatile uint8_t flag_control = 0;          // cada 50 ms
volatile uint8_t flag_reporte = 0;          // cada 200 ms
volatile uint8_t cnt_ticks = 0;
volatile uint8_t gyroTicks = 0;             // ticks de 10 ms desde la ultima lectura
volatile uint8_t ctrlTicks = 0;             // ticks de 10 ms desde el ultimo ciclo de control real

// --- IMU ---
uint8_t imuOK = 0, imuWhoAmI = 0, imuFallos = 0;
uint8_t holdActivo = 0, avisoHold = 0;
int16_t gyroRaw = 0;
float   gyroBiasRaw = 0.0f;
float   gyroRate = 0.0f;                    // grados/seg, positivo = horario
float   gyroSign = -1.0f;                   // -1 con el MPU plano y Z hacia arriba
float   heading = 0.0f;                     // grados acumulados
float   headingRef = 0.0f;                  // rumbo que se quiere mantener

// ==============================================================================
// 4. UTILIDADES
// ==============================================================================
float absf(float x){ return (x < 0.0f) ? -x : x; }

float limitarF(float v, float lim){
    if(v >  lim) return  lim;
    if(v < -lim) return -lim;
    return v;
}

int x100(float v){ return (int)(v * 100.0f + 0.5f); }

// Escribe v con 1 o 2 decimales y signo, ej "+12.3" / "-0.31" (sin %f)
void fmtDec(char *dst, float v, uint8_t dec){
    long x, den;
    char sg;
    if(dec == 2){ den = 100; x = (long)(v * 100.0f + ((v >= 0.0f) ? 0.5f : -0.5f)); }
    else        { den = 10;  x = (long)(v * 10.0f  + ((v >= 0.0f) ? 0.5f : -0.5f)); }
    sg = (x < 0) ? '-' : '+';
    if(x < 0) x = -x;
    if(dec == 2) sprintf(dst, "%c%d.%02d", sg, (int)(x / den), (int)(x % den));
    else         sprintf(dst, "%c%d.%d",   sg, (int)(x / den), (int)(x % den));
}

// ==============================================================================
// 5. UART
// ==============================================================================
void UART_Init(void){
    TRISCbits.TRISC6 = 0; TRISCbits.TRISC7 = 1;
    BAUDCONbits.BRG16 = 1; TXSTAbits.BRGH = 1;
    SPBRGH = 0; SPBRG = 138;                // 115200 baud @ 64 MHz
    TXSTA = 0x24; RCSTA = 0x90; PIE1bits.RCIE = 1;
}
void UART_Write(char c){ while(!TXSTAbits.TRMT); TXREG = c; }
void UART_WriteString(const char *s){ while(*s){ UART_Write(*s); s++; } }

// ==============================================================================
// 6. I2C (MSSP1, maestro, 100 kHz) - todas las esperas tienen timeout
// ==============================================================================
#define I2C_TIMEOUT 3000U
#define I2C_RW_BIT  0x04                    // bit R/W de SSP1STAT (transmision en curso)

void I2C_Init(void){
    TRISCbits.TRISC3 = 1;                   // SCL
    TRISCbits.TRISC4 = 1;                   // SDA
    SSP1STAT = 0x80;                        // SMP=1 (sin control de slew a 100 kHz)
    SSP1ADD  = 159;                         // 64e6 / (4 * 100e3) - 1
    SSP1CON1 = 0x28;                        // SSPEN=1, modo maestro I2C
    SSP1CON2 = 0x00;
}

void I2C_Recover(void){
    SSP1CON1bits.SSPEN = 0;
    __delay_us(20);
    SSP1CON1bits.SSPEN = 1;
    SSP1CON2 = 0x00;
}

uint8_t I2C_WaitIdle(void){
    uint16_t t = I2C_TIMEOUT;
    while((SSP1CON2 & 0x1F) || (SSP1STAT & I2C_RW_BIT)){
        if(--t == 0) return 0;
    }
    return 1;
}

uint8_t I2C_Start(void){
    uint16_t t = I2C_TIMEOUT;
    if(!I2C_WaitIdle()) return 0;
    SSP1CON2bits.SEN = 1;
    while(SSP1CON2bits.SEN){ if(--t == 0) return 0; }
    return 1;
}

uint8_t I2C_RepStart(void){
    uint16_t t = I2C_TIMEOUT;
    if(!I2C_WaitIdle()) return 0;
    SSP1CON2bits.RSEN = 1;
    while(SSP1CON2bits.RSEN){ if(--t == 0) return 0; }
    return 1;
}

void I2C_Stop(void){
    uint16_t t = I2C_TIMEOUT;
    if(!I2C_WaitIdle()) return;
    SSP1CON2bits.PEN = 1;
    while(SSP1CON2bits.PEN){ if(--t == 0) return; }
}

uint8_t I2C_Write(uint8_t data){
    uint16_t t = I2C_TIMEOUT;
    if(!I2C_WaitIdle()) return 0;
    SSP1BUF = data;
    while(SSP1STATbits.BF){ if(--t == 0) return 0; }
    if(!I2C_WaitIdle()) return 0;
    return !SSP1CON2bits.ACKSTAT;           // 1 = el esclavo respondio ACK
}

uint8_t I2C_ReadByte(uint8_t *data, uint8_t enviarAck){
    uint16_t t = I2C_TIMEOUT;
    if(!I2C_WaitIdle()) return 0;
    SSP1CON2bits.RCEN = 1;                  // RCEN esta en SSP1CON2
    while(!SSP1STATbits.BF){ if(--t == 0) return 0; }
    *data = SSP1BUF;
    if(!I2C_WaitIdle()) return 0;
    SSP1CON2bits.ACKDT = enviarAck ? 0 : 1; // 0 = ACK (siguen mas bytes), 1 = NACK
    SSP1CON2bits.ACKEN = 1;
    t = I2C_TIMEOUT;
    while(SSP1CON2bits.ACKEN){ if(--t == 0) return 0; }
    return 1;
}

// ==============================================================================
// 7. MPU9250 (solo giroscopio Z)
// ==============================================================================
uint8_t MPU_WriteReg(uint8_t reg, uint8_t val){
    uint8_t ok = I2C_Start();
    ok = ok && I2C_Write((uint8_t)(MPU_ADDR << 1));
    ok = ok && I2C_Write(reg);
    ok = ok && I2C_Write(val);
    I2C_Stop();
    if(!ok) I2C_Recover();
    return ok;
}

uint8_t MPU_ReadReg(uint8_t reg, uint8_t *val){
    uint8_t ok = I2C_Start();
    ok = ok && I2C_Write((uint8_t)(MPU_ADDR << 1));
    ok = ok && I2C_Write(reg);
    ok = ok && I2C_RepStart();
    ok = ok && I2C_Write((uint8_t)((MPU_ADDR << 1) | 1));
    ok = ok && I2C_ReadByte(val, 0);
    I2C_Stop();
    if(!ok) I2C_Recover();
    return ok;
}

uint8_t MPU_ReadGyroZ(int16_t *out){
    uint8_t h = 0, l = 0;
    uint8_t ok = I2C_Start();
    ok = ok && I2C_Write((uint8_t)(MPU_ADDR << 1));
    ok = ok && I2C_Write(0x47);                         // GYRO_ZOUT_H
    ok = ok && I2C_RepStart();
    ok = ok && I2C_Write((uint8_t)((MPU_ADDR << 1) | 1));
    ok = ok && I2C_ReadByte(&h, 1);
    ok = ok && I2C_ReadByte(&l, 0);
    I2C_Stop();
    if(!ok){ I2C_Recover(); return 0; }
    *out = (int16_t)(((uint16_t)h << 8) | l);
    return 1;
}

uint8_t MPU_Init(void){
    uint8_t who = 0;
    __delay_ms(100);
    if(!MPU_WriteReg(0x6B, 0x80)) return 0;             // PWR_MGMT_1: reset
    __delay_ms(100);
    MPU_WriteReg(0x6B, 0x01);                           // despierta, reloj PLL
    __delay_ms(50);
    MPU_WriteReg(0x1A, 0x03);                           // CONFIG: filtro pasa-bajos 41 Hz
    MPU_WriteReg(0x1B, 0x00);                           // GYRO_CONFIG: +-250 dps
    if(!MPU_ReadReg(0x75, &who)) return 0;              // WHO_AM_I
    imuWhoAmI = who;
    return (who == 0x71 || who == 0x73 || who == 0x70 || who == 0x68);
}

// Promedia 300 muestras con el sensor quieto. Devuelve 0 si se movio o no responde.
uint8_t calibrarGiroscopio(void){
    int32_t suma;
    int16_t raw, mn, mx;
    uint16_t i;
    uint8_t intento;

    for(intento = 0; intento < 3; intento++){
        suma = 0; mn = 32767; mx = -32768;
        for(i = 0; i < 300; i++){
            if(!MPU_ReadGyroZ(&raw)) return 0;
            suma += raw;
            if(raw < mn) mn = raw;
            if(raw > mx) mx = raw;
            __delay_ms(5);
        }
        if(((int32_t)mx - (int32_t)mn) <= 400){         // ~3 dps pico a pico
            gyroBiasRaw = (float)suma / 300.0f;
            return 1;
        }
    }
    return 0;
}

// Lee el giroscopio e integra el rumbo con los ticks de 10 ms realmente transcurridos
void actualizarGiroscopio(void){
    uint8_t ticks;
    int16_t raw;

    if(gyroTicks == 0) return;
    INTCONbits.GIE = 0; ticks = gyroTicks; gyroTicks = 0; INTCONbits.GIE = 1;
    if(!imuOK) return;

    if(!MPU_ReadGyroZ(&raw)){
        if(++imuFallos >= 5) imuOK = 0;                 // deja de intentar: no bloquea el control
        return;
    }
    imuFallos = 0;
    gyroRaw = raw;
    gyroRate = (((float)raw - gyroBiasRaw) / GYRO_LSB_POR_DPS) * gyroSign;
    heading += gyroRate * ((float)ticks * DT_TICK);
}

// ==============================================================================
// 8. CINEMATICA Y CONTROL DE LAS RUEDAS
// ==============================================================================
// Reparte (Vy, Vx, Wz + correccion de rumbo) entre las 4 ruedas
void recalcularCinematica(void){
    float tgt[4], sp;
    char nuevaDir;
    float wz = Wz + wzCorr;
    uint8_t i;

    tgt[0] = Vy + Vx + wz;      // M1 delantera izquierda
    tgt[1] = Vy - Vx - wz;      // M2 delantera derecha
    tgt[2] = Vy - Vx + wz;      // M3 trasera izquierda
    tgt[3] = Vy + Vx - wz;      // M4 trasera derecha

    for(i = 0; i < 4; i++){
        if(tgt[i] > 0.01f)       { nuevaDir = 'F'; sp =  tgt[i]; }
        else if(tgt[i] < -0.01f) { nuevaDir = 'B'; sp = -tgt[i]; }
        else                     { nuevaDir = 'S'; sp = 0.0f; }

        if(nuevaDir != direccion[i]){ integral[i] = 0.0f; errorPrev[i] = 0.0f; }
        direccion[i] = nuevaDir;
        spRpm[i] = (sp > MAX_RPM_CMD) ? MAX_RPM_CMD : sp;
    }
}

void detenerTodo(void){
    uint8_t i;
    Vx = 0.0f; Vy = 0.0f; Wz = 0.0f; wzCorr = 0.0f;
    modoRueda = 0;

    INTCONbits.GIE = 0;
    th_M1A = 0; th_M1B = 0; th_M2A = 0; th_M2B = 0;
    th_M3A = 0; th_M3B = 0; th_M4A = 0; th_M4B = 0;
    for(i = 0; i < 4; i++) pulsosAnt[i] = pulsos[i];    // evita un salto falso de RPM al reanudar
    INTCONbits.GIE = 1;

    for(i = 0; i < 4; i++){
        direccion[i] = 'S'; spRpm[i] = 0.0f;
        integral[i] = 0.0f; errorPrev[i] = 0.0f; dutyActual[i] = 0;
    }
}

// Gira UNA sola rueda (1..4 = M1..M4); las demas quedan detenidas
void comandoRueda(uint8_t rueda, int rpm){
    uint8_t i = rueda - 1;
    float v = limitarF((float)rpm, MAX_RPM_CMD);
    detenerTodo();
    modoRueda = 1;
    if(v > 0.0f)      { direccion[i] = 'F'; spRpm[i] = v; }
    else if(v < 0.0f) { direccion[i] = 'B'; spRpm[i] = -v; }
}

// Feedforward (duty segun la recta medida) + PID de correccion sobre el error de RPM.
// 'dt' es el tiempo REAL (segundos) transcurrido desde el calculo anterior, medido
// con ticks de hardware - nunca se asume un periodo fijo, porque el bucle principal
// puede demorarse (I2C de la MPU, transmision UART del reporte) y un dt asumido
// incorrecto se traduce directo en una RPM medida incorrecta.
void calcularControlRueda(uint8_t i, int32_t actual, float dt){
    float m, c, duty_ff, error, derivada, trim, salida;
    int32_t delta = actual - pulsosAnt[i];
    pulsosAnt[i] = actual;

    if(direccion[i] == 'S' || spRpm[i] <= 0.0f){
        integral[i] = 0.0f; errorPrev[i] = 0.0f;
        dutyActual[i] = 0; rpmMedido[i] = 0.0f;
        return;
    }

    if(delta < 0) delta = -delta;       // magnitud; el sentido ya lo conocemos
    rpmMedido[i] = ((float)delta * 60.0f) / (dt * PULSOS_POR_VUELTA);

    if(direccion[i] == 'F'){ m = FF_M_FWD[i]; c = FF_C_FWD[i]; }
    else                    { m = FF_M_REV[i]; c = FF_C_REV[i]; }

    duty_ff = (spRpm[i] - c) / m;
    if(duty_ff < 0.0f)   duty_ff = 0.0f;
    if(duty_ff > 100.0f) duty_ff = 100.0f;

    error = spRpm[i] - rpmMedido[i];

    if(duty_ff < 100.0f || error < 0.0f){               // anti-windup
        integral[i] += error * dt;
    }
    if(integral[i] >  30.0f) integral[i] =  30.0f;
    if(integral[i] < -30.0f) integral[i] = -30.0f;

    derivada = (error - errorPrev[i]) / dt;
    errorPrev[i] = error;

    trim = (Kp * error) + (Ki * integral[i]) + (Kd * derivada);

    salida = duty_ff + trim;
    if(salida < 0.0f)   salida = 0.0f;
    if(salida > 100.0f) salida = 100.0f;
    dutyActual[i] = (uint8_t)salida;
}

// Convierte (direccion, duty) de cada rueda en los 8 PWM (RPWM/LPWM) y los publica de una vez
void aplicarSalidasTodas(void){
    uint8_t a[4] = {0, 0, 0, 0};            // canal A (RPWM)
    uint8_t b[4] = {0, 0, 0, 0};            // canal B (LPWM)
    uint8_t i, usarB;

    for(i = 0; i < 4; i++){
        if(direccion[i] == 'S') continue;
        usarB = (direccion[i] == 'B');
        if(INVERTIDO[i]) usarB = !usarB;
        if(usarB) b[i] = dutyActual[i]; else a[i] = dutyActual[i];
    }

    INTCONbits.GIE = 0;
    th_M1A = a[0]; th_M1B = b[0];
    th_M2A = a[1]; th_M2B = b[1];
    th_M3A = a[2]; th_M3B = b[2];
    th_M4A = a[3]; th_M4B = b[3];
    INTCONbits.GIE = 1;
}

// ==============================================================================
// 9. MANTENIMIENTO DE RUMBO
// ==============================================================================
// Solo actua si: hold activo + IMU ok + traslacion pura (sin Wz) y velocidad suficiente.
// En cualquier otro caso el rumbo de referencia sigue al rumbo actual.
// PID completo sobre el rumbo: P (Kh*err) + I (Kih*integral, con anti-windup) +
// D (-Khd*gyroRate, usando la tasa ya medida en vez de derivar el error otra vez).
// 'dt' es el tiempo real del ciclo (el mismo que ya se usa para las ruedas).
void actualizarCorreccionRumbo(float dt){
    float vel = absf(Vx) + absf(Vy);
    float err, limite, corr, corrSinLimite;
    uint8_t traslacion = (absf(Wz) < 0.5f) && (vel >= RUMBO_VEL_MIN);
    uint8_t saturado, mismoSigno;

    if(!holdActivo || !imuOK || !traslacion){
        headingRef = heading;
        wzCorr = 0.0f;
        integralHeading = 0.0f;             // evita arrastrar acumulado de un tramo a otro
        return;
    }

    err = headingRef - heading;
    if(absf(err) > RUMBO_ERR_MAX){          // seguro: algo va mal (p.ej. signo invertido)
        holdActivo = 0; avisoHold = 1;
        headingRef = heading; wzCorr = 0.0f; integralHeading = 0.0f;
        return;
    }

    limite = 0.5f * vel;                    // nunca invierte el sentido de una rueda
    if(limite > RUMBO_CORR_MAX) limite = RUMBO_CORR_MAX;

    // Anti-windup: si la salida ya esta saturada Y el error la empuja aun mas
    // hacia ese mismo lado, no se sigue acumulando integral (igual criterio que
    // ya se usa en el PID de cada rueda).
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

// ==============================================================================
// 10. CICLO DE CONTROL (cada 50 ms)
// ==============================================================================
void cicloControl(void){
    int32_t snap[4];
    uint8_t i, ticks;
    float dt;

    INTCONbits.GIE = 0;                     // lectura atomica: contadores de 32 bits + ticks reales
    for(i = 0; i < 4; i++) snap[i] = pulsos[i];
    ticks = ctrlTicks; ctrlTicks = 0;
    INTCONbits.GIE = 1;

    if(ticks == 0) ticks = 1;               // nunca dividir por 0 si se llamo demasiado seguido
    dt = (float)ticks * DT_TICK;            // tiempo REAL transcurrido, no asumido

    if(!modoRueda){
        actualizarCorreccionRumbo(dt);
        recalcularCinematica();
    } else {
        headingRef = heading;
    }

    for(i = 0; i < 4; i++) calcularControlRueda(i, snap[i], dt);
    aplicarSalidasTodas();
}

// ==============================================================================
// 11. INTERRUPCIONES
// ==============================================================================
void __interrupt() interrupciones(void){
    static uint8_t cntPwm = 0;
    char c;

    // PWM por software (~26.5 kHz)
    if(PIR1bits.TMR2IF){
        if(++cntPwm >= 100) cntPwm = 0;
        LATBbits.LATB0 = (cntPwm < th_M1A); LATBbits.LATB1 = (cntPwm < th_M1B);
        LATBbits.LATB2 = (cntPwm < th_M2A); LATBbits.LATB3 = (cntPwm < th_M2B);
        LATDbits.LATD0 = (cntPwm < th_M3A); LATDbits.LATD2 = (cntPwm < th_M3B);
        LATDbits.LATD3 = (cntPwm < th_M4A); LATDbits.LATD5 = (cntPwm < th_M4B);
        PIR1bits.TMR2IF = 0;
    }

    // UART RX -> FIFO
    if(PIR1bits.RCIF){
        if(RCSTAbits.OERR){ RCSTAbits.CREN = 0; RCSTAbits.CREN = 1; }
        if(RCSTAbits.FERR){ c = RCREG; }
        while(PIR1bits.RCIF){
            rx_fifo[rx_head] = RCREG;
            rx_head = (rx_head + 1) & (RX_BUFFER_SIZE - 1);
        }
    }

    // Encoders en cuadratura: canal A por interrupcion en RB4-RB7, canal B por lectura
    if(INTCONbits.RBIF){
        uint8_t pb = PORTB;
        uint8_t cambiaron = pb ^ puerto_b_ant;
        if(cambiaron & 0x10){ if(pb & 0x10){ if(ENCB_M1 == 0) pulsos[0]++; else pulsos[0]--; } }
        if(cambiaron & 0x20){ if(pb & 0x20){ if(ENCB_M2 == 0) pulsos[1]++; else pulsos[1]--; } }
        if(cambiaron & 0x40){ if(pb & 0x40){ if(ENCB_M3 == 0) pulsos[2]++; else pulsos[2]--; } }
        if(cambiaron & 0x80){ if(pb & 0x80){ if(ENCB_M4 == 0) pulsos[3]++; else pulsos[3]--; } }
        puerto_b_ant = pb;
        INTCONbits.RBIF = 0;
    }

    // Base de tiempo de 10 ms
    if(INTCONbits.TMR0IF){
        TMR0H = 0xF6; TMR0L = 0x3C;
        if(gyroTicks < 250) gyroTicks++;
        if(ctrlTicks < 250) ctrlTicks++;
        cnt_ticks++;
        if(cnt_ticks % 5 == 0) flag_control = 1;
        if(cnt_ticks >= 20){ cnt_ticks = 0; flag_reporte = 1; }
        INTCONbits.TMR0IF = 0;
    }
}

// ==============================================================================
// 12. REPORTE
// ==============================================================================
// Una sola linea compacta: menos texto = menos tiempo bloqueado transmitiendo por UART.
// Las ganancias (Kp/Ki/Kd/Kh/Khd) ya no van en cada linea (se confirman al cambiarlas).
void enviarReporte(void){
    char msg[150];
    char hd[12], wc[12];

    fmtDec(hd, heading, 1);
    fmtDec(wc, wzCorr, 1);

    sprintf(msg,
        "V1 %d/%d/%d V2 %d/%d/%d V3 %d/%d/%d V4 %d/%d/%d | Vx=%d Vy=%d Wz=%d | Hd=%s Wc=%s HH=%d\r\n",
        (int)spRpm[0], (int)rpmMedido[0], dutyActual[0],
        (int)spRpm[1], (int)rpmMedido[1], dutyActual[1],
        (int)spRpm[2], (int)rpmMedido[2], dutyActual[2],
        (int)spRpm[3], (int)rpmMedido[3], dutyActual[3],
        (int)Vx, (int)Vy, (int)Wz, hd, wc, holdActivo);
    UART_WriteString(msg);

    if(!imuOK) UART_WriteString("IMU no disponible.\r\n");

    if(avisoHold){
        avisoHold = 0;
        UART_WriteString("AVISO: error de rumbo > 30 grados, hold desactivado. Revisa el signo (N).\r\n");
    }
}

// ==============================================================================
// 13. COMANDOS
// ==============================================================================
uint8_t robotQuieto(void){
    uint8_t i;
    if(absf(Vx) > 0.5f || absf(Vy) > 0.5f || absf(Wz) > 0.5f) return 0;
    for(i = 0; i < 4; i++) if(spRpm[i] > 0.0f) return 0;
    return 1;
}

void comandoCalibrar(void){
    char msg[60], bs[12];
    if(!imuOK){ UART_WriteString("IMU no disponible.\r\n"); return; }
    if(!robotQuieto()){ UART_WriteString("Primero deten el chasis (S).\r\n"); return; }

    UART_WriteString("Calibrando giroscopio... no muevas el chasis.\r\n");
    if(calibrarGiroscopio()){
        fmtDec(bs, gyroBiasRaw / GYRO_LSB_POR_DPS, 2);
        sprintf(msg, "Calibrado. Sesgo = %s dps\r\n", bs);
        UART_WriteString(msg);
    } else {
        UART_WriteString("FALLO: el chasis se movio o la IMU no responde. Repite con C.\r\n");
    }
    INTCONbits.GIE = 0; gyroTicks = 0; INTCONbits.GIE = 1;
    heading = 0.0f; headingRef = 0.0f;
}

void comandoHold(int val){
    if(val >= 1){
        if(!imuOK){ UART_WriteString("IMU no disponible: hold no activado.\r\n"); return; }
        holdActivo = 1; headingRef = heading;
        UART_WriteString("Hold de rumbo ACTIVADO.\r\n");
    } else {
        holdActivo = 0; wzCorr = 0.0f;
        UART_WriteString("Hold de rumbo DESACTIVADO.\r\n");
    }
}

// Aplica un comando numerico ya completo (llego el Enter)
void aplicarValor(char modo, int val, uint8_t ruedaSel){
    if(modo == 'P' || modo == 'I' || modo == 'D' || modo == 'G' || modo == 'K'){
        if(val < 0) val = 0;                 // las ganancias no son negativas
    }
    switch(modo){
        case 'Y': Vy = limitarF((float)val, MAX_RPM_CMD); modoRueda = 0; recalcularCinematica(); UART_WriteString("Vy aplicado.\r\n"); break;
        case 'X': Vx = limitarF((float)val, MAX_RPM_CMD); modoRueda = 0; recalcularCinematica(); UART_WriteString("Vx aplicado.\r\n"); break;
        case 'W': Wz = limitarF((float)val, MAX_RPM_CMD); modoRueda = 0; recalcularCinematica(); UART_WriteString("Wz aplicado.\r\n"); break;
        case 'P': Kp  = val / 100.0f; UART_WriteString("Kp aplicado.\r\n");  break;
        case 'I': Ki  = val / 100.0f; UART_WriteString("Ki aplicado.\r\n");  break;
        case 'D': Kd  = val / 100.0f; UART_WriteString("Kd aplicado.\r\n");  break;
        case 'G': Kh  = val / 100.0f; UART_WriteString("Kh aplicado.\r\n");  break;
        case 'K': Khd = val / 100.0f; UART_WriteString("Khd aplicado.\r\n"); break;
        case 'Q': Kih = val / 100.0f; UART_WriteString("Kih aplicado.\r\n"); break;
        case 'J': comandoHold(val); break;
        case 'A': comandoRueda(ruedaSel, val); UART_WriteString("Rueda individual aplicada.\r\n"); break;
        default: break;
    }
}

// Maquina de estados que interpreta un caracter a la vez
void procesarCaracter(char c){
    static char    modo = 0;         // captura numerica en curso: Y X W P I D H G K  o  A (rueda)
    static char    prefijo = 0;      // 'D' o 'T' esperando la segunda letra de una rueda
    static uint8_t ruedaSel = 0;
    static char    buf[6];
    static uint8_t idx = 0;
    char u = (c >= 'a' && c <= 'z') ? (char)(c - 32) : c;      // a mayuscula

    // 1) segunda letra de un comando de rueda (DI DD DR TI TD TR)
    if(prefijo){
        char p = prefijo;
        uint8_t sel = 0;
        prefijo = 0;
        if(u == 'I')                  sel = (p == 'D') ? 1 : 3;     // DI = M1, TI = M3
        else if(u == 'D' || u == 'R') sel = (p == 'D') ? 2 : 4;     // DD/DR = M2, TD/TR = M4
        if(sel){ ruedaSel = sel; modo = 'A'; idx = 0; return; }
        if(p == 'T'){ modo = 0; idx = 0; return; }
        // p == 'D' sin letra de rueda: era Kd ("D2"); se sigue procesando 'u' abajo
    }

    // 2) comandos de una sola letra y prefijos de captura numerica
    switch(u){
        case 'S': detenerTodo(); modo = 0; idx = 0; UART_WriteString("STOP.\r\n"); return;
        case 'C': comandoCalibrar(); modo = 0; idx = 0; return;
        case 'Z': heading = 0.0f; headingRef = 0.0f; UART_WriteString("Rumbo a 0.\r\n"); return;
        case 'N': gyroSign = -gyroSign; heading = 0.0f; headingRef = 0.0f; UART_WriteString("Signo invertido.\r\n"); return;
        case 'T': prefijo = 'T'; modo = 0; idx = 0; return;
        case 'Y': case 'X': case 'W': case 'P': case 'I': case 'D': case 'J': case 'G': case 'K': case 'Q':
            modo = u; idx = 0;
            if(u == 'D') prefijo = 'D';          // puede ser Kd o DI/DD/DR
            return;
        // --- MODO MANUAL (interfaz web original): comandos instantaneos, sin Enter ---
        case 'F': Vy =  RPM_MANUAL; modoRueda = 0; recalcularCinematica(); return;   // avanzar
        case 'B': Vy = -RPM_MANUAL; modoRueda = 0; recalcularCinematica(); return;   // retroceder
        case 'L': Vx = -RPM_MANUAL; modoRueda = 0; recalcularCinematica(); return;   // lateral izq
        case 'R': Vx =  RPM_MANUAL; modoRueda = 0; recalcularCinematica(); return;   // lateral der
        default: break;
    }

    // 2.b) '1'/'2' del Modo Manual (rotar antihorario/horario) SOLO si no hay una
    // captura numerica en curso de otro comando (ej. "Y1" debe seguir siendo Vy=1,
    // no "rotar" seguido de un 1 suelto). Este es el mismo riesgo de ambigüedad de
    // digitos que ya se corrigio en la Capa 0 con el prefijo "M".
    if(modo == 0 && prefijo == 0){
        if(u == '1'){ Wz = -RPM_MANUAL; modoRueda = 0; recalcularCinematica(); return; }
        if(u == '2'){ Wz =  RPM_MANUAL; modoRueda = 0; recalcularCinematica(); return; }
    }

    // 3) captura del numero y Enter
    if(!modo) return;
    if(u == '-' && idx == 0){ buf[idx++] = '-'; return; }
    if(u >= '0' && u <= '9'){ if(idx < 4) buf[idx++] = u; return; }
    if(u == '\n' || u == '\r'){
        if(idx > 0){ buf[idx] = '\0'; aplicarValor(modo, atoi(buf), ruedaSel); }
        modo = 0; idx = 0;
    }
}

// ==============================================================================
// 14. PROGRAMA PRINCIPAL
// ==============================================================================
void main(void){
    char msg[70];
    char bs[12];
    char c;

    OSCCON = 0x70; OSCTUNEbits.PLLEN = 1;
    ANSELA = 0; ANSELB = 0; ANSELC = 0; ANSELD = 0; ANSELE = 0;

    // Puentes H deshabilitados hasta que todo este listo
    TRISEbits.TRISE0 = 0;
    EN_GLOBAL = 0;

    // Salidas PWM (RPWM/LPWM) en 0
    TRISBbits.TRISB0 = 0; TRISBbits.TRISB1 = 0; TRISBbits.TRISB2 = 0; TRISBbits.TRISB3 = 0;
    TRISDbits.TRISD0 = 0; TRISDbits.TRISD2 = 0; TRISDbits.TRISD3 = 0; TRISDbits.TRISD5 = 0;
    LATB &= 0b11110000;
    LATDbits.LATD0 = 0; LATDbits.LATD2 = 0; LATDbits.LATD3 = 0; LATDbits.LATD5 = 0;

    // Encoders: canal A en RB4-RB7 (interrupcion por cambio), canal B en RA0-RA3
    TRISB |= 0b11110000;
    TRISA |= 0b00001111;
    IOCB = 0b11110000;
    puerto_b_ant = PORTB;

    UART_Init();
    I2C_Init();

    T2CON = 0x04; PR2 = 150; PIE1bits.TMR2IE = 1;                       // PWM por software
    T0CON = 0x85; TMR0H = 0xF6; TMR0L = 0x3C; INTCONbits.TMR0IE = 1;   // base de 10 ms
    INTCONbits.RBIE = 1; INTCONbits.PEIE = 1; INTCONbits.GIE = 1;

    UART_WriteString("\r\n=== CAPA 4: Mecanum + PID + rumbo MPU9250 ===\r\n");

    imuOK = MPU_Init();
    sprintf(msg, "MPU WHOAMI = 0x%02X -> %s\r\n", imuWhoAmI, imuOK ? "OK" : "NO DISPONIBLE");
    UART_WriteString(msg);

    if(imuOK){
        UART_WriteString("Calibrando giroscopio... NO muevas el chasis.\r\n");
        if(calibrarGiroscopio()){
            fmtDec(bs, gyroBiasRaw / GYRO_LSB_POR_DPS, 2);
            sprintf(msg, "Giroscopio calibrado. Sesgo = %s dps\r\n", bs);
            UART_WriteString(msg);
        } else {
            UART_WriteString("AVISO: calibracion inestable. Con el chasis quieto y detenido envia C.\r\n");
        }
        INTCONbits.GIE = 0; gyroTicks = 0; INTCONbits.GIE = 1;
        heading = 0.0f; headingRef = 0.0f;
    }

    EN_GLOBAL = 1;                          // ahora si se habilitan los puentes H

    UART_WriteString("Listo. Y/X/W<rpm> | S | DI/DD/TI/TD<rpm> | P/I/D | H0/H1 G/K/F C Z N\r\n");

    while(1){
        while(rx_tail != rx_head){
            c = rx_fifo[rx_tail];
            rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);
            procesarCaracter(c);
        }

        actualizarGiroscopio();

        if(flag_control){
            flag_control = 0;
            cicloControl();
        }
        if(flag_reporte){
            flag_reporte = 0;
            enviarReporte();
        }
    }
}

```