// ==============================================================================
// CAPA 4a - PRUEBA DEL MPU9250 SIN MOTORES
// Objetivo: comprobar el I2C, el sesgo del giroscopio, el SIGNO y la ESCALA del
// rumbo antes de meter la MPU al control del chasis.
//
// HARDWARE: SCL = RC3 (pin 18), SDA = RC4 (pin 23), AD0 a GND (direccion 0x68).
// Corre esta prueba con la BATERIA DE POTENCIA DE LOS BTS7960 DESCONECTADA.
//
// Comandos por UART (115200, FTDI en RC6/RC7), una letra, sin Enter:
//   C -> recalibra el sesgo del giroscopio (el sensor debe estar QUIETO)
//   Z -> pone el rumbo en 0
//   N -> invierte el signo del rumbo
//
// Convencion: rumbo POSITIVO = giro en sentido HORARIO visto desde arriba
// (el mismo sentido en que gira el chasis con Wz positivo).
//
// Reporte cada 200 ms:
//   "Hd=+12.3 Rate=+0.1 Bias=+0.31 Sg=- Raw=-12 [OK]"
// ==============================================================================

#pragma config FOSC = INTIO67, WDTEN = OFF, LVP = OFF, PBADEN = OFF

#include <xc.h>
#include <stdint.h>
#include <stdio.h>

#define _XTAL_FREQ 64000000

#define MPU_ADDR          0x68
#define GYRO_LSB_POR_DPS  131.0f   // rango +-250 dps
#define DT_TICK           0.01f    // el TMR0 interrumpe cada 10 ms

// ----------------------------------------------------------------------------
// ESTADO
// ----------------------------------------------------------------------------
volatile uint8_t gyroTicks = 0;     // ticks de 10 ms desde la ultima lectura
volatile uint8_t flagReporte = 0;

uint8_t imuOK = 0, imuWhoAmI = 0, imuFallos = 0;
int16_t gyroRaw = 0;
float gyroBiasRaw = 0.0f;
float gyroRate = 0.0f;              // grados/seg, positivo = horario
float gyroSign = -1.0f;             // -1: un MPU montado plano con Z hacia arriba
float heading = 0.0f;               // grados acumulados

// ----------------------------------------------------------------------------
// UTILIDADES
// ----------------------------------------------------------------------------
float absf(float x){ return (x < 0.0f) ? -x : x; }

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

// ----------------------------------------------------------------------------
// UART
// ----------------------------------------------------------------------------
void UART_Init(void){
    TRISCbits.TRISC6 = 0; TRISCbits.TRISC7 = 1;
    BAUDCONbits.BRG16 = 1; TXSTAbits.BRGH = 1;
    SPBRGH = 0; SPBRG = 138;            // 115200 baud @ 64 MHz
    TXSTA = 0x24; RCSTA = 0x90;
}
void UART_Write(char c){ while(!TXSTAbits.TRMT); TXREG = c; }
void UART_WriteString(const char *s){ while(*s){ UART_Write(*s); s++; } }

// ----------------------------------------------------------------------------
// I2C (MSSP1, maestro, 100 kHz) - todas las esperas tienen timeout
// ----------------------------------------------------------------------------
#define I2C_TIMEOUT 3000U
#define I2C_RW_BIT  0x04            // bit R/W de SSP1STAT (transmision en curso)

void I2C_Init(void){
    TRISCbits.TRISC3 = 1;           // SCL
    TRISCbits.TRISC4 = 1;           // SDA
    SSP1STAT = 0x80;                // SMP=1 (sin control de slew a 100 kHz)
    SSP1ADD  = 159;                 // 64e6 / (4 * 100e3) - 1
    SSP1CON1 = 0x28;                // SSPEN=1, modo maestro I2C
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
    return !SSP1CON2bits.ACKSTAT;   // 1 = el esclavo respondio ACK
}

uint8_t I2C_ReadByte(uint8_t *data, uint8_t enviarAck){
    uint16_t t = I2C_TIMEOUT;
    if(!I2C_WaitIdle()) return 0;
    SSP1CON2bits.RCEN = 1;          // RCEN esta en SSP1CON2
    while(!SSP1STATbits.BF){ if(--t == 0) return 0; }
    *data = SSP1BUF;
    if(!I2C_WaitIdle()) return 0;
    SSP1CON2bits.ACKDT = enviarAck ? 0 : 1;   // 0 = ACK (siguen mas bytes), 1 = NACK
    SSP1CON2bits.ACKEN = 1;
    t = I2C_TIMEOUT;
    while(SSP1CON2bits.ACKEN){ if(--t == 0) return 0; }
    return 1;
}

// ----------------------------------------------------------------------------
// MPU9250 (solo giroscopio Z)
// ----------------------------------------------------------------------------
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
    if(!MPU_WriteReg(0x6B, 0x80)) return 0;   // PWR_MGMT_1: reset
    __delay_ms(100);
    MPU_WriteReg(0x6B, 0x01);                 // despierta, reloj PLL
    __delay_ms(50);
    MPU_WriteReg(0x1A, 0x03);                 // CONFIG: filtro pasa-bajos 41 Hz
    MPU_WriteReg(0x1B, 0x00);                 // GYRO_CONFIG: +-250 dps
    if(!MPU_ReadReg(0x75, &who)) return 0;    // WHO_AM_I
    imuWhoAmI = who;
    return (who == 0x71 || who == 0x73 || who == 0x70 || who == 0x68);
}

// Promedia 300 muestras con el sensor quieto. Devuelve 0 si se movio.
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
        if(((int32_t)mx - (int32_t)mn) <= 400){      // ~3 dps pico a pico
            gyroBiasRaw = (float)suma / 300.0f;
            return 1;
        }
    }
    return 0;
}

void actualizarGiroscopio(void){
    uint8_t ticks;
    int16_t raw;

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
    heading += gyroRate * ((float)ticks * DT_TICK);   // usa los ticks reales transcurridos
}

// ----------------------------------------------------------------------------
// INTERRUPCION: solo la base de tiempo de 10 ms
// ----------------------------------------------------------------------------
void __interrupt() interrupciones(void){
    static uint8_t cnt200 = 0;
    if(INTCONbits.TMR0IF){
        TMR0H = 0xF6; TMR0L = 0x3C;
        if(gyroTicks < 250) gyroTicks++;
        if(++cnt200 >= 20){ cnt200 = 0; flagReporte = 1; }
        INTCONbits.TMR0IF = 0;
    }
}

// ----------------------------------------------------------------------------
// REPORTE
// ----------------------------------------------------------------------------
void enviarReporte(void){
    char msg[90], hd[12], rt[12], bs[12];
    fmtDec(hd, heading, 1);
    fmtDec(rt, gyroRate, 1);
    fmtDec(bs, gyroBiasRaw / GYRO_LSB_POR_DPS, 2);
    sprintf(msg, "Hd=%s Rate=%s Bias=%s Sg=%c Raw=%d [%s]\r\n",
            hd, rt, bs, (gyroSign > 0.0f) ? '+' : '-', gyroRaw,
            imuOK ? "OK" : "ERROR I2C");
    UART_WriteString(msg);
}

void calibrarYReportar(void){
    char msg[60], bs[12];
    UART_WriteString("Calibrando... no muevas el sensor.\r\n");
    if(calibrarGiroscopio()){
        fmtDec(bs, gyroBiasRaw / GYRO_LSB_POR_DPS, 2);
        sprintf(msg, "Calibrado. Sesgo = %s dps\r\n", bs);
        UART_WriteString(msg);
    } else {
        UART_WriteString("FALLO: el sensor se movio o no responde. Repite con C.\r\n");
    }
    INTCONbits.GIE = 0; gyroTicks = 0; INTCONbits.GIE = 1;
    heading = 0.0f;
}

// ----------------------------------------------------------------------------
void main(void){
    char msg[80];
    char c;

    OSCCON = 0x70; OSCTUNEbits.PLLEN = 1;
    ANSELC = 0;

    UART_Init();
    I2C_Init();

    T0CON = 0x85; TMR0H = 0xF6; TMR0L = 0x3C;
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIE = 1;

    UART_WriteString("\r\n=== CAPA 4a: PRUEBA MPU9250 (sin motores) ===\r\n");

    imuOK = MPU_Init();
    sprintf(msg, "WHOAMI = 0x%02X (esperado 0x71; tambien valen 0x70, 0x73, 0x68)\r\n", imuWhoAmI);
    UART_WriteString(msg);

    if(!imuOK){
        UART_WriteString("ERROR: no hay comunicacion I2C valida. Revisa SCL/SDA, voltaje, pull-ups y AD0.\r\n");
    } else {
        calibrarYReportar();
        UART_WriteString("Comandos: C recalibra | Z rumbo a 0 | N invierte signo\r\n");
    }

    while(1){
        if(PIR1bits.RCIF){
            if(RCSTAbits.OERR){ RCSTAbits.CREN = 0; RCSTAbits.CREN = 1; }
            c = RCREG;
            if(c == 'C' || c == 'c'){ if(imuOK) calibrarYReportar(); }
            else if(c == 'Z' || c == 'z'){ heading = 0.0f; UART_WriteString("Rumbo a 0.\r\n"); }
            else if(c == 'N' || c == 'n'){ gyroSign = -gyroSign; heading = 0.0f; UART_WriteString("Signo invertido.\r\n"); }
        }

        actualizarGiroscopio();

        if(flagReporte){
            flagReporte = 0;
            enviarReporte();
        }
    }
}
