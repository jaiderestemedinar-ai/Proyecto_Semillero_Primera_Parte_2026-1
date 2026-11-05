// ==============================================================================
// PASO 1 - ESCANER DE BUS I2C (sin logica de MPU9250)
// Unico objetivo: confirmar que el PIC puede hablar por I2C y encontrar que
// direccion responde. No asume nada sobre el sensor todavia.
//
// HARDWARE: SCL = RC3 (pin 18), SDA = RC4 (pin 23).
// UART: FTDI en RC6/RC7, 115200 baudios.
//
// Repite el escaneo completo cada 3 segundos. Reporta cada direccion que
// responda con ACK. El MPU9250 deberia aparecer en 0x68 (AD0 a GND) o 0x69
// (AD0 a VCC).
// ==============================================================================

#pragma config FOSC = INTIO67, WDTEN = OFF, LVP = OFF, PBADEN = OFF

#include <xc.h>
#include <stdint.h>
#include <stdio.h>

#define _XTAL_FREQ 64000000

// ---------------- UART ----------------
void UART_Init(void){
    TRISCbits.TRISC6 = 0; TRISCbits.TRISC7 = 1;
    BAUDCONbits.BRG16 = 1; TXSTAbits.BRGH = 1;
    SPBRGH = 0; SPBRG = 138; // 115200 baud @ 64MHz
    TXSTA = 0x24; RCSTA = 0x90;
}
void UART_Write(char c){ while(!TXSTAbits.TRMT); TXREG = c; }
void UART_WriteString(const char *s){ while(*s){ UART_Write(*s); s++; } }

// ---------------- I2C (MSSP1, maestro, 100 kHz), con timeout en cada espera ----------------
#define I2C_TIMEOUT 3000U

void I2C_Init(void){
    TRISCbits.TRISC3 = 1; // SCL
    TRISCbits.TRISC4 = 1; // SDA
    SSP1STAT = 0x80;      // SMP=1
    SSP1ADD  = 159;       // 100kHz @ 64MHz
    SSP1CON1 = 0x28;      // SSPEN=1, maestro I2C
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
    while((SSP1CON2 & 0x1F) || (SSP1STAT & 0x04)){
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

void I2C_Stop(void){
    uint16_t t = I2C_TIMEOUT;
    if(!I2C_WaitIdle()) return;
    SSP1CON2bits.PEN = 1;
    while(SSP1CON2bits.PEN){ if(--t == 0) return; }
}

// Devuelve 1 si el esclavo respondio ACK, 0 si no hubo respuesta o hubo timeout
uint8_t I2C_Write(uint8_t data){
    uint16_t t = I2C_TIMEOUT;
    if(!I2C_WaitIdle()) return 0;
    SSP1BUF = data;
    while(SSP1STATbits.BF){ if(--t == 0) return 0; }
    if(!I2C_WaitIdle()) return 0;
    return !SSP1CON2bits.ACKSTAT;
}

// Intenta direccionar 'addr7' en modo escritura. Devuelve 1 si hubo ACK.
uint8_t I2C_ProbeAddr(uint8_t addr7){
    uint8_t ok = I2C_Start();
    ok = ok && I2C_Write((uint8_t)(addr7 << 1));
    I2C_Stop();
    if(!ok) I2C_Recover();
    return ok;
}

void main(void){
    char msg[50];
    uint8_t addr;
    uint8_t encontrados;

    OSCCON = 0x70; OSCTUNEbits.PLLEN = 1;
    ANSELC = 0;

    UART_Init();
    I2C_Init();

    UART_WriteString("\r\n=== PASO 1: ESCANER I2C ===\r\n");
    UART_WriteString("Buscando dispositivos cada 3 segundos...\r\n");

    while(1){
        UART_WriteString("--- Escaneo ---\r\n");
        encontrados = 0;

        for(addr = 0x08; addr <= 0x77; addr++){
            if(I2C_ProbeAddr(addr)){
                sprintf(msg, "  Dispositivo encontrado en 0x%02X\r\n", addr);
                UART_WriteString(msg);
                encontrados++;
            }
            __delay_ms(2);
        }

        if(encontrados == 0){
            UART_WriteString("  NINGUNO. Revisa cableado, pull-ups y alimentacion.\r\n");
        } else {
            sprintf(msg, "Total: %d dispositivo(s).\r\n", encontrados);
            UART_WriteString(msg);
        }

        __delay_ms(3000);
    }
}
