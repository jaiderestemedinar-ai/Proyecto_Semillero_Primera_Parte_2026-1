// ==============================================================================
// CAPA 0 v3 - PROTOCOLO CORREGIDO (sin ambiguedad digitos/seleccion) +
// INVERSION AUTOMATICA DE M2/M4 (fisicamente montados/cableados al reves)
//
// BUG CORREGIDO en v2: los digitos '1'-'4' se usaban a la vez para
// seleccionar motor Y como parte del numero de duty, asi que escribir
// un duty como "45" o "23" disparaba un cambio de motor no deseado.
//
// Protocolo UART1 (FTDI, 115200 baud, 8N1) - NUEVO:
//   'M' luego un digito '1'-'4'  -> selecciona motor (dos teclas seguidas,
//                                    ej: presiona 'M' y luego '1' para elegir M1)
//   'F'               -> direccion adelante logica (ya compensa inversion de M2/M4)
//   'B'               -> direccion atras logica
//   'S'               -> detiene el motor activo (duty = 0)
//   "<numero>\n"      -> fija duty (0-100) del motor activo, ej "45\n" (ya no
//                         choca con la seleccion de motor)
//
// Reporte automatico cada 200ms:
//   "M<sel> dir=<F/B/S> duty=<pwm> pulsos=<contador_con_signo>\n"
//
// El signo de "pulsos" y el sentido fisico de "F" ya quedan coherentes para
// los 4 motores: si M2 y M4 estan invertidos fisicamente, este firmware lo
// compensa internamente (ver INVERT_M2 / INVERT_M4).
// ==============================================================================

#pragma config FOSC = INTIO67, WDTEN = OFF, LVP = OFF, PBADEN = OFF

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 64000000

#define EN_GLOBAL LATEbits.LATE0

// Canal B de cada encoder (da el signo)
#define ENCB_M1 PORTAbits.RA0
#define ENCB_M2 PORTAbits.RA1
#define ENCB_M3 PORTAbits.RA2
#define ENCB_M4 PORTAbits.RA3

// Confirmado en pruebas: M2 y M4 giran al reves de lo que su cableado RPWM/LPWM
// sugiere. Se compensa aqui en software para que 'F' sea SIEMPRE "adelante real"
// en los 4 motores, sin tener que recordar la excepcion en ningun otro lado.
#define INVERT_M2 1
#define INVERT_M4 1

volatile uint8_t motorActivo = 0;   // 0=ninguno, 1-4=motor seleccionado
volatile uint8_t duty = 0;          // 0-100
volatile char direccion = 'S';      // 'F','B','S'

volatile int32_t pulsos_0=0, pulsos_1=0, pulsos_2=0, pulsos_3=0;
uint8_t puerto_b_ant = 0;

volatile uint8_t th_M1A=0, th_M1B=0, th_M2A=0, th_M2B=0;
volatile uint8_t th_M3A=0, th_M3B=0, th_M4A=0, th_M4B=0;

#define RX_BUFFER_SIZE 32
volatile char rx_fifo[RX_BUFFER_SIZE];
volatile uint8_t rx_head=0, rx_tail=0;

volatile uint8_t flag_reporte = 0;
volatile uint8_t cnt_10ms = 0;

void aplicarSalidas(void){
    // Seguridad: apaga todo primero, nunca dos motores activos en esta capa
    th_M1A=0; th_M1B=0; th_M2A=0; th_M2B=0;
    th_M3A=0; th_M3B=0; th_M4A=0; th_M4B=0;

    uint8_t val = (direccion == 'S') ? 0 : duty;
    char dirEfectiva = direccion; // direccion logica pedida por el usuario

    switch(motorActivo){
        case 1:
            if(dirEfectiva=='F') th_M1A = val; else if(dirEfectiva=='B') th_M1B = val;
            break;
        case 2:
            #if INVERT_M2
                if(dirEfectiva=='F') th_M2B = val; else if(dirEfectiva=='B') th_M2A = val;
            #else
                if(dirEfectiva=='F') th_M2A = val; else if(dirEfectiva=='B') th_M2B = val;
            #endif
            break;
        case 3:
            if(dirEfectiva=='F') th_M3A = val; else if(dirEfectiva=='B') th_M3B = val;
            break;
        case 4:
            #if INVERT_M4
                if(dirEfectiva=='F') th_M4B = val; else if(dirEfectiva=='B') th_M4A = val;
            #else
                if(dirEfectiva=='F') th_M4A = val; else if(dirEfectiva=='B') th_M4B = val;
            #endif
            break;
        default: break;
    }
}

void __interrupt() interrupciones(void){
    static uint8_t cnt = 0;
    char c;

    // --- PWM por software, dos canales por motor (RPWM / LPWM), ~26.5kHz ---
    if(PIR1bits.TMR2IF){
        if(++cnt >= 100) cnt = 0;
        LATBbits.LATB0 = (cnt < th_M1A); LATBbits.LATB1 = (cnt < th_M1B);
        LATBbits.LATB2 = (cnt < th_M2A); LATBbits.LATB3 = (cnt < th_M2B);
        LATDbits.LATD0 = (cnt < th_M3A); LATDbits.LATD2 = (cnt < th_M3B);
        LATDbits.LATD3 = (cnt < th_M4A); LATDbits.LATD5 = (cnt < th_M4B);
        PIR1bits.TMR2IF = 0;
    }

    // --- UART RX a FIFO ---
    if(PIR1bits.RCIF){
        if(RCSTAbits.OERR){ RCSTAbits.CREN=0; RCSTAbits.CREN=1; }
        if(RCSTAbits.FERR){ c = RCREG; }
        while(PIR1bits.RCIF){
            rx_fifo[rx_head] = RCREG;
            rx_head = (rx_head + 1) & (RX_BUFFER_SIZE - 1);
        }
    }

    // --- Encoders en cuadratura via Interrupt-on-Change (RB4-RB7 = canal A) ---
    if(INTCONbits.RBIF){
        uint8_t pb = PORTB;
        uint8_t cambiaron = pb ^ puerto_b_ant;

        // M1: canal A = RB4, canal B = RA0
        if(cambiaron & 0x10){
            if(pb & 0x10){ if(ENCB_M1 == 0) pulsos_0++; else pulsos_0--; }
        }
        // M2: canal A = RB5, canal B = RA1
        if(cambiaron & 0x20){
            if(pb & 0x20){ if(ENCB_M2 == 0) pulsos_1++; else pulsos_1--; }
        }
        // M3: canal A = RB6, canal B = RA2
        if(cambiaron & 0x40){
            if(pb & 0x40){ if(ENCB_M3 == 0) pulsos_2++; else pulsos_2--; }
        }
        // M4: canal A = RB7, canal B = RA3
        if(cambiaron & 0x80){
            if(pb & 0x80){ if(ENCB_M4 == 0) pulsos_3++; else pulsos_3--; }
        }

        puerto_b_ant = pb;
        INTCONbits.RBIF = 0;
    }

    // --- Base de tiempo 10ms -> marca de reporte cada 200ms ---
    if(INTCONbits.TMR0IF){
        TMR0H = 0xF6; TMR0L = 0x3C;
        if(++cnt_10ms >= 20){
            cnt_10ms = 0;
            flag_reporte = 1;
        }
        INTCONbits.TMR0IF = 0;
    }
}

void UART_Init(void){
    TRISCbits.TRISC6=0; TRISCbits.TRISC7=1;
    BAUDCONbits.BRG16=1; TXSTAbits.BRGH=1;
    SPBRGH=0; SPBRG=138; // 115200 baud @ 64MHz
    TXSTA=0x24; RCSTA=0x90; PIE1bits.RCIE=1;
}

void UART_Write(char c){ while(!TXSTAbits.TRMT); TXREG = c; }
void UART_WriteString(char* s){ while(*s){ UART_Write(*s); s++; } }

void enviarReporte(void){
    char msg[64];
    int32_t p = 0;
    switch(motorActivo){
        case 1: p = pulsos_0; break;
        case 2: p = pulsos_1; break;
        case 3: p = pulsos_2; break;
        case 4: p = pulsos_3; break;
        default: break;
    }
    sprintf(msg, "M%d dir=%c duty=%d pulsos=%ld\r\n", motorActivo, direccion, duty, p);
    UART_WriteString(msg);
}

void main(void){
    char numBuf[6];
    uint8_t numIdx = 0;
    char c;
    uint8_t esperandoDigitoMotor = 0; // 1 justo despues de recibir 'M'

    OSCCON = 0x70; OSCTUNEbits.PLLEN = 1;
    ANSELA = 0; ANSELB = 0; ANSELC = 0; ANSELD = 0; ANSELE = 0;

    // EN_GLOBAL en LOW desde el arranque (seguridad primero)
    TRISEbits.TRISE0 = 0;
    EN_GLOBAL = 0;

    // Salidas RPWM/LPWM de los 4 motores
    TRISBbits.TRISB0=0; TRISBbits.TRISB1=0; TRISBbits.TRISB2=0; TRISBbits.TRISB3=0;
    TRISDbits.TRISD0=0; TRISDbits.TRISD2=0; TRISDbits.TRISD3=0; TRISDbits.TRISD5=0;
    LATB &= 0b11110000; // asegura RB0-3 en 0 al arrancar
    LATDbits.LATD0=0; LATDbits.LATD2=0; LATDbits.LATD3=0; LATDbits.LATD5=0;

    // Encoders: canal A en RB4-RB7 (entrada + IOC), canal B en RA0-RA3 (entrada simple)
    TRISB |= 0b11110000;
    TRISA |= 0b00001111;
    IOCB = 0b11110000;
    puerto_b_ant = PORTB;

    UART_Init();

    // PWM por software via TMR2
    T2CON = 0x04; PR2 = 150; PIE1bits.TMR2IE = 1;

    // Base de tiempo 10ms via TMR0
    T0CON = 0x85; TMR0H = 0xF6; TMR0L = 0x3C; INTCONbits.TMR0IE = 1;

    INTCONbits.RBIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    __delay_ms(50); // margen para que el PWM ya este generando 0% estable
    EN_GLOBAL = 1;  // ahora si habilitamos los 4 puentes H

    UART_WriteString("=== CAPA 0 v3 LISTO (protocolo corregido + inversion M2/M4) ===\r\n");
    UART_WriteString("Comandos: M1..M4 selecciona motor, F/B direccion, S detiene, numero+ENTER duty\r\n");
    UART_WriteString("Ejemplo: M1 luego F luego 45 y ENTER -> Motor 1 adelante al 45%\r\n");

    while(1){
        while(rx_tail != rx_head){
            c = rx_fifo[rx_tail];
            rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);

            if(c=='M' || c=='m'){
                esperandoDigitoMotor = 1; // el proximo caracter decide el motor
            }
            else if(esperandoDigitoMotor && c>='1' && c<='4'){
                motorActivo = c - '0';
                duty = 0; direccion = 'S';
                aplicarSalidas();
                numIdx = 0;
                esperandoDigitoMotor = 0;
                UART_WriteString("Motor seleccionado.\r\n");
            }
            else if(esperandoDigitoMotor){
                // Se presiono 'M' pero el siguiente caracter no fue 1-4: se ignora
                esperandoDigitoMotor = 0;
            }
            else if(c=='F' || c=='f'){
                direccion = 'F'; aplicarSalidas();
                UART_WriteString("Direccion: ADELANTE\r\n");
            }
            else if(c=='B' || c=='b'){
                direccion = 'B'; aplicarSalidas();
                UART_WriteString("Direccion: ATRAS\r\n");
            }
            else if(c=='S' || c=='s'){
                direccion = 'S'; duty = 0; aplicarSalidas();
                numIdx = 0;
                UART_WriteString("Motor detenido.\r\n");
            }
            else if(c >= '0' && c <= '9' && numIdx < 5){
                numBuf[numIdx++] = c;
            }
            else if(c=='\n' || c=='\r'){
                if(numIdx > 0 && motorActivo != 0){
                    numBuf[numIdx] = '\0';
                    int val = atoi(numBuf);
                    if(val < 0) val = 0;
                    if(val > 100) val = 100;
                    duty = (uint8_t)val;
                    aplicarSalidas();
                    UART_WriteString("Duty aplicado.\r\n");
                }
                numIdx = 0;
            }
        }

        if(flag_reporte){
            flag_reporte = 0;
            enviarReporte();
        }
    }
}
