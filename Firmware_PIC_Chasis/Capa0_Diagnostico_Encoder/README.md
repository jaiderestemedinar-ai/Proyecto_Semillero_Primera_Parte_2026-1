# Capa 0 — Diagnóstico de Motor/Encoder (lazo abierto)

Primer firmware del proyecto. Objetivo único: permitir medir el periodo crudo del encoder de **un** motor a la vez, con PWM fijo conocido, para empezar la caracterización empírica de la constante de pulsos por vuelta (ver `Especificaciones_del_Motor_y_Resolucion_Espacial/`).

- Sin PID, sin cinemática — lazo completamente abierto.
- Captura del periodo entre pulsos vía módulos CCP1–CCP4, usando Timer1 como base de tiempo.
- Protocolo: `1`/`2`/`3`/`4` selecciona el motor activo, `S` lo detiene, `<numero>` + Enter fija el duty (0–100) del motor seleccionado.
- Reporte automático cada 200 ms: `M<sel> duty=<pwm> per=<periodo_crudo> pulsos=<contador>`.

**Limitación encontrada y resuelta en la siguiente capa:** este protocolo usa dígitos tanto para seleccionar motor como para el valor de duty, lo que puede generar ambigüedad al interpretar comandos — corregido en `Capa0_v2_Cuadratura_BTS7960/`.

``` c
// ==============================================================================
// CAPA 0 - DIAGNOSTICO DE MOTOR/ENCODER (LAZO ABIERTO, SIN PID, SIN CINEMATICA)
// Objetivo unico: permitir medir el periodo crudo del encoder (per_X) de UN
// motor a la vez, con PWM fijo conocido, para calcular la constante real de
// conversion a RPM de forma EMPIRICA (no teorica).
//
// Protocolo por UART1 (FTDI, 115200 baud, 8N1):
//   '1','2','3','4'  -> selecciona el motor activo (detiene los demas)
//   'S'               -> detiene el motor activo (duty = 0)
//   "<numero>\n"      -> fija el duty (0-100) del motor activo, ej: "45\n"
//
// Reporte automatico cada 200ms por UART1:
//   "M<sel> duty=<pwm> per=<valor_crudo_CCP> pulsos=<contador>\n"
//
//   per=0 significa que no ha llegado ningun pulso reciente (motor detenido
//   o encoder desconectado). pulsos = contador acumulado, util para verificar
//   que efectivamente esta contando y no esta pegado en un valor fijo por ruido.
// ==============================================================================

#pragma config FOSC = INTIO67, WDTEN = OFF, LVP = OFF, PBADEN = OFF
#pragma config CCP2MX = PORTC1, CCP3MX = PORTB5

#include <xc.h>
#include <stdint.h>
#include <stdio.h>

#define _XTAL_FREQ 64000000

// --- Estado del motor seleccionado ---
volatile uint8_t motorActivo = 0;      // 0 = ninguno, 1-4 = motor seleccionado
volatile uint8_t duty = 0;             // 0-100 %, siempre direccion "adelante" (RPWM)

// --- Capturas crudas de encoder (periodo entre pulsos, en ticks de Timer1) ---
volatile unsigned int t_ant_0=0, t_ant_1=0, t_ant_2=0, t_ant_3=0;
volatile unsigned int per_0=0, per_1=0, per_2=0, per_3=0;
volatile uint32_t pulsos_0=0, pulsos_1=0, pulsos_2=0, pulsos_3=0;
volatile int inact_0=0, inact_1=0, inact_2=0, inact_3=0;

// --- PWM por software (mismo esquema que el proyecto original) ---
volatile uint8_t th_M1A=0, th_M1B=0, th_M2A=0, th_M2B=0;
volatile uint8_t th_M3A=0, th_M3B=0, th_M4A=0, th_M4B=0;

// --- Recepcion UART por FIFO simple ---
#define RX_BUFFER_SIZE 32
volatile char rx_fifo[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0, rx_tail = 0;

// --- Flag de reporte periodico (200ms) ---
volatile uint8_t flag_reporte = 0;
volatile uint8_t cnt_10ms = 0;

void aplicarDuty(void){
    // Apaga todos los motores primero (seguridad: nunca dos activos a la vez en esta capa)
    th_M1A=0; th_M1B=0; th_M2A=0; th_M2B=0;
    th_M3A=0; th_M3B=0; th_M4A=0; th_M4B=0;

    switch(motorActivo){
        case 1: th_M1A = duty; break;
        case 2: th_M2A = duty; break;
        case 3: th_M3A = duty; break;
        case 4: th_M4A = duty; break;
        default: break; // 0 = todo apagado
    }
}

void __interrupt() interrupciones(void){
    static uint8_t cnt=0;
    char c;

    // --- PWM por software (26.5kHz aprox, igual que el proyecto original) ---
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
        if(RCSTAbits.OERR){ RCSTAbits.CREN = 0; RCSTAbits.CREN = 1; }
        if(RCSTAbits.FERR){ c = RCREG; }
        while(PIR1bits.RCIF){
            rx_fifo[rx_head] = RCREG;
            rx_head = (rx_head + 1) & (RX_BUFFER_SIZE - 1);
        }
    }

    // --- Captura cruda de encoders (identico esquema al proyecto original) ---
    if(PIR1bits.CCP1IF){ per_0 = CCPR1 - t_ant_0; t_ant_0 = CCPR1; inact_0 = 0; pulsos_0++; PIR1bits.CCP1IF = 0; }
    if(PIR2bits.CCP2IF){ per_1 = CCPR2 - t_ant_1; t_ant_1 = CCPR2; inact_1 = 0; pulsos_1++; PIR2bits.CCP2IF = 0; }
    if(PIR4bits.CCP3IF){ per_2 = CCPR3 - t_ant_2; t_ant_2 = CCPR3; inact_2 = 0; pulsos_2++; PIR4bits.CCP3IF = 0; }
    if(PIR4bits.CCP4IF){
        unsigned int t = (CCPR4H << 8) | CCPR4L;
        per_3 = t - t_ant_3; t_ant_3 = t; inact_3 = 0; pulsos_3++;
        PIR4bits.CCP4IF = 0;
    }

    // --- Base de tiempo: marca de reporte cada 200ms + deteccion de inactividad ---
    if(INTCONbits.TMR0IF){
        TMR0H = 0xF6; TMR0L = 0x3C; // mismo recarga de 10ms del proyecto original

        // Si no llega pulso en ~100ms (10 ciclos de 10ms), consideramos motor detenido
        if(inact_0++ > 10) per_0 = 0;
        if(inact_1++ > 10) per_1 = 0;
        if(inact_2++ > 10) per_2 = 0;
        if(inact_3++ > 10) per_3 = 0;

        if(++cnt_10ms >= 20){ // 20 x 10ms = 200ms
            cnt_10ms = 0;
            flag_reporte = 1;
        }
        INTCONbits.TMR0IF = 0;
    }
}

void UART_Init(void){
    TRISCbits.TRISC6=0; TRISCbits.TRISC7=1;
    BAUDCONbits.BRG16=1; TXSTAbits.BRGH=1;
    SPBRGH=0; SPBRG=138; // 115200 baud @ 64MHz (identico al proyecto original)
    TXSTA=0x24; RCSTA=0x90; PIE1bits.RCIE=1;
}

void UART_Write(char c){
    while(!TXSTAbits.TRMT);
    TXREG = c;
}

void UART_WriteString(char* s){
    while(*s){ UART_Write(*s); s++; }
}

void enviarReporte(void){
    char msg[64];
    unsigned int per_actual = 0;
    uint32_t pulsos_actual = 0;

    switch(motorActivo){
        case 1: per_actual = per_0; pulsos_actual = pulsos_0; break;
        case 2: per_actual = per_1; pulsos_actual = pulsos_1; break;
        case 3: per_actual = per_2; pulsos_actual = pulsos_2; break;
        case 4: per_actual = per_3; pulsos_actual = pulsos_3; break;
        default: break;
    }

    sprintf(msg, "M%d duty=%d per=%u pulsos=%lu\r\n",
            motorActivo, duty, per_actual, pulsos_actual);
    UART_WriteString(msg);
}

void main(void){
    char numBuf[6];
    uint8_t numIdx = 0;
    char c;

    OSCCON = 0x70; OSCTUNEbits.PLLEN = 1;
    ANSELB = 0; ANSELC = 0; ANSELD = 0;

    // Salidas PWM (RPWM/LPWM) de los 4 motores
    TRISBbits.TRISB0=0; TRISBbits.TRISB1=0; TRISBbits.TRISB2=0; TRISBbits.TRISB3=0;
    TRISDbits.TRISD0=0; TRISDbits.TRISD2=0; TRISDbits.TRISD3=0; TRISDbits.TRISD5=0;

    // Entradas de encoder (CCP1=RC2, CCP2=RC1, CCP3=RB5, CCP4=RD1)
    TRISCbits.TRISC2=1; TRISCbits.TRISC1=1; TRISBbits.TRISB5=1; TRISDbits.TRISD1=1;

    UART_Init();

    // PWM por software via TMR2 (identico al proyecto original)
    T2CON = 0x04; PR2 = 150; PIE1bits.TMR2IE = 1;

    // Base de tiempo de 10ms via TMR0
    T0CON = 0x85; TMR0H = 0xF6; TMR0L = 0x3C; INTCONbits.TMR0IE = 1;

    // Captura de encoders: Timer1 (T1CON=0x31) + CCP1-4 en modo "cada flanco de subida"
    T1CON = 0x31;
    CCP1CON = 5; CCP2CON = 5; CCP3CON = 5; CCP4CON = 5;
    PIE1bits.CCP1IE=1; PIE2bits.CCP2IE=1; PIE4bits.CCP3IE=1; PIE4bits.CCP4IE=1;

    INTCONbits.PEIE = 1; INTCONbits.GIE = 1;

    UART_WriteString("=== CAPA 0 - DIAGNOSTICO LISTO ===\r\n");
    UART_WriteString("Comandos: 1/2/3/4 selecciona motor, S detiene, numero+ENTER fija duty 0-100\r\n");

    while(1){
        while(rx_tail != rx_head){
            c = rx_fifo[rx_tail];
            rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);

            if(c=='1' || c=='2' || c=='3' || c=='4'){
                motorActivo = c - '0';
                duty = 0;
                aplicarDuty();
                numIdx = 0;
                UART_WriteString("Motor seleccionado.\r\n");
            }
            else if(c=='S' || c=='s'){
                duty = 0;
                aplicarDuty();
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
                    aplicarDuty();
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
```