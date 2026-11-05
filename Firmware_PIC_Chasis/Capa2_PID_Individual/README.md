# Capa 2 — PID Individual por Rueda

Primer firmware con control de velocidad en lazo cerrado real, aplicado a **un motor a la vez** (no los 4 simultáneos todavía — eso llega en la Capa 3).

- Control PID + feedforward sobre el error de RPM (ver el diseño matemático completo en `PID_Caracterizacion_constantes/`).
- Ganancias ajustables en vivo por UART, sin reprogramar: `P<n>` / `I<n>` / `D<n>` (valor × 100, ej. `I30` = Ki 0.30).
- Permitió validar el controlador y las curvas de feedforward motor por motor antes de arriesgarse a coordinar los 4 al mismo tiempo con cinemática Mecanum.


```c
// ==============================================================================
// CAPA 2 - PID DE VELOCIDAD INDIVIDUAL POR RUEDA, CON FEEDFORWARD
// Un motor a la vez (en el aire), setpoint en RPM real, no en duty crudo.
//
// Protocolo UART1 (FTDI, 115200 baud, 8N1):
//   M1..M4        -> selecciona motor
//   F / B         -> direccion (resetea el lazo de control)
//   S             -> detiene (setpoint=0, resetea integral)
//   V<num>\n      -> fija el setpoint en RPM, ej "V60\n" = 60 RPM
//   P<num>\n      -> fija Kp = num/100.0, ej "P30\n" = Kp 0.30
//   I<num>\n      -> fija Ki = num/100.0
//   D<num>\n      -> fija Kd = num/100.0
//
// Reporte cada 200ms:
//   "M<sel> dir=<> sp=<rpm_pedido> rpm=<rpm_medido> duty=<duty_aplicado>
//    Kp=<> Ki=<> Kd=<>\n"
// ==============================================================================

#pragma config FOSC = INTIO67, WDTEN = OFF, LVP = OFF, PBADEN = OFF

#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define _XTAL_FREQ 64000000
#define EN_GLOBAL LATEbits.LATE0

#define ENCB_M1 PORTAbits.RA0
#define ENCB_M2 PORTAbits.RA1
#define ENCB_M3 PORTAbits.RA2
#define ENCB_M4 PORTAbits.RA3

#define INVERT_M2 1
#define INVERT_M4 1

// --- Coeficientes de feedforward medidos: RPM = m*duty + c ---
// Indice 0..3 = M1..M4
const float FF_M_FWD[4] = {1.226f, 1.208f, 1.290f, 1.209f};
const float FF_C_FWD[4] = {-2.54f, -1.12f, -4.10f, -1.61f};
const float FF_M_REV[4] = {1.140f, 1.220f, 1.279f, 1.284f};
const float FF_C_REV[4] = {-1.00f,  2.05f, -4.20f, -3.50f};

#define PULSOS_POR_VUELTA 1090.0f

volatile uint8_t motorActivo = 0;
volatile char direccion = 'S';
volatile float setpoint_rpm = 0.0f;

volatile int32_t pulsos_0=0, pulsos_1=0, pulsos_2=0, pulsos_3=0;
int32_t pulsos_ant = 0;
uint8_t puerto_b_ant = 0;

volatile uint8_t th_M1A=0, th_M1B=0, th_M2A=0, th_M2B=0;
volatile uint8_t th_M3A=0, th_M3B=0, th_M4A=0, th_M4B=0;

float Kp = 0.30f, Ki = 0.05f, Kd = 0.0f;
float integral = 0.0f, error_prev = 0.0f;

float rpm_medido = 0.0f;
uint8_t duty_actual = 0;

#define RX_BUFFER_SIZE 32
volatile char rx_fifo[RX_BUFFER_SIZE];
volatile uint8_t rx_head=0, rx_tail=0;

volatile uint8_t flag_reporte = 0;
volatile uint8_t flag_control = 0;
volatile uint8_t cnt_ticks = 0;

void resetControl(void){
    integral = 0.0f;
    error_prev = 0.0f;
    pulsos_ant = 0;
    duty_actual = 0;
}

void aplicarDuty(uint8_t val){
    th_M1A=0; th_M1B=0; th_M2A=0; th_M2B=0;
    th_M3A=0; th_M3B=0; th_M4A=0; th_M4B=0;

    switch(motorActivo){
        case 1:
            if(direccion=='F') th_M1A=val; else if(direccion=='B') th_M1B=val;
            break;
        case 2:
            #if INVERT_M2
                if(direccion=='F') th_M2B=val; else if(direccion=='B') th_M2A=val;
            #else
                if(direccion=='F') th_M2A=val; else if(direccion=='B') th_M2B=val;
            #endif
            break;
        case 3:
            if(direccion=='F') th_M3A=val; else if(direccion=='B') th_M3B=val;
            break;
        case 4:
            #if INVERT_M4
                if(direccion=='F') th_M4B=val; else if(direccion=='B') th_M4A=val;
            #else
                if(direccion=='F') th_M4A=val; else if(direccion=='B') th_M4B=val;
            #endif
            break;
        default: break;
    }
}

int32_t leerPulsosActivo(void){
    switch(motorActivo){
        case 1: return pulsos_0;
        case 2: return pulsos_1;
        case 3: return pulsos_2;
        case 4: return pulsos_3;
        default: return 0;
    }
}

void __interrupt() interrupciones(void){
    static uint8_t cnt = 0;
    char c;

    if(PIR1bits.TMR2IF){
        if(++cnt >= 100) cnt = 0;
        LATBbits.LATB0 = (cnt < th_M1A); LATBbits.LATB1 = (cnt < th_M1B);
        LATBbits.LATB2 = (cnt < th_M2A); LATBbits.LATB3 = (cnt < th_M2B);
        LATDbits.LATD0 = (cnt < th_M3A); LATDbits.LATD2 = (cnt < th_M3B);
        LATDbits.LATD3 = (cnt < th_M4A); LATDbits.LATD5 = (cnt < th_M4B);
        PIR1bits.TMR2IF = 0;
    }

    if(PIR1bits.RCIF){
        if(RCSTAbits.OERR){ RCSTAbits.CREN=0; RCSTAbits.CREN=1; }
        if(RCSTAbits.FERR){ c = RCREG; }
        while(PIR1bits.RCIF){
            rx_fifo[rx_head] = RCREG;
            rx_head = (rx_head + 1) & (RX_BUFFER_SIZE - 1);
        }
    }

    if(INTCONbits.RBIF){
        uint8_t pb = PORTB;
        uint8_t cambiaron = pb ^ puerto_b_ant;
        if(cambiaron & 0x10){ if(pb & 0x10){ if(ENCB_M1==0) pulsos_0++; else pulsos_0--; } }
        if(cambiaron & 0x20){ if(pb & 0x20){ if(ENCB_M2==0) pulsos_1++; else pulsos_1--; } }
        if(cambiaron & 0x40){ if(pb & 0x40){ if(ENCB_M3==0) pulsos_2++; else pulsos_2--; } }
        if(cambiaron & 0x80){ if(pb & 0x80){ if(ENCB_M4==0) pulsos_3++; else pulsos_3--; } }
        puerto_b_ant = pb;
        INTCONbits.RBIF = 0;
    }

    if(INTCONbits.TMR0IF){
        TMR0H=0xF6; TMR0L=0x3C;
        cnt_ticks++;
        if(cnt_ticks % 5 == 0) flag_control = 1;    // cada 50ms
        if(cnt_ticks >= 20){ cnt_ticks = 0; flag_reporte = 1; } // cada 200ms
        INTCONbits.TMR0IF = 0;
    }
}

void UART_Init(void){
    TRISCbits.TRISC6=0; TRISCbits.TRISC7=1;
    BAUDCONbits.BRG16=1; TXSTAbits.BRGH=1;
    SPBRGH=0; SPBRG=138;
    TXSTA=0x24; RCSTA=0x90; PIE1bits.RCIE=1;
}
void UART_Write(char c){ while(!TXSTAbits.TRMT); TXREG=c; }
void UART_WriteString(char* s){ while(*s){ UART_Write(*s); s++; } }

void enviarReporte(void){
    char msg[100];
    char kpBuf[8], kiBuf[8], kdBuf[8];
    // sprintf con %f no siempre esta disponible en XC8 free sin config extra;
    // formateamos manualmente con 2 decimales multiplicando por 100
    sprintf(msg, "M%d dir=%c sp=%d rpm=%d duty=%d Kp=%d Ki=%d Kd=%d\r\n",
            motorActivo, direccion,
            (int)setpoint_rpm, (int)rpm_medido, duty_actual,
            (int)(Kp*100), (int)(Ki*100), (int)(Kd*100));
    UART_WriteString(msg);
}

void calcularControl(void){
    if(motorActivo == 0 || direccion == 'S' || setpoint_rpm <= 0.0f){
        resetControl();
        aplicarDuty(0);
        rpm_medido = 0.0f;
        return;
    }

    int32_t actual = leerPulsosActivo();
    int32_t delta = actual - pulsos_ant;
    pulsos_ant = actual;
    if(delta < 0) delta = -delta; // magnitud, la direccion ya la sabemos por comando

    // Periodo de control = 50ms -> RPM = delta * (60/0.05) / PULSOS_POR_VUELTA
    rpm_medido = ((float)delta * 1200.0f) / PULSOS_POR_VUELTA;

    // --- Feedforward segun motor y direccion ---
    float m, c;
    uint8_t idx = motorActivo - 1;
    if(direccion == 'F'){ m = FF_M_FWD[idx]; c = FF_C_FWD[idx]; }
    else { m = FF_M_REV[idx]; c = FF_C_REV[idx]; }

    float duty_ff = (setpoint_rpm - c) / m;
    if(duty_ff < 0) duty_ff = 0;
    if(duty_ff > 100) duty_ff = 100;

    // --- PID de corrección (trim) sobre el error de RPM ---
    float error = setpoint_rpm - rpm_medido;

    float salida_sin_trim = duty_ff; // para anti-windup
    if(salida_sin_trim < 100.0f || error < 0){
        integral += error * 0.05f; // dt = 50ms = 0.05s
    }
    if(integral > 30.0f) integral = 30.0f;
    if(integral < -30.0f) integral = -30.0f;

    float derivada = (error - error_prev) / 0.05f;
    error_prev = error;

    float trim = (Kp * error) + (Ki * integral) + (Kd * derivada);

    float salida = duty_ff + trim;
    if(salida < 0) salida = 0;
    if(salida > 100) salida = 100;

    duty_actual = (uint8_t)salida;
    aplicarDuty(duty_actual);
}

void main(void){
    char numBuf[6];
    uint8_t numIdx = 0;
    char c;
    uint8_t esperandoDigitoMotor = 0;
    char modoEntrada = 0; // 'V','P','I','D' o 0 = ninguno

    OSCCON = 0x70; OSCTUNEbits.PLLEN = 1;
    ANSELA=0; ANSELB=0; ANSELC=0; ANSELD=0; ANSELE=0;

    TRISEbits.TRISE0 = 0;
    EN_GLOBAL = 0;

    TRISBbits.TRISB0=0; TRISBbits.TRISB1=0; TRISBbits.TRISB2=0; TRISBbits.TRISB3=0;
    TRISDbits.TRISD0=0; TRISDbits.TRISD2=0; TRISDbits.TRISD3=0; TRISDbits.TRISD5=0;
    LATB &= 0b11110000;
    LATDbits.LATD0=0; LATDbits.LATD2=0; LATDbits.LATD3=0; LATDbits.LATD5=0;

    TRISB |= 0b11110000;
    TRISA |= 0b00001111;
    IOCB = 0b11110000;
    puerto_b_ant = PORTB;

    UART_Init();
    T2CON=0x04; PR2=150; PIE1bits.TMR2IE=1;
    T0CON=0x85; TMR0H=0xF6; TMR0L=0x3C; INTCONbits.TMR0IE=1;
    INTCONbits.RBIE=1; INTCONbits.PEIE=1; INTCONbits.GIE=1;

    __delay_ms(50);
    EN_GLOBAL = 1;

    UART_WriteString("=== CAPA 2 LISTO (PID + Feedforward por rueda) ===\r\n");
    UART_WriteString("M1..M4 motor | F/B direccion | S alto | V<rpm> P<kp*100> I<ki*100> D<kd*100>\r\n");

    while(1){
        while(rx_tail != rx_head){
            c = rx_fifo[rx_tail];
            rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);

            if(c=='M' || c=='m'){
                esperandoDigitoMotor = 1;
            }
            else if(esperandoDigitoMotor && c>='1' && c<='4'){
                motorActivo = c - '0';
                direccion = 'S'; setpoint_rpm = 0;
                resetControl(); aplicarDuty(0);
                esperandoDigitoMotor = 0;
                UART_WriteString("Motor seleccionado.\r\n");
            }
            else if(esperandoDigitoMotor){
                esperandoDigitoMotor = 0;
            }
            else if(c=='F' || c=='f'){
                direccion='F'; resetControl();
                UART_WriteString("Direccion: ADELANTE\r\n");
            }
            else if(c=='B' || c=='b'){
                direccion='B'; resetControl();
                UART_WriteString("Direccion: ATRAS\r\n");
            }
            else if(c=='S' || c=='s'){
                direccion='S'; setpoint_rpm=0; resetControl(); aplicarDuty(0);
                UART_WriteString("Motor detenido.\r\n");
            }
            else if(c=='V'||c=='v'||c=='P'||c=='p'||c=='I'||c=='i'||c=='D'||c=='d'){
                modoEntrada = (c=='V'||c=='v') ? 'V' : (c=='P'||c=='p') ? 'P' : (c=='I'||c=='i') ? 'I' : 'D';
                numIdx = 0;
            }
            else if(modoEntrada && c>='0' && c<='9' && numIdx<5){
                numBuf[numIdx++] = c;
            }
            else if(modoEntrada && (c=='\n' || c=='\r')){
                if(numIdx > 0){
                    numBuf[numIdx]='\0';
                    int val = atoi(numBuf);
                    switch(modoEntrada){
                        case 'V': setpoint_rpm = (float)val; resetControl(); UART_WriteString("Setpoint aplicado.\r\n"); break;
                        case 'P': Kp = val/100.0f; UART_WriteString("Kp aplicado.\r\n"); break;
                        case 'I': Ki = val/100.0f; UART_WriteString("Ki aplicado.\r\n"); break;
                        case 'D': Kd = val/100.0f; UART_WriteString("Kd aplicado.\r\n"); break;
                    }
                }
                modoEntrada = 0;
                numIdx = 0;
            }
        }

        if(flag_control){
            flag_control = 0;
            calcularControl();
        }
        if(flag_reporte){
            flag_reporte = 0;
            enviarReporte();
        }
    }
}

```