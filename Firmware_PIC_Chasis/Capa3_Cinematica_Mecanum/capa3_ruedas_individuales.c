// ==============================================================================
// CAPA 3 - CINEMATICA MECANUM CON PID + FEEDFORWARD SIMULTANEO (4 RUEDAS)
// Primera vez que los 4 motores corren su lazo de control al mismo tiempo,
// alimentados por Vx (lateral), Vy (adelante/atras), Wz (rotacion).
//
// Protocolo UART1 (FTDI o XBee, 115200 baud, 8N1):
//   Y<num>\n   -> fija Vy (adelante positivo, atras negativo), ej "Y40\n" o "Y-40\n"
//   X<num>\n   -> fija Vx (lateral: derecha positivo, izquierda negativo)
//   W<num>\n   -> fija Wz (rotacion)
//   S          -> detiene todo (Vx=Vy=Wz=0, resetea los 4 lazos)
//
//   RUEDA INDIVIDUAL (diagnostico de contacto con el piso):
//   DI<num>\n  -> SOLO la Delantera Izquierda (M1) a <num> RPM (negativo = atras)
//   DD<num>\n  -> SOLO la Delantera Derecha  (M2)   (tambien acepta DR)
//   TI<num>\n  -> SOLO la Trasera Izquierda  (M3)
//   TD<num>\n  -> SOLO la Trasera Derecha    (M4)   (tambien acepta TR)
//                 Ej: "TD50\n" gira unicamente la trasera derecha a 50 RPM.
//                 Cada comando de rueda individual detiene las demas.
//   Nota: "D<digitos>" sigue siendo Kd (ej "D2"); solo "D" seguido de una
//         letra I/D/R se interpreta como rueda.
//   P<num>\n   -> Kp compartido = num/100.0 (aplica a los 4 motores)
//   I<num>\n   -> Ki compartido = num/100.0
//   D<num>\n   -> Kd compartido = num/100.0
//
// Reporte cada 200ms:
//   "V1 sp=.. rpm=.. duty=.. | V2 .. | V3 .. | V4 .. | Vx=.. Vy=.. Wz=.. Kp=.. Ki=.. Kd=..\n"
//
// NOTA: las ganancias Kp/Ki/Kd son COMPARTIDAS entre los 4 motores para
// simplificar la sintonizacion inicial. Si en las pruebas de Capa 2 alguno de
// los 4 necesito valores claramente distintos, edita los arrays KP_M[4],
// KI_M[4], KD_M[4] mas abajo con los valores especificos por rueda.
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

// Coeficientes de feedforward medidos: RPM = m*duty + c   [indice 0..3 = M1..M4]
const float FF_M_FWD[4] = {1.226f, 1.208f, 1.290f, 1.209f};
const float FF_C_FWD[4] = {-2.54f, -1.12f, -4.10f, -1.61f};
const float FF_M_REV[4] = {1.140f, 1.220f, 1.279f, 1.284f};
const float FF_C_REV[4] = {-1.00f,  2.05f, -4.20f, -3.50f};

#define PULSOS_POR_VUELTA 1090.0f

// Ganancias FINALES sintonizadas en pruebas reales con el chasis en el piso
// (equivalen a P11, I7, D2 por UART). Chasis estable en lateral con ~1.5-2cm
// de oscilacion maxima. PUNTO DE RESPALDO ANTES DE INTEGRAR LA MPU9250.
float Kp = 0.11f, Ki = 0.07f, Kd = 0.02f;

// --- Estado de las 4 ruedas ---
volatile int32_t pulsos[4] = {0,0,0,0};      // acumulado con signo (cuadratura)
int32_t pulsosAnt[4] = {0,0,0,0};
float integral[4] = {0,0,0,0};
float errorPrev[4] = {0,0,0,0};
float rpmMedido[4] = {0,0,0,0};
uint8_t dutyActual[4] = {0,0,0,0};
char direccion[4] = {'S','S','S','S'};
float spRpm[4] = {0,0,0,0};                  // magnitud (siempre positiva)

float Vx = 0.0f, Vy = 0.0f, Wz = 0.0f;

uint8_t puerto_b_ant = 0;

volatile uint8_t th_M1A=0, th_M1B=0, th_M2A=0, th_M2B=0;
volatile uint8_t th_M3A=0, th_M3B=0, th_M4A=0, th_M4B=0;

#define RX_BUFFER_SIZE 32
volatile char rx_fifo[RX_BUFFER_SIZE];
volatile uint8_t rx_head=0, rx_tail=0;

volatile uint8_t flag_reporte = 0;
volatile uint8_t flag_control = 0;
volatile uint8_t cnt_ticks = 0;

void recalcularCinematica(void){
    float tgt[4];
    tgt[0] = Vy + Vx + Wz; // M1
    tgt[1] = Vy - Vx - Wz; // M2
    tgt[2] = Vy - Vx + Wz; // M3
    tgt[3] = Vy + Vx - Wz; // M4

    for(uint8_t i=0; i<4; i++){
        if(tgt[i] > 0.01f){ direccion[i] = 'F'; spRpm[i] = tgt[i]; }
        else if(tgt[i] < -0.01f){ direccion[i] = 'B'; spRpm[i] = -tgt[i]; }
        else { direccion[i] = 'S'; spRpm[i] = 0.0f; }
    }
}

void detenerTodo(void){
    Vx = 0; Vy = 0; Wz = 0;
    for(uint8_t i=0; i<4; i++){
        direccion[i]='S'; spRpm[i]=0;
        integral[i]=0; errorPrev[i]=0; pulsosAnt[i]=0; dutyActual[i]=0;
    }
    th_M1A=0; th_M1B=0; th_M2A=0; th_M2B=0;
    th_M3A=0; th_M3B=0; th_M4A=0; th_M4B=0;
}

// Gira UNA sola rueda (rueda = 1..4 equivale a M1..M4) a 'rpm' (el signo da el
// sentido). Las otras tres quedan detenidas.
void comandoRueda(uint8_t rueda, int rpm){
    uint8_t i = rueda - 1;
    detenerTodo();

    // detenerTodo deja pulsosAnt en 0: se resincroniza con el contador actual
    // para que el primer calculo de RPM no vea un salto enorme de pulsos.
    INTCONbits.GIE = 0;
    for(uint8_t k = 0; k < 4; k++) pulsosAnt[k] = pulsos[k];
    INTCONbits.GIE = 1;

    if(rpm > 0){ direccion[i] = 'F'; spRpm[i] = (float)rpm; }
    else if(rpm < 0){ direccion[i] = 'B'; spRpm[i] = (float)(-rpm); }
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
        if(cambiaron & 0x10){ if(pb & 0x10){ if(ENCB_M1==0) pulsos[0]++; else pulsos[0]--; } }
        if(cambiaron & 0x20){ if(pb & 0x20){ if(ENCB_M2==0) pulsos[1]++; else pulsos[1]--; } }
        if(cambiaron & 0x40){ if(pb & 0x40){ if(ENCB_M3==0) pulsos[2]++; else pulsos[2]--; } }
        if(cambiaron & 0x80){ if(pb & 0x80){ if(ENCB_M4==0) pulsos[3]++; else pulsos[3]--; } }
        puerto_b_ant = pb;
        INTCONbits.RBIF = 0;
    }

    if(INTCONbits.TMR0IF){
        TMR0H=0xF6; TMR0L=0x3C;
        cnt_ticks++;
        if(cnt_ticks % 5 == 0) flag_control = 1;
        if(cnt_ticks >= 20){ cnt_ticks = 0; flag_reporte = 1; }
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
    char msg[150];
    sprintf(msg,
        "V1 sp=%d rpm=%d duty=%d | V2 sp=%d rpm=%d duty=%d | V3 sp=%d rpm=%d duty=%d | V4 sp=%d rpm=%d duty=%d | Vx=%d Vy=%d Wz=%d Kp=%d Ki=%d Kd=%d\r\n",
        (int)spRpm[0], (int)rpmMedido[0], dutyActual[0],
        (int)spRpm[1], (int)rpmMedido[1], dutyActual[1],
        (int)spRpm[2], (int)rpmMedido[2], dutyActual[2],
        (int)spRpm[3], (int)rpmMedido[3], dutyActual[3],
        (int)Vx, (int)Vy, (int)Wz,
        (int)(Kp*100), (int)(Ki*100), (int)(Kd*100));
    UART_WriteString(msg);
}

void calcularControlRueda(uint8_t idx){
    if(direccion[idx] == 'S' || spRpm[idx] <= 0.0f){
        integral[idx]=0; errorPrev[idx]=0; pulsosAnt[idx]=pulsos[idx]; dutyActual[idx]=0;
        rpmMedido[idx]=0;
        return;
    }

    int32_t actual = pulsos[idx];
    int32_t delta = actual - pulsosAnt[idx];
    pulsosAnt[idx] = actual;
    if(delta < 0) delta = -delta;

    rpmMedido[idx] = ((float)delta * 1200.0f) / PULSOS_POR_VUELTA;

    float m, c;
    if(direccion[idx] == 'F'){ m = FF_M_FWD[idx]; c = FF_C_FWD[idx]; }
    else { m = FF_M_REV[idx]; c = FF_C_REV[idx]; }

    float duty_ff = (spRpm[idx] - c) / m;
    if(duty_ff < 0) duty_ff = 0;
    if(duty_ff > 100) duty_ff = 100;

    float error = spRpm[idx] - rpmMedido[idx];

    if(duty_ff < 100.0f || error < 0){
        integral[idx] += error * 0.05f;
    }
    if(integral[idx] > 30.0f) integral[idx] = 30.0f;
    if(integral[idx] < -30.0f) integral[idx] = -30.0f;

    float derivada = (error - errorPrev[idx]) / 0.05f;
    errorPrev[idx] = error;

    float trim = (Kp*error) + (Ki*integral[idx]) + (Kd*derivada);

    float salida = duty_ff + trim;
    if(salida < 0) salida = 0;
    if(salida > 100) salida = 100;

    dutyActual[idx] = (uint8_t)salida;
}

void aplicarSalidasTodas(void){
    th_M1A=0; th_M1B=0; th_M2A=0; th_M2B=0;
    th_M3A=0; th_M3B=0; th_M4A=0; th_M4B=0;

    if(direccion[0]=='F') th_M1A = dutyActual[0]; else if(direccion[0]=='B') th_M1B = dutyActual[0];

    #if INVERT_M2
        if(direccion[1]=='F') th_M2B = dutyActual[1]; else if(direccion[1]=='B') th_M2A = dutyActual[1];
    #else
        if(direccion[1]=='F') th_M2A = dutyActual[1]; else if(direccion[1]=='B') th_M2B = dutyActual[1];
    #endif

    if(direccion[2]=='F') th_M3A = dutyActual[2]; else if(direccion[2]=='B') th_M3B = dutyActual[2];

    #if INVERT_M4
        if(direccion[3]=='F') th_M4B = dutyActual[3]; else if(direccion[3]=='B') th_M4A = dutyActual[3];
    #else
        if(direccion[3]=='F') th_M4A = dutyActual[3]; else if(direccion[3]=='B') th_M4B = dutyActual[3];
    #endif
}

void main(void){
    char numBuf[7];
    uint8_t numIdx = 0;
    char c;
    char modoEntrada = 0;
    char prefijoRueda = 0;   // 'D' o 'T' mientras se espera la segunda letra (DI/DD/TI/TD)
    uint8_t ruedaSel = 0;    // 1..4 = M1..M4, para el comando de rueda individual

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

    UART_WriteString("=== CAPA 3 LISTO (Cinematica Mecanum 4 ruedas simultaneas) ===\r\n");
    UART_WriteString("Y<rpm> Vy | X<rpm> Vx | W<rpm> Wz | S alto | P/I/D<val*100> ganancias\r\n");
    UART_WriteString("Rueda sola: DI/DD/TI/TD<rpm>  ej: TD50 (trasera derecha a 50 RPM)\r\n");

    while(1){
        while(rx_tail != rx_head){
            c = rx_fifo[rx_tail];
            rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);

            // --- Segunda letra de un comando de rueda individual (DI/DD/DR/TI/TD/TR) ---
            if(prefijoRueda){
                char p = prefijoRueda;
                uint8_t sel = 0;
                prefijoRueda = 0;
                if(c=='I' || c=='i')                        sel = (p=='D') ? 1 : 3; // DI=M1, TI=M3
                else if(c=='D'||c=='d'||c=='R'||c=='r')     sel = (p=='D') ? 2 : 4; // DD/DR=M2, TD/TR=M4
                if(sel){
                    ruedaSel = sel;
                    modoEntrada = 'A';
                    numIdx = 0;
                    continue;
                }
                if(p == 'T'){ modoEntrada = 0; numIdx = 0; continue; }
                // p=='D' y no era letra de rueda: era el comando Kd (ej "D2"),
                // el caracter se procesa normalmente mas abajo.
            }

            if(c=='S' || c=='s'){
                detenerTodo();
                modoEntrada = 0; numIdx = 0; prefijoRueda = 0;
                UART_WriteString("STOP.\r\n");
            }
            else if(c=='T' || c=='t'){
                prefijoRueda = 'T'; modoEntrada = 0; numIdx = 0;
            }
            else if(c=='Y'||c=='y'||c=='X'||c=='x'||c=='W'||c=='w'||
                    c=='P'||c=='p'||c=='I'||c=='i'||c=='D'||c=='d'){
                modoEntrada = (c=='Y'||c=='y') ? 'Y' :
                              (c=='X'||c=='x') ? 'X' :
                              (c=='W'||c=='w') ? 'W' :
                              (c=='P'||c=='p') ? 'P' :
                              (c=='I'||c=='i') ? 'I' : 'D';
                numIdx = 0;
                if(c=='D' || c=='d') prefijoRueda = 'D'; // puede ser Kd o DI/DD/DR
            }
            else if(modoEntrada && c=='-' && numIdx==0){
                numBuf[numIdx++] = c;
            }
            else if(modoEntrada && c>='0' && c<='9' && numIdx<6){
                numBuf[numIdx++] = c;
            }
            else if(modoEntrada && (c=='\n' || c=='\r')){
                if(numIdx > 0){
                    numBuf[numIdx]='\0';
                    int val = atoi(numBuf);
                    switch(modoEntrada){
                        case 'Y': Vy = (float)val; recalcularCinematica(); UART_WriteString("Vy aplicado.\r\n"); break;
                        case 'X': Vx = (float)val; recalcularCinematica(); UART_WriteString("Vx aplicado.\r\n"); break;
                        case 'W': Wz = (float)val; recalcularCinematica(); UART_WriteString("Wz aplicado.\r\n"); break;
                        case 'P': Kp = val/100.0f; UART_WriteString("Kp aplicado.\r\n"); break;
                        case 'I': Ki = val/100.0f; UART_WriteString("Ki aplicado.\r\n"); break;
                        case 'D': Kd = val/100.0f; UART_WriteString("Kd aplicado.\r\n"); break;
                        case 'A': comandoRueda(ruedaSel, val); UART_WriteString("Rueda individual aplicada.\r\n"); break;
                    }
                }
                modoEntrada = 0;
                numIdx = 0;
            }
        }

        if(flag_control){
            flag_control = 0;
            for(uint8_t i=0; i<4; i++) calcularControlRueda(i);
            aplicarSalidasTodas();
        }
        if(flag_reporte){
            flag_reporte = 0;
            enviarReporte();
        }
    }
}
