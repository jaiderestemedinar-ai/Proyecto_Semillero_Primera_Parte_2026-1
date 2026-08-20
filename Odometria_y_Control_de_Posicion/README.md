# Odometría y Control de Posición: La Prueba de los 50 cm y Caracterización Manual

Para lograr que el robot avanzara distancias exactas en el mundo real, tuvimos que construir un Lazo de Posición (Lazo Externo) que gobernara nuestro PID de velocidad (Lazo Interno). Esta implementación la dividimos en tres fases estratégicas:

## A. Reestructuración de Variables: Velocidad vs. Posición Absoluta

En las primeras fases, nuestros encoders contaban los pulsos en variables (p_m1, p_m2...) que se borraban a cero cada 50 milisegundos por medio de la interrupción del Timer0. Esto era útil para calcular la velocidad (pulsos por intervalo de tiempo), pero nos dejaba "ciegos" respecto a la distancia total recorrida.

La Solución en Código:
Implementamos un sistema de Acumuladores Absolutos. Creamos nuevas variables (pos_m1, pos_m2...) dentro de la rutina de interrupción (ISR).

```
if((c & 0x10) && (pb & 0x10)) { 
    if(ENCB_M1 == 0) { 
        p_m1++;       // Variable relativa (Se borra cada 50ms para el PID)
        pos_m1++;     // Variable absoluta (Odometría: Cuenta hasta el infinito)
    } else { 
        p_m1--; 
        pos_m1--; 
    } 
}
```

De esta forma, el PID seguía teniendo su dato de velocidad, mientras el "cerebro" principal ahora sabía exactamente en qué coordenada (en pulsos) se encontraba cada llanta desde que se encendió el robot. Promediando las 4 llantas (pos_m1 + pos_m2 + pos_m3 + pos_m4) / 4, obtuvimos la posición global y ultra precisa del chasis.

## B. El FTDI para Caracterización Manual

Antes de programar centímetros, necesitábamos saber empíricamente cuántos pulsos leía nuestro sistema por cada revolución completa de la llanta (360 grados). Intentar hacer esto con los motores encendidos era imposible porque el PID pelearía contra nuestra mano.

La Estrategia de Hardware:
En lugar de escribir un código de prueba complejo, usamos una técnica de aislamiento eléctrico:

Alimentación Lógica (5V): Dejamos conectado el módulo FTDI por USB. Esto mantuvo encendido el microcontrolador PIC18 y energizó los LEDs infrarrojos de los encoders.

Corte de Potencia (12V): Desconectamos físicamente la batería LiPo. Esto desenergizó la etapa de potencia de los puentes H (BTS7960).

Ejecución: Al estar los motores libres de torque, pudimos hacer una marca en la llanta, imprimir la variable absoluta (pos_m1) en el Monitor Serial y girar la rueda exactamente una vuelta con la mano.
Resultado: El monitor serial arrojó una lectura de ~1076 pulsos. Esto validó empíricamente la hoja de datos del motor (que indicaba 1080 pulsos teóricos), atribuyendo el pequeñísimo error al backlash (juego mecánico) de la caja reductora.

## C. Algoritmo de Navegación: Frenado Proporcional y Prueba 

Con la equivalencia de pulsos por centímetro calculada (que detallaremos en el Punto 3), enfrentamos un nuevo problema físico: La Inercia.
Si le decíamos al robot que avanzara a velocidad 35 y apagara los motores bruscamente al llegar al pulso objetivo de los 50 cm, el peso del chasis lo hacía resbalar y pasarse de la meta por casi 2.5 cm.

La Solución en Código (Máquina de Estados de Posición):
Diseñamos un algoritmo de aproximación que evalúa constantemente la distancia restante (meta_pulsos - pos_promedio):

Velocidad de Crucero: Si la distancia a la meta es grande (> 200 pulsos / ~4 cm), el Setpoint se mantiene alto (35).

Frenado de Aproximación: Al entrar en la zona de los últimos 4 centímetros, el Setpoint baja drásticamente (15). El robot "desacelera" para matar su inercia cinética.

Parada Seca: Al tocar el pulso 0, el Setpoint se vuelve 0 y la "Amnesia Matemática" del PID clava el chasis en el milímetro exacto.

Validación Bidireccional (Prueba Boomerang):
Para garantizar que el sistema no acumulara errores ni derrapes, programamos la Fase 5.1 como un viaje de ida y vuelta autónomo:

ESTADO 0: Avanzar hasta el equivalente a 50 cm positivos (+).

ESTADO 1: Pausa de 2 segundos para disipar fuerzas cinéticas residuales.

ESTADO 2: Invertir el Setpoint (reversa) hasta que los acumuladores absolutos volvieran a 0.

Conclusión: El robot regresó milimétricamente a su marca de origen, validando que el enrutador de dirección para los BTS7960 y la matemática de odometría funcionaban de manera simétrica en ambos sentidos.