Cinemática Inversa (Mecanum)
1. Objetivo

Deducir, a partir de la geometría de un chasis Mecanum de 4 ruedas, la relación matemática que convierte una velocidad deseada del chasis completo (avance, desplazamiento lateral y rotación) en la velocidad individual que debe tener cada rueda — la cinemática inversa, que es la que efectivamente se implementó y corre en el firmware del PIC de chasis.

Nota de alcance, por honestidad documental: esta sección cubre únicamente la cinemática inversa de la plataforma (Vx, Vy, Wz → RPM de cada rueda), que es lo que realmente se implementó y validó en este proyecto. No se implementó cinemática directa de la plataforma (odometría — reconstruir posición/orientación a partir de los encoders) ni cinemática del brazo (directa o inversa). La odometría queda explícitamente como trabajo futuro, registrado en Odometria_y_Control_de_Posicion/. La cinemática del brazo no forma parte del alcance de control implementado en este semillero y no se documenta aquí como si existiera.

2. Por qué una rueda Mecanum permite desplazamiento lateral

Una rueda Mecanum tiene una serie de rodillos libres montados en su circunferencia a 45° respecto al eje de giro de la rueda. Esto significa que la fuerza de contacto que la rueda transmite al suelo no es puramente tangencial (como en una rueda convencional) sino que tiene una componente lateral, cuya dirección depende de la orientación de los rodillos (que alterna en diagonal “/” o “” según la posición de la rueda en el chasis). Combinando las 4 ruedas con sus orientaciones cruzadas correctamente, es posible generar desplazamiento lateral puro, diagonal, rotación pura, o cualquier combinación, sin mover ningún elemento mecánico adicional.

3. Convención de ejes y variables
Vy: velocidad de avance/retroceso del chasis (adelante positivo).
Vx: velocidad de desplazamiento lateral (derecha positiva).
Wz: velocidad angular de rotación sobre el eje vertical (horario positivo, visto desde arriba).
M1..M4: velocidad individual de cada rueda, numeradas M1 = delantera izquierda, M2 = delantera derecha, M3 = trasera izquierda, M4 = trasera derecha.
4. Deducción de la matriz de cinemática inversa

Para un chasis Mecanum con la disposición estándar de rodillos (delanteras y traseras en configuración cruzada, de modo que el patrón de rodillos sea simétrico en X), la velocidad de cada rueda es la suma algebraica de las tres contribuciones independientes:

La contribución de Vy es la misma para las 4 ruedas (todas deben girar igual para avanzar en línea recta).
La contribución de Vx tiene signo opuesto entre el lado izquierdo y el lado derecho del chasis (para que las fuerzas laterales de los rodillos se combinen en vez de cancelarse).
La contribución de Wz tiene signo opuesto entre el lado delantero-izquierdo/trasero-derecho y el lado delantero-derecho/trasero-izquierdo (patrón diagonal, característico de la rotación en un chasis Mecanum de 4 ruedas).

Esto da la matriz de cinemática inversa adoptada e implementada:

M1 = Vy + Vx + Wz      (delantera izquierda)
M2 = Vy - Vx - Wz      (delantera derecha)
M3 = Vy - Vx + Wz      (trasera izquierda)
M4 = Vy + Vx - Wz      (trasera derecha)

Cada Mi resultante es la magnitud de RPM deseada para esa rueda (con signo, donde el signo indica el sentido de giro: adelante o reversa). Esta es exactamente la relación implementada en la función recalcularCinematica() del firmware (Firmware_PIC_Chasis/, Capas 3 y 4), que se ejecuta cada vez que llega un nuevo comando Y, X o W, o un comando de Modo Manual (F/B/L/R/1/2).

5. De la magnitud con signo a comando físico del motor

El resultado de la matriz (Mi) tiene signo, pero el controlador físico de cada rueda (PID + feedforward) trabaja con magnitud siempre positiva + una dirección explícita ('F'/'R' en la variable direccion[i], ver Caracterizacion_de_las_Constantes_PID_Kp_Ki_Kd/), porque las curvas de feedforward son distintas para adelante y para reversa. El firmware separa estos dos aspectos:

c
if(tgt[i] > 0.01f)      { nuevaDir = 'F'; sp =  tgt[i]; }
else if(tgt[i] < -0.01f){ nuevaDir = 'R'; sp = -tgt[i]; }
else                    { nuevaDir = 'S'; sp = 0; }

donde tgt[i] es el resultado de la matriz para la rueda i. Este paso es el puente concreto entre la cinemática (matemática pura) y el control (PID físico por rueda).

6. Límite de magnitud

Antes de aplicarse, cada componente (Vy, Vx, Wz) individualmente — y el resultado de cada rueda — se acota a MAX_RPM_CMD (120 RPM), tanto por seguridad mecánica como para evitar pedir al controlador una consigna que el motor físicamente no puede alcanzar (lo que llevaría a saturación permanente y, sin el anti-windup del PID, a un error integral descontrolado).

7. Validación práctica realizada

Se validó experimentalmente que la matriz produce el comportamiento esperado en los 4 modos de movimiento básicos (avance, lateral, diagonal y rotación pura), observando el chasis físico y confirmando telemetría de las 4 ruedas coherente con los signos esperados de la matriz en cada caso. El hallazgo de deriva durante el movimiento lateral puro (documentado en Especificaciones_Iniciales_de_Hardware.md, sección de ensamblaje) se investigó inicialmente como un posible error en esta matriz o en las ganancias del PID, y se descartó explícitamente ambas hipótesis mediante telemetría individual de las 4 ruedas antes de identificar la causa mecánica real — un ejemplo documentado de cómo la telemetría (ver Captura_de_Encoders_y_Telemetria/) se usó para descartar sistemáticamente hipótesis de software antes de mirar hacia el hardware.