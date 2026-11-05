# Captura de Encoders y Telemetría

## 1. Objetivo de esta sección

Describir cómo el firmware convierte los pulsos físicos de los encoders en una magnitud de velocidad utilizable (RPM), y cómo esa información —junto con el estado completo del control— se empaqueta y transmite en tiempo real para alimentar el gemelo digital.

## 2. Captura de pulsos: decodificación en cuadratura

Cada motor GM37-520 entrega dos canales de encoder desfasados 90° (A y B), lo que permite no solo contar pulsos sino determinar el sentido de giro sin necesidad de inferirlo del comando enviado al puente H — un dato críticamente importante, porque el sentido mecánico real puede no coincidir con el sentido eléctrico esperado (como ocurrió con M2 y M4, que giran invertidos por montaje).

Esquema adoptado:

Canal A de cada uno de los 4 encoders se conecta a un pin del puerto B (RB4–RB7), configurado para generar una interrupción por cambio de estado (Interrupt-on-Change, flag RBIF).
Canal B de cada encoder se lee de forma directa (sin interrupción) en el mismo instante en que se atiende el cambio del canal A, únicamente para decidir el signo.

```c
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
```

Cada motor mantiene un contador acumulado con signo (int32_t pulsos[4]), que crece o decrece según el sentido real de giro detectado por el canal B. Este contador nunca se “limpia” dentro del lazo de control — en su lugar, cada ciclo de control toma la diferencia respecto al valor leído en el ciclo anterior (pulsosAnt[4]), lo que da la cantidad de pulsos ocurridos en ese intervalo, con su signo.

Esta elección (acumulador continuo + diferencia por ciclo, en vez de “contar y resetear”) es la misma base necesaria para una futura odometría: el acumulado íntegro de pulsos, si se preserva entre ciclos de control, es directamente la magnitud que habría que escalar a centímetros — lo cual queda marcado como trabajo futuro en Odometria_y_Control_de_Posicion/.

## 3. De pulsos a RPM: la base de tiempo real

La conversión de “pulsos en un intervalo” a “RPM” requiere dividir por el tiempo real transcurrido, no por un tiempo asumido. Este punto tiene su propio apartado dedicado — y su propio caso de estudio de depuración — en Caracterizacion_de_las_Constantes_PID_Kp_Ki_Kd/ (sección 5 de ese documento), porque fue la causa raíz de un defecto real del sistema. En resumen aquí:

```c
rpm = (pulsos_en_el_intervalo / PULSOS_POR_VUELTA) * (60.0f / dt_real_segundos)
```


donde dt_real_segundos se mide con un contador de hardware (ctrlTicks, incrementado cada 10 ms por la interrupción de Timer0) en vez de asumirse fijo en 50 ms. PULSOS_POR_VUELTA = 1090, constante obtenida empíricamente (ver Especificaciones_del_Motor_y_Resolucion_Espacial/).

## 4. Empaquetado de la telemetría

Cada ~200 ms (controlado por el mismo contador de 10 ms, cnt_ticks), el firmware construye y transmite una sola línea compacta por UART:

```c
sprintf(msg,
    "V1 %d/%d/%d V2 %d/%d/%d V3 %d/%d/%d V4 %d/%d/%d | Vx=%d Vy=%d Wz=%d | Hd=%s Wc=%s HH=%d\r\n",
    (int)spRpm[0], (int)rpmMedido[0], dutyActual[0],
    ...
    (int)Vx, (int)Vy, (int)Wz, hd, wc, holdActivo);
```

Cada rueda reporta tres valores — consigna de RPM, RPM medida real y duty aplicado (0–100) — lo que permite, sin ningún equipo adicional, verificar en vivo si el PID está siguiendo la consigna o si una rueda específica tiene un problema físico (como ocurrió en el diagnóstico de la deriva lateral, ver Caracterizacion_de_las_Constantes_PID_Kp_Ki_Kd/). Luego se reporta el estado de movimiento global (Vx, Vy, Wz) y el estado del mantenimiento de rumbo (heading actual, corrección de Wz aplicada, y si el hold está activo).

Decisión de diseño deliberada — línea corta: en una versión anterior la línea de reporte era más larga (incluía además las ganancias del PID en cada envío). Al integrar la IMU, esa longitud adicional alargó el tiempo bloqueante de transmisión UART lo suficiente para contribuir al problema de jitter descrito en la sección de PID. Se acortó el reporte a lo estrictamente necesario para operación en vivo; las ganancias ahora solo se confirman por UART cuando se cambian, no en cada ciclo de reporte.

## 5. Ruta de la telemetría hacia el gemelo digital
PIC (UART) → XBee Esclavo → XBee Maestro → ESP8266 (UART→WebSocket) → Navegador

En el navegador, la interfaz web parsea esta línea y actualiza en tiempo real: la rotación visual de las 4 ruedas del modelo URDF (a partir de RPM medida) y, cuando se añada el panel correspondiente, el estado de rumbo. Esta es la implementación concreta del “Gemelo Digital” descrito en el README del repositorio: no es una animación simulada, es la reproducción visual de datos físicos reales reportados por el PIC.