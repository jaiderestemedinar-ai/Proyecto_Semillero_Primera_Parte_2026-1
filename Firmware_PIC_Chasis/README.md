# Firmware PIC — Chasis Mecanum

Código fuente completo del PIC18F45K22 de tracción, mostrado en su **evolución real**, de principio a fin. No se muestra solo la versión final: se conserva cada capa intermedia porque cada una resolvió un problema concreto y documentado, y en conjunto son evidencia directa de la metodología empírica seguida en este proyecto (ver `Especificaciones_del_Motor_y_Resolucion_Espacial/` y `PID_Caracterizacion_constantes/` para el detalle matemático de cada decisión).

## Orden de evolución

| Capa | Carpeta | Propósito | Qué resolvió / qué añadió |
|---|---|---|---|
| 0 | `Capa0_Diagnostico_Encoder/` | Diagnóstico de un solo motor, lazo abierto | Primera lectura cruda de encoder (periodo entre pulsos vía CCP), sin PID ni cinemática. Protocolo mínimo: seleccionar motor, fijar duty, leer periodo. |
| 0.v2 | `Capa0_v2_Cuadratura_BTS7960/` | Diagnóstico de un solo motor, cuadratura real | Añade decodificación en cuadratura (sentido de giro real, no asumido), pin `EN_GLOBAL` de seguridad, flags `INVERT_M2`/`INVERT_M4`, y corrige el bug de ambigüedad de dígitos del protocolo original (ver más abajo). |
| 2 | `Capa2_PID_Individual/` | Control de velocidad, un motor a la vez | Primer PID+feedforward funcional por rueda, con ganancias ajustables en vivo por UART (`P`/`I`/`D`). |
| 3 | `Capa3_Cinematica_Mecanum/` | Las 4 ruedas simultáneas | Cinemática inversa Mecanum completa (`Cinematicas_Brazo_y_Plataforma/Cinematica_Inversa_mecanum/`) + PID por rueda corriendo en paralelo. Incluye 3 variantes: la base (`capa3_cinematica_mecanum.c`), la de ganancias finales ya sintonizadas (`capa3_final_estable.c`), y la que añade comandos de diagnóstico de rueda individual (`capa3_ruedas_individuales.c`) usada para encontrar la deriva lateral. |
| — | `Diagnostico_I2C_y_MPU/` | Herramientas de diagnóstico de la IMU | Escáner I2C genérico (`paso1_escaner_i2c.c`) y firmware de prueba exclusivo de la MPU9250 (`capa4a_mpu9250_diagnostico.c`), usados **antes** de tocar el firmware de control para validar el hardware de forma aislada. |
| 4 | `Capa4_Rumbo_MPU9250/` | **Firmware final** | Capa 3 + mantenimiento de rumbo con la MPU9250 (PID con anti-windup) + comandos de rueda individual + compatibilidad dual de protocolo (nuevo protocolo numérico + Modo Manual heredado de la interfaz web original). Es el que se sube al PIC en la configuración actual del robot. |

## Por qué se conservan todas las capas y no solo la final

Cada capa resolvió un problema real y aislado, y mostrar solo el resultado final ocultaría el proceso de ingeniería que llevó hasta ahí:

- **Capa 0 → Capa 0.v2**: se corrigió un bug de protocolo real (los dígitos `1`–`4` se usaban a la vez para seleccionar motor y como valor de duty, causando lecturas erróneas como "45" interpretado como "motor 4, duty 5"). La solución (prefijo `M` + dígito) es la misma técnica de desambiguación que después se reaplicó para resolver la convivencia de protocolos en la Capa 4 (ver `Interconexion_del_sistema/`).
- **Capa 2 → Capa 3**: paso de control de un motor aislado a los 4 motores coordinados por cinemática Mecanum.
- **Capa 3 → Capa 3 (ruedas individuales)**: se añadieron comandos de diagnóstico por rueda específicamente para diagnosticar la deriva lateral del chasis (ver `Especificaciones_Iniciales_Hardware/`), que terminó siendo un problema mecánico, no de software — pero no se habría podido descartar el software sin esta herramienta.
- **Capa 3 → Capa 4**: se integró la IMU y apareció (y se corrigió) el bug del tiempo de ciclo fijo documentado en detalle en `PID_Caracterizacion_constantes/` (sección 5).

## Protocolo de comandos

El protocolo completo de la Capa 4 (el firmware final) está documentado en el encabezado del propio archivo `capa4_rumbo_mpu9250.c` y en `Interconexion_del_sistema/`, que explica además por qué coexisten dos protocolos (el nuevo, numérico, y el heredado de un solo carácter para el Modo Manual de la interfaz web).

## Hardware asociado

Ver el detalle completo de motores, encoders, puentes H y pines en `Especificaciones_Iniciales_Hardware/`.
