# Capa 4 — Firmware Final del Chasis (Rumbo con MPU9250)

**Este es el firmware que corre actualmente en el PIC del chasis.** Es la Capa 3 (`capa3_ruedas_individuales.c`) con la integración completa de la MPU9250 para mantenimiento activo de rumbo, y con compatibilidad dual de protocolo de comandos.

## Qué añade sobre la Capa 3

- Lectura del giroscopio Z (MPU9250 por I2C) y mantenimiento de rumbo: PID completo con anti-windup (`Kh`, `Khd`, `Kih`) — diseño matemático completo en `PID_Caracterizacion_constantes/` y `MPU_9250/`.
- El hold de rumbo arranca **apagado** por defecto (`J0`); se activa con `J1`. Si la IMU no responde en el arranque, el chasis se comporta exactamente igual que la Capa 3 (degradación segura, no un fallo silencioso).
- Corrección del bug de tiempo de ciclo fijo que apareció al integrar la IMU (ver `PID_Caracterizacion_constantes/`, sección 5): el cálculo de RPM pasó de asumir un `dt` fijo de 50 ms a medir el tiempo real transcurrido con un contador de hardware.
- Compatibilidad con el **Modo Manual** heredado de la interfaz web original (`F`/`B`/`L`/`R`/`1`/`2`/`S`), coexistiendo con el protocolo numérico nuevo — ver la resolución completa de colisiones de letra y de ambigüedad de dígitos en `Interconexion_del_sistema/`.

## Protocolo de comandos completo

Documentado íntegramente en el encabezado del propio archivo `capa4_rumbo_mpu9250.c`. Resumen:

| Comando | Función |
|---|---|
| `Y<rpm>` / `X<rpm>` / `W<rpm>` | Vy / Vx / Wz, protocolo nuevo |
| `DI`/`DD`/`TI`/`TD<rpm>` | Rueda individual (diagnóstico) |
| `P`/`I`/`D<n>` | Ganancias PID de rueda |
| `G`/`K`/`Q<n>` | Ganancias PID de rumbo (P/D/I) |
| `J1`/`J0` | Mantenimiento de rumbo on/off |
| `C` / `Z` / `N` | Calibrar IMU / zero rumbo / invertir signo |
| `F`/`B`/`L`/`R`/`1`/`2`/`S` | Modo Manual (compatibilidad interfaz web original) |

## Hardware

MPU9250: `SCL` → RC3 (pin 18), `SDA` → RC4 (pin 23), `AD0` → GND. Resto del hardware sin cambios respecto a la Capa 3 — ver `Especificaciones_Iniciales_Hardware/`.
