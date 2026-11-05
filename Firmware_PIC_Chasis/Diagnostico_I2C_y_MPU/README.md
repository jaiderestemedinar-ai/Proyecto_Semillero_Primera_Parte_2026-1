# Herramientas de Diagnóstico — I2C y MPU9250

Firmwares independientes del control del chasis, usados exclusivamente para validar el hardware de la IMU **antes** de integrarla al firmware de producción. Metodología completa en `MPU_9250/`.

| Archivo | Qué hace |
|---|---|
| `paso1_escaner_i2c.c` | Escáner I2C mínimo y genérico (no específico de la MPU9250): recorre las direcciones 0x08–0x77 buscando ACK. Permitió diagnosticar el bus I2C desde cero, encontrando que la ausencia de resistencias pull-up producía 56 "dispositivos" falsos — resuelto al añadir pull-ups de 4.7 kΩ. |
| `capa4a_mpu9250_diagnostico.c` | Firmware de prueba exclusivo de la MPU9250 (sin control de motores): calibración de sesgo del giroscopio, integración de heading, comandos `C` (recalibrar) / `Z` (poner rumbo en 0) / `N` (invertir signo). Usado para validar WHO_AM_I, estabilidad del sesgo y precisión de rotaciones de 90°/360° antes de integrar la IMU al firmware de control del chasis (Capa 4). |

Este orden (diagnosticar el sensor de forma aislada antes de integrarlo al lazo de control) es deliberado: cualquier error de hardware o de driver I2C se habría confundido fácilmente con un error de sintonización del PID si se hubiera integrado directamente.
