# Capa 3 — Cinemática Mecanum Completa (4 ruedas)

Primer firmware que controla las 4 ruedas simultáneamente mediante la matriz de cinemática inversa Mecanum (deducción completa en `Cinematicas_Brazo_y_Plataforma/Cinematica_Inversa_mecanum/`), cada una con su propio lazo PID + feedforward corriendo en paralelo.

Comandos: `Y<rpm>` / `X<rpm>` / `W<rpm>` para Vy/Vx/Wz, `S` para detener, `P`/`I`/`D` para las ganancias compartidas.

## Las tres variantes de esta carpeta

| Archivo | Qué es |
|---|---|
| `capa3_cinematica_mecanum.c` | Versión base: cinemática + PID funcionando con las primeras ganancias de prueba. |
| `capa3_final_estable.c` | Misma base, con las ganancias ya sintonizadas tras pruebas de movimiento real (valores finales adoptados posteriormente en la Capa 4: Kp=0.15, Ki=0.25, Kd=0.02 — ver `PID_Caracterizacion_constantes/`). |
| `capa3_ruedas_individuales.c` | Añade comandos de diagnóstico de rueda individual (`DI`/`DD`/`TI`/`TD<rpm>`), usados específicamente para diagnosticar la deriva observada durante el movimiento lateral puro — telemetría por rueda que permitió descartar el software como causa antes de encontrar el origen mecánico real (ver `Especificaciones_Iniciales_Hardware/`). |

Esta última variante es la base directa de la Capa 4: los mismos comandos de rueda individual se conservan en el firmware final.
