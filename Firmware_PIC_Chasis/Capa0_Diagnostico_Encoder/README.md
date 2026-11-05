# Capa 0 — Diagnóstico de Motor/Encoder (lazo abierto)

Primer firmware del proyecto. Objetivo único: permitir medir el periodo crudo del encoder de **un** motor a la vez, con PWM fijo conocido, para empezar la caracterización empírica de la constante de pulsos por vuelta (ver `Especificaciones_del_Motor_y_Resolucion_Espacial/`).

- Sin PID, sin cinemática — lazo completamente abierto.
- Captura del periodo entre pulsos vía módulos CCP1–CCP4, usando Timer1 como base de tiempo.
- Protocolo: `1`/`2`/`3`/`4` selecciona el motor activo, `S` lo detiene, `<numero>` + Enter fija el duty (0–100) del motor seleccionado.
- Reporte automático cada 200 ms: `M<sel> duty=<pwm> per=<periodo_crudo> pulsos=<contador>`.

**Limitación encontrada y resuelta en la siguiente capa:** este protocolo usa dígitos tanto para seleccionar motor como para el valor de duty, lo que puede generar ambigüedad al interpretar comandos — corregido en `Capa0_v2_Cuadratura_BTS7960/`.
