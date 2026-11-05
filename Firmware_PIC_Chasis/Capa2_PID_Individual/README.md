# Capa 2 — PID Individual por Rueda

Primer firmware con control de velocidad en lazo cerrado real, aplicado a **un motor a la vez** (no los 4 simultáneos todavía — eso llega en la Capa 3).

- Control PID + feedforward sobre el error de RPM (ver el diseño matemático completo en `PID_Caracterizacion_constantes/`).
- Ganancias ajustables en vivo por UART, sin reprogramar: `P<n>` / `I<n>` / `D<n>` (valor × 100, ej. `I30` = Ki 0.30).
- Permitió validar el controlador y las curvas de feedforward motor por motor antes de arriesgarse a coordinar los 4 al mismo tiempo con cinemática Mecanum.
