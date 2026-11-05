# Capa 0.v2 — Diagnóstico con Cuadratura Real y Protección BTS7960

Evolución directa de la Capa 0, con tres mejoras concretas encontradas durante las pruebas físicas:

1. **Decodificación en cuadratura real** (canal A + canal B), en vez de solo contar pulsos de un canal — permite conocer el sentido de giro real del motor, no el asumido a partir del comando enviado.
2. **Pin `EN_GLOBAL`**: habilitación conjunta de los 4 módulos BTS7960 (R_EN + L_EN unidos, ya que es una señal binaria de seguridad común a los 4 puentes).
3. **Flags `INVERT_M2` / `INVERT_M4`**: corrección por software del sentido de giro invertido de los motores M2 y M4 (causado por el montaje físico), sin necesidad de recablear.

## Bug corregido: ambigüedad de protocolo

En la Capa 0, los dígitos `1`–`4` se usaban simultáneamente para seleccionar el motor activo **y** podían aparecer como parte de un valor de duty, causando lecturas erróneas (p. ej. `"45"` podía interpretarse parcialmente como selección de motor en vez de como el número 45). Se corrigió introduciendo un protocolo de dos caracteres: `M` + dígito (`M1`–`M4`) para seleccionar motor, de modo que ningún dígito suelto dispare una selección por accidente.

Esta misma clase de bug —y la misma técnica de solución— reaparece y se resuelve de nuevo en la Capa 4 al reconciliar el protocolo nuevo con el Modo Manual heredado de la interfaz web (ver `Interconexion_del_sistema/`).
