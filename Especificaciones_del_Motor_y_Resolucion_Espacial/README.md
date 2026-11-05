# Modelado Matemático: Especificaciones del Motor y Resolución Espacial

Para traducir una orden de software (ej. "avanza 50 cm") en movimientos físicos reales, es necesario vincular las especificaciones electromecánicas del motor GM37-520 con la geometría de la llanta Mecanum y la resolución real del encoder. Este documento detalla la caracterización teórica inicial, por qué se abandonó, y la caracterización empírica aplicada que finalmente se utilizó.

---

## 1. Datos de partida (datasheet)

| Parámetro | Valor |
|---|---|
| Relación de reducción | 1:90 |
| Pulsos por revolución del encoder (eje del motor, antes de la reducción) | 12 PPR |
| Velocidad nominal a 12V | 110 RPM (medida en el eje de salida, después de la reducción) |

**Pulsos por vuelta teóricos (eje de salida):** `12 PPR × 90 (reducción) = 1080 pulsos/vuelta`

Este número teórico depende de que el encoder esté efectivamente ubicado antes de la reducción (en el eje del motor) y de que cada pulso del canal A se cuente una sola vez por flanco. Es un valor de referencia, **no la constante final utilizada** — ver la sección 2 para entender por qué.

---

## 2. Por qué se descartó el cálculo puramente teórico de la conversión pulso→RPM

En una primera aproximación, se intentó calcular la constante de conversión de "período entre pulsos" a "RPM" a partir de la configuración de los registros del temporizador del PIC (prescaler de Timer1, frecuencia de reloj del sistema). Este método **se abandonó deliberadamente** tras cometer dos errores de cálculo consecutivos al interpretar manualmente los bits de configuración del temporizador (`T1CON`) sin verificarlos contra el datasheet con suficiente rigor — cada intento produjo una constante distinta y ninguna fue confiable sin medición real.

**Decisión metodológica:** en lugar de confiar en una deducción teórica propensa a errores de interpretación de registros, se adoptó una **calibración completamente empírica**, descrita a continuación. Este es un ejemplo deliberado de buena práctica de ingeniería: cuando un cálculo teórico depende de múltiples supuestos no verificados, es preferible medir directamente.

---

## 3. Metodología de calibración empírica

### 3.1 Procedimiento

1. Se aplicó al motor un duty cycle fijo y conocido, en lazo abierto (sin control de velocidad).
2. Se contaron los pulsos acumulados del encoder durante una ventana de tiempo exacta, cronometrada externamente (10, 20 y 30 segundos, en pruebas independientes, para reducir el error relativo de medición).
3. De forma simultánea, se marcó físicamente la rueda y se contaron las vueltas reales completadas en la misma ventana de tiempo (conteo manual, contrastado en las tres duraciones para minimizar el error de fracciones de vuelta mal contadas).
4. La constante se obtiene de: `pulsos_por_vuelta = pulsos_contados / vueltas_contadas`

### 3.2 Datos obtenidos (duty = 50%, ventana de 10 segundos)

| Rueda | Pulsos contados | Vueltas contadas | Pulsos/vuelta |
|---|---|---|---|
| M1 | 10890 | ≈9.95 | ≈1094 |
| M2 | 11608 | ≈10.9 | ≈1065 |
| M3 | 11139 | 10 | ≈1114 |
| M4 | 10940 | 10 | ≈1094 |

### 3.3 Resultado final

**Promedio adoptado: 1090 pulsos/vuelta**, aplicado como constante única para las 4 ruedas. La dispersión observada entre ruedas (≈1065 a ≈1114) se atribuye principalmente a la imprecisión inherente al conteo manual de vueltas fraccionarias, no a una diferencia mecánica real entre encoders — por lo que no se justificaba usar una constante distinta por rueda para este parámetro (a diferencia de las curvas de velocidad, donde sí existen diferencias reales entre motores, ver `Caracterizacion_de_las_Constantes_PID_Kp_Ki_Kd/`).

Es notable que el valor empírico (1090) resulta muy cercano al valor teórico calculado en la sección 1 (1080) — una diferencia de apenas ~0.9%, lo que sirve como validación cruzada de que ambos números son razonables y consistentes entre sí, pese a que el método final adoptado fue el empírico.

---

## 4. Validación cruzada contra la especificación del fabricante

Con la constante de 1090 pulsos/vuelta aplicada, se calculó la RPM real a distintos niveles de duty cycle:

| Duty (%) | M1 (RPM) | M2 (RPM) | M3 (RPM) | M4 (RPM) |
|---|---|---|---|---|
| 50 | 59.9 | 63.9 | 61.3 | 60.2 |
| 70 | 83.8 | 88.0 | 86.2 | 86.1 |
| 90 | 108.2 | 115.4 | 111.4 | 110.8 |

A duty = 90% (cercano al máximo de operación), la velocidad medida (~108–115 RPM) es consistente con la especificación nominal del fabricante de **110 RPM a 12V**, confirmando que tanto la constante de conversión como el procedimiento de medición son válidos.

---

## 5. Encoder de cuadratura: resolución de dirección real

A diferencia de un encoder de un solo canal (que solo permite contar pulsos, infiriendo la dirección del comando enviado), el encoder utilizado dispone de **2 canales (A/B)**, lo que permite:

- Canal A: genera la interrupción de conteo (por cambio de estado).
- Canal B: se lee en el instante del flanco de A para determinar el signo (sentido de giro real).

Esto permitió, durante la puesta en marcha, **detectar por hardware** qué motores giraban en sentido contrario al comandado (M2 y M4, documentado en `Caracterizacion_de_las_Constantes_PID_Kp_Ki_Kd/`), en vez de depender de la observación visual para cada prueba.

---

## 6. Uso de esta constante en el resto del sistema

El valor de 1090 pulsos/vuelta es la base de:
- El cálculo de RPM en tiempo real dentro del lazo de control PID (`Caracterizacion_de_las_Constantes_PID_Kp_Ki_Kd/`).
- La futura conversión de pulsos acumulados a distancia lineal recorrida, necesaria para la odometría (`Odometria_y_Control_de_Posicion/`), donde se calibrará además el factor pulsos→cm de forma independiente para desplazamiento lateral y frontal, dado que la geometría de rodadura de una rueda Mecanum no es idéntica en ambos ejes.