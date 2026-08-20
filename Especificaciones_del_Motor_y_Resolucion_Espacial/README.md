# Modelado Matemático: Especificaciones del Motor y Resolución Espacial

Para traducir una orden de software (ej. "Avanza 50 cm") en movimientos físicos reales, construimos un modelo matemático vinculando las especificaciones electromecánicas del motor GM37-520 con la geometría de la llanta Mecanum.

A continuación, se detalla la caracterización teórica y aplicada:

A. Análisis de la Transmisión Electromecánica (Motor GM37-520)

La placa de características del motor nos proporcionó tres variables fundamentales que definen el rendimiento del chasis:

![Etiqueta motor GM37-520](Motor_Mecanum.jpg)

1. 12 PPR (Fase AB): El encoder magnético de cuadratura acoplado al eje interno del motor genera 12 pulsos eléctricos por cada revolución completa de dicho eje (antes de pasar por los engranajes).

2. Relación de Reducción (1:90): La caja reductora metálica exige que el motor interno gire exactamente 90 veces para que el eje de salida (donde va la llanta) complete una sola vuelta.

3. 110 RPM (a 12V): La velocidad máxima teórica del eje de salida sin carga.

Cálculo de Resolución del Encoder (Pulsos por Vuelta):

Para saber cuántos pulsos lee nuestro microcontrolador por cada giro completo de la llanta, multiplicamos la resolución base por la caja reductora:

Pulsos Totales = 12 PPR x 90(Reducción) = 1080 pulsos por vuelta.

Este valor teórico de 1080 fue validado empíricamente en laboratorio usando lectura manual, obteniendo un margen de error mecánico por backlash inferior al 0.4%.
