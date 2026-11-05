# Caracterización, Control PID y Mantenimiento de Rumbo — Chasis Mecanum

## 1. Hardware del chasis

| Componente | Especificación |
|---|---|
| Motores | 4× GM37-520, 12V, reducción 1:90, encoder en cuadratura 12 PPR |
| Driver | 4× BTS7960 (puente H), habilitados mediante un único pin `EN_GLOBAL` |
| Encoders | Cuadratura (canal A por interrupción, canal B para el signo) |
| Controlador | PIC18F45K22 @ 64 MHz (PLL activo) |
| Batería | CNHL 3S 5200mAh 90C |
| IMU | MPU9250 (I2C, solo eje Z del giroscopio utilizado) |

---

## 2. Caracterización experimental del encoder

### 2.1 Metodología

Se optó por una calibración **empírica**, no teórica, para evitar errores de interpretación de los registros del temporizador del PIC. El procedimiento:

1. Se aplicó un duty cycle fijo y conocido a un motor.
2. Se contaron los pulsos acumulados del encoder en una ventana de tiempo exacta (10, 20 y 30 segundos, cronometrados).
3. Simultáneamente, se contaron las vueltas físicas reales de la rueda (marca de referencia + cronómetro, validado con múltiples duraciones para reducir el error relativo de conteo manual).

### 2.2 Resultado

Pulsos por vuelta obtenidos (promedio de las 4 ruedas): **≈ 1090 pulsos/vuelta**

Validación cruzada: a duty = 90% (cercano al máximo), la velocidad medida fue de ≈ 108–110 RPM, consistente con la especificación nominal del fabricante (110 RPM @ 12V), confirmando la validez de la constante.

---

## 3. Cinemática inversa Mecanum

Con los ejes definidos como `Vx` (lateral, derecha positivo), `Vy` (longitudinal, adelante positivo) y `Wz` (rotación, horario positivo), la velocidad objetivo de cada rueda se obtiene de:

```
M1 (delantera izq.)  =  Vy + Vx + Wz
M2 (delantera der.)  =  Vy − Vx − Wz
M3 (trasera izq.)    =  Vy − Vx + Wz
M4 (trasera der.)    =  Vy + Vx − Wz
```

El signo del resultado define la dirección (adelante/atrás) de cada rueda; la magnitud es el setpoint de velocidad en RPM.

---

## 4. Feedforward por rueda y por dirección

### 4.1 Justificación

Un barrido de duty cycle (30–90%) mostró que, aunque los 4 motores son nominalmente idénticos, existe una variación real de hasta ~8% en RPM entre ruedas a la misma duty — y una asimetría adicional entre la dirección de avance y la de reversa en algunos motores, atribuible a diferencias de resistencia interna entre ambas mitades del puente H. Esto se resolvió **no** ajustando el hardware, sino incorporando esa variación directamente al modelo de control: cada rueda, en cada dirección, tiene su propia recta de feedforward.

### 4.2 Modelo

Se confirmó una relación lineal entre duty cycle y RPM en el rango de operación (30–90%), por lo que se ajustó una recta por mínimos cuadrados para cada rueda y dirección:

```
RPM = m · duty + c        =>        duty_necesario = (RPM_deseado − c) / m
```

### 4.3 Coeficientes finales

| Rueda | m (adelante) | c (adelante) | m (reversa) | c (reversa) |
|---|---|---|---|---|
| M1 | 1.226 | −2.54 | 1.140 | −1.00 |
| M2 | 1.208 | −1.12 | 1.220 | +2.05 |
| M3 | 1.290 | −4.10 | 1.279 | −4.20 |
| M4 | 1.209 | −1.61 | 1.284 | −3.50 |

Este feedforward entrega al PID un punto de partida cercano al correcto, reduciendo el esfuerzo de corrección necesario y, con ello, el tiempo de asentamiento.

---

## 5. PID de velocidad por rueda

### 5.1 Diseño

Lazo cerrado de velocidad (RPM) por rueda, con salida = feedforward + corrección PID:

```
error = setpoint_RPM − RPM_medida
salida_duty = duty_feedforward + (Kp·error + Ki·∫error·dt + Kd·d(error)/dt)
```

### 5.2 Anti-windup

El término integral solo se acumula si la salida no está saturada, o si el error la empuja a salir de la saturación — evita que el integrador "se cargue" de más durante arranques o paradas bruscas.

### 5.3 Ganancias finales

Sintonizadas experimentalmente (método manual: P hasta el borde de oscilación, I para eliminar error permanente, D solo para amortiguar sobre-impulso):

```
Kp = 0.20     Ki = 0.40     Kd = 0.02
```

Compartidas entre las 4 ruedas — justificación: la variación entre motores ya es absorbida por el feedforward individualizado (sección 4), por lo que el PID solo corrige el error residual común a la dinámica del sistema (mismo modelo de motor, misma reducción, mismo tipo de encoder).

### 5.4 Caso de depuración relevante: error de temporización

Durante la integración del giroscopio, el bucle principal comenzó a realizar transacciones I2C y transmisiones UART más largas, lo que introdujo una variación real en la duración de cada ciclo de control. El cálculo de RPM asumía un período fijo de 50 ms, lo que producía una medición incorrecta cuando el ciclo real se alargaba — visible como una oscilación sincronizada y violenta en las 4 ruedas simultáneamente.

**Corrección:** se reemplazó el período asumido por el tiempo **real** transcurrido, medido con un contador de ticks de hardware (base de tiempo de 10 ms), igual que ya se hacía para la integración del rumbo. Esto eliminó por completo la oscilación, y es consistente con el principio de no asumir temporización fija cuando el tiempo de ejecución del bucle puede variar.

---

## 6. Diagnóstico mecánico: caso real de deriva en movimiento lateral

Durante las pruebas de movimiento lateral puro (`Vx ≠ 0`, `Vy = 0`), el chasis presentaba una deriva sistemática hacia adelante o atrás pese a que las 4 ruedas seguían su setpoint de RPM con precisión (confirmado por telemetría).

**Metodología de aislamiento del problema:**
1. Se descartó el control (el error de RPM por rueda era mínimo y consistente).
2. Se sospechó inicialmente de una asimetría de tracción por la distribución de peso del brazo robótico.
3. Mediante inspección física directa (prueba de contacto con tira de papel bajo cada rueda), se identificó que una de las ruedas no apoyaba completamente en el piso.
4. **Causa raíz:** una arandela de más en el ensamblaje de la placa superior (donde se ancla el brazo) generaba una tensión mecánica que levantaba ligeramente un lateral del chasis, reduciendo el contacto de esa rueda con el piso.

Este hallazgo refuerza un principio metodológico aplicado durante todo el proyecto: **antes de ajustar ganancias de control, se descarta primero la causa mecánica**, verificando con pruebas físicas directas en lugar de iterar a ciegas sobre el software.

---

## 7. Mantenimiento de rumbo con MPU9250

### 7.1 Justificación

Incluso con las 4 ruedas siguiendo su setpoint de RPM con precisión y sin fallas mecánicas, persiste una deriva de orientación mínima e inevitable (tolerancias de fabricación, micro-deslizamiento) que no es detectable por los encoders, ya que estos solo miden la rotación del eje del motor, no el desplazamiento real del chasis sobre el piso. Se incorporó el eje Z del giroscopio del MPU9250 para cerrar un lazo de control de orientación (yaw) en tiempo real.

### 7.2 Comunicación I2C

Driver I2C implementado directamente sobre el módulo MSSP1 del PIC18F45K22 (maestro, 100 kHz), con control de timeout en cada espera de bus para evitar bloqueos indefinidos ante un fallo de comunicación.

### 7.3 Calibración del sesgo (bias)

El giroscopio reporta un offset (bias) distinto de cero incluso en reposo. Se promedian 300 muestras con el sensor quieto; si la dispersión de esas muestras supera un umbral (~3°/s pico a pico), se descarta la calibración y se reintenta, para evitar calibrar con el sensor en movimiento.

### 7.4 Validación del sensor

| Prueba | Resultado |
|---|---|
| Deriva en reposo (26 s) | < 0.2° |
| Rotación controlada de 90° | +90.0° medido |
| Rotación controlada de 360° | +357.6° medido (diferencia atribuible a imprecisión de la rotación manual, no al sensor) |

### 7.5 PID de corrección de rumbo

Lazo de control completo (P + I + D) sobre el error de orientación, cuya salida se suma como corrección adicional al término `Wz` de la cinemática:

```
error_rumbo = rumbo_referencia − rumbo_actual
corrección = Kh·error + Kih·∫error·dt − Khd·(velocidad_angular_medida)
Wz_efectivo = Wz_comandado + corrección
```

- **Derivativo implementado sobre la tasa medida directamente** (no derivando el error), evitando amplificar ruido de muestra a muestra.
- **Anti-windup** con el mismo criterio que el PID de las ruedas (no acumula integral si la salida está saturada en la dirección del error).
- **Límite de seguridad:** si el error de rumbo supera 30°, el sistema desactiva automáticamente la corrección (posible fallo de signo o evento anómalo) en vez de intentar corregir un error fuera de rango esperado.
- El lazo solo actúa durante traslación pura (sin rotación comandada) y por encima de una velocidad mínima, para no interferir con giros intencionales.

### 7.6 Ganancias finales

```
Kh  = 0.80   (P)
Ki_h = 0.03  (I)
Khd = 0.05   (D)
```

### 7.7 Resultados de validación en campo

- **Prueba de línea recta/lateral con referencia visual:** desviación lateral prácticamente imperceptible a simple vista (solo apreciable mirando fijamente desde arriba con una referencia de línea fija).
- **Respuesta a perturbación manual:** ante una perturbación externa aplicada a mitad de movimiento, el sistema detecta el error de rumbo introducido y lo corrige de forma autónoma, retornando al rumbo de referencia sin intervención adicional.
- **Comportamiento del término integral:** transitorio único con sobre-impulso pequeño, seguido de asentamiento estable cercano a 0° — sin oscilación sostenida, indicando una sintonización apropiada para la dinámica del sistema.

---

## 8. Conclusiones de esta etapa

El sistema de tracción Mecanum logra:
1. Seguimiento preciso de velocidad individual por rueda (feedforward + PID con anti-windup).
2. Ejecución correcta de la cinemática completa (avance, lateral, diagonal, rotación).
3. Corrección activa de deriva de orientación mediante realimentación inercial, validada cuantitativamente (no solo observación cualitativa).

Esto constituye la base de control de bajo nivel sobre la cual se implementará la odometría y el control de posición (sección pendiente: `Odometria_y_Control_de_Posicion/`).