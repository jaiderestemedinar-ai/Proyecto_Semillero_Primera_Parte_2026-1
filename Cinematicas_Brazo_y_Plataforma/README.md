# Cinemática Inversa (El Cerebro del Chasis Mecanum)

En robótica móvil, la Cinemática Directa consiste en medir a qué velocidad gira cada llanta para adivinar hacia dónde se mueve el chasis. Nosotros hicimos lo contrario: aplicamos Cinemática Inversa. Esto significa que le dictamos al robot hacia dónde queremos que vaya en el espacio (los vectores globales) y el algoritmo calcula matemáticamente a qué velocidad exacta debe girar cada motor individual para lograrlo.

Para un chasis omnidireccional con 4 ruedas Mecanum, el movimiento total del robot se define mediante tres vectores principales:

1. **Vy** (Vector Frontal): Desplazamiento longitudinal (Adelante / Atrás).

2. **Vx** (Vector Lateral): Desplazamiento transversal (Izquierda / Derecha - Modo Cangrejo).

3. **W**  (Velocidad Angular): Rotación sobre su propio centro geométrico (Giro horario / antihorario)

## A. La Matriz Matemática (El Código)

Para traducir estos tres vectores en velocidades individuales (Setpoints) para cada uno de los 4 PIDs, implementamos el siguiente modelo de ecuaciones lineales en el firmware:

```
// SP = Setpoint (Velocidad objetivo para cada motor)
SP1 = Vy + Vx + W;   // Motor 1: Delantero Izquierdo
SP2 = Vy - Vx - W;   // Motor 2: Delantero Derecho
SP3 = Vy - Vx + W;   // Motor 3: Trasero Izquierdo
SP4 = Vy + Vx - W;   // Motor 4: Trasero Derecho
```

## B. Análisis Vectorial: ¿Cómo funciona la física de las llantas?

Las llantas Mecanum tienen rodillos pasivos inclinados a 45 grados. Cuando se instalan correctamente (formando una "X" vistos desde arriba), estos rodillos descomponen la fuerza del motor en dos vectores: uno longitudinal (hacia adelante/atrás) y uno transversal (hacia los lados).

Nuestra matriz cinemática explota esta física operando de tres maneras distintas:

1. **Desplazamiento Lineal (Vy ≠ 0, Vx = 0,W = 0 )** 

Si pedimos Vy = 35, las cuatro ecuaciones dan como resultado SP = 35.

**Física:** Los 4 motores giran hacia adelante a la misma velocidad. Las fuerzas transversales de los rodillos se anulan entre sí (las izquierdas empujan hacia el centro, las derechas empujan hacia el centro) y toda la energía se convierte en empuje frontal puro.


2. **Desplazamiento Lateral - Modo Cangrejo  (Vy = 0, Vx ≠ 0,W = 0 )** 

Si pedimos Vx = 35 (Moverse a la derecha), la matriz arroja:

SP1 = +35 (Avanza)

SP2 = -35 (Retrocede)

SP3 = -35 (Retrocede)

SP4 = +35 (Avanza)

**Física:** Al girar M1 y M4 hacia adelante, y M2 y M3 hacia atrás, las fuerzas longitudinales se anulan matemáticamente. El chasis no puede ir ni adelante ni atrás. Sin embargo, debido al ángulo de 45° de los rodillos, todas las fuerzas transversales se suman en la misma dirección, logrando que el robot se deslice lateralmente de forma mágica.


3. **Rotación Pura (Vy = 0, Vx = 0, W ≠ 0)**

Si pedimos W = 35 (Giro sobre su eje), la matriz calcula:

SP1 = +35 (Avanza)

SP3 = +35 (Avanza)

SP2 = -35 (Retrocede)

SP4 = -35 (Retrocede)

**Física:** El robot se comporta como un tanque. Las llantas del lado izquierdo traccionan hacia adelante y las del lado derecho traccionan hacia atrás. Al estar a la misma velocidad, el chasis no se desplaza de su coordenada, sino que gira perfectamente sobre su propio eje.

## C. Movimientos Combinados (Omnidireccionalidad Real)

El verdadero poder de esta matriz inversa radica en su capacidad de superposición. Si inyectamos valores simultáneos, por ejemplo, Vy = 30 y Vx = 30, la matriz calculará automáticamente velocidades asimétricas (SP1 = 60, SP2 = 0, SP3 = 0, SP4 = 60). Esto frenará la diagonal derecha y acelerará la diagonal izquierda, logrando que el chasis se mueva en un ángulo diagonal exacto de 45 grados.