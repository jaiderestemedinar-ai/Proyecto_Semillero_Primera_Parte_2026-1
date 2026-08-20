# Caracterización y Sintonización de las Constantes PID (Kp ,Ki, Kd)
En la industria, hay dos formas de obtener las constantes de un controlador PID:

## Analítica: 
Calculando la Función de Transferencia del motor mediante transformada de Laplace (requiere modelado físico avanzado).

## Heurística (Empírica):
 Mediante pruebas de lazo cerrado, observando la reacción física del sistema y ajustando las ganancias.

Para este chasis con motores GM37-520, utilizamos el método empírico estructurado (muy similar al método de Ziegler-Nichols), evaluando el comportamiento de las ruedas bajo la carga real del chasis (baterías, estructura y electrónica) con un tiempo de muestreo estricto de 50 milisegundos.

Así fue como dedujimos cada valor para llegar a nuestro arreglo base de Kp = 2.0, Ki = 0.5 y Kd = 0.2:

## Acción Proporcional (Kp = 2.0) - (se puede decir que es la potencia que pide el setpoint)

El término proporcional es la fuerza bruta instantánea. Responde a la pregunta: ¿Qué tan lejos estoy de mi objetivo ahora mismo?

La Ecuación: P = Kp x Error

El Proceso: Empezamos con Ki = 0 y Kd = 0. Le dimos al motor un Setpoint de 35 pulsos. Si poníamos un Kp muy bajo (ej. 0.5), el motor apenas recibía voltaje y no lograba vencer la inercia. Al subirlo a 2.0, el chasis lograba reaccionar rápidamente y llevar la velocidad cerca de los 35 pulsos, pero se quedaba un poco por debajo (ej. 32 o 33) debido al peso y la fricción contra el suelo. A este déficit se le llama Error en Estado Estacionario.

## Acción Integral ($K_i = 0.5$) - (se puede decir que es la memoria de el lazo)

El término integral se encarga de eliminar ese error en estado estacionario. Responde a: ¿Cuánto tiempo llevo sin alcanzar mi objetivo exacto?

La Ecuación: I = Ki x Σ Error

El Proceso: Para lograr que la llanta pasara de 33 a exactamente 35 pulsos, necesitábamos que el controlador acumulara ese pequeño error con el tiempo y empujara un poco más. Un Ki de 0.5 fue perfecto para dar ese empujón extra sin causar que el motor se descontrolara.

Problema resuelto (Anti-Windup): Nos dimos cuenta de que si una llanta se trababa físicamente, la integral acumulaba un error infinito, provocando un latigazo de PWM. Lo solucionamos introduciendo la saturación matemática: if(integral > 300) integral = 300;. Para el movimiento lateral, al ser más inestable, redujimos la ganancia a Ki = 0.05 y la saturación a 100.

Amnesia Matemática: Implementamos una regla vital. Si Setpoint == 0, la integral se borra (integral = 0). Esto garantizó el freno en seco perfecto, evitando que el robot se arrastrara tratando de "corregir" errores del pasado mientras estaba detenido.

## Acción Derivativa ($K_d = 0.2$) - (se puede decir que es la accion de "frenado" hacia futuro)

El término derivativo evalúa la tasa de cambio. Responde a: ¿A qué velocidad me estoy acercando a mi objetivo?

La Ecuación: D = Kd x Error Actual - Error Previo

El Proceso: Al tener un Kp fuerte y un Ki sumando fuerza, el motor corría el riesgo de pasarse de los 35 pulsos (Sobreimpulso u Overshoot). El Kd actúa como un amortiguador. Al detectar que el error se está reduciendo muy rápido, aplica un valor negativo al PWM para ir frenando el motor justo antes de llegar al Setpoint. Un Kd de 0.2 fue suficiente para suavizar la llegada sin volver el sistema "lento".

El Filtro Pasabajo: En las últimas versiones, aplicamos un filtro (ALPHA_FILTRO_D = 0.35f) porque a velocidades bajas, un salto de 1 solo pulso en el encoder causaba picos ruidosos en la derivada que se traducían en micro-temblores en las ruedas.