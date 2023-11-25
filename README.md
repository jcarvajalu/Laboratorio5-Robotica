# Laboratorio5-Robotica-2023_2
# Cinemátia inversa usando el Phantom X

* Juan Nicolás Carvajal Useche
* Edgar Giovanny Obregón Espitia
  
## Introducción
La mayoría de las aplicaciones en robótica se fundamentan en la determinación de rutas necesarias para guiar un manipulador dentro de su área de operación, definiendo diversas configuraciones mediante el conocimiento de la posición y orientación de su herramienta de trabajo. En este contexto, la resolución del problema cinemático inverso se ocupa de calcular las posiciones angulares del manipulador, facilitando así su control a través de un entorno de desarrollo software. En este laboratorio, se realizó la solución del modelo cinemático inverso para el robot PhantomXPincher de manera geométrica, mediante este modelo, se programaron una serie de trayectorias a través de MATLAB con el fin de ser dibujadas por el robot. Se mostrará el procedimiento para hallar este modelo, el procedimiento de programación mediante ROS y Python del robot, los resultados obtenidos y un análisis de estos incluyendo métricas de precisión y repetibilidad.

## Modelo cinemático inverso

Para hallar el modelo cinemático inverso, se realizó una solución geométrica

#### Articulación q1

$\theta_1$ depende solo de los valores de posición en dirección $x$ y $y$, ya que es la única articulación que actúa así, por lo que se tiene que:

```math
\theta_1=atan2(P_y,P_x)
```

#### Desacople de la muñeca

Con el objetivo de reducir la cantidad de variables, se realiza un desacoplamiento de la muñeca de la última articulación. En consecuencia, la posición del Punto de Control de la Herramienta (TCP) se desplaza una distancia de $-L_4$ en la dirección del vector de aproximación. Finalmente, la posición de la muñeca se calcula mediante la siguiente fórmula:

```math
w=
\begin{bmatrix}
    x_T\\
    y_T\\
    z_T
\end{bmatrix}
-L_4
\begin{bmatrix}
    a_x\\
    a_y\\
    a_z
\end{bmatrix}
```

donde $a_x$, $a_y$ y $a_z$ son las componentes del vector de aproximación en el marco de referencia del eslabón base, y estas componentes pueden obtenerse a partir de la matriz de rotación del TCP.

#### Mecanismo 2R
Con todas estas simplificaciones realizadas, el problema de la cinemática inversa del robot se reduce a un mecanismo 2R. Este mecanismo tiene dos soluciones posibles: con el codo hacia arriba y con el codo hacia abajo.

<p align="center">
  <img width="460" height="300" src="Imagenes/CI.png">
</p>

```math
\begin{gather*}
    r = \sqrt{x_w^2+y_w^2}\\
    h = z_w-L_1\\
    \\
    c = \sqrt{r^2+h^2}\\
    \\
    \beta = \arctan2{(L_m,L_2)}\\
    \psi = \frac{\pi}{2}-\beta\\
    L_r = \sqrt{L_m^2+L_2^2}\\
    \\
    \phi = \arccos{\frac{c^2-L_3^2-L_r^2}{-2L_rL_3}}\\
    \\
    \gamma = \arctan2{(h,r)}\\
    \alpha =  \arccos{\frac{L_3^2-L_r^2-c^2}{-2L_rc}}
\end{gather*}
```
#### q2 y q3 (Hombro y codo)
Para definir las soluciones con el codo hacia arriba y hacia abajo, se utilizan las siguientes ecuaciones:

Por convenciencia, para este laboratorio se selecciona la solución codo arriba del modelo, entonces:

```math
    \mathbf{q_2}  = \frac{\pi}{2}-\beta-\alpha-\gamma \\
    \mathbf{q_3} =           \pi-\psi-\phi  
```
Desarrollando estas ecuaciones se tendría que:

```math
\begin{align*}
    \theta_2 &= -2 \arctan\left(\frac{2D_rL_2 - \sqrt{-D_r^4 - 2D_r^2D_z^2 + 2D_r^2L_2^2 + 2D_r^2L_3^2 - D_z^4 + 2D_z^2L_2^2 + 2D_z^2L_3^2 - L_2^4 + 2L_2^2L_3^2 - L_3^4}}{D_r^2 + D_z^2 + 2D_zL_2 + L_2^2 - L_3^2}\right)\\
    \theta_3 &= -2 \arctan\left(\frac{\sqrt{(-D_r^2 - D_z^2 + L_2^2 + 2L_2L_3 + L_3^2)(D_r^2 + D_z^2 - L_2^2 + 2L_2L_3 - L_3^2)} - 2L_2L_3}{D_r^2 + D_z^2 - L_2^2 - L_3^2}\right)
\end{align*}
```

Donde:

```math
\begin{align}
    D_r=\sqrt{P_x^2+P_y^2}-L_4 cos(\beta): Distancia r (radial de los dos eslabones)\\
    D_z=P_z-L_4 sin(\beta)-L_1: Distancia en Z de los dos eslabones\\
\end{align}
```

#### Articulación de la muñeca (q4)

Una vez que se han definido los ángulos de la cintura, hombro y codo, se vuelve a conectar la muñeca y se define el ángulo de la muñeca de la siguiente manera:

```math
\begin{gather*}
    \theta_a=\arctan2{\left(\sqrt{x_a^2+y_a^2},z_a\right)}\\
    q_4=\theta_a-q_2-q_3-\frac{\pi}{2}
\end{gather*}
```
Donde esta parametrización está determinada por los valores de las dos articulaciones anteriores, y el ángulo de ataque deseado de la articulación final. Este modelo se programó en Matlab para hallar los valores de las articulaciónes para diferentes trayectorias, esto se puede ver en el código anexo.


## Portaherramienta y gripper cyberpunk

Con el fin de fijar la posición del marcador en el espacio de trabajo para su carga y descarga, y para asegurar el agarre en el gripper cuando dibuje, se diseñaron dos piezas, una para sostener la herramienta con el gripper, y otra para sostener el marcador en el tablero. A continuación se muestran imágenes de este par de herramientas.

<p align="center">
  <img width="460" height="300" src="Imagenes/portaherramienta.png">
</p>

<p align="center">
  <img width="460" height="300" src="Imagenes/gripper.png">
</p>

## Rutinas programadas

Las rutinas programadas para dibujar seguían la siguiente secuencia:

* Posición inicial en home: q = [0,0,0,pi/2,0\].
* Primera trayectoria de aproximación al punto inicial de la figura, letra o trazo a dibujar, esta aproximación se hace llevando el marcador a la coordenada XY de este punto, pero con un Z arriba del tablero.
* Trayectoria de aproximación al tablero, donde se deja fijo el XY del punto inicial, y se baja Z con una cantidad de puntos especificados.
* Inicio de la rutina de dibujo, donde dependiendo de los trazos (ya sean las iniciales, figura, o workspace) se dibujan estos hasta llegar al punto final del dibujo.
* Trayectoria de alejamiento del tablero, donde se deja XY constante y se sube en Z para alejarse del tablero con n cantidad de puntos.
* Regreso a home, donde tras subir y terminar el dibujo el robot vuelve a home.

Adicional a estas, se programaron dos rutinas para la carga de la herramienta, que seguían la secuencia:

* Posición inicial en home: q = [0,0,0,pi/2,0\].
* Punto de acercamiento a la herramienta.
* Aproximación horizontal al soporte del marcador.
* Cierre del gripper para la sujeción del marcador.
* Alejamiento en el eje z hacia arriba para sacar el marcador del portaherramienta.
* Regreso a home.

Y para la descarga fue similar a la anterior rutina pero invertida, para esto se hizo un ciclo for donde se recorría esta rutina pero de atrás para adelante. Para ver las rutinas más detalladamente, **dirigirse al matlab "CI_Trayectorias_Phantom_Lab.mlx", donde se encuentra implementado todo**.

### Espacio de trabajo
Los puntos para la rutina del espacio de trabajo consistieron en 2 circunferencias de diferente radio parametrizadas en coordenadas polares. En este caso, se escogió el barrido en un rango de ángulo de [-60°,60\] con un radio exterior de 280mm y un radio interior de 165mm. Esto se programó en matlab y la trayectoria teórica se muestra en la imagen a continuación.

<p align="center">
  <img width="460" height="300" src="Imagenes/WS.png">
</p>

Se implementó una función para generar los puntos en matlab, luego, a través de un bucle for se realizó la cinemática inversa de cada punto.

### Iniciales

Se hicieron los trazos para la letra N y G, como adición, se aplicó una matriz de rotación para la letra G. Se hizo usando trazos rectos. De igual manera, tras generar los trazos se realizó la cinemática inversa de cada punto y se guardó todas las posiciones en un único vector. La imagen a continuación muestra las letras.

<p align="center">
  <img width="460" height="300" src="Imagenes/iniciales.png">
</p>

### Figura libre

Se programaron los trazos para dibujar una carita feliz, como los demás casos, tras generar los puntos se hizo un for para obtener los valores de las articulaciones usando la cinemática inversa. La figura se muestra en la imagen a continuación.

<p align="center">
  <img width="460" height="300" src="Imagenes/figurita.png">
</p>

## Envío de rutinas al robot con el script de Python y creación de la interfaz.

_________ Gay

## Resultados

Los resultados de esto se pueden ver en los 2 siguientes videos. El siguiente video muestra la implementación de todas las rutinas.

<p align="center">
  <img width="460" height="300" src="Imagenes/WS.png">
</p>
Video 1

Dado que se pasó por alto algunos requerimientos de la interfaz gráfica, se modificó la interfaz que ya se tenía implementada ajustando estos requerimientos, la interfaz implementada completa se muestra en el siguiente video. Además, en este video se muestra una parte del análisis de repetitividad de trazos, donde se muestra el tablero ya con los trazos y la repetición de algunas rutinas. No se muestra todo el análisis puesto que el video quedaría muy extenso.

<p align="center">
  <img width="460" height="300" src="Imagenes/WS.png">
</p>
Video 2

Se observa que se implementaron todos los requerimientos en la interfaz gráfica y se realiza la secuencia de rutinas solicitada. El resultado de los trazos repetidos se muestra en la siguiente imagen. Se hizo una corrección de perspectiva dado que la superficie del tablero debe quedar lo más perpendicular a la cámara para que se tengan los resultados más reales.

<p align="center">
  <img width="460" height="300" src="Imagenes/results.png">
</p>

## Análisis de resultados

### Tiempos de ejecución

Los tiempos de ejecución de cada rutina se resumen en la siguiente tabla.

| Rutina | Tiempo (s) |
| ------------- | ------------- |
| Carga de herramienta  | Content Cell  |
| Workspace  | Content Cell  |
| Iniciales  | Content Cell  |
| Figura libre  | Content Cell  |
| Descargar herramienta  | Content Cell  |

### Calidad del trazo y comparación con valores teóricos

Ahora, se comparará cada trazo de cada rutina respecto al valor real, se tomará la desviación de os trazos generados en las rutinas identificado por computadora. La imagen a continuación muestra la comparación de esto.

<p align="center">
  <img width="460" height="300" src= "Imagenes/results2.png">
</p>

A través de la librería OpenCV en Python, las imágenes fueron comparadas usando el patrón de calibración de la cámara del celular, exportando las imágenes calibradas teniendo en cuenta que el trazo rojo es producto de una operación dilatación usando _cv2.dilate_, y se segmentó este trazo, luego, se umbralizó la imagen del tablero tomada con la cámara usando _cv2.threshold_ utilizando un umbral de 225, se segmentó también y calibrando esta imagen y sumándolas se obtuvo la intersección de las dos imágenes. Tras esto, se dispuso a hacer las cotas para la verificación dimensional generando los siguientes resultados.

#### Trayectoria del workspace

Se tomaron las desviaciones de el radio de curvatura y de la distancia entre punto de inicio y final. Los resultados se muestran en la siguiente tabla.

<p align="center">
  <img width="460" height="300" src="Imagenes/WIdim.png">
</p>

<p align="center">
  <img width="460" height="300" src="Imagenes/WOdim.png">
</p>

| Trazo | Valor real (mm) | Valor del dibujo (mm) | Error relativo del trazo |
| ------------- | ------------- | ------------- | ------------- |
| Radio interno | 165  | ------------- |
| Radio externo | 285  | 292.05 |  |
| Longitud RI | 242.57  | 236.14 |  |
| Longitud RO | 482.57  | ------------- |  |

Se observa que esta figura la hace considerablemente bien incluso cuando se repiten los trazos, se observa que al inicio de la trayectoria existe una línea atípica no programada, pero esto se debe a que cuando el marcador baja, se choca inicialmente contra el tablero y después se acomoda el gripper, por lo que este choque pequeño genera estos trazos que también se verán en otras figuras.

#### Trayectoria iniciales

Del mismo modo, los resultados para estos trazos son:

<p align="center">
  <img width="460" height="300" src="Imagenes/initdim.png">
</p>

<p align="center">
  <img width="460" height="300" src="Imagenes/initdim2.png">
</p>

| Trazo | Valor real (mm) | Valor del dibujo (mm) |
| ------------- | ------------- | ------------- |
| Trazo 1: Letra N | 165  | ------------- |
| Trazo 2: Letra N | 285  | ------------- |
| Trazo 3: Letra N | 242.57  | ------------- |
| Trazo 1: Letra G | 165  | ------------- |
| Trazo 2: Letra G | 285  | ------------- |
| Trazo 3: Letra G | 242.57  | ------------- |
| Trazo 4: Letra G | 242.57  | ------------- |
| Trazo 5: Letra G | 242.57  | ------------- |

Se observa que la letra G la hace mejor que la letra N, aquí ocurre el mismo fenómeno, que el primer trazo debe corregir la trayectoria desde el punto de inicio, por lo que es el trazo más deficiente. Sin embargo, se logra corregir esto y la letra G se dibuja de una mejor manera. Otra cosa a atribuir es que el marcador pudo haberse movido un poco debido a la escritura o choques con el tablero, lo cual genera inexactitudes en los trazos dibujados.

#### Trayectoria figura libre

Ahora, para la figura libre se tiene:

<p align="center">
  <img width="460" height="300" src="Imagenes/digRes.png">
</p>

| Trazo | Valor real (mm) | Valor del dibujo (mm) |
| ------------- | ------------- | ------------- |
| Trazo 1: Ojo 1 | 165  | ------------- |
| Trazo 2: Ojo 2 | 285  | ------------- |
| Trazo 3: Radio boca | 242.57  | ------------- |
| Trazo 1: Diámetro cara | 165  | ------------- |

