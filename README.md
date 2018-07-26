# plataforma_autonoma
<h1>Plataforma modular para desarrollos de conduccion autónoma</h1>

![alt text](https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/photo_2018-07-25_19-28-11.jpg?raw=true)

Esta vehículo tiene como objetivo, ser una plataforma modular para desarrollos académicos. Fue diseñada con el objetivo de poder implementar sistemas de control en un vehículo semejante a uno comercial pero a escala 1-10. 

Este vehículo es capaz de alcanzar un objetivo determinado por una coordenada x-y evitando obstáculos, utilización dead reackoing para estimar su poision. Esta primer versión utiliza una red de Arduinos nano interconectados por I2C que se encargan del control, la comunicación y evitar obstaculos.

Es posible comandar el vehículo remotamente desde una base manualmente con un joystick o mediante comandos serial a través de una computadora.

<h2>Fabricación:</h2>
<h3>Mecánica</h3>

<img src="https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/P01-02.jpg?raw=true" alt="Smiley face" height="567" width="800"></img>

Con el objetivo de poder modificar la plataforma para adaptarla a las necesidades futuras se dividió el diseño en tres modulos cada uno mecanicámenete independiente:

<ul>
  <li>Tracción</li>    
  <li>Dirección</li>
  <li>Chasis</li>
</ul>
  
Ademas de esos tres modulos, se diseñaron otros dos para poder aplciar un sistema de control y detección de obstáculos:

<ul>
  <li>Radar</li>    
  <li>Control</li>
</ul>

Todos los planos y los componentes estan detallados en la carpeta correspondiente -> link
A continuacion se muestra como es el ensamble y cuales son las piezas que deben utilziarse.
Todas las piezas impresas tienen una codificacion en el nombre que indica como se recomienda que sean impresas.

<b>XXXX-XXXX-I3D-L04-W20-IF50</b>

Esto indica que es una pieza impresa en 3D con capa de 0.4 mm pared de 2 mm y relleno de 50%.

<h4>Tracción</h4>
<img src="https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/P01-T04.jpg?raw=true" alt="Smiley face" height="567" width="800"></img>
<h4>Dirección</h4>
<img src="https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/P01-D04.jpg?raw=true" alt="Smiley face" height="567" width="800"></img>
<h4>Chasis</h4>
<img src="https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/P01-CH04.jpg?raw=true" alt="Smiley face" height="567" width="800"></img>
<h4>Radar</h4>
<img src="https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/P01-R02.jpg?raw=true" alt="Smiley face" height="567" width="800"></img>
<h4>Control</h4>
<img src="https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/P01-CT01.jpg?raw=true" alt="Smiley face" height="567" width="800"></img>

<h4>Ensable completo</h4>
<img src="https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/P01.jpg?raw=true" alt="Smiley face" height="567" width="800"></img>
<h3>Electrónica</h3>
La electrónica esta desarrollada en base a modulos comerciales y no es necesario fabricar ningun pcb. Esta versión fue fabricada utilziando placas multiperforadas y componentes básicos como conectores.

El detalle de construcción esta en la carpeta de electrónica -> link

<h3>Software</h3>
Todos los códigos estan realizados en Arduino y C++, se utilizaron librerias comerciales y se programaron otras. En la carpeta de Códigos se encuentra cada detalle.
