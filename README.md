# plataforma_autonoma
<h1>Plataforma modular para desarrollos de conduccion autónoma</h1>

![alt text](https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/Imagenes/P01-02.jpg?raw=true)


Esta vehículo tiene como objetivo, ser una plataforma modular para desarrollos académicos. Fue diseñada con el objetivo de poder implementar sistemas de control en un vehículo semejante a uno comercial pero a escala 1-10. 

Este vehículo es capaz de alcanzar un objetivo determinado por una coordenada x-y evitando obstáculos, utilización dead reackoing para estimar su poision. Esta primer versión utiliza una red de Arduinos nano interconectados por I2C que se encargan del control, la comunicación y evitar obstaculos.

Es posible comandar el vehículo remotamente desde una base manualmente con un joystick o mediante comandos serial a través de una computadora.

<h2>Fabricación:</h2>
<h3>Mecánica</h3>
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

<h3>Electrónica</h3>
La electrónica esta desarrollada en base a modulos comerciales y no es necesario fabricar ningun pcb. Esta versión fue fabricada utilziando placas multiperforadas y componentes básicos como conectores.

El detalle de construcción esta en la carpeta de electrónica -> link

<h3>Software</h3>
Todos los códigos estan realizados en Arduino y C++, se utilizaron librerias comerciales y se programaron otras. En la carpeta de Códigos se encuentra cada detalle.
