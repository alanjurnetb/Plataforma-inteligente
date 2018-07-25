# plataforma_autonoma
<h1>Plataforma modular para desarrollos de conduccion autonoma</h1>v

![alt text](https://github.com/alanjurnetb/plataforma_autonoma/blob/master/Planos/P01-02.jpg?raw=true)


Esta vehiculo tiene como objetivo, ser una plataforma modular para desarrollos academicos. Fue diseñada con el objetivo de poder implementar sistemas de control en un vehiculo semejante a uno comercial pero a escala 1-10. 

Este vehiculo es capaz de alcanzar un objetivo determinado por una coordenada x-y evitando obstaculos, utilizacion dead reackoing para estimar su poision. Esta primer version utiliza una red de Arduinos nano interconectados por I2C que se encargan del control, la comunicacion y evitar obstaculos.

Es posible comandar el vehiculo remotamente desde una base manualmente con un joystick o mediante comandos serial a traves de una computadora.

<h2>Fabricacion:</h2>
<h3>Mecanica</h3>
Con el objetivo de poder modificar la plataforma para adaptarla a las necesidades futuras se dividio el diseño en tres modulos cada uno mecanicamenete independiente:

<ul>
  <li>Traccion</li>    
  <li>Direccion</li>
  <li>Chasis</li>
</ul>
  
Ademas de esos tres modulos, se diseñaron otros dos para poder aplciar un sistema de control y deteccion de obstaculos:

<ul>
  <li>Radar</li>    
  <li>Control</li>
</ul>

Todos los planos y los componentes estan detallados en la carpeta correspondiente -> link

<h3>Electronica</h3>
La electronica esta desarrollada en base a modulos comerciales y no es neceario fabricar ningun pcb. Esta version fue fabricada utilziando placas multiperforadas y componentes basicos como conectores.

El detalle de construccion esta en la carpeta de electronica -> link

<h3>Software</h3>
Todos los codigos estan realizados en Arduino y C++, se utilizaron librerias comerciales y se programaron otras. En la carpeta de Codigos se encuentra cada detalle.
