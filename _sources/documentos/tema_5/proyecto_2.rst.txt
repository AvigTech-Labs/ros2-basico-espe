Proyecto 2 
===========

Introducción
-------------

En ROS 2, los robots normalmente se describen mediante archivos **URDF**
(*Universal Robot Description Format*). El URDF define la estructura del robot:
links, joints, geometría visual, colisiones e inercias.

Sin embargo, cuando un robot crece en complejidad o se necesita reutilizar el
mismo modelo varias veces, escribir todo directamente en URDF se vuelve poco
práctico.

Para esto se utiliza **XACRO** (*XML Macros*), una herramienta que permite
generar archivos URDF de forma más ordenada, reutilizable y parametrizable.
