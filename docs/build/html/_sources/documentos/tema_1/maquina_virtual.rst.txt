Instalacion de Ubuntu 22.04 con VirtualBox
==========================================

1. `Descargar
   VirtualBox <https://www.oracle.com/virtualization/technologies/vm/downloads/virtualbox-downloads.html>`__

.. figure:: ./img/0-VB0.jpeg
   :alt: Seleccion

   Seleccion

3. Configuracion de VirtualBOX

Selecciono la opción “nueva”

.. figure:: ./img/0-VB1.jpeg
   :alt: Seleccion

   Seleccion

Coloco un nombre para la máquina virtual

.. figure:: ./img/0-VB2.jpeg
   :alt: Seleccion

   Seleccion

Cargo la imagen ISO

.. figure:: ./img/0-VB3.jpeg
   :alt: Seleccion

   Seleccion

.. figure:: ./img/0-VB4.jpeg
   :alt: Seleccion

   Seleccion

Agrego un Usuario y contraseña para la máquina virtual

.. figure:: ./img/0-VB5.jpeg
   :alt: Seleccion

   Seleccion

Congifuro la memoria ram y el número de nucleos (se recomienda utilizar
el 50% de la memoria RAM Física)

.. figure:: ./img/0-VB6.jpeg
   :alt: Seleccion

   Seleccion

Para un uso básico se recomienda un total de 30 GB de memoria

.. figure:: ./img/0-VB7.jpeg
   :alt: Seleccion

   Seleccion

Finalizo la configuración

.. figure:: ./img/0-VB8.jpeg
   :alt: Seleccion

   Seleccion

Arrancar Máquina virtual - Al iniciar la máquina de forma automática
comenzará la instalación

.. figure:: ./img/0-VB9.jpeg
   :alt: Seleccion

   Seleccion

4. Instalación de locales

Al realizarse la instalación utilizando una Máquina virtual es posible
que no podamos acceder a la terminal de ubuntu, por lo cual es necesario
realizar la configuración de forma manual de los *locales*

Presionamos las teclas ctrl+alt+F3

Ingresamos nuestras credenciales

Accedemos al super usuario

::

   su

la contraseña es la misma que usamos para nuestro usuario regular.

a continuación realizamos la configuración de los locales.

::

   locale 

   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   locale  

y reiniciamos el sistema

::

   init 6

5. Actualizar software

::

   sudo apt upgrade -y 

5. Habilitar portapapeles Bidireccional

Para habilitar el portapapeles bidireccional se debe configurar la
máquina virtual de la siguiente forma:

.. figure:: ./img/0-VB10.jpg
   :alt: Seleccion

   Seleccion

montar el CD de complemento de invitado

.. figure:: ./img/0-VB11.jpg
   :alt: Seleccion

   Seleccion

ejecutar autorun.sh y reiniciar el sistema.

**Posibles Errores** -

1. **Tu Usuario** no está en el archivo sudousers

**solucion** en la terminal colocar

::

   su -

posteriormente agrego el nombre de usuario

::

   sudo adduser [su nombre de usuario] sudo

2. Al momento de ejecutar la máquina virtual NOT IN A HYPERVISOR
   PARTITION (HVP =0 )
   `solucion <https://www.youtube.com/watch?v=XkLHhqOZmmY&t=5s>`__
