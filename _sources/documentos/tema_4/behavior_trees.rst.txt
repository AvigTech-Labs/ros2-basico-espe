Behavior Trees en ROS 2 para el control de dos robots SCARA
===========================================================


Objetivo del documento
----------------------

Este documento explica cómo aplicar **Behavior Trees (BT)** en ROS 2 para controlar la secuencia de trabajo de dos robots SCARA de 3 grados de libertad:

- Robot 1: ``ra1``
- Robot 2: ``ra2``
- Robot 2: ``ra3``

El sistema base considerado es:

- Robots SCARA propios de 3 GDL: rotacional, rotacional, prismático.
- Modelo CAD convertido a ``xacro``/URDF.
- Dos robots cargados simultáneamente en RViz2 usando namespaces.
- Puente ROS 2 - MQTT para enviar objetivos de juntas a ESP32.
- ESP32 ejecutando movimiento con AccelStepper.
- El gemelo digital se actualiza con los movimientos reales o estimados.
- No se utiliza cinemática inversa; ya se conocen los objetivos articulares ``[q1, q2, q3]``.

La tarea que se quiere automatizar es:

.. code-block:: text

   Home -> Pick -> Activar efector -> Place -> Apagar efector

En el caso de dos robots, los ciclos no siempre se ejecutan en paralelo. El activador de máquina puede disparar el ciclo del robot 1 o del robot 2 de forma independiente. También puede ocurrir que, mientras el robot 1 está ejecutando su ciclo, se active el ciclo del robot 2.

Contexto general
----------------

En ROS 1 se utilizaba con frecuencia ``SMACH`` para crear máquinas de estado en Python. En ROS 2 no existe un reemplazo oficial único de SMACH. Las alternativas más utilizadas son:

- FSM manual en Python.
- ``SMACC2`` para máquinas de estado en C++.
- ``YASMIN`` para FSM en Python/C++.
- ``Behavior Trees`` para orquestación reactiva de tareas.
- Planificación simbólica con ``PlanSys2`` para tareas más complejas.

En este documento se desarrolla el camino de **Behavior Trees en Python**, usando ``py_trees`` y ``py_trees_ros``.

Referencias útiles:

- py_trees_ros: https://py-trees-ros.readthedocs.io/
- py_trees: https://py-trees.readthedocs.io/
- py_trees_ros en ROS 2 Humble: https://docs.ros.org/en/humble/p/py_trees_ros/
- Nav2 Behavior Trees: https://docs.nav2.org/behavior_trees/
- BehaviorTree.CPP: https://www.behaviortree.dev/

Concepto de Behavior Tree
-------------------------

Un **Behavior Tree** es una estructura jerárquica que define cómo se ejecuta una tarea. A diferencia de una máquina de estados clásica, un BT no depende únicamente de un estado activo y transiciones explícitas, sino que evalúa periódicamente el árbol mediante ciclos llamados **ticks**.

Cada nodo del árbol devuelve uno de tres estados:

.. list-table:: Estados de retorno de un nodo BT
   :header-rows: 1
   :widths: 20 80

   * - Estado
     - Significado
   * - ``SUCCESS``
     - La acción o condición terminó correctamente.
   * - ``FAILURE``
     - La acción o condición falló.
   * - ``RUNNING``
     - La acción todavía está en ejecución.

Ejemplo aplicado al robot:

.. list-table:: Ejemplos de retorno
   :header-rows: 1
   :widths: 30 30 40

   * - Nodo
     - Estado devuelto
     - Interpretación
   * - ``MoveHome``
     - ``RUNNING``
     - El robot todavía se está moviendo hacia Home.
   * - ``MoveHome``
     - ``SUCCESS``
     - El robot llegó a Home.
   * - ``MovePick``
     - ``FAILURE``
     - El robot no completó el movimiento dentro del timeout.

Elementos principales de un Behavior Tree
-----------------------------------------

Behavior / Action node
~~~~~~~~~~~~~~~~~~~~~~

Un ``Behaviour`` es un nodo básico del árbol. Puede representar una acción o una condición.

Ejemplos para el SCARA:

.. code-block:: text

   MoveHome
   MovePick
   EffectorOn
   MovePlace
   EffectorOff
   CheckPieceDetected
   CheckEmergencyStop

Sequence
~~~~~~~~

Una ``Sequence`` ejecuta sus hijos en orden.

Reglas:

- Si un hijo devuelve ``SUCCESS``, pasa al siguiente.
- Si un hijo devuelve ``RUNNING``, la secuencia devuelve ``RUNNING``.
- Si un hijo devuelve ``FAILURE``, la secuencia devuelve ``FAILURE``.

Para la tarea principal:

.. code-block:: text

   Sequence: PalletizingTask
   ├── WaitStart
   ├── MoveHome
   ├── MovePick
   ├── EffectorOn
   ├── MovePlace
   └── EffectorOff

Selector / Fallback
~~~~~~~~~~~~~~~~~~~

Un ``Selector`` o ``Fallback`` prueba alternativas.

Reglas:

- Si un hijo devuelve ``FAILURE``, prueba el siguiente.
- Si un hijo devuelve ``SUCCESS``, el selector devuelve ``SUCCESS``.
- Si un hijo devuelve ``RUNNING``, el selector devuelve ``RUNNING``.

Ejemplo con recuperación:

.. code-block:: text

   Selector: Root
   ├── MainTask
   └── Recovery

Si ``MainTask`` falla, se ejecuta ``Recovery``.

Decorator
~~~~~~~~~

Un ``Decorator`` modifica el comportamiento de un nodo hijo.

Ejemplos:

- Reintentar una acción.
- Invertir el resultado.
- Aplicar timeout.
- Repetir una acción.

Ejemplo conceptual:

.. code-block:: text

   Retry
   └── MovePick

Blackboard
~~~~~~~~~~

El ``Blackboard`` es una memoria compartida entre nodos del árbol. Puede almacenar:

- Estado del robot.
- Último ``motion_state`` recibido.
- Resultado de sensores.
- Parámetros de receta.
- Variables de coordinación entre robots.

En este primer ejercicio usaremos una estructura simple de Python como estado compartido:

.. code-block:: python

   self.robot_status = {
       "motion_state": None,
       "start_requested": False,
   }

Diferencia entre FSM y Behavior Tree
------------------------------------

Una FSM manual para tu secuencia se ve así:

.. code-block:: text

   IDLE -> MOVE_HOME -> MOVE_PICK -> EFFECTOR_ON -> MOVE_PLACE -> EFFECTOR_OFF -> DONE

Un Behavior Tree equivalente se ve así:

.. code-block:: text

   Sequence: PalletizingTask
   ├── WaitStart
   ├── MoveHome
   ├── MovePick
   ├── EffectorOn
   ├── MovePlace
   └── EffectorOff

La FSM es muy clara para secuencias pequeñas. El BT empieza a ser más conveniente cuando se agregan:

- Reintentos.
- Condiciones de sensores.
- Fallos y recuperación.
- Bloqueo de zonas compartidas.
- Coordinación entre robots.
- Tareas alternativas.
- Comportamiento reactivo.

Instalación
-------------

BT se puede utilizar directamente con python. 

.. code-block:: bash

   pip install py-trees
   apt-get install -y graphviz
   pip install py-trees pydot

Diccionarios Python
--------------------

Para agregar un mayor nivel de control con respecto a las variables del sistema es importante manejar una memoria de datos,
en este caso se utiliza un diccionario con las variables más importantes,

.. code-block:: python

	CAMINOS = {
		"malla_A": {
			"0A": [0, 3, 6],
			"0B": [0, 3, 4, 7],
			"0C": [0, 3, 4, 5, 8],

			"1A": [1, 4, 3, 6],
			"1B": [1, 4, 7],
			"1C": [1, 4, 5, 8],

			"2A": [2, 5, 4, 3, 6],
			"2B": [2, 5, 4, 7],
			"2C": [2, 5, 8],
		},

		"malla_B": {
			"0A": [0, 3, 6],
			"0B": [0, 3, 4, 7],
			"0C": [0, 3, 4, 5, 8],

			"1A": [1, 4, 3, 6],
			"1B": [1, 4, 7],
			"1C": [1, 4, 5, 8],

			"2A": [2, 5, 4, 3, 6],
			"2B": [2, 5, 4, 7],
			"2C": [2, 5, 8],
		},

		"malla_C": {
			"0A": [0, 3, 6],
			"0B": [0, 3, 4, 7],
			"0C": [0, 3, 4, 5, 8],

			"1A": [1, 4, 3, 6],
			"1B": [1, 4, 7],
			"1C": [1, 4, 5, 8],

			"2A": [2, 5, 4, 3, 6],
			"2B": [2, 5, 4, 7],
			"2C": [2, 5, 8],
		},
	}

	def obtener_camino(nombre_malla, codigo_camino):
		"""
		Busca un camino dentro del diccionario CAMINOS.

		Parámetros:
			nombre_malla: nombre de la malla. Ejemplo: "malla_A"
			codigo_camino: código de camino. Ejemplo: "0B"

		Retorna:
			Lista con los puntos del camino. Ejemplo: [0, 3, 4, 7]
		"""

		nombre_malla = nombre_malla.strip()
		codigo_camino = codigo_camino.upper().strip()

		if nombre_malla not in CAMINOS:
			raise ValueError(
					f"La malla '{nombre_malla}' no existe. "
					f"Mallas disponibles: {list(CAMINOS.keys())}"
			)

		if codigo_camino not in CAMINOS[nombre_malla]:
			raise ValueError(
					f"El camino '{codigo_camino}' no existe en '{nombre_malla}'. "
					f"Caminos disponibles: {list(CAMINOS[nombre_malla].keys())}"
			)

		return CAMINOS[nombre_malla][codigo_camino]


	# ============================================================
	# PRUEBAS
	# ============================================================

	camino_1 = obtener_camino("malla_A", "0B")
	camino_2 = obtener_camino("malla_B", "2C")
	camino_3 = obtener_camino("malla_C", "1A")

	print("Camino malla_A 0B:", camino_1)
	print("Camino malla_B 2C:", camino_2)
	print("Camino malla_C 1A:", camino_3)

	caminos_b = CAMINOS

	caminos_b["malla_A"]["0B"] = [0,0,1]

	print(CAMINOS["malla_A"]["0B"])


BT para el control
------------------

En este ejemplo se utiliza usa secuenca de control simple para un robot, simulando cada movimiento a través de un timer.

.. code-block:: python

	# ============================================================
	# IMPORTACIÓN DE LIBRERÍAS
	# ============================================================

	import time
	import os
	import py_trees


	# ============================================================
	# DATO PRINCIPAL: caminos de todas las mallas 3x3
	# ============================================================

	CAMINOS = {
		"malla_A": {
			"0A": [0, 3, 6],
			"0B": [0, 3, 4, 7],
			"0C": [0, 3, 4, 5, 8],

			"1A": [1, 4, 3, 6],
			"1B": [1, 4, 7],
			"1C": [1, 4, 5, 8],

			"2A": [2, 5, 4, 3, 6],
			"2B": [2, 5, 4, 7],
			"2C": [2, 5, 8],
		},

		"malla_B": {
			"0A": [0, 3, 6],
			"0B": [0, 3, 4, 7],
			"0C": [0, 3, 4, 5, 8],

			"1A": [1, 4, 3, 6],
			"1B": [1, 4, 7],
			"1C": [1, 4, 5, 8],

			"2A": [2, 5, 4, 3, 6],
			"2B": [2, 5, 4, 7],
			"2C": [2, 5, 8],
		},

		"malla_C": {
			"0A": [0, 3, 6],
			"0B": [0, 3, 4, 7],
			"0C": [0, 3, 4, 5, 8],

			"1A": [1, 4, 3, 6],
			"1B": [1, 4, 7],
			"1C": [1, 4, 5, 8],

			"2A": [2, 5, 4, 3, 6],
			"2B": [2, 5, 4, 7],
			"2C": [2, 5, 8],
		},
	}


	# ============================================================
	# FUNCIÓN: buscar camino según nombre de malla y código
	# ============================================================

	def obtener_camino(nombre_malla, codigo_camino):
		"""
		Busca un camino dentro del diccionario CAMINOS.

		Ejemplo:
			obtener_camino("malla_A", "0B")

		Retorna:
			[0, 3, 4, 7]
		"""

		nombre_malla = nombre_malla.strip()
		codigo_camino = codigo_camino.upper().strip()

		if nombre_malla not in CAMINOS:
			raise ValueError(
					f"La malla '{nombre_malla}' no existe. "
					f"Mallas disponibles: {list(CAMINOS.keys())}"
			)

		if codigo_camino not in CAMINOS[nombre_malla]:
			raise ValueError(
					f"El camino '{codigo_camino}' no existe en '{nombre_malla}'. "
					f"Caminos disponibles: {list(CAMINOS[nombre_malla].keys())}"
			)

		return CAMINOS[nombre_malla][codigo_camino]


	# ============================================================
	# CLASE: SOLICITAR CAMINO
	# ============================================================

	class SolicitarCamino(py_trees.behaviour.Behaviour):
		"""
		Primer comportamiento del árbol.

		Este nodo pide al usuario:
		- nombre de la malla
		- código del camino

		Si el camino existe:
			retorna SUCCESS

		Si el camino no existe:
			retorna FAILURE
		"""

		def __init__(self, name, memoria):
			super().__init__(name=name)

			# memoria es un diccionario compartido entre comportamientos
			self.memoria = memoria

			# Variable para evitar pedir datos más de una vez
			self.consulta_realizada = False

		def initialise(self):
			print("\n=== Solicitud de camino ===")

		def update(self):

			# Si ya se hizo la consulta, no la repetimos
			if self.consulta_realizada:
					return py_trees.common.Status.SUCCESS

			try:
					# Pedir datos al usuario
					nombre_malla = input("Ingrese nombre de la malla (malla_A, malla_B, malla_C): ")
					codigo_camino = input("Ingrese código del camino (ejemplo: 0A, 0B, 2C): ")

					# Buscar el camino en el diccionario
					camino = obtener_camino(nombre_malla, codigo_camino)

					# Guardar datos en memoria compartida
					self.memoria["nombre_malla"] = nombre_malla.strip()
					self.memoria["codigo_camino"] = codigo_camino.upper().strip()
					self.memoria["camino"] = camino

					print("\nCamino encontrado correctamente:")
					print(f"Malla: {self.memoria['nombre_malla']}")
					print(f"Código: {self.memoria['codigo_camino']}")
					print(f"Camino: {self.memoria['camino']}")

					# Marcar que ya se consultó
					self.consulta_realizada = True

					# Si el camino existe, el comportamiento termina con éxito
					return py_trees.common.Status.SUCCESS

			except ValueError as error:
					print("\nError en la búsqueda del camino:")
					print(error)

					# Si hay error, el comportamiento falla
					return py_trees.common.Status.FAILURE


	# ============================================================
	# CLASE: ACCIÓN SIMPLE
	# ============================================================

	class AccionSimple(py_trees.behaviour.Behaviour):
		"""
		Comportamiento simple que simula una acción con tiempo.

		Ejemplo:
		- Mover a HOME
		- Activar efector
		- Apagar efector
		"""

		def __init__(self, name, tiempo=1.0):
			super().__init__(name=name)
			self.tiempo = tiempo
			self.tiempo_inicio = None

		def initialise(self):
			self.tiempo_inicio = time.time()
			print(f"Iniciando: {self.name}")

		def update(self):
			tiempo_actual = time.time()
			tiempo_transcurrido = tiempo_actual - self.tiempo_inicio

			if tiempo_transcurrido >= self.tiempo:
					print(f"Finalizado: {self.name}")
					return py_trees.common.Status.SUCCESS

			return py_trees.common.Status.RUNNING


	# ============================================================
	# CLASE: RECORRER CAMINO
	# ============================================================

	class RecorrerCamino(py_trees.behaviour.Behaviour):
		"""
		Este comportamiento recorre los puntos del camino encontrado.

		Por ejemplo, si el usuario ingresó:
			malla_A, 0B

		El camino será:
			[0, 3, 4, 7]

		Este nodo simula mover el robot punto por punto.
		"""

		def __init__(self, name, memoria, tiempo_por_punto=1.0):
			super().__init__(name=name)

			self.memoria = memoria
			self.tiempo_por_punto = tiempo_por_punto

			self.indice_actual = 0
			self.tiempo_inicio = None

		def initialise(self):
			self.indice_actual = 0
			self.tiempo_inicio = time.time()

			print("\nIniciando recorrido del camino...")

		def update(self):

			# Verificar que exista un camino guardado
			if "camino" not in self.memoria:
					print("No existe un camino cargado en memoria.")
					return py_trees.common.Status.FAILURE

			camino = self.memoria["camino"]

			# Si ya recorrimos todos los puntos, terminamos
			if self.indice_actual >= len(camino):
					print("Camino recorrido completamente.")
					return py_trees.common.Status.SUCCESS

			tiempo_actual = time.time()
			tiempo_transcurrido = tiempo_actual - self.tiempo_inicio

			# Cada cierto tiempo avanzamos al siguiente punto
			if tiempo_transcurrido >= self.tiempo_por_punto:
					punto = camino[self.indice_actual]

					print(f"Robot moviéndose al punto: {punto}")

					# Aquí en un robot real publicarías:
					# /raX/joint_goals_q
					# y esperarías /raX/motion_state

					self.indice_actual += 1
					self.tiempo_inicio = time.time()

			return py_trees.common.Status.RUNNING


	# ============================================================
	# FUNCIÓN: CREAR ÁRBOL DE COMPORTAMIENTO
	# ============================================================

	def crear_arbol():

		# Memoria compartida entre nodos del árbol
		memoria = {}

		# Nodo raíz tipo Sequence
		root = py_trees.composites.Sequence(
			name="Secuencia_SCARA",
			memory=True
		)

		# Primer comportamiento:
		# El usuario ingresa la malla y el camino
		solicitar_camino = SolicitarCamino(
			name="Solicitar camino",
			memoria=memoria
		)

		# Segundo comportamiento:
		# Simula mover a HOME
		mover_home = AccionSimple(
			name="Mover a HOME",
			tiempo=2.0
		)

		# Tercer comportamiento:
		# Recorre los puntos del camino encontrado
		recorrer_camino = RecorrerCamino(
			name="Recorrer camino",
			memoria=memoria,
			tiempo_por_punto=1.0
		)

		# Cuarto comportamiento:
		# Activa efector
		activar_efector = AccionSimple(
			name="Activar efector",
			tiempo=1.0
		)

		# Quinto comportamiento:
		# Apaga efector
		apagar_efector = AccionSimple(
			name="Apagar efector",
			tiempo=1.0
		)

		# Agregar comportamientos al árbol
		root.add_children([
			solicitar_camino,
			mover_home,
			recorrer_camino,
			activar_efector,
			apagar_efector
		])

		return root


	# ============================================================
	# FUNCIÓN: GRAFICAR ÁRBOL EN PNG
	# ============================================================

	def graficar_arbol_png(arbol, nombre_archivo="arbol_scara"):

		print("\nÁrbol en formato texto:")
		print(py_trees.display.ascii_tree(arbol))

		py_trees.display.render_dot_tree(
			root=arbol,
			name=nombre_archivo,
			target_directory="."
		)

		archivo_dot = f"{nombre_archivo}.dot"
		archivo_svg = f"{nombre_archivo}.svg"
		archivo_png = f"{nombre_archivo}.png"

		if os.path.exists(archivo_dot):
			os.remove(archivo_dot)

		if os.path.exists(archivo_svg):
			os.remove(archivo_svg)

		print(f"\nImagen PNG generada: {archivo_png}")

		try:
			from IPython.display import Image, display
			display(Image(filename=archivo_png))
		except Exception:
			print(f"Abre manualmente el archivo: {archivo_png}")


	# ============================================================
	# FUNCIÓN PRINCIPAL
	# ============================================================

	def main():

		arbol = crear_arbol()

		# Graficar estructura inicial
		graficar_arbol_png(arbol, "arbol_scara_con_busqueda")

		print("\nEjecutando árbol de comportamiento...\n")

		while arbol.status not in [
			py_trees.common.Status.SUCCESS,
			py_trees.common.Status.FAILURE
		]:
			arbol.tick_once()
			time.sleep(0.1)

		if arbol.status == py_trees.common.Status.SUCCESS:
			print("\nTarea completa correctamente.")

		elif arbol.status == py_trees.common.Status.FAILURE:
			print("\nLa tarea falló.")

		print("\nÁrbol con estados finales:")
		print(py_trees.display.ascii_tree(arbol, show_status=True))


	# ============================================================
	# PUNTO DE ENTRADA
	# ============================================================

	if __name__ == "__main__":
		main()

BT y ROS2 
---------

.. code-block:: bash

   sudo apt update 
   sudo apt install ros-humble-py-trees

Integración del proceso con BT y ROS2.


.. tabs:: 

	.. group-tab:: BT

		.. code:: python

			import rclpy
			from rclpy.node import Node

			from std_msgs.msg import String
			from std_msgs.msg import Float32MultiArray
			from std_msgs.msg import UInt8

			import py_trees
			import time

			# ============================================================
			# BASE DE DATOS DEL SISTEMA
			# ============================================================

			SISTEMAS_DB = {
				"sis_a": {
					"robot_articular": "ra1",
					"robot_movil": "rm_sis_a",

					"home_q": [0.0, 0.0, 0.0],
					"recolectar_q": [45.0, 10.0, 0.0],

					"estaciones_storage_q": {
							0: [0.0, 0.0, 0.0],
							1: [10.0, 0.0, 0.0],
							2: [20.0, 0.0, 0.0],
							3: [0.0, 10.0, 0.0],
							4: [10.0, 10.0, 0.0],
							5: [20.0, 10.0, 0.0],
							6: [0.0, 20.0, 0.0],
							7: [10.0, 20.0, 0.0],
							8: [20.0, 20.0, 0.0],
					},

					"caminos_movil": {
							"0A": [0, 3, 6],
							"0B": [0, 3, 4, 7],
							"0C": [0, 3, 4, 5, 8],
							"1A": [1, 4, 3, 6],
							"1B": [1, 4, 7],
							"1C": [1, 4, 5, 8],
							"2A": [2, 5, 4, 3, 6],
							"2B": [2, 5, 4, 7],
							"2C": [2, 5, 8],
					},
				},

				"sis_b": {
					"robot_articular": "ra2",
					"robot_movil": "rm_sis_b",

					"home_q": [0.0, 0.0, 0.0],
					"recolectar_q": [35.0, 10.0, 0.0],

					"estaciones_storage_q": {
							0: [0.0, 0.0, 0.0],
							1: [10.0, 0.0, 0.0],
							2: [20.0, 0.0, 0.0],
							3: [0.0, 10.0, 0.0],
							4: [10.0, 10.0, 0.0],
							5: [20.0, 10.0, 0.0],
							6: [0.0, 20.0, 0.0],
							7: [10.0, 20.0, 0.0],
							8: [20.0, 20.0, 0.0],
					},

					"caminos_movil": {
							"0A": [0, 3, 6],
							"0B": [0, 3, 4, 7],
							"0C": [0, 3, 4, 5, 8],
							"1A": [1, 4, 3, 6],
							"1B": [1, 4, 7],
							"1C": [1, 4, 5, 8],
							"2A": [2, 5, 4, 3, 6],
							"2B": [2, 5, 4, 7],
							"2C": [2, 5, 8],
					},
				},

				"sis_c": {
					"robot_articular": "ra3",
					"robot_movil": "rm_sis_c",

					"home_q": [0.0, 0.0, 0.0],
					"recolectar_q": [35.0, 10.0, 0.0],

					"estaciones_storage_q": {
							0: [0.0, 0.0, 0.0],
							1: [10.0, 0.0, 0.0],
							2: [20.0, 0.0, 0.0],
							3: [0.0, 10.0, 0.0],
							4: [10.0, 10.0, 0.0],
							5: [20.0, 10.0, 0.0],
							6: [0.0, 20.0, 0.0],
							7: [10.0, 20.0, 0.0],
							8: [20.0, 20.0, 0.0],
					},

					"caminos_movil": {
							"0A": [0, 3, 6],
							"0B": [0, 3, 4, 7],
							"0C": [0, 3, 4, 5, 8],
							"1A": [1, 4, 3, 6],
							"1B": [1, 4, 7],
							"1C": [1, 4, 5, 8],
							"2A": [2, 5, 4, 3, 6],
							"2B": [2, 5, 4, 7],
							"2C": [2, 5, 8],
					},
				},
			}


			# ============================================================
			# FUNCIÓN: CONSULTAR TAREA
			# ============================================================

			def consultar_tarea(nombre_sistema, codigo_camino, estacion_storage):
				nombre_sistema = nombre_sistema.strip().lower()
				codigo_camino = codigo_camino.upper().strip()

				try:
					estacion_storage = int(estacion_storage)
				except ValueError:
					raise ValueError("La estación del storage debe ser un número entre 0 y 8.")

				if nombre_sistema not in SISTEMAS_DB:
					raise ValueError(f"El sistema '{nombre_sistema}' no existe.")

				sistema = SISTEMAS_DB[nombre_sistema]

				if codigo_camino not in sistema["caminos_movil"]:
					raise ValueError(
							f"El camino móvil '{codigo_camino}' no existe en '{nombre_sistema}'."
					)

				if estacion_storage not in sistema["estaciones_storage_q"]:
					raise ValueError(
							f"La estación '{estacion_storage}' no existe en '{nombre_sistema}'."
					)

				tarea = {
					"nombre_sistema": nombre_sistema,
					"codigo_camino": codigo_camino,
					"estacion_storage": estacion_storage,

					"robot_articular": sistema["robot_articular"],
					"robot_movil": sistema["robot_movil"],

					"camino_movil": sistema["caminos_movil"][codigo_camino],

					"home_q": sistema["home_q"],
					"recolectar_q": sistema["recolectar_q"],
					"storage_q": sistema["estaciones_storage_q"][estacion_storage],
				}

				return tarea


			# ============================================================
			# BEHAVIOR: ESPERAR Y VALIDAR TAREA
			# ============================================================

			class EsperarTarea(py_trees.behaviour.Behaviour):

				def __init__(self, name, node, memoria):
					super().__init__(name=name)
					self.node = node
					self.memoria = memoria

				def update(self):
					if not self.memoria["nueva_tarea"]:
							return py_trees.common.Status.RUNNING

					texto_tarea = self.memoria["texto_tarea"]
					self.node.get_logger().info(f"Tarea recibida: {texto_tarea}")

					partes = texto_tarea.split()

					if len(partes) != 3:
							self.node.get_logger().error(
								"Formato incorrecto. Use: 'sis_a 0B 7'"
							)
							self.memoria["nueva_tarea"] = False
							return py_trees.common.Status.FAILURE

					nombre_sistema = partes[0]
					codigo_camino = partes[1]
					estacion_storage = partes[2]

					try:
							tarea = consultar_tarea(
								nombre_sistema,
								codigo_camino,
								estacion_storage
							)

							self.memoria["tarea"] = tarea
							self.memoria["robot_articular"] = tarea["robot_articular"]
							self.memoria["robot_movil"] = tarea["robot_movil"]
							self.memoria["nueva_tarea"] = False

							self.node.get_logger().info("Tarea validada correctamente.")
							self.node.get_logger().info(
								f"Robot móvil: {tarea['robot_movil']}"
							)
							self.node.get_logger().info(
								f"Robot articular: {tarea['robot_articular']}"
							)
							self.node.get_logger().info(
								f"Camino móvil: {tarea['camino_movil']}"
							)
							self.node.get_logger().info(
								f"Storage estación {tarea['estacion_storage']} -> q={tarea['storage_q']}"
							)

							return py_trees.common.Status.SUCCESS

					except ValueError as error:
							self.node.get_logger().error(str(error))
							self.memoria["nueva_tarea"] = False
							return py_trees.common.Status.FAILURE


			# ============================================================
			# BEHAVIOR: INICIAR SISTEMA
			# ============================================================

			class IniciarSistema(py_trees.behaviour.Behaviour):

				def __init__(self, name, node, memoria, tiempo_espera=0.2):
					super().__init__(name=name)
					self.node = node
					self.memoria = memoria
					self.tiempo_espera = tiempo_espera
					self.tiempo_inicio = None

				def initialise(self):
					self.tiempo_inicio = time.time()

					msg = UInt8()
					msg.data = 1

					self.node.publisher_system_start.publish(msg)
					self.node.get_logger().info("Inicio del sistema enviado.")

				def update(self):
					if time.time() - self.tiempo_inicio >= self.tiempo_espera:
							return py_trees.common.Status.SUCCESS

					return py_trees.common.Status.RUNNING


			# ============================================================
			# BEHAVIOR: ENVIAR CAMINO AL ROBOT MÓVIL
			# ============================================================

			class EnviarCaminoMovil(py_trees.behaviour.Behaviour):

				def __init__(self, name, node, memoria, timeout=20.0):
					super().__init__(name=name)
					self.node = node
					self.memoria = memoria
					self.timeout = timeout
					self.tiempo_inicio = None
					self.seen_moving = False

				def initialise(self):
					self.tiempo_inicio = time.time()
					self.seen_moving = False

					tarea = self.memoria["tarea"]
					robot_movil = tarea["robot_movil"]

					msg = Float32MultiArray()
					msg.data = [float(punto) for punto in tarea["camino_movil"]]

					self.node.publishers_mobile_path[robot_movil].publish(msg)

					self.node.get_logger().info(
							f"[{robot_movil}] Camino móvil enviado: {msg.data}"
					)

				def update(self):
					tarea = self.memoria["tarea"]
					robot_movil = tarea["robot_movil"]

					motion_state = self.memoria["mobile_motion_state"][robot_movil]

					if motion_state == 1:
							self.seen_moving = True
							return py_trees.common.Status.RUNNING

					if self.seen_moving and motion_state == 0:
							self.node.get_logger().info(
								f"[{robot_movil}] Camino móvil terminado."
							)
							return py_trees.common.Status.SUCCESS

					if time.time() - self.tiempo_inicio > self.timeout:
							self.node.get_logger().error(
								f"[{robot_movil}] Timeout esperando fin del camino móvil."
							)
							return py_trees.common.Status.FAILURE

					return py_trees.common.Status.RUNNING


			# ============================================================
			# BEHAVIOR: MOVER ROBOT ARTICULAR
			# ============================================================

			class MoverJuntas(py_trees.behaviour.Behaviour):

				def __init__(self, name, node, memoria, target_key, timeout=10.0):
					super().__init__(name=name)

					self.node = node
					self.memoria = memoria
					self.target_key = target_key
					self.timeout = timeout

					self.tiempo_inicio = None
					self.seen_moving = False

				def initialise(self):
					self.tiempo_inicio = time.time()
					self.seen_moving = False

					tarea = self.memoria["tarea"]
					robot_articular = tarea["robot_articular"]

					q = tarea[self.target_key]

					msg = Float32MultiArray()
					msg.data = [float(valor) for valor in q]

					self.node.publishers_joint_goals[robot_articular].publish(msg)

					self.node.get_logger().info(
							f"[{robot_articular}] {self.name} -> q={msg.data}"
					)

				def update(self):
					tarea = self.memoria["tarea"]
					robot_articular = tarea["robot_articular"]

					motion_state = self.memoria["articular_motion_state"][robot_articular]

					if motion_state == 1:
							self.seen_moving = True
							return py_trees.common.Status.RUNNING

					if self.seen_moving and motion_state == 0:
							self.node.get_logger().info(
								f"[{robot_articular}] {self.name} terminado."
							)
							return py_trees.common.Status.SUCCESS

					if time.time() - self.tiempo_inicio > self.timeout:
							self.node.get_logger().error(
								f"[{robot_articular}] Timeout en {self.name}."
							)
							return py_trees.common.Status.FAILURE

					return py_trees.common.Status.RUNNING


			# ============================================================
			# BEHAVIOR: EFECTOR ON/OFF
			# ============================================================

			class ComandoEfector(py_trees.behaviour.Behaviour):

				def __init__(self, name, node, memoria, activar=True, tiempo_espera=5):
					super().__init__(name=name)

					self.node = node
					self.memoria = memoria
					self.activar = activar
					self.tiempo_espera = tiempo_espera
					self.tiempo_inicio = None

				def initialise(self):
					self.tiempo_inicio = time.time()

					tarea = self.memoria["tarea"]
					robot_articular = tarea["robot_articular"]

					msg = UInt8()
					msg.data = 1 if self.activar else 0

					self.node.publishers_effector[robot_articular].publish(msg)

					estado = "ON" if self.activar else "OFF"

					self.node.get_logger().info(
							f"[{robot_articular}] Efector {estado}"
					)

				def update(self):
					if time.time() - self.tiempo_inicio >= self.tiempo_espera:
							return py_trees.common.Status.SUCCESS

					return py_trees.common.Status.RUNNING


			# ============================================================
			# NODO ROS 2 PRINCIPAL
			# ============================================================

			class ScaraBTNode(Node):

				def __init__(self):
					super().__init__("scara_bt_node")

					self.memoria = {
							"nueva_tarea": False,
							"texto_tarea": "",
							"tarea": None,

							"robot_articular": None,
							"robot_movil": None,

							"arbol_ocupado": False,

							"articular_motion_state": {
								"ra1": None,
								"ra2": None,
								"ra3": None,
							},

							"mobile_motion_state": {
								"rm_sis_a": None,
								"rm_sis_b": None,
								"rm_sis_c": None,
							},
					}

					self.publisher_system_start = self.create_publisher(
							UInt8,
							"/system/start",
							10
					)

					self.publishers_mobile_path = {
							"rm_sis_a": self.create_publisher(
								Float32MultiArray,
								"/rm_sis_a/path_cmd",
								10
							),
							"rm_sis_b": self.create_publisher(
								Float32MultiArray,
								"/rm_sis_b/path_cmd",
								10
							),
							"rm_sis_c": self.create_publisher(
								Float32MultiArray,
								"/rm_sis_c/path_cmd",
								10
							),
					}

					self.publishers_joint_goals = {
							"ra1": self.create_publisher(
								Float32MultiArray,
								"/ra1/joint_goals_q",
								10
							),
							"ra2": self.create_publisher(
								Float32MultiArray,
								"/ra2/joint_goals_q",
								10
							),
							"ra3": self.create_publisher(
								Float32MultiArray,
								"/ra3/joint_goals_q",
								10
							),
					}

					self.publishers_effector = {
							"ra1": self.create_publisher(UInt8, "/ra1/effector_cmd", 10),
							"ra2": self.create_publisher(UInt8, "/ra2/effector_cmd", 10),
							"ra3": self.create_publisher(UInt8, "/ra3/effector_cmd", 10),
					}

					self.sub_task = self.create_subscription(
							String,
							"/task_request",
							self.callback_task_request,
							10
					)

					self.sub_mobile_state_a = self.create_subscription(
							UInt8,
							"/rm_sis_a/motion_state",
							lambda msg: self.callback_mobile_motion_state("rm_sis_a", msg),
							10
					)

					self.sub_mobile_state_b = self.create_subscription(
							UInt8,
							"/rm_sis_b/motion_state",
							lambda msg: self.callback_mobile_motion_state("rm_sis_b", msg),
							10
					)

					self.sub_mobile_state_c = self.create_subscription(
							UInt8,
							"/rm_sis_c/motion_state",
							lambda msg: self.callback_mobile_motion_state("rm_sis_c", msg),
							10
					)

					self.sub_articular_state_ra1 = self.create_subscription(
							UInt8,
							"/ra1/motion_state",
							lambda msg: self.callback_articular_motion_state("ra1", msg),
							10
					)

					self.sub_articular_state_ra2 = self.create_subscription(
							UInt8,
							"/ra2/motion_state",
							lambda msg: self.callback_articular_motion_state("ra2", msg),
							10
					)

					self.sub_articular_state_ra3 = self.create_subscription(
							UInt8,
							"/ra3/motion_state",
							lambda msg: self.callback_articular_motion_state("ra3", msg),
							10
					)

					self.tree = self.crear_arbol()

					self.timer = self.create_timer(0.1, self.tick_tree)

					self.get_logger().info("Nodo Behavior Tree iniciado.")
					self.get_logger().info(
							"Envía tareas con: /task_request String 'sis_a 0B 7'"
					)

				def callback_task_request(self, msg):
					if self.memoria["arbol_ocupado"]:
							self.get_logger().warn(
								"Árbol ocupado. Tarea ignorada hasta terminar la actual."
							)
							return

					self.memoria["texto_tarea"] = msg.data
					self.memoria["nueva_tarea"] = True
					self.memoria["arbol_ocupado"] = True

					self.tree.stop(py_trees.common.Status.INVALID)

					self.get_logger().info(f"Nueva tarea recibida: {msg.data}")

				def callback_mobile_motion_state(self, robot_movil, msg):
					self.memoria["mobile_motion_state"][robot_movil] = int(msg.data)

				def callback_articular_motion_state(self, robot_articular, msg):
					self.memoria["articular_motion_state"][robot_articular] = int(msg.data)

				def crear_arbol(self):
					root = py_trees.composites.Sequence(
							name="BT_MOVIL_ARTICULAR_STORAGE",
							memory=True
					)

					esperar_tarea = EsperarTarea(
							name="Esperar y validar tarea",
							node=self,
							memoria=self.memoria
					)

					iniciar_sistema = IniciarSistema(
							name="Iniciar sistema",
							node=self,
							memoria=self.memoria
					)

					enviar_camino_movil = EnviarCaminoMovil(
							name="Enviar camino móvil",
							node=self,
							memoria=self.memoria,
							timeout=20.0
					)

					mover_recoleccion = MoverJuntas(
							name="Mover articular a recolección",
							node=self,
							memoria=self.memoria,
							target_key="recolectar_q",
							timeout=10.0
					)

					activar_efector = ComandoEfector(
							name="Activar efector",
							node=self,
							memoria=self.memoria,
							activar=True,
							tiempo_espera=0.2
					)

					mover_storage = MoverJuntas(
							name="Mover articular a storage",
							node=self,
							memoria=self.memoria,
							target_key="storage_q",
							timeout=10.0
					)

					apagar_efector = ComandoEfector(
							name="Apagar efector",
							node=self,
							memoria=self.memoria,
							activar=False,
							tiempo_espera=0.2
					)

					mover_home = MoverJuntas(
							name="Mover articular a HOME",
							node=self,
							memoria=self.memoria,
							target_key="home_q",
							timeout=10.0
					)

					root.add_children([
							esperar_tarea,
							iniciar_sistema,
							enviar_camino_movil,
							mover_recoleccion,
							activar_efector,
							mover_storage,
							apagar_efector,
							mover_home,
					])

					return root

				def limpiar_tarea(self):
					self.memoria["nueva_tarea"] = False
					self.memoria["texto_tarea"] = ""
					self.memoria["tarea"] = None
					self.memoria["robot_articular"] = None
					self.memoria["robot_movil"] = None
					self.memoria["arbol_ocupado"] = False

				def apagar_efector_seguro(self):
					robot_articular = self.memoria.get("robot_articular")

					if robot_articular in self.publishers_effector:
							msg = UInt8()
							msg.data = 0
							self.publishers_effector[robot_articular].publish(msg)

				def tick_tree(self):
					self.tree.tick_once()

					if self.tree.status == py_trees.common.Status.SUCCESS:
							self.get_logger().info("Tarea completada correctamente.")
							self.limpiar_tarea()
							self.tree.stop(py_trees.common.Status.INVALID)

					elif self.tree.status == py_trees.common.Status.FAILURE:
							self.get_logger().error("La tarea falló.")
							self.apagar_efector_seguro()
							self.limpiar_tarea()
							self.tree.stop(py_trees.common.Status.INVALID)


			# ============================================================
			# MAIN
			# ============================================================

			def main():
				rclpy.init()

				node = ScaraBTNode()

				try:
					rclpy.spin(node)
				except KeyboardInterrupt:
					pass

				node.destroy_node()
				rclpy.shutdown()


			if __name__ == "__main__":
				main()

	.. group-tab:: Mqtt

		.. code:: python

			import rclpy
			from rclpy.node import Node
			from rclpy.duration import Duration
			from std_msgs.msg import Float32MultiArray,UInt8
			from sensor_msgs.msg import JointState

			import paho.mqtt.client as mqtt
			import json
			import math


			class MQTTBridge(Node):
				def __init__(self):
					super().__init__('mqtt_bridge_proyecto')

					# ROS publishers (gemelos digitales)
					self.publisher_r1 = self.create_publisher(JointState, 'r1/joint_states', 1)
					self.publisher_r2 = self.create_publisher(JointState, 'r2/joint_states', 1)

					# ROS publishets (estados)
					self.publisher_state_r1 = self.create_publisher(UInt8, 'ra1/motion_state', 1)
					self.publisher_state_r2 = self.create_publisher(UInt8, 'ra2/motion_state', 1)

					# ROS subscribers (metas)
					self.create_subscription(Float32MultiArray, 'ra1/joint_goals_q', self.listener_ros_ra1, 10)
					self.create_subscription(Float32MultiArray, 'ra2/joint_goals_q', self.listener_ros_ra2, 10)

					# ROS subscribers (metas)
					self.create_subscription(UInt8, 'ra1/effector_cmd', self.listener_efector_ra1, 1)
					self.create_subscription(UInt8, 'ra2/effector_cmd', self.listener_efector_ra2, 1)

					# Watchdog - Verificacion de conexiones esp32
					self.active_ra1 = False
					self.active_ra2 = False
					self.last_mqtt_time_ra1 = self.get_clock().now()
					self.last_mqtt_time_ra2 = self.get_clock().now()
					self.timer_watchdog_ra1 = self.create_timer(0.5, self.check_timeout_ra1)
					self.timer_watchdog_ra2 = self.create_timer(0.5, self.check_timeout_ra2)

					# MQTT topics
					self.topic_sub_r1     = "ra_1/sensores" # datos de juntas, sensores y estados
					self.topic_juntas_r1  = "ra_1/juntas"   # publicador
					self.topic_efector_r1 = "ra_1/efector"   # publicador

					self.topic_sub_r2     = "ra_2/sensores" # datos de juntas, sensores y estados
					self.topic_juntas_r2  = "ra_2/juntas"   # publicador
					self.topic_efector_r2 = "ra_2/efector"   # publicador

					# Paso (grados) para cuantizar comandos (según hardware)
					self.paso_q1 = 0.01929  # aproximación para PG47 + 400 pulse/rev
					self.paso_q2 = 0.9      # si tus NEMA17 están a 0.9°/paso (half-step)
					self.paso_q3 = 0.9

					# MQTT client
					self.mqtt_client = mqtt.Client()
					self.mqtt_client.on_connect = self.on_connect
					self.mqtt_client.on_message = self.on_message
					self.mqtt_client.connect("192.168.100.208", 1883, 60)
					self.mqtt_client.loop_start()

				# ---------- ROS -> MQTT (metas) ----------
				def listener_ros_ra1(self, msg: Float32MultiArray):
					if len(msg.data) < 3:
							self.get_logger().warn("ra1/joint_goals_q debe tener 3 valores.")
							return

					# Conversion a grados
					q1_deg = math.degrees(msg.data[0])
					q2_deg = math.degrees(msg.data[1])

					# Relacion de movimiento engranaje-cremallera
					angulo_q3_deg = math.degrees(msg.data[2] / 0.0225)

					payload = {
							"q1": self.mover_a_angulo(q1_deg, paso=self.paso_q1),
							"q2": self.mover_a_angulo(q2_deg, paso=self.paso_q2),
							"q3": self.mover_a_angulo(angulo_q3_deg, paso=self.paso_q3)
					}
					self.mqtt_client.publish(self.topic_juntas_r1, json.dumps(payload))
					self.get_logger().info("Meta enviada a ra1 (MQTT).")

				def listener_ros_ra2(self, msg: Float32MultiArray):
					if len(msg.data) < 3:
							self.get_logger().warn("ra2/joint_goals_q debe tener 3 valores.")
							return

					q1_deg = math.degrees(msg.data[0])
					q2_deg = math.degrees(msg.data[1])
					angulo_q3_deg = math.degrees(msg.data[2] / 0.0225)

					payload = {
							"q1": self.mover_a_angulo(q1_deg, paso=self.paso_q1),
							"q2": self.mover_a_angulo(q2_deg, paso=self.paso_q2),
							"q3": self.mover_a_angulo(angulo_q3_deg, paso=self.paso_q3)
					}
					self.mqtt_client.publish(self.topic_juntas_r2, json.dumps(payload))
					self.get_logger().info("Meta enviada a ra2 (MQTT).")
				

				def listener_efector_ra1(self, msg: UInt8):
					efector = msg.data
					payload = {
							"efector": efector
					}
					self.mqtt_client.publish(self.topic_efector_r1, json.dumps(payload))
					self.get_logger().info("Meta enviada a ra1 efector  (MQTT).")
				
				def listener_efector_ra2(self, msg: UInt8):
					efector = msg.data
					payload = {
							"efector": efector
					}
					self.mqtt_client.publish(self.topic_efector_r2, json.dumps(payload))
					self.get_logger().info("Meta enviada a ra2 efector  (MQTT).")

				def mover_a_angulo(self, angulo_objetivo: float, paso: float) -> float:
					"""
					Cuantiza el ángulo objetivo al múltiplo de 'paso' más cercano.
					Devuelve el ángulo cuantizado en grados.
					"""
					desplazamiento_valido = round(angulo_objetivo / paso) * paso
					return round(desplazamiento_valido, 4)

				# ---------- MQTT -> ROS (feedback) ----------
				def on_connect(self, client, userdata, flags, rc):
					if rc == 0:
							self.get_logger().info("Conectado al broker MQTT.")
							client.subscribe(self.topic_sub_r1)
							client.subscribe(self.topic_sub_r2)
					else:
							self.get_logger().error(f"Error de conexión MQTT: rc={rc}")

				def on_message(self, client, userdata, msg):
					try:
							data = json.loads(msg.payload.decode("utf-8"))

							# Esperado desde ESP32: q1_mov,q2_mov,q3_mov en GRADOS
							q1 = float(data.get("q1_mov", 0.0))
							q2 = float(data.get("q2_mov", 0.0))
							q3 = float(data.get("q3_mov", 0.0))

							js = JointState()
							js.header.stamp = self.get_clock().now().to_msg()

							if msg.topic == self.topic_sub_r1:
								self.active_ra1 = True
								self.last_mqtt_time_ra1 = self.get_clock().now()

								js.name = ['r1_brazo_joint', 'r1_antebrazo_joint', 'r1_efector_joint']
								js.position = [math.radians(q1), math.radians(q2), math.radians(q3)*0.0225]
								self.publisher_r1.publish(js)

								estado_r1 = UInt8()
								estado_r1.data = int(data.get("estado", 0))
								self.publisher_state_r1.publish(estado_r1)

								self.get_logger().warn(f'[{estado_r1}] estado de movimiento r1')

							elif msg.topic == self.topic_sub_r2:
								self.active_ra2 = True
								self.last_mqtt_time_ra2 = self.get_clock().now()

								js.name = ['r2_brazo_joint', 'r2_antebrazo_joint', 'r2_efector_joint']
								js.position = [math.radians(q1), math.radians(q2), math.radians(q3)*0.0225]
								self.publisher_r2.publish(js)

								estado_r2 = UInt8()
								estado_r2.data = int(data.get("estado", 0))
								self.publisher_state_r2.publish(estado_r2)

								self.get_logger().warn(f'[{estado_r2}] estado de movimiento r2')


					except Exception as e:
							self.get_logger().error(f"Error procesando MQTT: {e}")

				# ---------- Watchdogs ----------
				def check_timeout_ra1(self):
					now = self.get_clock().now()
					if self.active_ra1 and (now - self.last_mqtt_time_ra1 > Duration(seconds=15.0)):
							self.get_logger().warn("RA1: no llegan datos MQTT, se detiene publicación.")
							self.active_ra1 = False

				def check_timeout_ra2(self):
					now = self.get_clock().now()
					if self.active_ra2 and (now - self.last_mqtt_time_ra2 > Duration(seconds=2.0)):
							self.get_logger().warn("RA2: no llegan datos MQTT, se detiene publicación.")
							self.active_ra2 = False


			def main(args=None):
				rclpy.init(args=args)
				node = MQTTBridge()
				try:
					rclpy.spin(node)
				except KeyboardInterrupt:
					pass
				finally:
					node.destroy_node()
					rclpy.shutdown()

	.. group-tab:: Lauch

		.. code:: python
	
			from launch import LaunchDescription

			from launch_ros.actions import Node
			from launch_ros.substitutions import FindPackageShare
			from launch_ros.parameter_descriptions import ParameterValue

			from launch.substitutions import Command
			from launch.substitutions import PathJoinSubstitution


			def generate_launch_description():

				# Robot 1 y Robot 2 iguales
				xacro_file = PathJoinSubstitution([
					FindPackageShare("mi_pkg_python"),
					"urdf_2",
					"ensamblaje.urdf.xacro"
				])

				robot1_description = ParameterValue(
					Command([
							"ros2 run xacro xacro ",
							xacro_file,
							" prefix:=r1_"
					]),
					value_type=str
				)

				robot2_description = ParameterValue(
					Command([
							"ros2 run xacro xacro ",
							xacro_file,
							" prefix:=r2_"
					]),
					value_type=str
				)

				# Robot 3 CR10 A

				xacro_file_r3 = PathJoinSubstitution([
					FindPackageShare("mi_pkg_python"),
					"urdf_3",
					"CR10A_2.urdf.xacro"
				])

				robot3_description = ParameterValue(
					Command([
							"ros2 run xacro xacro ",
							xacro_file_r3,
							" prefix:=r3_"
					]),
					value_type=str
				)

				# =========================
				# Robot State Publishers
				# =========================
				robot_state_publisher_r1 = Node(
					package="robot_state_publisher",
					executable="robot_state_publisher",
					name="robot_state_publisher",
					namespace="r1",
					output="screen",
					parameters=[
							{"robot_description": robot1_description},
							{"publish_robot_description": True}
					],
					remappings=[
							("/joint_states", "joint_states"),
							("/robot_description", "robot_description")
					]
				)

				robot_state_publisher_r2 = Node(
					package="robot_state_publisher",
					executable="robot_state_publisher",
					name="robot_state_publisher",
					namespace="r2",
					output="screen",
					parameters=[
							{"robot_description": robot2_description},
							{"publish_robot_description": True}
					],
					remappings=[
							("/joint_states", "joint_states"),
							("/robot_description", "robot_description")
					]
				)

				robot_state_publisher_r3 = Node(
					package="robot_state_publisher",
					executable="robot_state_publisher",
					name="robot_state_publisher",
					namespace="r3",
					output="screen",
					parameters=[
							{"robot_description": robot3_description},
							{"publish_robot_description": True}
					],
					remappings=[
							("/joint_states", "joint_states"),
							("/robot_description", "robot_description")
					]
				)

				# Joint State Publishers 
				joint_state_publisher_r1 = Node(
					package="joint_state_publisher",
					executable="joint_state_publisher",
					name="joint_state_publisher",
					namespace="r1",
					output="screen",
					parameters=[
							{"robot_description": robot1_description}
					],
					remappings=[
							("/robot_description", "robot_description"),
							("/joint_states", "joint_states")
							]
					)

				joint_state_publisher_r2 = Node(
					package="joint_state_publisher",
					executable="joint_state_publisher",
					name="joint_state_publisher",
					namespace="r2",
					output="screen",
					parameters=[
							{"robot_description": robot2_description}
					],
					remappings=[
							("/robot_description", "robot_description"),
							("/joint_states", "joint_states")
					]
				)

				joint_state_publisher_r3 = Node(
					package="joint_state_publisher",
					executable="joint_state_publisher",
					name="joint_state_publisher",
					namespace="r3",
					output="screen",
					parameters=[
							{"robot_description": robot3_description}
					],
					remappings=[
							("/robot_description", "robot_description"),
							("/joint_states", "joint_states")
					]
				)

				# Static TF al mundo

				static_tf_r1 = Node(
					package="tf2_ros",
					executable="static_transform_publisher",
					name="static_tf_r1",
					output="screen",
					arguments=[
							"0.1", "0.1", "0",
							"0", "0", "0",
							"world",
							"r1_base_link"
					]
				)

				static_tf_r2 = Node(
					package="tf2_ros",
					executable="static_transform_publisher",
					name="static_tf_r2",
					output="screen",
					arguments=[
							"1.0", "0", "0",
							"0", "0", "0",
							"world",
							"r2_base_link"
					]
				)

				static_tf_r3 = Node(
					package="tf2_ros",
					executable="static_transform_publisher",
					name="static_tf_r3",
					output="screen",
					arguments=[
							"0.5", "1.0", "0",
							"0", "0", "0",
							"world",
							"r3_base_link"
					]
				)

				# =========================
				# RViz2
				# =========================
				rviz2 = Node(
					package="rviz2",
					executable="rviz2",
					name="rviz2",
					output="screen"
				)

				return LaunchDescription([
					robot_state_publisher_r1,
					robot_state_publisher_r2,
					robot_state_publisher_r3,

					joint_state_publisher_r1,
					joint_state_publisher_r2,
					joint_state_publisher_r3,

					static_tf_r1,
					static_tf_r2,
					static_tf_r3,

					rviz2
				])

Comandos para el control
------------------------

Arranque del sistema:

.. code:: bash

	ros2 topic pub /task_request std_msgs/msg/String "{data: 'sis_a 0B 7'}" -1

Control del robot móvil:

.. code:: bash

	ros2 topic pub /rm_sis_a/motion_state std_msgs/msg/UInt8 "{data: 1}" -1
	ros2 topic pub /rm_sis_a/motion_state std_msgs/msg/UInt8 "{data: 0}" -1


Control del robot articular:

.. code:: bash

	ros2 topic pub /ra1/motion_state std_msgs/msg/UInt8 "{data: 1}" -1
	ros2 topic pub /ra1/motion_state std_msgs/msg/UInt8 "{data: 0}" -1

Agregar recuperación de errores
-------------------------------

La versión inicial retorna ``FAILURE`` si un movimiento supera el timeout. Luego puedes agregar un árbol de recuperación.

Estructura conceptual:

.. code-block:: text

   Selector: RootWithRecovery
   ├── Sequence: MainTask
   │   ├── WaitStart
   │   ├── MoveHome
   │   ├── MovePick
   │   ├── EffectorOn
   │   ├── MovePlace
   │   └── EffectorOff
   └── Sequence: Recovery
       ├── EffectorOff
       └── MoveHome

Interpretación:

- Primero intenta la tarea principal.
- Si la tarea principal falla, ejecuta recuperación.
- La recuperación apaga el efector y manda el robot a Home.

Ejemplo parcial de código:

.. code-block:: python

   root = py_trees.composites.Selector(
       name=f"RootWithRecovery_{self.robot_ns}",
       memory=False
   )

   main = py_trees.composites.Sequence(
       name=f"MainTask_{self.robot_ns}",
       memory=True
   )

   recovery = py_trees.composites.Sequence(
       name=f"Recovery_{self.robot_ns}",
       memory=True
   )

   main.add_children([
       WaitStart(...),
       MoveJointGoal(...),
       MoveJointGoal(...),
       EffectorCommand(...),
       MoveJointGoal(...),
       EffectorCommand(...),
   ])

   recovery.add_children([
       EffectorCommand(
           name=f"RecoveryEffectorOff_{self.robot_ns}",
           node=self,
           pub_effector=self.pub_effector,
           on=False,
           settle_sec=0.2
       ),
       MoveJointGoal(
           name=f"RecoveryMoveHome_{self.robot_ns}",
           node=self,
           pub_goal=self.pub_goal,
           robot_status=self.robot_status,
           target=t["home"],
           timeout_sec=10.0
       ),
   ])

   root.add_children([main, recovery])

Agregar reintentos
------------------

Puedes envolver un movimiento con ``Retry``.

Ejemplo:

.. code-block:: python

   move_pick = MoveJointGoal(
       name=f"MovePick_{self.robot_ns}",
       node=self,
       pub_goal=self.pub_goal,
       robot_status=self.robot_status,
       target=t["pick"],
       timeout_sec=10.0
   )

   move_pick_retry = py_trees.decorators.Retry(
       name=f"RetryMovePick_{self.robot_ns}",
       child=move_pick,
       num_failures=2
   )

Luego en la secuencia agregas ``move_pick_retry`` en vez de ``move_pick``.

Agregar condición de pieza
--------------------------

Si más adelante agregas un sensor de presencia o visión artificial, puedes añadir un nodo de condición.

Tópico sugerido:

.. code-block:: text

   /raX/piece_detected

Tipo:

.. code-block:: text

   std_msgs/msg/Bool

Árbol modificado:

.. code-block:: text

   Sequence
   ├── WaitStart
   ├── MoveHome
   ├── MovePick
   ├── CheckPieceDetected
   ├── EffectorOn
   ├── MovePlace
   └── EffectorOff

Ejemplo conceptual:

.. code-block:: python

   class CheckPieceDetected(py_trees.behaviour.Behaviour):
       def __init__(self, name, robot_status):
           super().__init__(name=name)
           self.robot_status = robot_status

       def update(self):
           if self.robot_status.get("piece_detected", False):
               return py_trees.common.Status.SUCCESS
           return py_trees.common.Status.FAILURE
