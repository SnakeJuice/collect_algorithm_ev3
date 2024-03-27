#!/usr/bin/env pybricks-micropython
##########################################################
# Algoritmo de recolección y evasión para robot LEGO EV3 #
#                                                        #
# Autores:                                               #
# - Cristian Anjari                                      #
# - Marcos Medina                                        #
#                                                        #
# Para proyecto de Tesis 2024                            #
#                                                        #
# Universidad de Santiago de Chile                       #
# Facultad de Ciencia                                    #
#                                                        #
# Licenciatura en Ciencia de la Computación/             #
# Analista en Computación Científica                     #
#                                                        #
# Santiago, Chile                                        #
# 25/03/2024                                             #
##########################################################

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.messaging import BluetoothMailboxClient, TextMailbox
import itertools
import math
###################################################################################

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
servo_motor = Motor(Port.C)

# SETTINGS RECOLECTOR
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=140)
robot.settings(straight_speed = 50, straight_acceleration = 100, turn_rate= 50, turn_acceleration = 50)

"""
    Convierte coordenadas en formato string en una lista de tuplas.
    
    Parámetros:
        string (str): El string de coordenadas convertir.
        is_obstacle (bool): Si las coordenadas representan un obstáculo.
        
    Devuelve:
        coords: Una lista de tuplas que representan las coordenadas.
"""
def string_to_coordinates(string, is_obstacle=False):
    string = string[1:-1]
    parts = string.split("), (")
    coords = []
    for part in parts:
        x, y = part.split(", ")
        x = int(x.strip("("))
        y = int(y.strip(")"))
        if is_obstacle:
            x = abs(x - 10)
        coords.append((x, y))
    return coords
#################################################

"""
    Expande un obstáculo en todas las direcciones por un margen dado.
    
    Parámetros:
        obstacle (tuple): Las coordenadas del obstáculo.
        margin (int): El margen de expansión.
        
    Devuelve:
        expanded: Una lista de tuplas que representan las coordenadas del obstáculo expandido.
"""
def expand_obstacle(obstacle, margin):
    x, y = obstacle
    expanded = []
    for dx in range(-margin, margin+1):
        for dy in range(-margin, margin+1):
            expanded.append((x+dx, y+dy))
    return expanded
#################################################

"""
    Expande todos los obstáculos en una lista por un margen dado.
    
    Parámetros:
        obstacles (list): Una lista de tuplas que representan los obstáculos.
        margin (int): El margen de expansión.
        
    Devuelve:
        expanded: Una lista de tuplas que representan las coordenadas de los obstáculos expandidos.
"""
def expand_obstacles(obstacles, margin):
    expanded = []
    for obstacle in obstacles:
        expanded.extend(expand_obstacle(obstacle, margin))
    return expanded
#################################################

"""
    Calcula la distancia de Manhattan entre dos puntos.
    
    Parámetros:
        point1 (tuple): El primer punto.
        point2 (tuple): El segundo punto.
        
    Devuelve:
        int: La distancia de Manhattan entre los dos puntos.
"""
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
#################################################

"""
    Obtiene los vecinos de un nodo en una cuadrícula.
    
    Parámetros:
        node (tuple): Las coordenadas del nodo.
        
    Devuelve:
        neighbors: Una lista de tuplas que representan las coordenadas de los vecinos.
"""
def get_neighbors(node):
    x, y = node
    # Vecinos a la izquierda, derecha, arriba y abajo
    neighbors = [(x-1, y), (x+1, y), (x, y-17), (x, y+17)]
    # Elimina los vecinos que están por debajo de y=17
    neighbors = [(x, y) for x, y in neighbors if y >= 17]
    return neighbors
#################################################

"""
    Implementa el algoritmo A* para encontrar el camino más corto entre dos puntos en una cuadrícula con obstáculos.
    
    Parámetros:
        start (tuple): Las coordenadas del punto de inicio.
        goal (tuple): Las coordenadas del punto de destino.
        obstacles (list): Una lista de tuplas que representan los obstáculos.
        
    Devuelve:
        list: Una lista de tuplas que representan el camino más corto, o None si no se encontró un camino.
"""
# VERIFICAR COMPLEJIDAD.
def a_star(start, goal, obstacles):
    # Inicializa las listas abierta y cerrada
    open_list = [start]
    closed_list = []

    # Inicializa el mapa de costos
    g = {start: 0}
    h = {start: distance(start, goal)}
    f = {start: h[start]}

    # Inicializa el mapa de nodos anteriores
    came_from = {}

    # Mientras la lista abierta no esté vacía
    while open_list:
        # Encuentra el nodo con el menor valor de f en la lista abierta
        current = min(open_list, key=lambda node: f[node])

        # Si el nodo actual es el objetivo, reconstruye y devuelve el camino
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(current)
            return path[::-1]

        # Elimina el nodo actual de la lista abierta y lo agrega a la lista cerrada
        open_list.remove(current)
        closed_list.append(current)

        # Para cada vecino del nodo actual
        for neighbor in get_neighbors(current):
            # Si el vecino está en la lista cerrada o es un obstáculo, ignóralo
            if neighbor in closed_list or neighbor in obstacles:
                continue

            # Calcula el costo tentativo del vecino a través del nodo actual
            dx, dy = neighbor[0] - current[0], neighbor[1] - current[1]
            tentative_g = g[current] + math.sqrt(dx**2 + (dy/17)**2)  # El costo de moverse a un vecino

            # Si el vecino no está en la lista abierta o el costo tentativo es menor que el costo actual
            # actualiza el mapa de costos y agrega el vecino a la lista abierta
            if neighbor not in open_list or tentative_g < g[neighbor]:
                came_from[neighbor] = current
                g[neighbor] = tentative_g
                h[neighbor] = distance(neighbor, goal)
                f[neighbor] = g[neighbor] + h[neighbor]
                if neighbor not in open_list:
                    open_list.append(neighbor)
                #print("Vecino agregado a la lista abierta: ", neighbor)

    # Si no se encontró un camino, devuelve None
    return None
#################################################

"""
    Implementa el algoritmo 2-opt para mejorar una ruta.
    
    Parámetros:
        coordinates (list): Una lista de tuplas que representan la ruta.
        
    Devuelve:
        list: Una lista de tuplas que representan la ruta optimizada.
"""
# Complejidad O(n^2)
# n = numero de coordenadas
def two_opt(coordinates):
    # Calcula la matriz de distancias
    n = len(coordinates)
    dist = [[distance(coordinates[i], coordinates[j]) for j in range(n)] for i in range(n)]

    # Inicializa la ruta con el orden de las coordenadas
    route = list(range(n))
    
    improvement = True
    while improvement:
        improvement = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                # Si intercambiar las "ciudades" i y j resulta en una ruta más corta, haz el intercambio
                if dist[route[i-1]][route[j]] + dist[route[i]][route[(j+1)%n]] < dist[route[i-1]][route[i]] + dist[route[j]][route[(j+1)%n]]:
                    route[i:j+1] = list(reversed(route[i:j+1]))
                    improvement = True

    # Devuelve las coordenadas en el orden de la ruta óptima
    return [coordinates[i] for i in route]
#################################################

"""
    Implementa un algoritmo codicioso para moverse a lo largo de un camino en una cuadrícula con obstáculos.
    
    Parámetros:
        coordinates (list): Una lista de tuplas que representan el camino.
        obstacles (list): Una lista de tuplas que representan los obstáculos.
        
    Devuelve:
        list: Una lista de tuplas que representan el camino seguido, o None si no se encontró un camino.
"""
# Complejidad O(n^3 * m) VERIFICAR SI ESTA CORRECTO
# n = coordenadas, m = longitud promedio del camino de A*
def move_along_path_greedy(coordinates, obstacles):
    start_position = (0,17)
    current_position = start_position
    full_path = []
    visited_coordinates = [current_position]
    
    # Ordena las coordenadas con el algoritmo de 2-opt
    coordinates = two_opt(coordinates)
    
    # Direcciones posibles
    RIGHT = 0 # Derecha
    UP = 1  # Arriba
    LEFT = 2 # Izquierda
    DOWN = 3 # Abajo
    
    # Inicializa la dirección actual del robot
    current_direction = RIGHT

    while len(visited_coordinates) < len(coordinates) + 1:
        # Encuentra la coordenada no visitada más cercana
        unvisited = [coord for coord in coordinates if coord not in visited_coordinates]
        closest = min(unvisited, key=lambda coord: distance(current_position, coord))

        # Encuentra el camino más corto a la coordenada más cercana
        path = a_star(current_position, closest, obstacles)
        if path is None:
            print("No se encontró un camino a ",{closest[0]},{closest[1]})
            return
        full_path.extend(path)
        current_position = closest
        visited_coordinates.append(closest)

        print("Camino a : {}, {}".format(closest[0], closest[1]))
        print(path)
        print('\n')

        #Abre la reja
        servo_motor.run_angle(150, 50)

        # Para cada paso en el camino
        for i in range(len(path) - 1):
            # Obtiene las coordenadas del paso actual y el próximo paso
            current_x, current_y = path[i]
            next_x, next_y = path[i + 1]
            
            # Inicializamos la próxima dirección a None
            next_direction = None
            
            # Calculamos la diferencia en X y Y entre el paso actual y el próximo paso
            dx = next_x - current_x
            dy = next_y - current_y
            
            # Si dx y dy son ambos cero, continuamos con la siguiente iteración del bucle
            if dx == 0 and dy == 0:
                continue
            
            # Dependiendo de la diferencia, establecemos la próxima dirección
            if dx > 0:
                next_direction = RIGHT
            elif dx < 0:
                next_direction = LEFT
            elif dy > 0:
                next_direction = UP
            elif dy < 0:
                next_direction = DOWN
            #print("Próxima dirección: ", next_direction)

            # Calculamos el ángulo de giro comparando la próxima dirección con la dirección actual del robot
            turn_angle = (next_direction - current_direction) % 4
            # Si en el extraño caso de que el ángulo de giro sea 2, el robot debe dar la vuelta
            if turn_angle == 2:
                # Da la vuelta
                robot.turn(180)
                wait(10000)
            # Si el ángulo de giro es 1, el robot debe girar a la izquierda
            if turn_angle == 1:
                # Gira a la izquierda
                robot.turn(-90)
                wait(10000)
            # Si el ángulo de giro es 3, el robot debe girar a la derecha
            elif turn_angle == 3:
                # Gira a la derecha
                robot.turn(90)
                wait(10000)
            
            '''
            # Si el robot debe subir o bajar en el eje Y
            # Se le da un pequeño empujon
            if current_x == next_x and current_y != next_y:
                dist = math.ceil(dy / 17) * 30
                #print("Dist: ",dist)
                robot.straight(abs(dist))
            '''
        
            # Actualiza la dirección actual del robot a la próxima dirección
            current_direction = next_direction
            
            # Calculamos la distancia al próximo paso
            step_distance = math.sqrt(dx**2 + dy**2) * 10  # cada unidad es mm
            
            # Movemos el robot la distancia calculada
            robot.straight(step_distance)
            
        # Luego de llegar a la coordenada objetivo se espera 1 segundo
        wait(1000)
        #Cierra la reja
        servo_motor.run_angle(150, -50)

    # Al finalizar el recorrido que vaya a la posición (9,136)
    #path = a_star(current_position, (9,136), obstacles)
    #if path is None:
    #    print("No se encontró un camino a (9,136)")
    #    return
    #full_path.extend(path)

    return full_path
#################################################

#################################################
##----------------- main ----------------------##
#################################################

SERVER = 'Master'

client = BluetoothMailboxClient()

rbox = TextMailbox('rec1', client)

print('establishing connection...')
client.connect(SERVER)
ev3.screen.print('connected!')

while True:
    rbox.send('Recolector1 conectado')
    rbox.wait()
    #rbox.read()
    #print("rbox.read ",rbox.read())
    if(rbox.read()[0] == '['):
        rojos = rbox.read()
        ev3.screen.print("lista roja",rojos)
    rbox.send('rojo ok')
    rbox.wait_new()
    if(rbox.read()[0] == '['):
        verdes = rbox.read()
        ev3.screen.print("lista verde",verdes)
    rbox.send('verde ok')
    rbox.wait_new()    

    if(rbox.read()=='Recolector1 muevete'):
        #robot.straight(100)
        
        coordinates = string_to_coordinates(rojos)
        obstacles = string_to_coordinates(verdes)

        print(coordinates)
        print(obstacles)
        print('\n')

        margin = 15
        expanded_obstacles = expand_obstacles(obstacles, margin)
        
        wait(5000)
        ev3.speaker.beep()
        wait(1000)
        full_path = move_along_path_greedy(coordinates, expanded_obstacles)
        #print(full_path)
    rbox.send('termine')