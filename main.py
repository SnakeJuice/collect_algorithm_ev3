#!/usr/bin/env pybricks-micropython
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


# Convierte un string a una lista de tuplas
def string_to_coordinates(string):
    string = string[1:-1]
    parts = string.split("), (")
    coords = []
    for part in parts:
        x, y = part.split(", ")
        coords.append((int(x.strip("(")), int(y.strip(")"))))
    return coords


#Mejor heuristica para movimientos en 4 direcciones
def distance(point1, point2):
    # Calcula la distancia de Manhattan entre dos puntos
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

def get_neighbors(node):
    x, y = node
    return [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
    # Vecinos a la izquierda, derecha, arriba y abajo

# Algoritmo A* para encontrar el camino más corto entre dos puntos
# Complejidad O(n^2)
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
            tentative_g = g[current] + 1  # El costo de moverse a un vecino es siempre 1

            # Si el vecino no está en la lista abierta o el costo tentativo es menor que el costo actual
            # actualiza el mapa de costos y agrega el vecino a la lista abierta
            if neighbor not in open_list or tentative_g < g[neighbor]:
                came_from[neighbor] = current
                g[neighbor] = tentative_g
                h[neighbor] = distance(neighbor, goal)
                f[neighbor] = g[neighbor] + h[neighbor]
                if neighbor not in open_list:
                    open_list.append(neighbor)

    # Si no se encontró un camino, devuelve None
    return None

# Algoritmo 2-opt - Heurística de mejora de ruta
# Complejidad O(n^2)
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
                # Si intercambiar las ciudades i y j resulta en una ruta más corta, haz el intercambio
                if dist[route[i-1]][route[j]] + dist[route[i]][route[(j+1)%n]] < dist[route[i-1]][route[i]] + dist[route[j]][route[(j+1)%n]]:
                    route[i:j+1] = reversed(route[i:j+1])
                    improvement = True

    # Devuelve las coordenadas en el orden de la ruta óptima
    return [coordinates[i] for i in route]

# Algoritmo Greedy para moverse a lo largo de un camino
# Complejidad O(n^2)
def move_along_path_greedy(coordinates, obstacles):
    start_position = (0,0)
    current_position = start_position
    full_path = []
    visited_coordinates = [current_position]
    # Ordena las coordenadas con el algoritmo de 2-opt
    coordinates = two_opt(coordinates)

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

        #print(f'Camino a ({closest[0]},{closest[1]}): {path}')
        print('\n')

    # Encuentra el camino de regreso al punto de inicio
    path = a_star(current_position, start_position, obstacles)
    if path is None:
        print("No se encontró un camino de regreso al inicio")
        return
    #full_path.extend(path)

    return full_path


#################################--Main--#####################################################
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=125)
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

        full_path = move_along_path_greedy(coordinates, obstacles)
        print(full_path)
    rbox.send('termine')

   
