#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.messaging import BluetoothMailboxClient, TextMailbox

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

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
    print("rbox.read ",rbox.read())
    if(rbox.read()=='Recolector1 muevete'):
        robot.straight(100)
# Write your program here.
   
