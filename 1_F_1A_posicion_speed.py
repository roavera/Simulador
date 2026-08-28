from collections import deque
import math
import time
import csv
import os
import logging
from xmlrpc import client
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

#=========================================================
# PARAMETROS
#=========================================================

# Conversión de unidades
FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444    # 1 nudo = 0.514444 m/s
METERS_TO_FT = 1 / FT_TO_METERS

# =========================================================
# CONDICIONES INICIALES
# =========================================================

LAT = -34.554          # grados, se lo puede ajustar para iniciar en un lugar específico del mapa
LON = -58.425          # grados, se lo puede ajustar para iniciar en un lugar específico del mapa
ALTITUDE = 4500        # ft - altitud que se desea mantener durante el vuelo nivelado
INITIAL_SPEED = 50     # kts - airspeed

FINAL_SPEED = 120      # kts - airspeed
VERTICAL_SPEED = 0     # ft/min - vertical speed 
SPEED_ROLL = 0         # kts - velocidad a la que el helicóptero comienza a inclinarse 

ACCELERATION_RATE = 0.5 # kts por iteración - velocidad de incremento de la velocidad
SAMPLE_TIME = 0.05      # Tiempo de muestreo para el controlador (20 Hz)

# =========================================================
# DARAREFS
# =========================================================
DATAREFS = [
    'sim/flightmodel/position/latitude',                  # 0: latitud [°]
    'sim/flightmodel/position/longitude',                 # 1: longitud [°]
    'sim/flightmodel/position/y_agl',                      # 2: altitud [m]
    'sim/cockpit2/gauges/indicators/airspeed_kts_pilot'    # 3: velocidad [kt]
]

def ramp_speed(client):
    current_speed = INITIAL_SPEED

    while current_speed < FINAL_SPEED:
        vx = current_speed * KTS_TO_METERS_PER_SEC  # Convertir a m/s
        client.sendDREF("sim/flightmodel/position/local_vx", vx)

        client.sendDREF("sim/flightmodel/position/local_vy", 0)
        client.sendDREF("sim/flightmodel/position/local_vz", 0)

        #Incremento suave de la velocidad 
        current_speed += ACCELERATION_RATE  # Incremento de 0.5 kts por iteración

        #Tiempo de muestreo para el controlador (20 Hz)
        time.sleep(SAMPLE_TIME)

def main():
    with xpc.XPlaneConnect() as client:

        values = [LAT, LON, ALTITUDE , 0, 0, 0] # lat, lon, alt, pitch, roll, heading 
        client.sendPOSI(values, 0)

        time.sleep(2) # Esperar a que el simulador estabilice la posición

        vx = INITIAL_SPEED * KTS_TO_METERS_PER_SEC  # Velocidad inicial en m/s

        # Enviar la velocidad inicial al simulador    
        client.sendDREF("sim/flightmodel/position/local_vx", vx)

        ramp_speed(client)
        

if __name__ == "__main__":
    main()
    