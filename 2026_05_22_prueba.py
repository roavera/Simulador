from collections import deque
import math
import time
import csv
import os
import logging
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =========================================================
# PARAMETROS Y CONVERSIONES
# =========================================================

FT_TO_METERS = 0.3048
KTS_TO_MPS = 0.514444         
FPM_TO_MPS = 0.3048 / 60      # Factor para ft/min a m/s

# =========================================================
# CONDICIONES INICIALES
# =========================================================

LAT = -34.554          
LON = -58.425          
ALTITUDE = 4500        # ft 
INITIAL_SPEED = 50     # kts 
FINAL_SPEED = 120      # kts 
SPEED_ROLL = 0         # grados 
SAMPLE_RATE = 0.05     # Segundos (20 Hz)

# =========================================================
# CONTROLADOR 
# =========================================================

class FlightController:
    def __init__(self, client):
        self.client = client
        self.flight_phase = 'cruise'
        self._init_pids()

        self.datarefs = [
            'sim/flightmodel/position/y_agl',                                   # 0: altitud [m]
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot',    # 1: heading [grados]
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',                # 2: airspeed [kts]
            'sim/flightmodel/position/local_vx',                                # 3: v_lat [m/s]
            'sim/flightmodel/position/local_vy',                                # 4: climb_rate [m/s]
            'sim/flightmodel/position/local_vz',                                # 5: v_long [m/s]
            'sim/flightmodel/position/local_x',                                 # 6: pos_x [m]
            'sim/flightmodel/position/local_y',                                 # 7: pos_y [m]
            'sim/flightmodel/position/local_z',                                 # 8: pos_z [m]
            'sim/flightmodel/position/theta',                                   # 9: pitch [grados]
            'sim/flightmodel/position/phi',                                     # 10: roll [grados]
            'sim/flightmodel/position/psi',                                     # 11: yaw [grados]
            'sim/flightmodel/position/latitude',                                # 12: lat [grados]
            'sim/flightmodel/position/longitude',                               # 13: lon [grados]
            'sim/cockpit2/engine/actuators/prop_angle_degrees'                  # 14: collective [grados]
        ]

    def _init_pids(self):
        # COLLECTIVE (Altitud)
        self.pid_collective = PID(1.0, 0.01, 0.1, setpoint=0, sample_time=SAMPLE_RATE)
        self.pid_collective.output_limits = (-3.0, 3.0)

        # PITCH (Velocidad Longitudinal) -> Ganancias NEGATIVAS
        # Si V_target > V_current, se requiere pitch negativo (empujar mando) para acelerar
        self.pid_pitch = PID(-0.05, -0.0001, -0.015, setpoint=FINAL_SPEED, sample_time=SAMPLE_RATE)
        self.pid_pitch.output_limits = (-0.7, 0.7)             
        
        # ROLL (Estabilización Lateral)
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=SPEED_ROLL, sample_time=SAMPLE_RATE)
        self.pid_roll.output_limits = (-0.7, 0.7)
        
        # YAW (Rumbo)
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0, sample_time=SAMPLE_RATE) 
        self.pid_yaw.output_limits = (-1.0, 1.0) 
    
    def calculate_collective_control(self, target_altitude, current_altitude, climb_rate, current_collective):
        MAX_CLIMB_RATE = 100 * FPM_TO_MPS  
        
        error_alt = 0.1 * (target_altitude - current_altitude) 
        w_ref = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, error_alt))
        
        self.pid_collective.setpoint = w_ref
        collective_change = self.pid_collective(climb_rate)
        
        new_collective = current_collective + collective_change
        return max(-4.0, min(11.0, new_collective))
    
    def calculate_pitch_control(self, current_speed):
        return self.pid_pitch(current_speed)
    
    def calculate_roll_control(self, current_roll):
        return self.pid_roll(current_roll)
    
    def calculate_yaw_control(self, target_heading, current_heading):
        # Normalizar el error para lidiar con el cruce circular de los 360 grados
        error = (target_heading - current_heading + 180) % 360 - 180
        self.pid_yaw.setpoint = 0
        return self.pid_yaw(-error)
    
    def run(self, target_altitude_ft):
        print("Iniciando lazo de control PID...")
        data = self.client.getDREFs(self.datarefs)
        heading_ref = data[1][0] 
        target_altitude_m = target_altitude_ft * FT_TO_METERS

        while True:
            loop_start = time.time()

            # Leer datos indexando el primer elemento del arreglo devuelto
            data = self.client.getDREFs(self.datarefs)
            
            # y_agl en X-Plane viene en metros nativamente
            altitude_agl = data[0][0] 
            heading = data[1][0]
            airspeed = data[2][0]
            climb_rate = data[4][0]
            current_roll = data[10][0]
            current_collective = data[14][0]

            # Calcular controles
            collective_cmd = self.calculate_collective_control(target_altitude_m, altitude_agl, climb_rate, current_collective)
            pitch_cmd = self.calculate_pitch_control(airspeed)
            roll_cmd = self.calculate_roll_control(current_roll)
            yaw_cmd = self.calculate_yaw_control(heading_ref, heading)

            # Enviar comandos como escalares puros (sin listas/corchetes)
            self.client.sendDREF("sim/cockpit2/controls/yoke_pitch_ratio", pitch_cmd)
            self.client.sendDREF("sim/cockpit2/controls/yoke_roll_ratio", roll_cmd)
            self.client.sendDREF("sim/cockpit2/controls/yoke_heading_ratio", yaw_cmd)
            self.client.sendDREF("sim/cockpit2/engine/actuators/prop_angle_degrees", collective_cmd)

            # Control de la frecuencia de muestreo del lazo (evita saturar el canal UDP)
            elapsed = time.time() - loop_start
            time.sleep(max(0, SAMPLE_RATE - elapsed))

if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        # Posicionamiento inicial
        values = [LAT, LON, ALTITUDE * FT_TO_METERS, 0, 0, 0] 
        client.sendPOSI(values, 0)
        
        # Override de la velocidad en los instrumentos (escalares puros)
        client.sendDREF("sim/operation/override/override_ias", 1)
        client.sendDREF("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", INITIAL_SPEED)
        time.sleep(2) 
        client.sendDREF("sim/operation/override/override_ias", 0)

        controller = FlightController(client)
        controller.run(ALTITUDE)