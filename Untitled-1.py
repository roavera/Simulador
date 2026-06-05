# ========================= 
# 1_F_1A_1_Vuelo Nivelado.py
# Crucero
# Validar el desempeño a velocidades por encima de la velocidad de maxima resistencia. Determinar la performance durante un vuelo en crucero.

# Condiciones Iniciales: 
    # Altitud:      4500 ft
    # Velocidad:    50 kts

# Condiciones finales:       
    # Altitud:      4500 ft
    # Velocidad:    120 kts
# =========================

from collections import deque
import math
import time
import csv
import os
import logging
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

LAT = -34.554          # grados
LON = -58.425          # grados
ALTITUDE = 4500        # ft - altitud que se desea mantener durante el vuelo nivelado
INITIAL_SPEED = 50     # kts - airspeed

FINAL_SPEED = 120      # kts - airspeed
VERTICAL_SPEED = 0     # ft/min - vertical speed 
SPEED_ROLL = 0         # kts - velocidad a la que el helicóptero comienza a inclinarse 


# =========================================================
# CONTROLADOR 
# =========================================================

class FlightController:
    def __init__(self, client):
        self.client = client
        self.flight_phase = 'cruise'

        # Inicia el set point de velocidad en la velocidad inicial para evitar comandos bruscos al inicio del vuelo
        self.current_target_speed = INITIAL_SPEED

        # Inicializar PIDs con parámetros por defecto
        self._init_pids()

        # Configurar datarefs
        self.datarefs = [
            'sim/flightmodel/position/y_agl',                                   # 0: altitud    [m]
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot',    # 1: heading    [grados magnéticos]
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',                # 2: airspeed   [knots]
            'sim/flightmodel/position/local_vx',                                # 3: v_lat      [m/s]
            'sim/flightmodel/position/local_vy',                                # 4: climb_rate [m/s]
            'sim/flightmodel/position/local_vz',                                # 5: v_long     [m/s]
            'sim/flightmodel/position/local_x',                                 # 6: pos_x      [m]
            'sim/flightmodel/position/local_y',                                 # 7: pos_y      [m]
            'sim/flightmodel/position/local_z',                                 # 8: pos_z      [m]
            'sim/flightmodel/position/theta',                                   # 9: pitch      [grados]
            'sim/flightmodel/position/phi',                                     # 10: roll      [grados]
            'sim/flightmodel/position/psi',                                     # 11: yaw       [grados]
            'sim/flightmodel/position/latitude',                                # 12: lat       [grados]
            'sim/flightmodel/position/longitude',                               # 13: lon       [grados]
            'sim/cockpit2/engine/actuators/prop_angle_degrees'                  # 14: collective[grados]
        ]

    # =========================================================
    # INICIALIZACIÓN DE PIDs
    # =========================================================

    def _init_pids(self):
        # PID para velocidad vertical (climb rate)
        self.pid_collective = PID(1.0, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-2.5, 2.5)

        # PID para pitch (theta) - cabeceo
        self.pid_pitch = PID(0.08, 0.0001, 0.03, setpoint=INITIAL_SPEED)  
        self.pid_pitch.output_limits = (-0.25 , 0.3)             
        
        # PID para roll (phi) - Rolido
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=SPEED_ROLL)
        self.pid_roll.output_limits = (-0.7,0.7)
        
        # PID de yaw (psi) - pedales
        self.pid_yaw = PID(0.04, 0.0, 0.001, setpoint=0)  
        self.pid_yaw.output_limits = (-1.0,1.0)

# =============================================
    # CONTROL COLLECTIVE (ACTUALIZADO)
    # =============================================
    def calculate_collective_control(self, target_altitude, current_altitude, climb_rate, current_collective, pitch_actual):
        # 1. Mayor agresividad en el lazo exterior de altitud
        K_p_alt = 0.5 # Aumentado desde 0.1 para una respuesta más rápida
        error_alt = K_p_alt * (target_altitude - current_altitude)  
        
        # Ampliamos ligeramente el límite de tasa vertical para permitir correcciones fuertes
        MAX_CLIMB_RATE = 8.0 
        w_ref = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, error_alt))
        
        # Lazo interno: Aplicar PID al climb rate actual
        self.pid_collective.setpoint = w_ref
        collective_change = self.pid_collective(climb_rate)
        
        # Calcular nuevo valor base
        new_collective = current_collective + collective_change

        # 2. ACOPLAMIENTO (Feedforward) basado en el ESTADO FÍSICO REAL (Theta)
        # En X-Plane, un theta negativo indica morro abajo.
        if pitch_actual < 0:
            # Ganancia empírica de acoplamiento referenciada a los grados de inclinación.
            # Ej: Si el helicóptero se inclina -8 grados, inyecta +1.6 grados extra de colectivo inmediato.
            K_feedforward = 0.20 
            compensation_pitch = abs(pitch_actual) * K_feedforward
            new_collective += compensation_pitch  
        
        # Limitar rango físico del collective (-4 a 11 grados en un rotor típico)
        new_collective = max(-4.0, min(11.0, new_collective))
        return new_collective

    # =============================================
    # CONTROL DE ÁNGULOS
    # ============================================= 

    def calculate_pitch_control(self, target_speed, current_speed):
        self.pid_pitch.setpoint = target_speed
        return self.pid_pitch(current_speed)
    
    def calculate_roll_control(self, target_roll, current_roll):
        self.pid_roll.setpoint = target_roll
        return self.pid_roll(current_roll)
    
    def calculate_yaw_control(self, target_heading, current_heading):
        # Distancia angular más corta en un círculo de 360 grados
        error_heading = (target_heading - current_heading + 360) % 360
        if error_heading > 180:
            error_heading -= 360
        
        self.pid_yaw.setpoint = 0
        return self.pid_yaw(-error_heading)

    # =============================================
    # BUCLE PRINCIPAL
    # =============================================

    def run(self, target_altitude):
        # Capturar heading de referencia al inicio del vuelo
        data = self.client.getDREFs(self.datarefs) 
        heading_ref = float(data[1][0])  # Heading magnético inicial

        # Convertir altitud objetivo a metros para cálculos internos
        target_altitude_m = target_altitude * FT_TO_METERS
        
        while True:
            # Leer datos del simulador
            data = self.client.getDREFs(self.datarefs)
            altitude_agl        = float(data[0][0])
            heading             = float(data[1][0])
            airspeed            = float(data[2][0])
            climb_rate          = float(data[4][0])
            pitch_actual        = float(data[9][0])
            roll_actual         = float(data[10][0])
            current_collective  = float(data[14][0]) 

            # Gestión de la RAMPA de velocidad (Con mitigación de Windup)
            error_velocidad = self.current_target_speed - airspeed
            if self.current_target_speed < FINAL_SPEED:
                if error_velocidad < 3.0: # Solo avanza la rampa si la física del helicóptero "sigue el ritmo"
                    self.current_target_speed += 0.02  
                if self.current_target_speed > FINAL_SPEED:
                    self.current_target_speed = FINAL_SPEED

            # Cálculo del Pitch (Inversión de signo aplicada para bajar morro)
            pitch_pid_out = self.calculate_pitch_control(self.current_target_speed, airspeed)
            pitch_cmd = -pitch_pid_out 

            # Cálculo del Colectivo (Pasando correctamente los parámetros sin 'self' extra y sumando 'pitch_cmd')
            collective_cmd = self.calculate_collective_control(
                target_altitude_m, altitude_agl, climb_rate, current_collective, pitch_cmd
            )
            
            # Cálculo de Roll (Alas niveladas en 0 grados)
            roll_cmd = self.calculate_roll_control(0.0, roll_actual)
            
            # Cálculo de Yaw (Mantener rumbo corrigiendo discontinuidad)
            yaw_cmd = self.calculate_yaw_control(heading_ref, heading)

            # Enviar comandos al simulador limitados de forma segura
            self.client.sendDREF("sim/cockpit2/controls/yoke_pitch_ratio", max(-0.7, min(0.7, pitch_cmd)))
            self.client.sendDREF("sim/cockpit2/controls/yoke_roll_ratio", max(-0.7, min(0.7, roll_cmd)))
            self.client.sendDREF("sim/cockpit2/controls/yoke_heading_ratio", max(-1.0, min(1.0, yaw_cmd)))
            self.client.sendDREF("sim/cockpit2/engine/actuators/prop_angle_degrees", collective_cmd)

            # Control de frecuencia: pausa de 20 ms (50 Hz) para que los PID operen de forma estable
            time.sleep(0.02)

# =========================================================
# CONEXIÓN CON X-PLANE
# =========================================================
if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        
        # Posición inicial 
        values = [LAT, LON, ALTITUDE , 0, 0, 0] # lat, lon, alt, pitch, roll, heading
        client.sendPOSI(values, 0)
        # Activar el override de IAS 
        client.sendDREF("sim/operation/override/override_ias", 1)
        # Forzar la velocidad inicial
        client.sendDREF("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", INITIAL_SPEED)
        time.sleep(2) # Esperar a que el simulador estabilice la posición

        # Desactivar el override IAS 
        client.sendDREF("sim/operation/override/override_ias", 0)

        controller = FlightController(client)
        controller.run(ALTITUDE)