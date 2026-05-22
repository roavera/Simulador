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

# Datarefs de interes para esta prueba:
# sim/flightmodel/engine/ENGN_TRQ			
# sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot			
# sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot			
# sim/cockpit2/controls/yoke_pitch_ratio			
# sim/cockpit2/controls/yoke_roll_ratio			
# sim/cockpit2/controls/yoke_heading_ratio			
# sim/cockpit2/engine/actuators/prop_ratio_all			

# Datarefs para parametros iniciales
# sim/flightmodel/weight/m_fixed		
# sim/flightmodel/weight/m_fuel1		
# sim/cockpit2/gauges/indicators/CG_indicator		

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

LAT = -34.554          # grados, se lo puede ajustar para iniciar en un lugar específico del mapa
LON = -58.425          # grados, se lo puede ajustar para iniciar en un lugar específico del mapa
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

        self.target_speed_setpoint = INITIAL_SPEED

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
        # Controla el collective para mantener la altitud

        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-2.5, 2.5)

        # -------------------------------------------- #
        # ÁNGULOS 

        # PID para pitch (theta) - cabeceo
        # Nariz arriba para aumentar altitud, nariz abajo para descender
        self.pid_pitch = PID(0.05, 0.0001, 0.015, setpoint=FINAL_SPEED)
        self.pid_pitch.output_limits = (-0.7 , 0.7)             
        
        # PID para roll (phi) - Rolido
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=SPEED_ROLL)
        self.pid_roll.output_limits = (-0.7,0.7)
        
        # PID de yaw (psi) - pedales
        # Controla el rotor de cola
        # contrarresta la inercia y el torque del rotor
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0)  # Parámetros optimizados
        self.pid_yaw.output_limits = (-5,5 )

        # -------------------------------------------- #
    
    # =============================================
    # CONTROL COLLECTIVE
    # =============================================
    # Velocidad vertical (climb rate) -> Collective
    # Control suave del collective con limitaciones dinámicas para evitar cambios bruscos que puedan desestabilizar el vuelo.
    # Args:
    #        target_altitude_m: Altitud objetivo en metros
    #        current_altitude_m: Altitud actual en metros
    #        climb_rate: Velocidad vertical actual en m/s
    #        current_collective: Ángulo actual del collective en grados
    # Returns:
    #        new_collective: Nuevo ángulo del collective en grados, limitado a un rango seguro
    def calculate_collective_control(self, target_altitude, current_altitude, climb_rate,current_collective):
        
        # Limitar la referencia de velocidad vertical para evitar comandos extremos
        # Evita ascensos violentos e inestabilidad
        MAX_CLIMB_RATE = 100 * KTS_TO_METERS_PER_SEC  # 51.444 m/s
        
        # Error de altitud (suavizado para evitar cambios bruscos)
        # Movimientos mas suaves, estables y lentos para evitar oscilaciones 
        error_alt = 0.1 * (target_altitude - current_altitude)  # Factor de ganancia reducido
        
        # Limitar la referencia de velocidad vertical
        w_ref = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, error_alt))
        
        # Aplicar PID al climb rate actual
        self.pid_collective.setpoint = w_ref
        collective_change = self.pid_collective(climb_rate)
        
        # Calcular nuevo valor con cambio incremental (suavizado)
        new_collective = current_collective + collective_change
        
        # Limitar rango físico del collective (-4 a 11 grados)
        new_collective = max(-4.0, min(11.0, new_collective))
        return new_collective

    # =============================================
    # CONTROL DE ÁNGULOS
    # ============================================= 

    # Pitch 
    # Args:
    #        target_speed: Velocidad objetivo en nudos
    #        current_speed: Velocidad actual en nudos
    # Returns:
    # pitch_cmd: Comando de pitch (yoke_pitch_ratio) entre -0.7 y 0.7

    def calculate_pitch_control(self, target_speed, current_speed):
        self.pid_pitch.setpoint = target_speed
        return self.pid_pitch(current_speed)
    
    # Roll
    # Args:
    #        target_speed: Velocidad a la que el helicóptero comienza a inclinarse en nudos
    #        current_speed: Velocidad actual en nudos
    # Returns:
    # roll_cmd: Comando de roll (yoke_roll_ratio) entre -0.7 y 0.7
    def calculate_roll_control(self, target_speed, current_speed):
        self.pid_roll.setpoint = target_speed
        return self.pid_roll(current_speed)
    
    # Yaw
    # Args:
    #        target_heading: Heading objetivo en grados magnéticos
    #        current_heading: Heading actual en grados magnéticos 
    # Returns:
    # yaw_cmd: Comando de yaw (yoke_heading_ratio) entre -5 y 5
    def calculate_yaw_control(self, target_heading, current_heading):
        self.pid_yaw.setpoint = target_heading
        return self.pid_yaw(current_heading)
    
    # =============================================


    def run(self, target_altitude):
        # Capturar heading de referencia al inicio del vuelo
        data = self.client.getDREFs(self.datarefs) 
        heading_ref = float(data[1][0])  # Heading magnético inicial
        
        while True:
            # Leer datos del simulador
            data = self.client.getDREFs(self.datarefs) #
            altitude_agl = float(data[0][0])  * FT_TO_METERS  # Convertir a metros
            heading = float(data[1][0])
            airspeed = float(data[2][0])
            climb_rate = float(data[4][0])
            current_collective = float(data[14][0]) 

            # Calcular controles
            collective_cmd = self.calculate_collective_control(target_altitude * FT_TO_METERS, altitude_agl, climb_rate, current_collective)
            pitch_cmd = self.calculate_pitch_control(FINAL_SPEED, airspeed)
            roll_cmd = self.calculate_roll_control(SPEED_ROLL, airspeed)
            yaw_cmd = self.calculate_yaw_control(heading_ref, heading)

            # Enviar comandos al simulador
            self.client.sendDREF("sim/cockpit2/controls/yoke_pitch_ratio", pitch_cmd)
            self.client.sendDREF("sim/cockpit2/controls/yoke_roll_ratio", roll_cmd)
            self.client.sendDREF("sim/cockpit2/controls/yoke_heading_ratio", yaw_cmd)
            self.client.sendDREF("sim/cockpit2/engine/actuators/prop_angle_degrees", collective_cmd)

# =========================================================
# CONEXIÓN CON X-PLANE
# =========================================================
if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        
        #Posición inicial 
        values = [LAT, LON, ALTITUDE , 0, 0, 0] # lat, lon, alt, pitch, roll, heading
        client.sendPOSI(values, 0)
        #Activar el override de IAS 
        client.sendDREF("sim/operation/override/override_ias", 1)
        # Forzar la velocidad inicial
        client.sendDREF("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", INITIAL_SPEED)
        time.sleep(2) # Esperar a que el simulador estabilice la posición

        # Desactivar el override IAS 
        client.sendDREF("sim/operation/override/override_ias", 0)

        controller = FlightController(client)
        controller.run(ALTITUDE)

