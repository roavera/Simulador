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

# =========================================================
# CONDICIONES INICIALES
# =========================================================

LAT = -34.554
LON = -58.425
ALTITUDE = 4500        # ft
INITIAL_SPEED = 50     # kts - airspeed

FINAL_SPEED = 120      # kts - airspeed
VERTICAL_SPEED = 0     # ft/min - vertical speed 
SPEED_ROLL = 0         # kts - velocidad a la que el helicóptero comienza a inclinarse 

# =========================================================
# CONTROLADOR PID
# =========================================================

class FlightController:
    def __init__(self, client):
        self.client = client
        self.flight_phase = 'cruise'

        # Inicializar PIDs con parámetros por defecto
        self._initialize_pids()
        
    def _initialize_pids(self):
        # PID para velocidad vertical (climb rate)
        # Controla el collective para mantener la altitud

        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-3, 3)

        # -------------------------------------------- #
        # ÁNGULOS 

        # PID para pitch (theta) - cabeceo
        # Nariz arriba para aumentar altitud, nariz abajo para descender
        self.pid_pitch = PID(0.05, 0.0001, 0.015, setpoint=FINAL_SPEED)
        self.pid_pitch.output_limits = (-0.7 , 0.7)             
        
        # PID para roll (phi) - Rolido
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=0)
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
    def calculate_collective_control(self, target_altitude, current_altitude, climb_rate,current_collective):
        
        # Limitar la referencia de velocidad vertical para evitar comandos extremos
        # Evita ascensos violentos e inestabilidad
        MAX_CLIMB_RATE = 100 * 0.00508  # 100 ft/min en m/s
        
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
        return max(-4.0, min(11.0, new_collective))
    
    
    


# =========================================================
# CONEXIÓN CON X-PLANE
# =========================================================
if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        
        #Posición inicial 
        values = [LAT, LON, ALTITUDE, 0, 0, 0] # lat, lon, alt, pitch, roll, heading
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

# client.sendDREF("sim/flightmodel/position/local_vz", INITIAL_SPEED * KTS_TO_METERS_PER_SEC)