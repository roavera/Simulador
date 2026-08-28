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

SAMPLE_TIME = 0.05     # Tiempo de muestreo para el controlador (20 Hz)

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
            'sim/cockpit2/engine/actuators/prop_angle_degrees',                # 14: collective[grados]
        
            # Datarefs de interés para registro
            'sim/flightmodel/engine/ENGN_TRQ',                                  # 15: torque motor         
            'sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot',            # 16: pitch (vacuómetro)   [°]
            'sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot',             # 17: roll  (vacuómetro)   [°]
            'sim/cockpit2/controls/yoke_pitch_ratio',                           # 18: mando pitch          
            'sim/cockpit2/controls/yoke_roll_ratio',                            # 19: mando roll           
            'sim/cockpit2/controls/yoke_heading_ratio',                         # 20: mando yaw            
            'sim/cockpit2/engine/actuators/prop_ratio_all',                     # 21: prop ratio           
            # Parámetros iniciales / peso
            'sim/flightmodel/weight/m_fixed',                                   # 22: peso fijo            [kg]
            'sim/flightmodel/weight/m_fuel1',                                   # 23: combustible          [kg]
            'sim/cockpit2/gauges/indicators/CG_indicator',                      # 24: CG indicator

        ]

    # =========================================================
    # INICIALIZACIÓN DE PIDs
    # =========================================================

    def _init_pids(self):
        # PID para velocidad vertical (climb rate)
        # Controla el collective para mantener la altitud

        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-4, 4)
        # -------------------------------------------- #
        # ÁNGULOS 

        # PID para pitch (theta) - cabeceo
        # Nariz arriba para aumentar altitud, nariz abajo para descender
        self.pid_pitch = PID(0.05, 0.005, 0.015, setpoint=INITIAL_SPEED)  # PRUEBA , podriar ser INICIAL_SPEED
        self.pid_pitch.output_limits = (-0.25 , 0.25)             
        
        # PID para roll (phi) - Rolido
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=SPEED_ROLL)
        self.pid_roll.output_limits = (-0.7,0.7)
        
        # PID de yaw (psi) - pedales
        # Controla el rotor de cola
        # contrarresta la inercia y el torque del rotor
        self.pid_yaw = PID(0.04, 0.0, 0.001, setpoint=0)  # Parámetros optimizados
        self.pid_yaw.output_limits = (-1.0,1.0)

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
    def calculate_collective_control(self, target_altitude, current_altitude, climb_rate,current_collective, pitch_cmd):
        
        # Limitar la referencia de velocidad vertical para evitar comandos extremos
        # Evita ascensos violentos e inestabilidad
        MAX_CLIMB_RATE = 3 * FT_TO_METERS / 60
        # MAX_CLIMB_RATE = 100 * KTS_TO_METERS_PER_SEC  # 51.444 m/s
        
        # Error de altitud (suavizado para evitar cambios bruscos)
        # Movimientos mas suaves, estables y lentos para evitar oscilaciones 
        error_alt = 0.1 * (target_altitude - current_altitude)  # Factor de ganancia reducido
        
        # Limitar la referencia de velocidad vertical
        w_ref = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, error_alt))
        
        # Aplicar PID al climb rate actual
        self.pid_collective.setpoint = w_ref
        
        base_collective = current_collective + self.pid_collective(climb_rate)*0.1
        
        # Calcular nuevo valor con cambio incremental (suavizado)
        new_collective = base_collective  # Cambio incremental aplicado al collective actual

        if pitch_cmd < -0.05:
            compensation_pitch = abs(pitch_cmd) * 5
            new_collective += compensation_pitch  # Factor de compensación para pitch negativo
        
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
        
        self.pid_speed.setpoint = target_speed
        pitch = self.pid_speed(current_speed)
        return pitch
    
    # Roll
    # Args:
    #        target_roll: Ángulo de roll objetivo en grados
    #        current_roll: Ángulo de roll actual en grados
    # Returns:
    # roll_cmd: Comando de roll (yoke_roll_ratio) entre -0.7 y 0.7
    def calculate_roll_control(self, target_roll, current_roll):
        self.pid_roll.setpoint = target_roll
        return self.pid_roll(current_roll)
    
    # Yaw
    # Args:
    #        target_heading: Heading objetivo en grados magnéticos
    #        current_heading: Heading actual en grados magnéticos 
    # Returns:
    # yaw_cmd: Comando de yaw (yoke_heading_ratio) entre -5 y 5
    def calculate_yaw_control(self, target_heading, current_heading):
        # Distancia angular más corta en un círculo de 360 grados
        error_heading = (target_heading - current_heading + 360) % 360
        if error_heading > 180:
            error_heading -= 360
        
        self.pid_yaw.setpoint = 0
        return self.pid_yaw(-error_heading)
    
    # =============================================


    def run(self):

        target_altitude = ALTITUDE * FT_TO_METERS  # Convertir altitud objetivo a metros
        
        # Capturar heading de referencia al inicio del vuelo
        data = self.client.getDREFs(self.datarefs) 
        heading_ref = float(data[1][0])  # Heading magnético inicial
        
        while True:
                    
            loop_start_time: float = time.time()

            # Leer datos del simulador
            data = self.client.getDREFs(self.datarefs)
            altitude_agl        = float(data[0][0])
            heading             = float(data[1][0])
            airspeed            = float(data[2][0])
            climb_rate          = float(data[4][0])
            pitch_actual        = float(data[9][0])
            roll_actual         = float(data[10][0])
            current_collective  = float(data[14][0])
            torque_motor        = float(data[15][0])

            # Gestión de la RAMPA de velocidad (Incremento suave por ciclo)
            if self.current_target_speed < FINAL_SPEED:
                self.current_target_speed += (2.5 * SAMPLE_TIME)  # Incremento suave de 2.5 kts por segundo
                if self.current_target_speed > FINAL_SPEED:
                    self.current_target_speed = FINAL_SPEED

            # Cálculo del Pitch (Inversión de signo aplicada para bajar morro)
            pitch_pid_out = self.calculate_pitch_control(self.current_target_speed, airspeed)
            pitch_cmd = -pitch_pid_out 

            # Cálculo del Colectivo (Pasando el pitch_cmd para ejecutar el acoplamiento)
            collective_cmd = self.calculate_collective_control(target_altitude, altitude_agl, climb_rate, current_collective, pitch_cmd)
            
            # Cálculo de Roll (Alas niveladas en 0 grados)
            roll_cmd = self.calculate_roll_control(0.0, roll_actual)
            
            # Cálculo de Yaw (Mantener rumbo corrigiendo discontinuidad)
            error_heading = (heading_ref - heading + 360) % 360
            if error_heading > 180: error_heading -= 360
            yaw_cmd = -self.pid_yaw(-error_heading)

            # Enviar comandos al simulador
            self.client.sendDREF("sim/cockpit2/controls/yoke_pitch_ratio", pitch_cmd)
            self.client.sendDREF("sim/cockpit2/controls/yoke_roll_ratio", roll_cmd)
            self.client.sendDREF("sim/cockpit2/controls/yoke_heading_ratio", yaw_cmd)
            self.client.sendDREF("sim/cockpit2/engine/actuators/prop_angle_degrees", collective_cmd)

            elapse_time = time.time() - loop_start_time
            if elapse_time < SAMPLE_TIME:
                time.sleep(SAMPLE_TIME - elapse_time)  # Mantener frecuencia de muestreo constante  
            else:
                logging.warning(f"Loop took {elapse_time:.3f} seconds, which is longer than the sample time of {SAMPLE_TIME} seconds.")

# CONEXIÓN CON X-PLANE
# =========================================================
if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        
        #Posición inicial 

        values = [LAT, LON, ALTITUDE , 0, 0, 0] # lat, lon, alt, pitch, roll, heading
        client.sendPOSI(values, 0)

        time.sleep(2) # Esperar a que el simulador estabilice la posición

        client.sendDREF("sim/cockpit2/controls/yoke_pitch_ratio", 0)
        client.sendDREF("sim/cockpit2/controls/yoke_roll_ratio", 0)
        client.sendDREF("sim/cockpit2/controls/yoke_heading_ratio", 0)
        client.sendDREF("sim/cockpit2/engine/actuators/prop_angle_degrees", 5)  # Colectivo en posición mínima
        
        time.sleep(10)

        data = client.getDREFs("sim/flightmodel/position/y_agl", "sim/cockpit2/gauges/indicators/airspeed_kts_pilot", "sim/flightmodel/engine/ENGN_TRQ")

        altitude = float(data[0][0])
        speed = float(data[1][0])
        torque = float(data[2][0])

        print("--------------------------------")
        print("Estado inicial:")
        print(f"Altitud : {altitude*METERS_TO_FT:.1f} ft")
        print(f"Velocidad : {speed:.1f} kt")
        print(f"Torque : {torque:.1f}")
        print("--------------------------------")

        controller = FlightController(client)
        controller.run()

