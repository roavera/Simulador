import math
import time
import csv
import os
import logging
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =============================================
# CONSTANTES Y CONFIGURACIÓN TÉCNICA
# =============================================
FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444

# Parámetros de la aeronave (Bell 407)
HOVER_HEIGHT_FT = 200
TRANSITION_SPEED = 25.0  # m/s
MAX_COLLECTIVE = 11.0    # Grados (Paso máximo)
MIN_COLLECTIVE = -4.0    # Grados (Paso mínimo)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='flight_controller.log'
)

class FlightController:
    def __init__(self):
        self.client = xpc.XPlaneConnect()
        self.current_point_index = 0
        self.last_point_time = time.time()
        self.flight_phase = 'hover'
        self.initial_position = None
        self.hover_ref = None

        self._initialize_pids()
        self.points = self._load_flight_path()
        
        # Datarefs necesarios para control de estado completo
        self.datarefs = [
            'sim/flightmodel/position/y_agl',                                   # 0: altitud
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot',    # 1: heading
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',                # 2: airspeed
            'sim/flightmodel/position/local_vx',                                # 3: v_lat
            'sim/flightmodel/position/local_vy',                                # 4: climb_rate (v_up)
            'sim/flightmodel/position/local_vz',                                # 5: v_long
            'sim/flightmodel/position/local_x',                                 # 6: pos_x
            'sim/flightmodel/position/local_y',                                 # 7: pos_y
            'sim/flightmodel/position/local_z',                                 # 8: pos_z
            'sim/flightmodel/position/theta',                                   # 9: pitch
            'sim/flightmodel/position/phi',                                     # 10: roll
            'sim/flightmodel/position/psi',                                     # 11: yaw (true)
            'sim/flightmodel/position/latitude',                                # 12: lat
            'sim/flightmodel/position/longitude',                               # 13: lon
            'sim/cockpit2/engine/actuators/prop_angle_degrees'                  # 14: collective actual
        ]

    def _initialize_pids(self):
        """
        Sintonización de PIDs normalizados [-1, 1].
        Basado en arquitectura de control de estabilidad (SAS).
        """
        # Control de Altitud (V_vertical -> Colectivo)
        # Kp=2.0 para una respuesta de 0.5 grados por cada m/s de error de trepada
        self.pid_collective = PID(2.5, 0.5, 0.1, setpoint=0)
        self.pid_collective.output_limits = (MIN_COLLECTIVE, MAX_COLLECTIVE)

        # Control de Pitch (Velocidad/Posición -> Attitude -> Cíclico Longitudinal)
        # Ganancias bajas para evitar oscilaciones inducidas por el rotor
        self.pid_pitch = PID(0.05, 0.005, 0.01, setpoint=0)
        self.pid_pitch.output_limits = (-1.0, 1.0)

        # Control de Roll (V_lateral/Posición -> Attitude -> Cíclico Lateral)
        self.pid_roll = PID(0.04, 0.005, 0.01, setpoint=0)
        self.pid_roll.output_limits = (-1.0, 1.0)

        # Control de Yaw (Error Heading -> Pedales)
        # Incrementamos Kp para vencer el torque reactivo
        self.pid_yaw = PID(0.035, 0.002, 0.001, setpoint=0)
        self.pid_yaw.output_limits = (-1.0, 1.0)

    def _load_flight_path(self):
        csv_file = 'trajectories/sabe_saez.csv'
        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"Archivo no encontrado: {csv_file}")
        
        points = []
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                points.append({
                    'altitud': float(row['Altitud (ft)']) * FT_TO_METERS,
                    'latitud': float(row['Lat']),
                    'longitud': float(row['Long']),
                    'airspeed': float(row['Airspeed (kias)']) * KTS_TO_METERS_PER_SEC
                })
        return points

    # --- Funciones Matemáticas de Navegación ---
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        return (math.degrees(math.atan2(x, y)) + 360) % 360

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000.0 # Radio Tierra en m
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat, dlon = lat2 - lat1, lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def calculate_heading_error(self, target, current):
        error = (target - current + 360) % 360
        return error - 360 if error > 180 else error

    # --- Lógica de Control de Actuadores ---
    def get_yaw_command(self, heading_error, collective, airspeed):
        """Implementa compensación de torque (Feed-forward) para Bell 407"""
        pid_out = self.pid_yaw(heading_error)
        
        # Feed-forward: El Bell 407 requiere pedal derecho (+) al subir colectivo
        # K_torque estimada: 0.06 unidades de pedal por cada grado de colectivo
        torque_comp = collective * 0.065
        
        # Estabilidad direccional por velocidad (Weathervane effect)
        speed_fade = max(0.2, 1.0 - (airspeed / 40.0))
        
        return max(-1.0, min(1.0, (pid_out + torque_comp) * speed_fade))

    def get_collective_command(self, target_alt, current_alt, climb_rate):
        """Control de velocidad vertical con límite de seguridad (Climb Rate)"""
        max_climb = 5.0 # m/s (aprox 1000 ft/min)
        error_alt = target_alt - current_alt
        
        # Lazo externo: Altitud -> Referencia de velocidad vertical
        v_ref = max(-max_climb, min(max_climb, error_alt * 0.5))
        
        self.pid_collective.setpoint = v_ref
        return self.pid_collective(climb_rate)

    def run(self):
        try:
            logging.info("Controlador Iniciado.")
            while True:
                # 1. Adquisición de Datos
                data = self.client.getDREFs(self.datarefs)
                if not data: continue
                
                # Unpack de variables
                alt, head, speed = data[0][0], data[1][0], data[2][0] * KTS_TO_METERS_PER_SEC
                v_climb = data[4][0]
                pos_local = (data[6][0], data[8][0]) # X, Z
                pitch, roll = data[9][0], data[10][0]
                lat, lon = data[12][0], data[13][0]
                coll_actual = data[14][0]

                # 2. Navegación
                target = self.points[self.current_point_index]
                dist = self.calculate_distance(lat, lon, target['latitud'], target['longitud'])
                bearing = self.calculate_bearing(lat, lon, target['latitud'], target['longitud'])
                
                # 3. Lógica de Fase de Vuelo
                phase = 'hover' if alt < 20 and speed < 5 else 'cruise'
                
                # Avanzar waypoint
                if dist < 40 and abs(target['altitud'] - alt) < 10:
                    if self.current_point_index < len(self.points) - 1:
                        self.current_point_index += 1
                        logging.info(f"Waypoint alcanzado. Siguiente: {self.current_point_index}")

                # 4. Cálculo de Errores de Actitud
                h_error = self.calculate_heading_error(bearing, head)
                
                # Pitch Control (Longitudinal)
                if phase == 'hover':
                    # En hover, usamos pitch para mantener posición Z
                    if self.hover_ref is None: self.hover_ref = pos_local
                    error_z = self.hover_ref[1] - pos_local[1]
                    self.pid_pitch.setpoint = max(-5, min(5, error_z * 0.2))
                else:
                    self.hover_ref = None
                    error_speed = target['airspeed'] - speed
                    self.pid_pitch.setpoint = max(-10, min(10, error_speed * 1.2))

                # 5. Generación de Comandos (Controladores)
                cmd_pitch = self.pid_pitch(pitch)
                
                # Roll simple (Mantener nivelado o coordinar giro suave)
                self.pid_roll.setpoint = 0 if phase == 'hover' else h_error * 0.1
                cmd_roll = self.pid_roll(roll)
                
                cmd_yaw = self.get_yaw_command(h_error, coll_actual, speed)
                cmd_coll = self.get_collective_command(target['altitud'], alt, v_climb)

                # 6. Salida a Simulador
                # sendCTRL: [Pitch, Roll, Yaw] -> Rango [-1, 1]
                self.client.sendCTRL([cmd_pitch, cmd_roll, cmd_yaw])
                # Colectivo es un dataref de ángulo físico
                self.client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees', cmd_coll)

                time.sleep(0.02) # 50Hz Loop

        except KeyboardInterrupt:
            logging.info("Interrumpido.")
        finally:
            self.client.close()

if __name__ == "__main__":
    controller = FlightController()
    controller.run()