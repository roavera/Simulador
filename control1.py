import math
import time
import csv
import os
import logging
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =============================================
# CONSTANTES Y CONFIGURACIÓN
# =============================================

# Conversión de unidades
FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444    # 1 nudo = 0.514444 m/s

#Umbrales y parámetros de control
PROXIMITY_THRESHOLD = 50            # metros para distancia horizontal (m)
ALTITUDE_THRESHOLD = 10             # metros para diferencia de altitud (m)
TRANSITION_SPEED = 25               # velocidad de transición entre hover y vuelo forward (m/s)
MIN_AIRSPEED = 5                    # velocidad mínima para considerar movimiento (m/s)
HOVER_HEIGHT = 200 * FT_TO_METERS   # altura para considerar en fase de despegue o hover (en metros)
MIN_TIME_BETWEEN_POINTS = 2.0       # tiempo mínimo entre transiciones de puntos (s)

# Configuración de logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='flight_controller.log'    # lo crea en el mismo directorio del script
    filemode='a',                        # No sobrescribir el log en cada ejecución, sino que se añada al final del archivo
)

# =============================================
# CLASE PRINCIPAL DEL CONTROLADOR DE VUELO
# =============================================

class FlightController:
    def __init__(self):
        self.client = xpc.XPlaneConnect()
        self.current_point_index = 0
        self.last_point_time = time.time()
        self.flight_phase = 'hover'
        self.initial_position = None  # Almacenará la posición inicial, inicializa en cero 
        self.hover_ref = None

        # Inicializar PIDs con parámetros por defecto
        self._initialize_pids()
        
        # Cargar puntos de trayectoria
        self.points = self._load_flight_path()
        
        # Configurar datarefs
        self.datarefs = [
            'sim/flightmodel/position/y_agl',                                   # 0: altitud [m]
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot',    # 1: heading [m]
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

        # Inicializar PIDs 

        # PID para velocidad vertical (climb rate)
        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-4, 4)

        # -------------------------------------------- #
        # ANGULOS 

        # PID para pitch Cabeceo
        self.pid_pitch = PID(0.04, 0.0001, 0.02, setpoint=0)
        self.pid_pitch.output_limits = (-0.7 , 0.7)              # PORQUE NO LO CAMBIO A (-0.7, 0,7)
        
        # PID para roll Rolido
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=0)
        self.pid_roll.output_limits = (-0.7,0.7)
        
        # PID de yaw unificado 
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0)  # Parámetros optimizados
        self.pid_yaw.output_limits = (-8, 8)

        # -------------------------------------------- #
    
    def calculate_collective_control(self, target_altitude, current_altitude, climb_rate,current_collective):
        """Control suave del collective con limitaciones dinámicas"""
        # Convertir 100 pies/min a m/s (unidad de X-Plane)
        MAX_CLIMB_RATE = 100 * 0.00508  # ≈ 0.508 m/s
        
        # Error de altitud (suavizado para evitar cambios bruscos)
        error_alt = 0.1 * (target_altitude - current_altitude)  # Factor de ganancia reducido
        
        # Limitar la referencia de velocidad vertical
        w_ref = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, error_alt))
        
        # Aplicar PID al climb rate actual
        self.pid_collective.setpoint = w_ref
        collective_change = self.pid_collective(climb_rate)
        
        # Obtener valor actual del collective (ángulo de palas)
        #current_collective = self.client.getDREF('sim/cockpit2/engine/actuators/prop_angle_degrees')[0]
        
        # Calcular nuevo valor con cambio incremental (suavizado)
        new_collective = current_collective + collective_change
        
        # Limitar rango físico del collective (-4 a 11 grados)
        return max(-4.0, min(11.0, new_collective))
    
    def run(self):
        """Bucle principal de control de vuelo"""
        try:
            logging.info("Iniciando controlador de vuelo")
            # Obtener posición inicial antes de comenzar
            self.get_initial_position()
            
            # Calcular heading inicial al primer punto
            if not self.points:
                raise ValueError("No hay puntos de trayectoria cargados")
            
            initial_target = self.points[0]
            initial_bearing = self.calculate_bearing(
                self.initial_position['lat'],
                self.initial_position['lon'],
                initial_target['latitud'],
                initial_target['longitud']
            )
            
            logging.info(f"Heading inicial requerido: {initial_bearing:.1f}°")
            logging.info(f"Heading actual de la aeronave: {self.initial_position['heading']:.1f}°")
            while True:
                print("\n=== CICLO DE CONTROL ===")
                print(f"Punto actual: {self.current_point_index}/{len(self.points)-1}")
                # Leer datos de X-Plane
                dref_values = self.client.getDREFs(self.datarefs)
                
                # Procesar datos (con validación)
                altitude = dref_values[0][0] if dref_values[0][0] is not None else 0.0
                heading = dref_values[1][0] if dref_values[1][0] is not None else 0.0
                airspeed = (dref_values[2][0] or 0.0) * KTS_TO_METERS_PER_SEC if dref_values[2][0] is not None else 0.0
                speed_vertical = dref_values[4][0] if dref_values[4][0] is not None else 0.0
                pos_x = dref_values[6][0] if dref_values[6][0] is not None else 0.0
                pos_z = dref_values[8][0] if dref_values[8][0] is not None else 0.0
                pitch = dref_values[9][0] if dref_values[9][0] is not None else 0.0
                roll = dref_values[10][0] if dref_values[10][0] is not None else 0.0
                lat = dref_values[12][0] if dref_values[12][0] is not None else 0.0
                lon = dref_values[13][0] if dref_values[13][0] is not None else 0.0
                collective_current = dref_values[14][0] if dref_values[14][0] is not None else 0.0
                
                # Determinar fase de vuelo
                self.flight_phase = self.get_flight_phase(altitude, airspeed)
                
                # Obtener punto objetivo actual
                target = self.points[self.current_point_index]
                
                # Calcular distancia y rumbo al punto objetivo
                distance = self.calculate_distance(lat, lon, target['latitud'], target['longitud'])
                bearing = self.calculate_bearing(lat, lon, target['latitud'], target['longitud'])
                
                # Corrección de distancia
                heading_correction = self.pid_distance(distance)
                
                # Error de heading
                heading_error = self.calculate_heading_error(
                    bearing + heading_correction * 0.1,
                    heading
                )

                # Verificar si se alcanzó el punto actual
                if (distance <= PROXIMITY_THRESHOLD and 
                    abs(target['altitud'] - altitude) <= ALTITUDE_THRESHOLD and
                    (time.time() - self.last_point_time) > MIN_TIME_BETWEEN_POINTS):
                    self.goto_next_point()
                    target = self.points[self.current_point_index]
                
                # =============================================
                # CÁLCULO DE CONTROLES
                # =============================================
                
                # Control de velocidad vertical (collective)
                #error_alt = target['altitud'] - altitude
                #w_ref = max(-5, min(5, error_alt))
                #self.pid_collective.setpoint = w_ref 
                #collective_output = collective_current + self.pid_collective(speed_vertical)
                #collective_output = max(-4, min(4, collective_output))
                # Reemplazar el control de collective existente con:
                collective_output = self.calculate_collective_control(
                    target['altitud'], 
                    altitude, 
                    speed_vertical,  # Dataref 'sim/flightmodel/position/local_vy'
                    collective_current
                )

                # Enviar comando a X-Plane
                #self.client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees', collective_output)
                
                # Control de pitch
                error_speed = target['airspeed'] - airspeed
                error_speed = max(-5, min(5, error_speed))
                
                # Solo considerar posición en hover
                if self.flight_phase == 'hover':
                    if self.hover_ref is None:
                        self.hover_ref = (pos_x, pos_z)

                    error_pos_x = self.hover_ref[0] - pos_x
                    error_pos_z = self.hover_ref[1] - pos_z

                    error_pos_x = max(-10, min(10, error_pos_x))
                    error_pos_z = max(-10, min(10, error_pos_z))
                else:
                    self.hover_ref = None
                    error_pos_x = 0
                    error_pos_z = 0
                
                if self.flight_phase == 'hover':
                    self.pid_pitch.setpoint = -error_pos_z * 0.5
                else:
                    self.pid_pitch.setpoint = error_speed * 0.7
                
                cyclic_pitch = self.pid_pitch(pitch)
                cyclic_pitch = max(-0.7, min(0.7, cyclic_pitch))
                
                # Control de roll
                g_heading = 0.0 if self.flight_phase == 'hover' else 0.25

                error_lat = 0 - (dref_values[3][0] or 0.0)
                error_lat = max(-2, min(2, error_lat))

                self.pid_roll.setpoint = (
                    error_lat * 0.3
                    - error_pos_x * 0.2
                    + heading_error * g_heading
                )

                cyclic_roll = self.pid_roll(roll)
                cyclic_roll = max(-0.7, min(0.7, cyclic_roll))
                                
                # Control de yaw
                pedals = self.calculate_yaw_control(heading_error, collective_current,airspeed)
                
                # =============================================
                # ENVÍO DE COMANDOS A X-PLANE
                # =============================================
                
                self.client.sendCTRL([cyclic_pitch, cyclic_roll, pedals]) # Pedañs = 0 
                self.client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees', collective_output)
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            logging.info("Controlador detenido por el usuario")
        except Exception as e:
            logging.error(f"Error crítico: {str(e)}", exc_info=True)
        finally:
            self.client.close()
            logging.info("Conexión con X-Plane cerrada")

# =============================================
# EJECUCIÓN PRINCIPAL
# =============================================

if __name__ == "__main__":
    controller = FlightController()
    controller.run()