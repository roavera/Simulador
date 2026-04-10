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

# Umbrales y parámetros de control
PROXIMITY_THRESHOLD = 50        # metros para distancia horizontal
ALTITUDE_THRESHOLD = 10         # metros para diferencia de altitud
TRANSITION_SPEED = 25           # velocidad de transición entre hover y vuelo forward (m/s)
MIN_AIRSPEED = 5                # velocidad mínima para considerar movimiento (m/s)
HOVER_HEIGHT = 200              # altura para considerar en fase de despegue ft
MIN_TIME_BETWEEN_POINTS = 2.0   # tiempo mínimo entre transiciones de puntos (s)

# Configuración de logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='flight_controller.log'
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
            'sim/flightmodel/position/y_agl',                    # altitud

            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot', # heading
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',  # airspeed
            
            # Vector completo de velocidad
            'sim/flightmodel/position/local_vx',                   # Speed roll
            'sim/flightmodel/position/local_vy',                   # climb rate
            'sim/flightmodel/position/local_vz',                   # Speed longitudinal
            
            # Posición
            'sim/flightmodel/position/local_x',                    # position lateral 
            'sim/flightmodel/position/local_y',                    # position up
            'sim/flightmodel/position/local_z',                    # position longitudinal
            
            'sim/flightmodel/position/theta',                     # pitch
            'sim/flightmodel/position/phi',                       # Roll
            'sim/flightmodel/position/psi',                       # Yaw                
            
            'sim/flightmodel/position/latitude',                  # latitude
            'sim/flightmodel/position/longitude',                 # longitude
            
            'sim/cockpit2/engine/actuators/prop_angle_degrees',   # collective
            
            'sim/joystick/yoke_pitch_ratio',                      # cyclic pitch
            'sim/joystick/yoke_roll_ratio',                       # cyclic roll
            'sim/joystick/yoke_heading_ratio',                    # pedals
        ]

    def _initialize_pids(self):

        # Inicialización de los PID
        
        # PID para velocidad vertical (climb rate)
        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-4, 4)
        
        # PID para pitch
        self.pid_pitch = PID(0.04, 0.0001, 0.02, setpoint=0)
        self.pid_pitch.output_limits = (-0.7 , 0.7)              # PORQUE NO LO CAMBIO A (-0.7, 0,7)
        
        # PID para roll
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=0)
        self.pid_roll.output_limits = (-0.7,0.7)
        
        # PID de yaw unificado
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0)  # Parámetros optimizados
        self.pid_yaw.output_limits = (-8, 8)
        
        # PID para distancia
        self.pid_distance = PID(0.8, 0.0, 0.05, setpoint=0)
        self.pid_distance.output_limits = (-45, 45)

    def _load_flight_path(self):
        """Carga los puntos de trayectoria desde el archivo CSV"""
        csv_file = 'trajectories/sabe_saez.csv'
        
        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"No se encontró el archivo de trayectoria: {csv_file}")
        
        points = []
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    point = {
                        'altitud': float(row['Altitud (ft)']) * FT_TO_METERS,
                        'latitud': float(row['Lat']),
                        'longitud': float(row['Long']),
                        'airspeed': float(row['Airspeed (kias)']) * KTS_TO_METERS_PER_SEC
                    }
                    points.append(point)
            
            if not points:
                raise ValueError("El archivo CSV no contiene datos válidos")
                
            logging.info(f"Cargamados {len(points)} puntos de trayectoria")
            return points
            
        except Exception as e:
            logging.error(f"Error al cargar trayectoria: {str(e)}")
            raise
        
    def get_initial_position(self):
        """Obtiene la posición inicial de la aeronave"""
        logging.info("Obteniendo posición inicial de la aeronave...")
        initial_data = self.client.getDREFs([
            'sim/flightmodel/position/latitude',
            'sim/flightmodel/position/longitude',
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot'
        ])
        
        if None in initial_data or any(d[0] is None for d in initial_data):
            raise ValueError("No se pudo obtener la posición inicial de la aeronave")
        
        self.initial_position = {
            'lat': initial_data[0][0],
            'lon': initial_data[1][0],
            'heading': initial_data[2][0]
        }
        
        logging.info(f"Posición inicial: Lat={self.initial_position['lat']:.6f}, "
                    f"Lon={self.initial_position['lon']:.6f}, "
                    f"Heading={self.initial_position['heading']:.1f}°")
                    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calcula el rumbo entre dos puntos geográficos"""
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        
        delta_lon = lon2 - lon1
        x = math.sin(delta_lon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
        
        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        return (bearing + 360) % 360

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calcula la distancia entre dos puntos usando la fórmula de Haversine"""
        R = 6371.0  # Radio de la Tierra en km
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c * 1000  # Convertir a metros

    def calculate_heading_error(self, desired_heading, current_heading):
        """Calcula la diferencia angular más corta entre dos headings"""
        error = (desired_heading - current_heading + 360) % 360
        return error - 360 if error > 180 else error

    def get_flight_phase(self, altitude, airspeed):
        """Determina la fase actual de vuelo"""
        if altitude < HOVER_HEIGHT*FT_TO_METERS and airspeed < MIN_AIRSPEED:
            return 'hover'
        elif airspeed < TRANSITION_SPEED:
            return 'transition'
        return 'cruise'

    def calculate_yaw_control(self, heading_error, collective, airspeed):

        # Deadzone
        if abs(heading_error) < 2:
            heading_error = 0

        # PID
        pid_output = self.pid_yaw(heading_error)

        # Compensación más suave
        collective_effect = collective * 0.2

        # Reducir en vuelo
        speed_factor = max(0.2, 1 - (airspeed / 25))

        yaw_control = (pid_output + collective_effect) * speed_factor

        return max(-8, min(8, yaw_control))


    def goto_next_point(self):
        """Avanza al siguiente punto de la trayectoria"""
        if self.current_point_index < len(self.points) - 1:
            self.current_point_index += 1
            self.last_point_time = time.time()
            target = self.points[self.current_point_index]
            
            logging.info(
                f"Nuevo punto {self.current_point_index}: "
                f"Lat={target['latitud']:.6f}, Lon={target['longitud']:.6f}, "
                f"Alt={target['altitud']:.1f}m, Vel={target['airspeed']:.1f}m/s"
            )
            return True
        else:
            logging.info("Trayectoria completada - último punto alcanzado")
            return False

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
                
                self.client.sendCTRL([cyclic_pitch, cyclic_roll, pedals])
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