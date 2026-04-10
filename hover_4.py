import math
import time
import logging
import numpy as np
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =============================================
# CONSTANTES Y CONFIGURACIÓN
# =============================================

# Conversión de unidades
FT_TO_METERS = 0.3048

# Parámetros de la misión
TARGET_ALTITUDE = 20.0  # Altura objetivo en metros
HOVER_DURATION = 20.0   # Tiempo de hover en segundos
ALTITUDE_TOLERANCE = 0.5  # Tolerancia de altitud en metros

# Límites de velocidad
MAX_VELOCITY_HORIZONTAL = 0#2.0  # m/s - Velocidad horizontal máxima permitida
MAX_VELOCITY_COMMAND = 0#1.5     # m/s - Comando máximo de velocidad desde control de posición

# Ganancias de control - POSICIÓN (lazo externo)
KP_POS_Z = 0.15  # Ganancia control posición longitudinal (reducida)
KP_POS_X = -0.10  # Ganancia control posición lateral (reducida)

# Ganancias eliminadas - ahora en los PIDs de velocidad
# Las ganancias de conversión velocidad→ángulo están en los constructores de PID

# Otras ganancias
EFECT_COLLECTIVE_HOVER = 0.5  # Efecto del collective en yaw

# MATRIZ DE DESACOPLAMIENTO
CROSS_COUPLING = {
    'pitch_to_roll': 0.0,
    'roll_to_pitch': 0.0,
    'collective_to_pitch': 0.0,#1,
    'collective_to_roll': 0.0#1
}

# Configuración de logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('hover_test_cascaded.log'),
        logging.StreamHandler()
    ]
)

# =============================================
# CLASE CONTROLADOR DE HOVER CON CONTROL EN CASCADA
# =============================================

class HoverTestController:


    def __init__(self):
        """Inicializa el controlador de hover"""
        try:
            self.client = xpc.XPlaneConnect()
            logging.info("Conexión con X-Plane establecida")
        except Exception as e:
            logging.error(f"Error al conectar con X-Plane: {e}")
            raise
        
        # Variables de estado
        self.initial_position = None
        self.initial_heading = None
        self.hover_reference_position = None
        self.mission_phase = 'init'
        self.hover_start_time = None
        
        # Variables para control en cascada
        self.velocity_long_cmd = 0.0  # Comando de velocidad longitudinal
        self.velocity_lat_cmd = 0.0   # Comando de velocidad lateral
        
        # Variables para compensación
        self.last_collective = 0.0
        
        # Inicializar PIDs
        self._initialize_pids()
        
        # Configurar datarefs
        self._setup_datarefs()

    def _initialize_pids(self):
        """Inicializa todos los controladores PID"""
        
        # PID para velocidad vertical (climb rate)
        self.pid_collective = PID(1.0, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-2.0, 2.0)
        
        # PID para velocidad longitudinal (m/s) → ángulo de pitch (grados)
        self.pid_velocity_long = PID(Kp=-2.5, Ki=-0.15, Kd=-0.3, setpoint=0)
        self.pid_velocity_long.output_limits = (-15, 15)  # Límite de comando de pitch en grados
        self.pid_velocity_long.sample_time = 0.05  # 20 Hz
        
        # PID para velocidad lateral (m/s) → ángulo de roll (grados)
        self.pid_velocity_lat = PID(Kp=-2.5, Ki=-0.15, Kd=-0.3, setpoint=0)
        self.pid_velocity_lat.output_limits = (-15, 15)  # Límite de comando de roll en grados
        self.pid_velocity_lat.sample_time = 0.05  # 20 Hz
        
        # PID para actitud pitch (lazo interno)
        self.pid_pitch = PID(0.05, 0.0001, 0.01, setpoint=0)
        self.pid_pitch.output_limits = (-10, 10)
        
        # PID para actitud roll (lazo interno)
        self.pid_roll = PID(0.10, 0.0005, 0.01, setpoint=0)
        self.pid_roll.output_limits = (-10, 10)
        
        # PID para yaw
        self.pid_yaw = PID(0.005, 0.001, 0.0, setpoint=0)
        self.pid_yaw.output_limits = (-15, 15)
        
        logging.info("PIDs inicializados correctamente")
        logging.info("Control en cascada: Posición → Velocidad (PID) → Actitud → Controles")

    def _setup_datarefs(self):
        """Configura los datarefs necesarios"""
        self.datarefs = [
            'sim/flightmodel/position/y_agl',                    # 0: altitud AGL
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot', # 1: heading
            'sim/flightmodel/position/local_vx',                   # 2: velocidad lateral
            'sim/flightmodel/position/local_vy',                   # 3: climb rate
            'sim/flightmodel/position/local_vz',                   # 4: velocidad longitudinal
            'sim/flightmodel/position/local_x',                    # 5: posición lateral 
            'sim/flightmodel/position/local_z',                    # 6: posición longitudinal
            'sim/flightmodel/position/theta',                     # 7: pitch
            'sim/flightmodel/position/phi',                       # 8: roll
            'sim/flightmodel/position/latitude',                  # 9: latitude
            'sim/flightmodel/position/longitude',                 # 10: longitude
            'sim/cockpit2/engine/actuators/prop_angle_degrees',   # 11: collective
        ]

    def saturate(self, value, min_val, max_val):
        """Satura un valor entre límites"""
        return max(min_val, min(max_val, value))
    
    def normalize(self, value):
        """Normaliza collective del rango [-4, 11] al rango [0, 1]"""
        min_original = -4
        max_original = 11
        saturated_value = max(min_original, min(max_original, value))
        return (saturated_value - min_original) / (max_original - min_original)

    def calculate_heading_error(self, desired_heading, current_heading):
        """Calcula la diferencia angular más corta entre dos headings"""
        error = (desired_heading - current_heading + 360) % 360
        return error - 360 if error > 180 else error

    def get_initial_position(self):
        """Obtiene la posición y heading inicial"""
        logging.info("Obteniendo posición inicial...")
        
        try:
            initial_data = self.client.getDREFs([
                'sim/flightmodel/position/local_x',
                'sim/flightmodel/position/local_z',
                'sim/flightmodel/position/latitude',
                'sim/flightmodel/position/longitude',
                'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot'
            ])
            
            if not all(d and d[0] is not None for d in initial_data):
                raise ValueError("Datos iniciales inválidos")
            
            self.initial_position = {
                'pos_x': initial_data[0][0],
                'pos_z': initial_data[1][0],
                'lat': initial_data[2][0],
                'lon': initial_data[3][0]
            }
            
            self.initial_heading = initial_data[4][0]
            
            # Establecer referencia de hover
            self.hover_reference_position = self.initial_position.copy()
            
            logging.info(f"Posición inicial: X={self.initial_position['pos_x']:.2f}, "
                        f"Z={self.initial_position['pos_z']:.2f}")
            logging.info(f"Heading inicial: {self.initial_heading:.1f}°")
            
        except Exception as e:
            logging.error(f"Error obteniendo posición inicial: {e}")
            raise

    def calculate_position_error(self, current_x, current_z):
        """Calcula el error de posición respecto a la referencia"""
        if not self.hover_reference_position:
            return 0.0, 0.0
        
        error_x = self.hover_reference_position['pos_x'] - current_x
        error_z = self.hover_reference_position['pos_z'] - current_z
        
        return error_x, error_z

    def calculate_velocity_command(self, error_pos_x, error_pos_z, dt):
        """
        LAZO EXTERNO: Convierte error de posición en comando de velocidad
        Este es el controlador más lento (frecuencia baja)
        """
        # Control proporcional de posición a velocidad
        vel_cmd_x = error_pos_x * KP_POS_X
        vel_cmd_z = error_pos_z * KP_POS_Z
        
        # Saturar comandos de velocidad
        vel_cmd_x = self.saturate(vel_cmd_x, -MAX_VELOCITY_COMMAND, MAX_VELOCITY_COMMAND)
        vel_cmd_z = self.saturate(vel_cmd_z, -MAX_VELOCITY_COMMAND, MAX_VELOCITY_COMMAND)
        
        return vel_cmd_x, vel_cmd_z

    def calculate_attitude_command(self, vel_cmd_x, vel_cmd_z, vel_actual_x, vel_actual_z):
        """
        LAZO INTERNO: Convierte velocidad deseada en comando de actitud (pitch/roll)
        
        Utiliza PIDs con anti-windup integrado de simple_pid.
        La conversión de velocidad (m/s) a ángulo (grados) se hace mediante las ganancias del PID.
        
        Física del helicóptero:
        - Para moverse hacia adelante: pitch negativo (nariz abajo)
        - Para moverse hacia atrás: pitch positivo (nariz arriba)
        - Para moverse a la derecha: roll positivo
        - Para moverse a la izquierda: roll negativo
        
        Por eso usamos ganancias negativas en los PIDs.
        """
        # Configurar setpoints (velocidades deseadas)
        self.pid_velocity_long.setpoint = vel_cmd_z
        self.pid_velocity_lat.setpoint = vel_cmd_x
        
        # Los PIDs calculan automáticamente el error y aplican P+I+D
        # El anti-windup está integrado en simple_pid mediante output_limits
        pitch_cmd = self.pid_velocity_long(vel_actual_z)
        roll_cmd = self.pid_velocity_lat(vel_actual_x)
        
        # Los PIDs ya tienen output_limits, pero por seguridad verificamos
        pitch_cmd = self.saturate(pitch_cmd, -15, 15)
        roll_cmd = self.saturate(roll_cmd, -15, 15)
        
        return pitch_cmd, roll_cmd

    def calculate_collective_control(self, target_altitude, current_altitude, climb_rate, current_collective):
        """Control del collective para alcanzar altitud objetivo"""
        try:
            MAX_CLIMB_RATE = 20 * 0.00508
            error_alt = target_altitude - current_altitude
            
            Kp_altitude = 0.4
            w_ref = Kp_altitude * error_alt
            w_ref = self.saturate(w_ref, -MAX_CLIMB_RATE, MAX_CLIMB_RATE)
            
            self.pid_collective.setpoint = w_ref
            collective_change = self.pid_collective(climb_rate)
            new_collective = current_collective + collective_change
            
            return self.saturate(new_collective, -4.0, 11.0)
            
        except Exception as e:
            logging.error(f"Error en control de collective: {e}")
            return current_collective

    def calculate_yaw_control(self, heading_error, collective):
        """Control de yaw para mantener heading"""
        try:
            pid_output = self.pid_yaw(heading_error)
            collective_normalize = self.normalize(collective)
            collective_effect = collective_normalize * EFECT_COLLECTIVE_HOVER
            yaw_control = pid_output + collective_effect
            return self.saturate(yaw_control, -0.7, 0.7)
        except Exception as e:
            logging.error(f"Error en control de yaw: {e}")
            return 0.0

    def apply_decoupling_matrix(self, cyclic_pitch_cmd, cyclic_roll_cmd, collective_change):
        """Aplica matriz de desacoplamiento"""
        collective_compensation_pitch = collective_change * CROSS_COUPLING['collective_to_pitch']
        collective_compensation_roll = collective_change * CROSS_COUPLING['collective_to_roll']
        
        cross_compensation_pitch = cyclic_roll_cmd * CROSS_COUPLING['roll_to_pitch']
        cross_compensation_roll = cyclic_pitch_cmd * CROSS_COUPLING['pitch_to_roll']
        
        pitch_compensated = cyclic_pitch_cmd + collective_compensation_pitch + cross_compensation_pitch
        roll_compensated = cyclic_roll_cmd + collective_compensation_roll + cross_compensation_roll
        
        return pitch_compensated, roll_compensated

    def transform_controls_to_body_frame(self, control_pitch, control_roll, heading, initial_heading):
        """Transforma comandos del frame de navegación al frame del helicóptero"""
        heading_diff = math.radians(heading - initial_heading)
        cos_h = math.cos(heading_diff)
        sin_h = math.sin(heading_diff)
        
        pitch_body = control_pitch * cos_h - control_roll * sin_h
        roll_body = control_pitch * sin_h + control_roll * cos_h
        
        return pitch_body, roll_body

    def run(self):
        """Bucle principal de control"""
        try:
            logging.info("=" * 70)
            logging.info("INICIANDO TEST DE HOVER CON CONTROL EN CASCADA")
            logging.info(f"Objetivo: Subir a {TARGET_ALTITUDE}m y mantener por {HOVER_DURATION}s")
            logging.info("Arquitectura: Posición → Velocidad → Actitud")
            logging.info("=" * 70)
            
            self.get_initial_position()
            self.mission_phase = 'takeoff'
            
            cycle_count = 0
            last_collective = 0.0
            last_time = time.time()
            
            while self.mission_phase != 'completed':
                try:
                    current_time = time.time()
                    dt = current_time - last_time
                    last_time = current_time
                    
                    if dt > 0.2:  # Saltar ciclos con dt muy grande
                        dt = 0.05
                    
                    # Leer datos de X-Plane
                    dref_values = self.client.getDREFs(self.datarefs)
                    
                    # Extraer datos
                    altitude = dref_values[0][0] if dref_values[0] else 0.0
                    heading = dref_values[1][0] if dref_values[1] else 0.0
                    speed_roll = dref_values[2][0] if dref_values[2] else 0.0
                    climb_rate = dref_values[3][0] if dref_values[3] else 0.0
                    speed_long = dref_values[4][0] if dref_values[4] else 0.0
                    pos_x = dref_values[5][0] if dref_values[5] else 0.0
                    pos_z = dref_values[6][0] if dref_values[6] else 0.0
                    pitch = dref_values[7][0] if dref_values[7] else 0.0
                    roll = dref_values[8][0] if dref_values[8] else 0.0
                    collective_current = dref_values[11][0] if dref_values[11] else 0.0
                    
                    # =============================================
                    # MÁQUINA DE ESTADOS
                    # =============================================
                    
                    if self.mission_phase == 'takeoff':
                        if altitude >= TARGET_ALTITUDE - ALTITUDE_TOLERANCE:
                            self.mission_phase = 'hover'
                            self.hover_start_time = time.time()
                            logging.info("=" * 70)
                            logging.info(f"ALTITUD OBJETIVO ALCANZADA: {altitude:.1f}m")
                            logging.info(f"INICIANDO HOVER POR {HOVER_DURATION} SEGUNDOS")
                            logging.info("=" * 70)
                    
                    elif self.mission_phase == 'hover':
                        elapsed_time = time.time() - self.hover_start_time
                        if elapsed_time >= HOVER_DURATION:
                            self.mission_phase = 'completed'
                            logging.info("=" * 70)
                            logging.info("MISIÓN COMPLETADA CON ÉXITO")
                            logging.info(f"Tiempo total en hover: {elapsed_time:.1f}s")
                            logging.info("=" * 70)
                            break
                    
                    # =============================================
                    # CONTROL EN CASCADA
                    # =============================================
                    
                    # 1. Control de altitud (collective)
                    collective_output = self.calculate_collective_control(
                        TARGET_ALTITUDE, altitude, climb_rate, collective_current
                    )
                    collective_change = collective_output - last_collective
                    last_collective = collective_output
                    
                    # 2. Errores de posición (en frame de navegación)
                    error_pos_x, error_pos_z = self.calculate_position_error(pos_x, pos_z)
                    error_pos_z = self.saturate(error_pos_z, -50, 50)
                    error_pos_x = self.saturate(error_pos_x, -50, 50)
                    
                    # 3. LAZO EXTERNO: Posición → Velocidad deseada
                    vel_cmd_x, vel_cmd_z = self.calculate_velocity_command(
                        error_pos_x, error_pos_z, dt
                    )
                    
                    # 4. LAZO INTERNO: Velocidad → Actitud deseada (usando PIDs)
                    pitch_cmd, roll_cmd = self.calculate_attitude_command(
                        vel_cmd_x, vel_cmd_z,
                        speed_roll, speed_long
                    )
                    
                    # 5. Transformar al frame del helicóptero
                    pitch_body, roll_body = self.transform_controls_to_body_frame(
                        pitch_cmd, roll_cmd, heading, self.initial_heading
                    )
                    
                    # 6. Aplicar desacoplamiento
                    pitch_decoupled, roll_decoupled = self.apply_decoupling_matrix(
                        pitch_body, roll_body, collective_change
                    )
                    
                    # 7. Control de actitud (PID de pitch y roll)
                    self.pid_pitch.setpoint = pitch_decoupled
                    cyclic_pitch = self.pid_pitch(pitch)
                    cyclic_pitch = self.saturate(cyclic_pitch, -0.7, 0.7)
                    
                    self.pid_roll.setpoint = roll_decoupled
                    cyclic_roll = self.pid_roll(roll)
                    cyclic_roll = self.saturate(cyclic_roll, -0.7, 0.7)
                    
                    # 8. Control de yaw
                    heading_error = self.calculate_heading_error(self.initial_heading, heading)
                    pedals = self.calculate_yaw_control(heading_error, collective_current)
                    
                    # =============================================
                    # ENVÍO DE COMANDOS
                    # =============================================
                    
                    self.client.sendCTRL([cyclic_pitch, cyclic_roll, -pedals])
                    self.client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees', collective_output)
                    
                    # Logging detallado
                    cycle_count += 1
                    if cycle_count % 20 == 0:
                        if self.mission_phase == 'takeoff':
                            logging.info(
                                f"ASCENSO - Alt: {altitude:.1f}m/{TARGET_ALTITUDE}m, "
                                f"Climb: {climb_rate:.2f}m/s | "
                                f"Pos_err: X={error_pos_x:.1f}m Z={error_pos_z:.1f}m | "
                                f"Vel: vx={speed_roll:.2f} vz={speed_long:.2f} | "
                                f"Vel_cmd: vx={vel_cmd_x:.2f} vz={vel_cmd_z:.2f} | "
                                f"Att_cmd: P={pitch_cmd:.1f}° R={roll_cmd:.1f}°"
                            )
                        elif self.mission_phase == 'hover':
                            elapsed = time.time() - self.hover_start_time
                            remaining = HOVER_DURATION - elapsed
                            logging.info(
                                f"HOVER - Alt: {altitude:.1f}m, Tiempo: {elapsed:.1f}s/{HOVER_DURATION}s | "
                                f"Pos_err: X={error_pos_x:.1f}m Z={error_pos_z:.1f}m | "
                                f"Vel: vx={speed_roll:.2f} vz={speed_long:.2f}"
                            )
                    
                    time.sleep(0.05)  # 20 Hz
                    
                except Exception as e:
                    logging.error(f"Error en ciclo de control: {e}")
                    time.sleep(0.1)
                    continue
            
            # Mantener controles finales
            logging.info("Manteniendo controles finales...")
            for _ in range(40):
                self.client.sendCTRL([cyclic_pitch, cyclic_roll, -pedals])
                self.client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees', collective_output)
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            logging.info("Test interrumpido por el usuario")
        except Exception as e:
            logging.error(f"Error crítico: {str(e)}", exc_info=True)
        finally:
            try:
                self.client.close()
                logging.info("Conexión con X-Plane cerrada")
            except:
                pass

# =============================================
# EJECUCIÓN PRINCIPAL
# =============================================

if __name__ == "__main__":
    try:
        controller = HoverTestController()
        controller.run()
    except Exception as e:
        logging.error(f"Error iniciando controlador: {e}")
        input("Presiona Enter para salir...")