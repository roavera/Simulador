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

# =============================================
# MAPEO DE DATAREFS
# =============================================
DREFS = {
    "altitude":     0,  # Altitud sobre el terreno (m)
    "heading":      1,  # Heading magnético (grados)
    "airspeed":     2,  # Velocidad del aire (knots)
    "v_lat":        3,  # Velocidad lateral (m/s)
    "climb_rate":   4,  # Tasa de ascenso (m/s)
    "v_long":       5,  # Velocidad longitudinal (m/s)
    "pos_x":        6,  # Posición X (m)
    "pos_y":        7,  # Posición Y (m)
    "pos_z":        8,  # Posición Z (m)
    "pitch":        9,  # Pitch (grados)
    "roll":         10, # Roll (grados)
    "yaw":          11, # Yaw (grados)
    "lat":          12, # Latitud (grados)
    "lon":          13, # Longitud (grados)
    "collective":   14  # Collective (grados)
}

# =============================================
# CLASE STATE (sensores)
# =============================================

class State:
    def __init__(self, dref_values):
        self._data = dref_values

    def _get(self, key, scale=1.0):
        idx = DREFS[key]
        value = self._data[idx][0]
        return (value if value is not None else 0.0) * scale

    @property
    def altitude(self):
        return self._get("altitude")

    @property
    def heading(self):
        return self._get("heading")

    @property
    def airspeed(self):
        return self._get("airspeed", KTS_TO_METERS_PER_SEC)

    @property
    def climb_rate(self):
        return self._get("climb_rate")

    @property
    def pos_x(self):
        return self._get("pos_x")

    @property
    def pos_z(self):
        return self._get("pos_z")

    @property
    def pitch(self):
        return self._get("pitch")

    @property
    def roll(self):
        return self._get("roll")

    @property
    def lat(self):
        return self._get("lat")

    @property
    def lon(self):
        return self._get("lon")

    @property
    def collective(self):
        return self._get("collective")

# Configuración de logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',   
    filename='flight_controller.log',                   # lo crea en el mismo directorio del script
    filemode='a',                                       # No sobrescribir el log en cada ejecución, sino que se añada al final del archivo
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
        # self.points = self._load_flight_path()
        
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

    def reset_simulation(self):
        self.client.sendDREF("sim/operation/reset_flight", 1)
        time.sleep(1)

        self.pid_collective.reset()
        self.pid_pitch.reset()
        self.pid_roll.reset()
        self.pid_yaw.reset()
    print("Simulación reiniciada")

    def _initialize_pids(self):

        # Inicializar PIDs 

        # PID para velocidad vertical (climb rate)
        # Controla el collective para mantener la altitud

        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-3, 3)

        # -------------------------------------------- #
        # ÁNGULOS 

        # PID para pitch (theta) - cabeceo
        # Nariz arriba para aumentar altitud, nariz abajo para descender
        self.pid_pitch = PID(0.05, 0.0001, 0.015, setpoint=0)
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
    
    # =============================================
    # LOOP PRINCIPAL
    # =============================================
    # 1. Lee el estado del helicóptero
    # 2. Decide a dónde quiere ir
    # 3. Calcula errores
    # 4. Aplica PIDs
    # 5. Manda comandos
    def run(self):
        try:
            logging.info("Iniciando controlador de vuelo")

            self.reset_simulation()
            
            while True:
                logging.info("\n=== CICLO DE CONTROL ===")
                
                # Lee todos los datarefs de una sola vez (altitud, velocidad, posición, orientación, collective)
                dref_values = self.client.getDREFs(self.datarefs)
                if not dref_values:
                    logging.warning("No se pudieron leer los datarefs")
                    continue

                # Crear objeto de estado para acceder a los datos de forma estructurada
                state = State(dref_values)

                # =========================
                # CONTROLES
                # =========================
                
                # Colective (control de altitud)
                target_altitude = 100                                    # Altitud objetivo (en metros)
                collective_output = self.calculate_collective_control(
                    target_altitude,    # Altitud objetivo
                    state.altitude,     # Altitud actual
                    state.climb_rate,   # Tasa de ascenso actual
                    state.collective    # Collective actual (ángulo de palas)    
                )
                
                # Pitch
                # Control de velocidad: nariz arriba para aumentar altitud, nariz abajo para descender
                # --- PID EXTERNO: genera pitch_deseado ---

                # Control de velocidad (fuera de hover)
                target_airspeed = 25.0   # Velocidad objetivo (m/s)
                error_speed = target_airspeed - state.airspeed
                error_speed = max(-5, min(5, error_speed))   # limitar error

                # Control de posición (solo en hover)
                if self.flight_phase == 'hover':
                    if self.hover_ref is None:
                        self.hover_ref = (state.pos_x, state.pos_z)

                    error_pos_x = self.hover_ref[0] - state.pos_x
                    error_pos_z = self.hover_ref[1] - state.pos_z

                    error_pos_x = max(-10, min(10, error_pos_x))
                    error_pos_z = max(-10, min(10, error_pos_z))
                else:
                    self.hover_ref = None
                    error_pos_x = 0
                    error_pos_z = 0

                # --- Cálculo del pitch deseado según fase de vuelo ---

                if self.flight_phase == 'hover':
                    # Mantener posición vertical → pitch deseado
                    pitch_deseado = -error_pos_z * 0.5
                else:
                    # Control de velocidad → pitch deseado
                    pitch_deseado = error_speed * 0.7

                # Limitar pitch deseado a un rango razonable
                pitch_deseado = max(-10, min(10, pitch_deseado))   # grados

                # --- PID INTERNO: controla el ángulo de pitch real ---

                self.pid_pitch.setpoint = pitch_deseado
                cyclic_pitch = self.pid_pitch(state.pitch)

                # Limitar salida del PID
                cyclic_pitch = max(-0.7, min(0.7, cyclic_pitch))

                # Roll (estabilidad lateral)

                # 1) Compensación por velocidad lateral (v_lat)
                # Si el helicóptero se mueve hacia la derecha, necesita inclinarse a la izquierda.

                lat_comp = -0.12 * state._get("v_lat")   

                # 2) Compensación por translational lift (cuando hay velocidad hacia adelante)
                # A mayor velocidad hacia adelante, más tendencia a inclinarse a la derecha.

                transl_comp = -0.015 * state._get("v_long")

                # 3) Setpoint del PID de roll
                # El helicóptero no siempre debe estar nivelado.        
                # A veces necesita un pequeño ángulo para mantenerse estable.                                   

                roll_setpoint = lat_comp + transl_comp

                # Limitar el setpoint a un rango razonable
                roll_setpoint = max(-5, min(5, roll_setpoint))

                # 4) PID interno de roll
                self.pid_roll.setpoint = roll_setpoint
                cyclic_roll = self.pid_roll(state.roll)

                # Limitar salida
                cyclic_roll = max(-0.7, min(0.7, cyclic_roll))

                # Yaw (mantener rumbo)
                target_heading = 0   # rumbo deseado

                # Calcular error de heading normalizado
                heading_error = target_heading - state.heading
                if heading_error > 180:
                    heading_error -= 360
                elif heading_error < -180:
                    heading_error += 360

                # El PID debe recibir el heading REAL, no el error
                self.pid_yaw.setpoint = target_heading
                pedals = self.pid_yaw(state.heading)

                # Limitar salida
                pedals = max(-5, min(5, pedals))
                
                # =============================================
                # ENVÍO DE COMANDOS A X-PLANE
                # =============================================
                
                self.client.sendCTRL([cyclic_pitch, cyclic_roll, pedals])
                self.client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees', collective_output)
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            logging.info("Controlador detenido")
        finally:
            self.client.close()
            logging.info("Conexión con X-Plane cerrada")
# =============================================
# EJECUCIÓN PRINCIPAL
# =============================================%%

if __name__ == "__main__":
    controller = FlightController()
    
    controller.run()