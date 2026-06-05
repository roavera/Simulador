import math
import time
import logging
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =========================================================
# CONSTANTES Y CONFIGURACIÓN
# =========================================================

# Conversión de unidades
FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444    # 1 nudo = 0.514444 m/s
METERS_TO_FT = 1 / FT_TO_METERS

# Condiciones Iniciales y Parámetros
LAT = -34.554          # grados
LON = -58.425          # grados
ALTITUDE = 4500        # ft - altitud que se desea mantener
INITIAL_SPEED = 50     # kts - airspeed inicial
FINAL_SPEED = 120      # kts - airspeed final
VERTICAL_SPEED = 0     # ft/min - vertical speed 
SPEED_ROLL = 0         # kts - velocidad de inclinación deseada

# Umbrales
MIN_AIRSPEED = 5       # velocidad mínima para considerar movimiento (m/s)
HOVER_HEIGHT = 200     # altura para considerar en fase de despegue ft

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
    # CORRECCIÓN 1: Recibir el 'client' en el __init__
    def __init__(self, client):
        self.client = client
        self.flight_phase = 'stabilizing'
        self.start_time = time.time()
        
        # Obtener y guardar el heading inicial al momento de instanciar
        self.initial_heading = self.client.getDREF('sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot')[0]

        # Inicializar PIDs
        self._initialize_pids()
        
        # Configurar datarefs
        self.datarefs = [
            'sim/flightmodel/position/y_agl',                                # 0: altitud    [m]
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot', # 1: heading    [grados magnéticos]
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',             # 2: airspeed   [knots]
            'sim/flightmodel/position/local_vx',                             # 3: v_lat      [m/s]
            'sim/flightmodel/position/local_vy',                             # 4: climb_rate [m/s]
            'sim/flightmodel/position/local_vz',                             # 5: v_long     [m/s]
            'sim/flightmodel/position/local_x',                              # 6: pos_x      [m]
            'sim/flightmodel/position/local_y',                              # 7: pos_y      [m]
            'sim/flightmodel/position/local_z',                              # 8: pos_z      [m]
            'sim/flightmodel/position/theta',                                # 9: pitch      [grados]
            'sim/flightmodel/position/phi',                                  # 10: roll      [grados]
            'sim/flightmodel/position/psi',                                  # 11: yaw       [grados]
            'sim/flightmodel/position/latitude',                             # 12: lat       [grados]
            'sim/flightmodel/position/longitude',                            # 13: lon       [grados]
            'sim/cockpit2/engine/actuators/prop_angle_degrees',              # 14: collective[grados]
        ]

    def _initialize_pids(self):
        """Inicializa todos los controladores PID con sus parámetros"""
        
        # PID para velocidad vertical (climb rate)
        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-4, 11) # Rango típico de colectivo
        
        # CORRECCIÓN 4: Ganancias de pitch negativas y apuntar a INITIAL_SPEED
        self.pid_pitch = PID(-0.04, -0.0001, -0.02, setpoint=INITIAL_SPEED)
        self.pid_pitch.output_limits = (-0.5, 0.5)
        
        # PID para roll
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=SPEED_ROLL)
        self.pid_roll.output_limits = (-0.5, 0.5)
        
        # PID de yaw
        self.pid_yaw = PID(0.015, 0.001, 0.002, setpoint=0)
        self.pid_yaw.output_limits = (-0.5, 0.5)

    def calculate_heading_error(self, desired_heading, current_heading):
        """Calcula la diferencia angular más corta entre dos headings"""
        error = (desired_heading - current_heading + 360) % 360
        return error - 360 if error > 180 else error

    def calculate_yaw_control(self, heading_error, collective, airspeed):
        """Control de yaw basado únicamente en heading y collective."""
        pid_output = self.pid_yaw(heading_error)
        collective_effect = collective * 0.0  
        speed_factor = max(0, 1 - (airspeed / 50))  
        yaw_control = (pid_output + pid_output*collective_effect)*speed_factor
        return max(-0.5, min(0.5, yaw_control))

    def calculate_collective_control(self, target_altitude_m, current_altitude_m, climb_rate, current_collective):
        """Control suave del collective con limitaciones dinámicas"""
        MAX_CLIMB_RATE = 100 * 0.00508  # ≈ 0.508 m/s
        error_alt = 0.1 * (target_altitude_m - current_altitude_m)  
        w_ref = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, error_alt))
        
        self.pid_collective.setpoint = w_ref
        collective_change = self.pid_collective(climb_rate)
        
        new_collective = current_collective + collective_change
        return max(-4.0, min(11.0, new_collective))
    
    # CORRECCIÓN 1: Se eliminó el argumento ALTITUDE de run() ya que lo lee de las constantes globales
    def run(self):
        """Bucle principal de control de vuelo"""
        try:
            logging.info("Iniciando controlador de vuelo nivelado")
            print(f"Manteniendo Rumbo: {self.initial_heading:.1f}° | Altitud: {ALTITUDE} ft")
            
            target_alt_meters = ALTITUDE * FT_TO_METERS

            while True:
                current_time = time.time() - self.start_time
                
                # Lógica de transición de velocidad: luego de 20s, acelerar a 120 kts
                if self.flight_phase == 'stabilizing' and current_time > 20.0:
                    self.flight_phase = 'cruising'
                    self.pid_pitch.setpoint = FINAL_SPEED
                    print(f"\n>>> Transición: Acelerando a la velocidad final: {FINAL_SPEED} kts <<<")

                # Leer datos de X-Plane
                dref_values = self.client.getDREFs(self.datarefs)
                
                # Procesar datos
                altitude = dref_values[0][0] if dref_values[0] else 0.0
                heading = dref_values[1][0] if dref_values[1] else 0.0
                airspeed = dref_values[2][0] if dref_values[2] else 0.0
                speed_vertical = dref_values[4][0] if dref_values[4] else 0.0
                pitch = dref_values[9][0] if dref_values[9] else 0.0
                roll = dref_values[10][0] if dref_values[10] else 0.0
                collective_current = dref_values[14][0] if dref_values[14] else 0.0
                
                # Error de heading
                heading_error = self.calculate_heading_error(self.initial_heading, heading)
                
                # Imprimir telemetría
                if int(current_time * 20) % 20 == 0: 
                    print(f"Alt: {altitude/FT_TO_METERS:.0f}ft | Vel: {airspeed:.1f}kts | Phase: {self.flight_phase}")
                
                # =============================================
                # CÁLCULO DE CONTROLES (CORRECCIÓN 3: Sin Waypoints)
                # =============================================
                
                # 1. Altitud -> Colectivo
                collective_output = self.calculate_collective_control(
                    target_alt_meters, 
                    altitude, 
                    speed_vertical,
                    collective_current
                )

                # 2. Velocidad -> Pitch
                # Le pasamos el airspeed real (en nudos). El PID se encarga del resto.
                cyclic_pitch = self.pid_pitch(airspeed)
                
                # 3. Roll -> Mantener nivelado
                cyclic_roll = self.pid_roll(roll)
                
                # 4. Yaw -> Pedales
                pedals = self.calculate_yaw_control(-heading_error, collective_current, airspeed)
                
                # =============================================
                # ENVÍO DE COMANDOS A X-PLANE
                # =============================================
                
                self.client.sendCTRL([cyclic_pitch, cyclic_roll, pedals])
                self.client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees', collective_output)
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            logging.info("Controlador detenido por el usuario")
            print("\nPrueba de vuelo detenida.")
        except Exception as e:
            logging.error(f"Error crítico: {str(e)}", exc_info=True)
        finally:
            logging.info("Saliendo del bucle principal.")

# =============================================
# EJECUCIÓN PRINCIPAL
# =============================================

if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        
        print("Ajustando posición inicial...")
        # CORRECCIÓN 2: Multiplicar ALTITUDE por FT_TO_METERS
        values = [LAT, LON, ALTITUDE * FT_TO_METERS, 0, 0, 0] # lat, lon, alt_metros, pitch, roll, heading
        client.sendPOSI(values, 0)
        
        # Activar el override de IAS 
        client.sendDREF("sim/operation/override/override_ias", 1)
        # Forzar la velocidad inicial
        client.sendDREF("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", INITIAL_SPEED)
        time.sleep(2) # Esperar a que el simulador estabilice la posición

        # Desactivar el override IAS 
        client.sendDREF("sim/operation/override/override_ias", 0)

        # CORRECCIÓN 1: Pasar client a la clase y ejecutar sin parámetros extra
        controller = FlightController(client)
        controller.run()