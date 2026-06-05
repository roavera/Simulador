import math
import time
import logging
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =============================================
# CONSTANTES Y CONFIGURACIÓN UNIFICADA
# =============================================

# Conversión de unidades
FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444  # 1 nudo = 0.514444 m/s
METERS_TO_FT = 1 / FT_TO_METERS

# Condiciones Iniciales y Finales
LAT = -34.554          # grados - Inicio en el mapa
LON = -58.425          # grados - Inicio en el mapa
ALTITUDE = 4500        # ft - Altitud a mantener
INITIAL_SPEED = 50     # kts - Velocidad inicial
FINAL_SPEED = 120      # kts - Velocidad final objetivo
VERTICAL_SPEED = 0     # ft/min - Velocidad vertical objetivo (manejada por el PID de altitud)
SPEED_ROLL = 0         # kts - Mantener alas niveladas

# Configuración de logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='prueba_vuelo_nivelado.log'
)

# =============================================
# CLASE PRINCIPAL DEL CONTROLADOR
# =============================================

class FlightController:
    def __init__(self):
        self.client = xpc.XPlaneConnect()
        self.start_time = time.time()
        self.flight_phase = 'stabilizing' 
        
        self._initialize_pids()
        
        # Datarefs de lectura esenciales para la prueba
        self.datarefs = [
            'sim/flightmodel/position/y_agl',                                # 0: altitud AGL (usaremos elevation si aplica, aquí usamos la de X-Plane)
            'sim/flightmodel/position/elevation',                            # 1: altitud MSL (Real)
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',             # 2: airspeed
            'sim/flightmodel/position/local_vy',                             # 3: climb rate
            'sim/flightmodel/position/theta',                                # 4: pitch
            'sim/flightmodel/position/phi',                                  # 5: Roll
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot', # 6: heading
            'sim/cockpit2/engine/actuators/prop_angle_degrees',              # 7: collective
        ]

    def _initialize_pids(self):
        """Inicializa los controladores PID para la prueba de crucero."""
        # PID de Colectivo (Mantiene Altitud)
        self.pid_collective = PID(0.5, 0.01, 0.1, setpoint=ALTITUDE * FT_TO_METERS)
        self.pid_collective.output_limits = (-4.0, 11.0)
        
        # PID de Pitch (Controla Velocidad) - Ganancias negativas porque picar (pitch negativo) acelera
        self.pid_pitch = PID(-0.02, -0.001, -0.01, setpoint=INITIAL_SPEED)
        self.pid_pitch.output_limits = (-0.5, 0.5)
        
        # PID de Roll (Mantiene 0 inclinación)
        self.pid_roll = PID(0.05, 0.001, 0.01, setpoint=SPEED_ROLL)
        self.pid_roll.output_limits = (-0.5, 0.5)
        
        # PID de Yaw (Mantiene Rumbo)
        self.pid_yaw = PID(0.01, 0.0001, 0.001, setpoint=0)  
        self.pid_yaw.output_limits = (-0.5, 0.5)

    def set_initial_conditions(self):
        """Teletransporta la aeronave a LAT, LON y ALTITUDE."""
        logging.info("Forzando condiciones iniciales de posición y altitud...")
        print("Ubicando la aeronave en el punto de inicio...")
        
        alt_metros = ALTITUDE * FT_TO_METERS
        # sendPOSI: [Lat, Lon, Alt(m), Pitch, Roll, Yaw, Gear]
        self.client.sendPOSI([LAT, LON, alt_metros, 0, 0, 0, 1])
        
        time.sleep(2) # Dar tiempo al simulador para reubicar la aeronave
        
        # Guardar rumbo inicial para mantenerlo recto
        self.initial_heading = self.client.getDREF('sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot')[0]
        self.pid_yaw.setpoint = self.initial_heading
        
        logging.info(f"Posición seteada. Rumbo inicial fijado en {self.initial_heading:.1f}°")

    def run(self):
        """Bucle principal de la prueba."""
        try:
            self.set_initial_conditions()
            logging.info("Iniciando bucle de control de vuelo nivelado.")
            print(f"Buscando estabilidad a {INITIAL_SPEED} kts y {ALTITUDE} ft...")

            while True:
                current_time = time.time() - self.start_time
                
                # Transición de INITIAL_SPEED a FINAL_SPEED luego de 20 segundos
                if self.flight_phase == 'stabilizing' and current_time > 20.0:
                    self.flight_phase = 'cruising'
                    self.pid_pitch.setpoint = FINAL_SPEED # Cambiamos el objetivo del PID de velocidad
                    print(f"\n>>> Fase estabilizada. Acelerando a la velocidad final: {FINAL_SPEED} kts <<<")
                    logging.info("Transición de velocidad: Iniciando aceleración.")

                # 1. Leer Datos
                dref_values = self.client.getDREFs(self.datarefs)
                
                # Extraer valores asumiendo validación simple
                altitude_m = dref_values[1][0] if dref_values[1] else 0.0
                airspeed_kts = dref_values[2][0] if dref_values[2] else 0.0
                pitch = dref_values[4][0] if dref_values[4] else 0.0
                roll = dref_values[5][0] if dref_values[5] else 0.0
                heading = dref_values[6][0] if dref_values[6] else 0.0
                collective_current = dref_values[7][0] if dref_values[7] else 0.0
                
                altitude_ft = altitude_m * METERS_TO_FT

                # 2. Calcular Controles (PIDs)
                # Colectivo -> Control de Altitud
                collective_change = self.pid_collective(altitude_m)
                # Suavizado del colectivo respecto a su posición actual
                new_collective = max(-4.0, min(11.0, collective_current + (collective_change * 0.1)))
                
                # Pitch -> Control de Velocidad
                cyclic_pitch = self.pid_pitch(airspeed_kts)
                
                # Roll -> Mantener nivelado
                cyclic_roll = self.pid_roll(roll)
                
                # Yaw -> Mantener rumbo (ajustando la lectura circular 0-360)
                heading_error = (self.initial_heading - heading + 360) % 360
                if heading_error > 180: heading_error -= 360
                pedals = self.pid_yaw(-heading_error)

                # 3. Enviar Comandos
                self.client.sendCTRL([cyclic_pitch, cyclic_roll, pedals])
                self.client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees', new_collective)

                # Monitoreo en consola cada 1 segundo
                if int(current_time * 20) % 20 == 0: 
                    print(f"Altitud: {altitude_ft:.0f} ft | Velocidad: {airspeed_kts:.1f} kts | Fase: {self.flight_phase}")
                
                time.sleep(0.05) # Ejecutar a 20Hz

        except KeyboardInterrupt:
            print("\nPrueba finalizada por el usuario.")
            logging.info("Prueba detenida.")
        finally:
            self.client.close()
            logging.info("Conexión con X-Plane cerrada.")

# =============================================
# EJECUCIÓN PRINCIPAL
# =============================================

if __name__ == "__main__":
    controller = FlightController()
    controller.run()