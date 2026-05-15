import time
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =========================================================
# CONVERSIÓN DE UNIDADES
# =========================================================
FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444

# =========================================================
# CONDICIONES INICIALES
# =========================================================
LAT = -34.554
LON = -58.425
ALTITUDE = 4500        # ft
INITIAL_SPEED = 0      # kts (hover, sin avance)
FINAL_SPEED = 0        # kts (hover, sin avance)

# =========================================================
# CONTROLADOR PID
# =========================================================
class FlightController:
    def __init__(self, client):
        self.client = client
        self._initialize_pids()
        self.current_collective = 0.0

    def _initialize_pids(self):
        # Collective (altitude control)
        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-3, 3)

        # Pitch -> controla velocidad longitudinal (vx)
        self.pid_pitch = PID(0.05, 0.0001, 0.015, setpoint=0)
        self.pid_pitch.output_limits = (-0.7, 0.7)

        # Roll -> controla velocidad lateral (vy)
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=0)
        self.pid_roll.output_limits = (-0.7, 0.7)

        # Yaw -> estabiliza orientación
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0)
        self.pid_yaw.output_limits = (-5, 5)

    def calculate_collective(self, target_altitude, current_altitude, climb_rate):
        MAX_CLIMB_RATE = 100 * 0.00508  # 100 ft/min en m/s
        error_alt = 0.1 * (target_altitude - current_altitude)
        w_ref = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, error_alt))
        self.pid_collective.setpoint = w_ref
        collective_change = self.pid_collective(climb_rate)
        self.current_collective += collective_change
        return max(-4.0, min(11.0, self.current_collective))

    def run(self, target_altitude):
        while True:
            # Leer estado actual
            posi = self.client.getPOSI(0)
            current_altitude = posi[2] / FT_TO_METERS
            climb_rate = self.client.getDREF("sim/flightmodel/position/vh_ind")[0]  # ft/min
            vx = self.client.getDREF("sim/flightmodel/position/local_vx")[0]  # m/s
            vy = self.client.getDREF("sim/flightmodel/position/local_vy")[0]  # m/s
            yaw_rate = self.client.getDREF("sim/flightmodel/position/y_agl")[0]

            # Calcular controles
            collective = self.calculate_collective(target_altitude, current_altitude, climb_rate)
            pitch_cmd = self.pid_pitch(vx)
            roll_cmd = self.pid_roll(vy)
            yaw_cmd = self.pid_yaw(yaw_rate)

            # Enviar comandos a X-Plane
            self.client.sendCTRL([roll_cmd, pitch_cmd, yaw_cmd, collective])

            time.sleep(0.05)

# =========================================================
# MAIN
# =========================================================
if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        # Posición inicial
        values = [LAT, LON, ALTITUDE, 0, 0, 0]
        client.sendPOSI(values, 0)

        # Activar override IAS para fijar velocidad inicial (0 kts)
        client.sendDREF("sim/operation/override/override_ias", 1)
        client.sendDREF("sim/flightmodel/position/true_airspeed", INITIAL_SPEED * KTS_TO_METERS_PER_SEC)

        time.sleep(2)  # esperar estabilización

        # 🔽 Desactivar override IAS antes de iniciar el control
        client.sendDREF("sim/operation/override/override_ias", 0)

        controller = FlightController(client)
        controller.run(ALTITUDE)
