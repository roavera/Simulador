import time
import math
import logging
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =========================================================
# CONFIGURACIÓN
# =========================================================

FT_TO_METERS = 0.3048
KTS_TO_MPS = 0.514444

TARGET_ALTITUDE_FT = 4500
TARGET_ALTITUDE_M = TARGET_ALTITUDE_FT * FT_TO_METERS

INITIAL_SPEED = 50
TARGET_SPEED = 120

# =========================================================
# LOGGING
# =========================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)

# =========================================================
# CONTROLADOR
# =========================================================

class LevelFlightController:

    def __init__(self):

        self.client = xpc.XPlaneConnect()

        self.setup_datarefs()

        self._init_pids()

        self.initial_heading = None

    # =====================================================
    # DATAREFS
    # =====================================================

    def setup_datarefs(self):

        self.datarefs = [

            # 0
            'sim/flightmodel/position/y_agl',

            # 1
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',

            # 2
            'sim/flightmodel/position/local_vy',

            # 3
            'sim/flightmodel/position/theta',

            # 4
            'sim/flightmodel/position/phi',

            # 5
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot',

            # 6
            'sim/cockpit2/engine/actuators/collective_ratio'
        ]

    # =====================================================
    # PID SETUP
    # =====================================================

    def _init_pids(self):
        # PID para velocidad vertical (climb rate)
        # Controla el collective para mantener la altitud

        self.pid_collective = PID(1, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-3, 3)

        # -------------------------------------------- #
        # ÁNGULOS 

        # PID para pitch (theta) - cabeceo
        # Nariz arriba para aumentar altitud, nariz abajo para descender
        self.pid_pitch = PID(0.05, 0.0001, 0.015, setpoint=TARGET_SPEED)
        self.pid_pitch.output_limits = (-0.7 , 0.7)             
        
        # PID para roll (phi) - Rolido
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint= 0)
        self.pid_roll.output_limits = (-0.7,0.7)
        
        # PID de yaw (psi) - pedales
        # Controla el rotor de cola
        # contrarresta la inercia y el torque del rotor
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0)  # Parámetros optimizados
        self.pid_yaw.output_limits = (-5,5 )
        # -------------------------------------------- #

    # =====================================================
    # UTILS
    # =====================================================

    def saturate(self, value, min_val, max_val):

        return max(min_val, min(max_val, value))

    def heading_error(self, target, current):

        return (target - current + 180) % 360 - 180

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def run(self):

        logging.info("Iniciando controlador...")

        dt = 0.05

        # Capturar heading inicial

        initial_data = self.client.getDREFs(self.datarefs)

        self.initial_heading = initial_data[5][0]

        logging.info(f"Heading referencia: {self.initial_heading:.1f}")

        while True:

            try:

                data = self.client.getDREFs(self.datarefs)

                altitude = data[0][0]
                airspeed = data[1][0]
                climb_rate = data[2][0]
                pitch = data[3][0]
                roll = data[4][0]
                heading = data[5][0]
                collective = data[6][0]

                # =================================================
                # VELOCIDAD -> PITCH REF
                # =================================================

                pitch_ref = self.pid_speed(airspeed)

                pitch_ref = self.saturate(
                    pitch_ref,
                    -10,
                    10
                )

                # =================================================
                # PITCH ATTITUDE LOOP
                # =================================================

                self.pid_pitch.setpoint = pitch_ref

                pitch_cmd = self.pid_pitch(pitch)

                # =================================================
                # ALTITUD -> VSI REF
                # =================================================

                vsi_ref = self.pid_altitude(altitude)

                # =================================================
                # VSI -> COLLECTIVE
                # =================================================

                self.pid_collective.setpoint = vsi_ref

                collective_delta = self.pid_collective(climb_rate)

                collective_cmd = collective + collective_delta

                collective_cmd = self.saturate(
                    collective_cmd,
                    0.0,
                    1.0
                )

                # =================================================
                # ROLL HOLD
                # =================================================

                roll_cmd = self.pid_roll(roll)

                # =================================================
                # YAW HOLD
                # =================================================

                yaw_error = self.heading_error(
                    self.initial_heading,
                    heading
                )

                yaw_cmd = self.pid_yaw(yaw_error)

                # =================================================
                # ENVIAR CONTROLES
                # =================================================

                self.client.sendCTRL([
                    pitch_cmd,
                    roll_cmd,
                    -yaw_cmd
                ])

                self.client.sendDREF(
                    'sim/cockpit2/engine/actuators/collective_ratio',
                    collective_cmd
                )

                # =================================================
                # LOG
                # =================================================

                logging.info(
                    f"ALT={altitude/FT_TO_METERS:.0f}ft | "
                    f"IAS={airspeed:.1f}kts | "
                    f"PITCH={pitch:.1f}° | "
                    f"COL={collective_cmd:.2f}"
                )

                time.sleep(dt)

            except Exception as e:

                logging.error(e)

                time.sleep(0.1)

# =========================================================
# MAIN
# =========================================================

if __name__ == "__main__":

    controller = LevelFlightController()

    client = controller.client

    # =====================================================
    # POSICIÓN INICIAL
    # =====================================================

    LAT = -34.554
    LON = -58.425

    posi = [

        LAT,
        LON,
        TARGET_ALTITUDE_FT,
        0,
        0,
        0
    ]

    client.sendPOSI(posi)

    time.sleep(3)

    # =====================================================
    # VELOCIDAD INICIAL
    # =====================================================

    initial_speed_mps = INITIAL_SPEED * KTS_TO_MPS

    client.sendDREF(
        'sim/flightmodel/position/local_vz',
        -initial_speed_mps
    )

    time.sleep(2)

    # =====================================================
    # INICIAR CONTROL
    # =====================================================

    controller.run()
