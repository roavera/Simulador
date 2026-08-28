# =========================
# 1_F_1A_1_Vuelo Nivelado.py
# Crucero
# Validar el desempeño a velocidades por encima de la velocidad de maxima resistencia.
# Determinar la performance durante un vuelo en crucero.
#
# Condiciones Iniciales:
#     Altitud:      4500 ft
#     Velocidad:    50 kts
#
# Condiciones finales:
#     Altitud:      4500 ft
#     Velocidad:    120 kts
# =========================

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

FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444

# Condiciones de vuelo
LAT             = -34.554
LON             = -58.425
ALTITUDE_FT     = 4500          # ft
INITIAL_SPEED   = 50            # kts
FINAL_SPEED     = 120           # kts
SPEED_RAMP_RATE = 0.05          # kts por iteración (~2.5 kts/s a 50Hz)

# Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    filename='vuelo_nivelado.log'
)

# =============================================
# CONTROLADOR DE VUELO NIVELADO
# =============================================

class FlightController:
    def __init__(self, client):
        self.client = client
        self.current_target_speed = INITIAL_SPEED   # Rampa de velocidad
        self._initialize_pids()

        self.datarefs = [
            'sim/flightmodel/position/y_agl',                                   # 0:  altitud AGL          [m]
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot',    # 1:  heading mag          [°]
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',                # 2:  airspeed             [kts]
            'sim/flightmodel/position/local_vx',                                # 3:  vel. lateral         [m/s]
            'sim/flightmodel/position/local_vy',                                # 4:  climb rate           [m/s]
            'sim/flightmodel/position/local_vz',                                # 5:  vel. longitudinal    [m/s]
            'sim/flightmodel/position/local_x',                                 # 6:  pos. X               [m]
            'sim/flightmodel/position/local_y',                                 # 7:  pos. Y               [m]
            'sim/flightmodel/position/local_z',                                 # 8:  pos. Z               [m]
            'sim/flightmodel/position/theta',                                   # 9:  pitch                [°]
            'sim/flightmodel/position/phi',                                     # 10: roll                 [°]
            'sim/flightmodel/position/psi',                                     # 11: yaw                  [°]
            'sim/flightmodel/position/latitude',                                # 12: latitud              [°]
            'sim/flightmodel/position/longitude',                               # 13: longitud             [°]
            'sim/cockpit2/engine/actuators/prop_angle_degrees',                 # 14: collective           [°]
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

    # =============================================
    # PIDs
    # =============================================

    def _initialize_pids(self):

        # Collective → mantiene altitud (controla climb rate)
        self.pid_collective = PID(1.0, 0.01, 0.1, setpoint=0)
        self.pid_collective.output_limits = (-4, 4)

        # Pitch → controla airspeed (nariz abajo = acelera)
        self.pid_pitch = PID(0.05, 0.0001, 0.015, setpoint=FINAL_SPEED)
        self.pid_pitch.output_limits = (-0.7, 0.7)

        # Roll → mantiene alas niveladas (phi = 0°)
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=0.0)
        self.pid_roll.output_limits = (-0.7, 0.7)

        # Yaw → mantiene heading de referencia
        self.pid_yaw = PID(0.04, 0.0, 0.001, setpoint=0)
        self.pid_yaw.output_limits = (-1.0, 1.0)

    # =============================================
    # CÁLCULO DE CONTROLES
    # =============================================

    def calculate_collective_control(self, target_altitude_m, current_altitude_m,
                                     climb_rate, current_collective, pitch_cmd):
        MAX_CLIMB_RATE = 0.508                                  # ~100 ft/min en m/s
        error_alt      = 0.1 * (target_altitude_m - current_altitude_m)
        w_ref          = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, error_alt))

        self.pid_collective.setpoint = w_ref
        collective_change = self.pid_collective(climb_rate)
        new_collective    = current_collective + collective_change

        # Compensación de acoplamiento pitch-colectivo
        if pitch_cmd < 0:
            new_collective += abs(pitch_cmd) * 2.5

        return max(-4.0, min(11.0, new_collective))

    def calculate_pitch_control(self, target_speed, current_speed):
        self.pid_pitch.setpoint = target_speed
        pitch_pid_out = self.pid_pitch(current_speed)
        return -pitch_pid_out                                   # Inversión: nariz abajo para acelerar

    def calculate_roll_control(self, current_roll):
        return self.pid_roll(current_roll)                      # Setpoint fijo en 0°

    def calculate_yaw_control(self, heading_ref, current_heading):
        error = (heading_ref - current_heading + 360) % 360
        if error > 180:
            error -= 360
        self.pid_yaw.setpoint = 0
        return self.pid_yaw(-error)

    # =============================================
    # REGISTRO CSV
    # =============================================

    def _init_csv(self):
        self.csv_file = open('vuelo_nivelado_datos.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'tiempo_s', 'airspeed_kts', 'altitud_ft', 'climb_rate_fpm',
            'pitch_deg', 'roll_deg', 'heading_deg',
            'collective_deg', 'pitch_cmd', 'roll_cmd', 'yaw_cmd',
            'torque', 'prop_ratio',
            'peso_fijo_kg', 'combustible_kg', 'CG'
        ])

    def _log_csv(self, t, data, pitch_cmd, roll_cmd, yaw_cmd, collective_cmd):
        airspeed    = data[2][0]
        altitude_m  = data[0][0]
        climb_rate  = data[4][0]
        pitch       = data[9][0]
        roll        = data[10][0]
        heading     = data[1][0]
        collective  = data[14][0]
        torque      = data[15][0]
        prop_ratio  = data[21][0]
        peso_fijo   = data[22][0]
        combustible = data[23][0]
        cg          = data[24][0]

        self.csv_writer.writerow([
            round(t, 2),
            round(airspeed, 2),
            round(altitude_m * (1/FT_TO_METERS), 1),
            round(climb_rate * 196.85, 1),          # m/s → ft/min
            round(pitch, 3),
            round(roll, 3),
            round(heading, 1),
            round(collective, 3),
            round(pitch_cmd, 4),
            round(roll_cmd, 4),
            round(yaw_cmd, 4),
            round(collective_cmd, 4),
            torque,
            prop_ratio,
            peso_fijo,
            combustible,
            cg
        ])

    # =============================================
    # BUCLE PRINCIPAL
    # =============================================

    def run(self):
        target_altitude_m = ALTITUDE_FT * FT_TO_METERS

        # Capturar heading de referencia al arrancar
        init_data   = self.client.getDREFs(self.datarefs)
        heading_ref = float(init_data[1][0])
        logging.info(f"Heading de referencia: {heading_ref:.1f}°")

        self._init_csv()
        t_start = time.time()

        try:
            while True:
                data = self.client.getDREFs(self.datarefs)

                altitude_m         = float(data[0][0])
                heading            = float(data[1][0])
                airspeed           = float(data[2][0])
                climb_rate         = float(data[4][0])
                roll_actual        = float(data[10][0])
                current_collective = float(data[14][0])

                # --- Rampa de velocidad ---
                if self.current_target_speed < FINAL_SPEED:
                    self.current_target_speed = min(
                        self.current_target_speed + SPEED_RAMP_RATE,
                        FINAL_SPEED
                    )

                # --- Comandos de control ---
                pitch_cmd      = self.calculate_pitch_control(self.current_target_speed, airspeed)
                collective_cmd = self.calculate_collective_control(
                    target_altitude_m, altitude_m, climb_rate, current_collective, pitch_cmd
                )
                roll_cmd       = self.calculate_roll_control(roll_actual)
                yaw_cmd        = self.calculate_yaw_control(heading_ref, heading)

                # --- Envío a X-Plane ---
                self.client.sendDREF("sim/cockpit2/controls/yoke_pitch_ratio",   pitch_cmd)
                self.client.sendDREF("sim/cockpit2/controls/yoke_roll_ratio",    roll_cmd)
                self.client.sendDREF("sim/cockpit2/controls/yoke_heading_ratio", yaw_cmd)
                self.client.sendDREF("sim/cockpit2/engine/actuators/prop_angle_degrees", collective_cmd)

                # --- Registro ---
                t_elapsed = time.time() - t_start
                self._log_csv(t_elapsed, data, pitch_cmd, roll_cmd, yaw_cmd, collective_cmd)

                logging.info(
                    f"t={t_elapsed:.1f}s | IAS={airspeed:.1f}kts (target={self.current_target_speed:.1f}) "
                    f"| Alt={altitude_m/FT_TO_METERS:.0f}ft | VS={climb_rate*196.85:.0f}fpm"
                )

                time.sleep(0.02)    # ~50 Hz

        except KeyboardInterrupt:
            logging.info("Vuelo detenido por el usuario")
        finally:
            self.csv_file.close()
            logging.info("Archivo CSV cerrado")


# =============================================
# EJECUCIÓN PRINCIPAL
# =============================================

if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:

        # Posición inicial (altitud en metros para sendPOSI)
        client.sendPOSI([LAT, LON, ALTITUDE_FT * FT_TO_METERS, 0, 0, 0], 0)

        # Forzar velocidad inicial con override IAS
        client.sendDREF("sim/operation/override/override_ias", 1)
        client.sendDREF("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", INITIAL_SPEED)
        time.sleep(2)
        client.sendDREF("sim/operation/override/override_ias", 0)

        controller = FlightController(client)
        controller.run()