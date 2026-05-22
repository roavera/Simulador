# =========================
# 1_F_1A_1_Vuelo Nivelado.py
# Crucero
# Validar el desempeño a velocidades por encima de la velocidad de máxima resistencia.
# Determinar la performance durante un vuelo en crucero.

# Condiciones Iniciales:
#   Altitud:    4500 ft
#   Velocidad:  50 kts

# Condiciones Finales:
#   Altitud:    4500 ft
#   Velocidad:  120 kts

# Datarefs de interés:
#   sim/flightmodel/engine/ENGN_TRQ
#   sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot
#   sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot
#   sim/cockpit2/controls/yoke_pitch_ratio
#   sim/cockpit2/controls/yoke_roll_ratio
#   sim/cockpit2/controls/yoke_heading_ratio
#   sim/cockpit2/engine/actuators/prop_ratio_all
#   sim/flightmodel/weight/m_fixed
#   sim/flightmodel/weight/m_fuel1
#   sim/cockpit2/gauges/indicators/CG_indicator
# =========================

import time
import csv
import os
import logging
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

# =========================================================
# LOGGING
# =========================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("vuelo_nivelado.log", mode="w"),
    ],
)
log = logging.getLogger(__name__)

# =========================================================
# CONVERSIÓN DE UNIDADES
# =========================================================

FT_TO_METERS        = 0.3048
KTS_TO_MPS          = 0.514444   # 1 nudo = 0.514444 m/s
FPM_TO_MPS          = 0.00508    # ft/min → m/s

# =========================================================
# CONDICIONES DE VUELO
# =========================================================

LAT             = -34.554   # grados
LON             = -58.425   # grados
ALTITUDE        = 4500      # ft
INITIAL_SPEED   = 50        # kts IAS
FINAL_SPEED     = 120       # kts IAS

# =========================================================
# PARÁMETROS DE CONTROL
# =========================================================

DT                  = 0.05      # s  – ciclo de control (20 Hz)
MAX_CLIMB_RATE_FPM  = 100       # ft/min  – máxima tasa vertical aceptable
ALT_TOLERANCE_FT    = 10        # ft      – margen de altitud considerado estable
SPEED_TOLERANCE_KTS = 2         # kts     – margen de velocidad para considerar estabilización
STABLE_CYCLES       = 100       # ciclos estables consecutivos para dar la prueba por finalizada

# Rampas de aceleración
ACCEL_RAMP_KTS_PER_SEC = 2.0    # kts/s – tasa máx. de incremento del setpoint de velocidad

# =========================================================
# DATAREFS
# =========================================================

DREF = {
    # Posición / cinemática
    "altitude_msl":     "sim/flightmodel/position/elevation",           # m MSL
    "ias_kts":          "sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
    "climb_rate_mps":   "sim/flightmodel/position/vh_ind",              # m/s
    # Actitud
    "pitch_deg":        "sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot",
    "roll_deg":         "sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot",
    "heading_deg":      "sim/flightmodel/position/psi",
    # Controles
    "collective":       "sim/cockpit2/controls/collective_pitch",
    "yoke_pitch":       "sim/cockpit2/controls/yoke_pitch_ratio",
    "yoke_roll":        "sim/cockpit2/controls/yoke_roll_ratio",
    "yoke_yaw":         "sim/cockpit2/controls/yoke_heading_ratio",
    # Motor
    "torque":           "sim/flightmodel/engine/ENGN_TRQ",
    "prop_ratio":       "sim/cockpit2/engine/actuators/prop_ratio_all",
    # Peso / CG
    "m_fixed":          "sim/flightmodel/weight/m_fixed",
    "m_fuel":           "sim/flightmodel/weight/m_fuel1",
    "cg":               "sim/cockpit2/gauges/indicators/CG_indicator",
}

# =========================================================
# CSV OUTPUT
# =========================================================

CSV_FILE    = "vuelo_nivelado_data.csv"
CSV_HEADERS = [
    "timestamp_s",
    "ias_kts",
    "altitude_ft",
    "climb_rate_fpm",
    "pitch_deg",
    "roll_deg",
    "heading_deg",
    "collective",
    "yoke_pitch",
    "yoke_roll",
    "yoke_yaw",
    "torque",
    "speed_setpoint_kts",
    "phase",
]

# =========================================================
# CONTROLADOR
# =========================================================

class FlightController:
    """Controlador PID para vuelo nivelado en helicóptero sobre X-Plane."""

    def __init__(self, client: xpc.XPlaneConnect):
        self.client = client
        self.phase  = "acceleration"   # "acceleration" | "cruise"
        self._speed_setpoint = float(INITIAL_SPEED) # 
        self._stable_count   = 0
        self._t0             = time.time()
        self._heading_ref    = None    # se captura al inicio del run()

        self._init_pids()
        log.info("FlightController inicializado.")

    # ----------------------------------------------------------
    # Inicialización de PIDs
    # ----------------------------------------------------------

    def _init_pids(self):
        max_climb_mps = MAX_CLIMB_RATE_FPM * FPM_TO_MPS

        # Collective – mantiene altitud controlando climb rate
        self.pid_collective = PID(1.0, 0.01, 0.1, setpoint=0.0)
        self.pid_collective.output_limits = (-3.0, 3.0)

        # Pitch (yoke longitudinal) – controla IAS
        self.pid_pitch = PID(0.05, 0.0001, 0.015, setpoint=FINAL_SPEED)
        self.pid_pitch.output_limits = (-0.5, 0.5)

        # Roll (yoke lateral) – mantiene alas niveladas
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=0.0)
        self.pid_roll.output_limits = (-0.7, 0.7)

        # Yaw (pedales) – mantiene heading
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0.0)
        self.pid_yaw.output_limits = (-5.0, 5.0)

    # ----------------------------------------------------------
    # Lectura de estado desde X-Plane
    # ----------------------------------------------------------

    def _read_state(self) -> dict:
        keys   = list(DREF.keys())
        drefs  = [DREF[k] for k in keys]
        values = self.client.getDREFs(drefs)
        state  = {k: v[0] for k, v in zip(keys, values)}

        # Convertir unidades a sistema consistente
        state["altitude_ft"]    = state["altitude_msl"] / FT_TO_METERS
        state["climb_rate_fpm"] = state["climb_rate_mps"] / FPM_TO_MPS
        return state

    # ----------------------------------------------------------
    # Control de collective (altitud)
    # ----------------------------------------------------------

    def _collective_output(self, state: dict, target_alt_ft: float) -> float:
        max_climb_mps = MAX_CLIMB_RATE_FPM * FPM_TO_MPS
        error_alt_m   = (target_alt_ft - state["altitude_ft"]) * FT_TO_METERS
        # Ganancia reducida para suavizar la respuesta
        w_ref = max(-max_climb_mps, min(max_climb_mps, 0.1 * error_alt_m))
        self.pid_collective.setpoint = w_ref
        delta = self.pid_collective(state["climb_rate_mps"])
        new_collective = state["collective"] + delta
        return max(-4.0, min(11.0, new_collective))

    # ----------------------------------------------------------
    # Control de pitch (velocidad)
    # ----------------------------------------------------------

    def _pitch_output(self, state: dict) -> float:
        self.pid_pitch.setpoint = self._speed_setpoint
        return self.pid_pitch(state["ias_kts"])

    # ----------------------------------------------------------
    # Control de roll (actitud lateral)
    # ----------------------------------------------------------

    def _roll_output(self, state: dict) -> float:
        return self.pid_roll(state["roll_deg"])

    # ----------------------------------------------------------
    # Control de yaw (heading)
    # ----------------------------------------------------------

    def _yaw_output(self, state: dict) -> float:
        # Error de heading con manejo de wrap-around ±180°
        error_heading = state["heading_deg"] - self._heading_ref
        if error_heading > 180:
            error_heading -= 360
        elif error_heading < -180:
            error_heading += 360
        self.pid_yaw.setpoint = 0.0
        return self.pid_yaw(error_heading)

    # ----------------------------------------------------------
    # Rampa de aceleración
    # ----------------------------------------------------------

    def _update_speed_setpoint(self):
        if self._speed_setpoint < FINAL_SPEED:
            self._speed_setpoint = min(
                FINAL_SPEED,
                self._speed_setpoint + ACCEL_RAMP_KTS_PER_SEC * DT,
            )
            self.pid_pitch.setpoint = self._speed_setpoint

    # ----------------------------------------------------------
    # Envío de controles a X-Plane
    # ----------------------------------------------------------

    def _send_controls(self, collective, yoke_pitch, yoke_roll, yoke_yaw):
        self.client.sendDREF(DREF["collective"],  collective)
        self.client.sendDREF(DREF["yoke_pitch"],  yoke_pitch)
        self.client.sendDREF(DREF["yoke_roll"],   yoke_roll)
        self.client.sendDREF(DREF["yoke_yaw"],    yoke_yaw)

    # ----------------------------------------------------------
    # Detección de estabilización
    # ----------------------------------------------------------

    def _is_stable(self, state: dict) -> bool:
        alt_ok   = abs(state["altitude_ft"] - ALTITUDE) < ALT_TOLERANCE_FT
        speed_ok = abs(state["ias_kts"] - FINAL_SPEED) < SPEED_TOLERANCE_KTS
        return alt_ok and speed_ok

    # ----------------------------------------------------------
    # Loop principal
    # ----------------------------------------------------------

    def run(self, target_altitude_ft: float):
        """Ejecuta el loop de control hasta alcanzar crucero estabilizado."""

        # Capturar heading de referencia
        state = self._read_state()
        self._heading_ref = state["heading_deg"]
        log.info(
            f"Iniciando loop | Alt: {state['altitude_ft']:.0f} ft | "
            f"IAS: {state['ias_kts']:.1f} kts | Heading ref: {self._heading_ref:.1f}°"
        )

        with open(CSV_FILE, "w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=CSV_HEADERS)
            writer.writeheader()

            try:
                while True:
                    t_loop = time.time()
                    state  = self._read_state()

                    # ── Actualizar fase ──────────────────────────
                    if self.phase == "acceleration":
                        self._update_speed_setpoint()
                        if abs(state["ias_kts"] - FINAL_SPEED) < SPEED_TOLERANCE_KTS:
                            self.phase = "cruise"
                            log.info(
                                f"Fase CRUISE alcanzada | IAS: {state['ias_kts']:.1f} kts"
                            )

                    # ── Cálculo de controles ─────────────────────
                    collective = self._collective_output(state, target_altitude_ft)
                    yoke_pitch = self._pitch_output(state)
                    yoke_roll  = self._roll_output(state)
                    yoke_yaw   = self._yaw_output(state)

                    self._send_controls(collective, yoke_pitch, yoke_roll, yoke_yaw)

                    # ── Registro CSV ─────────────────────────────
                    writer.writerow({
                        "timestamp_s":        round(time.time() - self._t0, 3),
                        "ias_kts":            round(state["ias_kts"], 2),
                        "altitude_ft":        round(state["altitude_ft"], 1),
                        "climb_rate_fpm":     round(state["climb_rate_fpm"], 1),
                        "pitch_deg":          round(state["pitch_deg"], 2),
                        "roll_deg":           round(state["roll_deg"], 2),
                        "heading_deg":        round(state["heading_deg"], 1),
                        "collective":         round(collective, 3),
                        "yoke_pitch":         round(yoke_pitch, 4),
                        "yoke_roll":          round(yoke_roll, 4),
                        "yoke_yaw":           round(yoke_yaw, 4),
                        "torque":             round(state["torque"], 2),
                        "speed_setpoint_kts": round(self._speed_setpoint, 1),
                        "phase":              self.phase,
                    })

                    # ── Condición de fin ─────────────────────────
                    if self._is_stable(state):
                        self._stable_count += 1
                        if self._stable_count >= STABLE_CYCLES:
                            log.info(
                                f"Vuelo estabilizado. IAS: {state['ias_kts']:.1f} kts | "
                                f"Alt: {state['altitude_ft']:.0f} ft. Finalizando."
                            )
                            break
                    else:
                        self._stable_count = 0

                    # ── Timing preciso ───────────────────────────
                    elapsed = time.time() - t_loop
                    sleep_t = max(0.0, DT - elapsed)
                    time.sleep(sleep_t)

            except KeyboardInterrupt:
                log.warning("Loop interrumpido por el usuario (Ctrl+C).")

        log.info(f"Datos guardados en '{CSV_FILE}'.")


# =========================================================
# PUNTO DE ENTRADA
# =========================================================

if __name__ == "__main__":
    try:
        with xpc.XPlaneConnect() as client:
            log.info("Conexión con X-Plane establecida.")

            # ── Posición y velocidad inicial ─────────────────
            client.sendPOSI([LAT, LON, ALTITUDE, 0, 0, 0], 0)

            client.sendDREF("sim/operation/override/override_ias", 1)
            client.sendDREF(DREF["ias_kts"], INITIAL_SPEED)
            time.sleep(2)
            client.sendDREF("sim/operation/override/override_ias", 0)

            log.info(
                f"Posición inicial: {LAT}°, {LON}° | "
                f"Alt: {ALTITUDE} ft | IAS: {INITIAL_SPEED} kts"
            )

            # ── Ejecutar controlador ─────────────────────────
            controller = FlightController(client)
            controller.run(ALTITUDE)

    except ConnectionRefusedError:
        log.error(
            "No se pudo conectar con X-Plane. "
            "Verificá que el simulador esté corriendo y que XPlaneConnect esté activo."
        )
    except Exception as e:
        log.exception(f"Error inesperado: {e}")