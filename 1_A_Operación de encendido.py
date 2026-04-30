# Prueba 1: IDLE

from collections import deque
import time
import csv
from XPlaneConnect.Python3.src import xpc

class EngineStabilityMonitor:
    def __init__(self):

        # =========================
        # CONEXIÓN
        # =========================
        self.client = xpc.XPlaneConnect()

        # =========================
        # TIEMPO
        # =========================
        self.fs = 20.0
        self.dt = 1.0 / self.fs

        # =========================
        # DATAREFS
        # =========================
        self.drefs = [
            "sim/flightmodel/engine/ENGN_TRQ",
            "sim/flightmodel/rotor/main_rotor_rpm",
            "sim/flightmodel/engine/ENGN_FF_",
            "sim/flightmodel2/engines/N1_percent",
            "sim/flightmodel2/engines/N2_percent",
            "sim/flightmodel2/engines/ITT_deg_C"
        ]

        self.keys = ["torque", "rpm", "ff", "n1", "n2", "itt"]

        # =========================
        # BUFFER (ventana móvil)
        # =========================
        self.window_size = int(10 * self.fs)

        self.buffers = {
            k: deque(maxlen=self.window_size) for k in self.keys
        }


    # =========================
    # CONTROL NEUTRO
    # =========================
    def send_neutral_controls(self):
        self.client.sendCTRL([
            0.0# pitch
            0.0,  # roll
            0.0,  # yaw
            0.0   # throttle / collective
        ])

    # =========================
    # LECTURA
    # =========================
    def read_data(self):
        values = self.client.getDREFs(self.drefs)
        #values = [v[1] for v in values]

        data = dict(zip(self.keys, values))

        for k in self.keys:
            self.buffers[k].append(data[k])

        return data

    # =========================
    # ESTABILIDAD
    # =========================
    def is_stable(self):

        # esperar a llenar ventana
        if any(len(buf) < self.window_size for buf in self.buffers.values()):
            return False

        for key, buffer in self.buffers.items():

            mean = sum(buffer) / len(buffer)
            max_dev = max(abs(x - mean) for x in buffer)

            tol, is_percent = self.criteria[key]

            if is_percent:
                limit = abs(mean) * (tol / 100.0)
            else:
                limit = tol

            if max_dev > limit:
                return False

        return True

    # =========================
    # TEST PRINCIPAL
    # =========================
    def run_test(self, duration=10):

        log = []
        t0 = time.time()

        while time.time() - t0 < duration:
            loop_start = time.time()

            # mantener neutro
            self.send_neutral_controls()

            # leer datos
            data = self.read_data()

            t = time.time() - t0

            log.append([
                t,
                data["torque"],
                data["rpm"],
                data["ff"],
                data["n1"],
                data["n2"],
                data["itt"],  
            ])

            print(f"t={t:.2f}s")

            # mantener frecuencia
            elapsed = time.time() - loop_start
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("Fin de prueba")
        return log

    # =========================
    # GUARDADO CSV
    # =========================
    def save_csv(self, log, filename="idle_test.csv"):

        headers = [
            "time_s",
            "torque",
            "rpm",
            "fuel_flow",
            "n1",
            "n2",
            "itt",
            "stable"
        ]

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(log)

        print(f"Datos guardados en {filename}")


# =========================
# MAIN
# =========================
if __name__ == "__main__":

    monitor = EngineStabilityMonitor()

    log = monitor.run_test(duration=10)

    monitor.save_csv(log)