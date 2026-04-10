import math
import time
import csv
import numpy as np
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

# Matriz K (Hay que modificarla, la calculo con matlab)
# PRUEBO CON UN EJEMPLO 
# Estado: [u, v, w, p, q, r, phi, theta, psi, x, y, z]

K = np.array([
    [0, 0, 2.0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5],   # collective
    [1.2, 0, 0, 0, 0.5, 0, 0, 1.5, 0, 0, 0, 0], # pitch
    [0, 1.2, 0, 0.5, 0, 0, 1.5, 0, 0, 0, 0, 0], # roll
    [0, 0, 0, 0, 0, 1.0, 0, 0, 1.5, 0, 0, 0]    # yaw
])

# Parámetros de la aeronave (Bell 407)
HOVER_HEIGHT_FT = 200   # Altitud de hover en pies
COLLECTIVE_TRIM = 5.5   # Grados de paso para mantener el hover

# =============================================
# CONTROLADOR 
# =============================================

class LQRController:
    def __init__(self):
        self.client = xpc.XPlaneConnect()

        self.points = self._load_flight_path()
        self.current_point_index = 0

        # Datarefs necesarios para control de estado completo
        self.datarefs = [
            'sim/flightmodel/position/y_agl',                                   # 0: altitud
            'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot',    # 1: heading
            'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',                # 2: airspeed
            'sim/flightmodel/position/local_vx',                                # 3: v_lat
            'sim/flightmodel/position/local_vy',                                # 4: climb_rate (v_up)
            'sim/flightmodel/position/local_vz',                                # 5: v_long
            'sim/flightmodel/position/local_x',                                 # 6: pos_x
            'sim/flightmodel/position/local_y',                                 # 7: pos_y
            'sim/flightmodel/position/local_z',                                 # 8: pos_z
            'sim/flightmodel/position/theta',                                   # 9: pitch
            'sim/flightmodel/position/phi',                                     # 10: roll
            'sim/flightmodel/position/psi',                                     # 11: yaw (true)
            'sim/flightmodel/position/latitude',                                # 12: lat
            'sim/flightmodel/position/longitude',                               # 13: lon
            'sim/cockpit2/engine/actuators/prop_angle_degrees'                  # 14: collective actual
        ]

    # =============================================
    # Carga de trayectoria desde CSV
    # ============================================= 
    def _load_flight_path(self):
        csv_file = 'trajectories/sabe_saez.csv'
        
        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"No se encontró el archivo de trayectoria: {csv_file}")

        points = []

        def safe_float(value):
            try:
                return float(value)
            except ValueError:
                logging.warning(f"Valor no numérico encontrado en CSV: '{value}', se asignará 0.0")
                return 0.0

        with open(csv_file, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                point = {
                        'altitud': safe_float(row['Altitud (ft)']) * FT_TO_METERS,
                        'latitud': safe_float(row['Lat']),
                        'longitud': safe_float(row['Long']),
                        'airspeed': safe_float(row['Airspeed (kias)']) * KTS_TO_METERS_PER_SEC
                }
                points.append(point)

            if not points:
                raise ValueError("El archivo CSV no contiene datos válidos")

            print(f"Se cargaron {len(points)} puntos de trayectoria desde {csv_file}")
            return points

    # =============================================
    # UTILIDADES 
    # =============================================
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

    # =============================================
    # ESTADO
    # =============================================

    def get_state(self):
        d = self.client.getDREFs(self.datarefs)

        # Velocidades (body/local)
        u = d[3][0]   # vz → forward (aprox)
        v = d[4][0]   # vx → lateral
        w = d[5][0]   # vy → vertical

        # Actitud
        theta = math.radians(d[9][0])   # pitch
        phi   = math.radians(d[10][0])  # roll
        psi   = math.radians(d[11][0])  # yaw

        # Posición
        x = d[6][0]
        y = d[7][0]
        z = d[8][0]

        # Geográficas
        lat = d[12][0]
        lon = d[13][0]

        # Velocidad aire
        airspeed = d[2][0] * KTS_TO_METERS_PER_SEC # convertir de nudos a m/s

        # Aproximación
        p = q = r = 0.0

        state = np.array([
            u, v, w,
            p, q, r,
            phi, theta, psi,
            x, y, z
        ])

        return state, lat, lon, airspeed

    # =============================================
    # REFERENCIA DESDE WAYPOINT
    # =============================================

    # Dado el estado actual, calcula la referencia de control para el siguiente waypoint
    def get_reference(self, lat, lon, x, y, z):

        target = self.points[self.current_point_index]
        
        # Calcular rumbo y distancia al waypoint actual
        bearing = self.calculate_bearing(lat, lon, target["latitud"], target["longitud"])
        distance = self.calculate_distance(lat, lon, target["latitud"], target["longitud"])

        psi_ref = math.radians(bearing) # rumbo hacia el waypoint (en radianes)

        STEP = min (distance, 100) # distancia de referencia a lo largo del rumbo (máximo 100 m)

        # proyección simple local
        x_ref = x + distance * math.cos(psi_ref)
        z_ref = z + distance * math.sin(psi_ref)
        y_ref = target["altitud"]

        u_ref = target["airspeed"]
        v_ref = 0
        w_ref = 0

        ref = np.array([
            u_ref, v_ref, w_ref,
            0, 0, 0,
            0, 0, psi_ref,
            x_ref, y_ref, z_ref
        ])

        if distance < 20:  # Si estamos cerca del waypoint, avanzar al siguiente
            if self.current_point_index < len(self.points) - 1:
                self.current_point_index += 1
                print(f"Nuevo waypoint: {self.current_point_index} - Lat: {target['latitud']:.6f}, Lon: {target['longitud']:.6f}, Alt: {target['altitud']:.1f} m    , Airspeed: {target['airspeed']:.1f} m/s        ")
        return ref

    # =============================================
    # CONTROL
    # =============================================

    def compute_control(self, x, x_ref):
        error = x - x_ref
        u = -K @ error

        # Saturaciones
        collective = np.clip(u[0], -4, 11)
        pitch = np.clip(u[1], -0.7, 0.7)
        roll = np.clip(u[2], -0.7, 0.7)
        yaw = np.clip(u[3], -1, 1)

        return collective, pitch, roll, yaw

    # =============================================
    # LOOP PRINCIPAL
    # =============================================

    def run(self):
        try:
            while True:

                x, lat, lon, airspeed = self.get_state()

                x_ref = self.get_reference(lat, lon, x[9], x[10], x[11])

                collective, pitch, roll, yaw = self.compute_control(x, x_ref)

                # enviar a X-Plane
                self.client.sendCTRL([pitch, roll, yaw])
                self.client.sendDREF(
                    'sim/cockpit2/engine/actuators/prop_angle_degrees',
                    collective
                )

                print(f"Pitch:{pitch:.2f} Roll:{roll:.2f} Yaw:{yaw:.2f} Col:{collective:.2f}")

                time.sleep(0.02)  # 50 Hz

        except KeyboardInterrupt:
            print("Control detenido")
        finally:
            self.client.close()

# =============================================
# MAIN
# =============================================

if __name__ == "__main__":
    controller = LQRController()
    controller.run()