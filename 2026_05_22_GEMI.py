# ========================================================= 
# 1_F_1A_1_Vuelo_Nivelado_Crucero_Estable.py
# Validar el desempeño a velocidades por encima de la velocidad de maxima resistencia. 
# Control multi-bucle en cascada absoluta y adquisición de datos a 20 Hz.
# =========================================================

import time
import csv
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc


# =========================================================
# PARÁMETROS CINEMÁTICOS Y CONFIGURACIÓN DE MUESTREO
# =========================================================
FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444

# Condiciones Iniciales y de Contorno
LAT = -34.554          
LON = -58.425          
ALTITUDE = 10000       # ft - Altitud objetivo (Setpoint Z)
INITIAL_SPEED = 50     # kts - Velocidad inicial

# Condiciones Finales (Setpoints de Estado Estacionario)
FINAL_SPEED = 120      # kts - Velocidad aerodinámica de crucero
VERTICAL_SPEED = 0     # ft/min - Velocidad vertical objetivo
SPEED_ROLL = 0         # deg - Ángulo de alabeo nulo para vuelo rectilíneo

# Parámetros del Sistema Discreto
SAMPLING_RATE = 20.0   # Frecuencia de muestreo del controlador (Hz)
SAMPLE_PERIOD = 1.0 / SAMPLING_RATE

# =========================================================
# LÓGICA DE CONTROL (PLANTA Y TELEMETRÍA)
# =========================================================
class FlightController:
    def __init__(self, client, log_filename="vuelo_crucero_120kts.csv"):
        self.client = client
        self.log_filename = log_filename
        self._init_pids()
        
    def _init_pids(self):
        """
        Sintonización de los controladores PID en el dominio del tiempo discreto.
        El lazo del colectivo ahora entrega valores ABSOLUTOS de paso de pala.
        """
        # Lazo Interno Vertical: Error de Vz (ft/min) -> Paso Colectivo Absoluto (grados)
        # Sintonizado para evitar sobreimpulsos masivos en el eje Z
        self.pid_collective = PID(0.012, 0.004, 0.001, setpoint=0)
        self.pid_collective.output_limits = (-4.0, 11.0) # Límites mecánicos del colectivo

        # Lazo Longitudinal: Velocidad Aerodinámica (kts) -> Cabeceo (Pitch)
        self.pid_pitch = PID(0.02, 0.005, 0.01, setpoint=FINAL_SPEED) 
        self.pid_pitch.output_limits = (-0.5, 0.5) 
        
        # Lazo Lateral: Alabeo (Roll) -> Cíclico Lateral
        self.pid_roll = PID(0.01, 0.005, 0.001, setpoint=SPEED_ROLL)
        self.pid_roll.output_limits = (-0.5, 0.5)
        
        # Lazo Direccional: Rumbo (Heading) -> Pedales / Rotor de cola
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0)
        self.pid_yaw.output_limits = (-0.5, 0.5)

    def calculate_collective_control(self, target_altitude, current_altitude, climb_rate_fpm):
        """
        Control de Altitud en Cascada Estricta.
        Lazo Externo: Error de posición -> Consigna cinemática de velocidad vertical (w_ref).
        """
        # Aumento de la autoridad del lazo secundario para transitorios de gran escala
        MAX_CLIMB_RATE_FPM = 1500.0  
        
        # Lazo Proporcional de Posición
        error_alt = target_altitude - current_altitude
        
        # Generación de la consigna saturada de velocidad vertical (w_ref) en ft/min
        w_ref_fpm = max(-MAX_CLIMB_RATE_FPM, min(MAX_CLIMB_RATE_FPM, 0.25 * error_alt))
        
        # Lazo Interno: Actualización del setpoint dinámico del PID de velocidad vertical
        self.pid_collective.setpoint = w_ref_fpm
        
        # El PID calcula directamente el paso absoluto necesario para anular el error cinemático
        absolute_collective = self.pid_collective(climb_rate_fpm)
        
        return absolute_collective

    def run(self, target_altitude, duration_sec=60.0):
        print(f"[*] Ejecutando lazo de control digital. Transitorio: {INITIAL_SPEED} kts -> {FINAL_SPEED} kts.")
        print(f"[*] Target Altitude: {target_altitude} ft | Estabilización Vz: {VERTICAL_SPEED} ft/min.")
        
        # Inicialización del Datalogger
        fields = [
            'Timestamp_s', 'Latitude_deg', 'Longitude_deg', 'Altitude_ft', 
            'Airspeed_kts', 'ClimbRate_fpm', 'Pitch_deg', 'Roll_deg', 'Heading_deg',
            'Ctrl_Pitch_Ratio', 'Ctrl_Roll_Ratio', 'Ctrl_Yaw_Ratio', 'Collective_deg',
            'Engine_Torque_Nm'
        ]
        
        csv_file = open(self.log_filename, mode='w', newline='')
        csv_writer = csv.DictWriter(csv_file, fieldnames=fields)
        csv_writer.writeheader()
        
        start_time = time.time()
        next_sample_time = start_time
        
        try:
            while (time.time() - start_time) < duration_sec:
                current_time = time.time()
                
                # Sincronismo del reloj para evitar desfases del periodo de muestreo (Jitter)
                if current_time >= next_sample_time:
                    elapsed_time = current_time - start_time
                    
                    # =====================================================
                    # 1. ADQUISICIÓN DE ESTADO (Muestreo del vector de estado)
                    # =====================================================
                    posi = self.client.getPOSI(0) 
                    lat, lon, alt_ft, pitch, roll, heading, gear = posi
                    
                    drefs = [
                        "sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
                        "sim/cockpit2/gauges/indicators/vvi_fpm_pilot",
                        "sim/flightmodel/engine/ENGN_TRQ"
                    ]
                    dref_values = self.client.getDREFs(drefs)
                    
                    if not (dref_values and len(dref_values) == 3 and all(len(d) > 0 for d in dref_values)):
                        raise RuntimeError(f"[!] Falla en bus UDP de telemetría. Payload: {dref_values}")
                    
                    airspeed = dref_values[0][0]
                    climb_rate_fpm = dref_values[1][0]
                    engine_torque = dref_values[2][0]
                    
                    # =====================================================
                    # 2. PROCESAMIENTO DE SEÑAL Y LEYES DE CONTROL
                    # =====================================================
                    # Cálculo del paso absoluto del colectivo basado en la velocidad vertical en ft/min
                    absolute_collective = self.calculate_collective_control(
                        target_altitude, alt_ft, climb_rate_fpm
                    )
                    
                    ctrl_pitch = self.pid_pitch(airspeed)
                    ctrl_roll = self.pid_roll(roll)
                    ctrl_yaw = self.pid_yaw(heading) 
                    
                    # =====================================================
                    # 3. ACTUACIÓN (Inyección de matrices de control)
                    # =====================================================
                    # INVERSIÓN DE LA GANANCIA DE PLANTA LONGITUDINAL: 
                    # Para acelerar (e_v > 0), se requiere momento de cabeceo negativo (nariz abajo).
                    actuator_pitch = -ctrl_pitch 
                    
                    # Envío síncrono del cíclico longitudinal, lateral y pedales
                    self.client.sendCTRL([actuator_pitch, ctrl_roll, ctrl_yaw, -1]) 
                    
                    # Mapeo normalizado del paso colectivo al rango [0.0, 1.0] requerido por X-Plane
                    # Rango físico de simulación: -4 a 11 grados (amplitud total de 15 unidades)
                    normalized_collective = (absolute_collective + 4.0) / 15.0
                    self.client.sendDREF("sim/cockpit2/engine/actuators/prop_ratio_all", normalized_collective) 
                    
                    # =====================================================
                    # 4. REGISTRO MATRICIAL (CSV Logging)
                    # =====================================================
                    data_row = {
                        'Timestamp_s': round(elapsed_time, 3),
                        'Latitude_deg': lat,
                        'Longitude_deg': lon,
                        'Altitude_ft': alt_ft,
                        'Airspeed_kts': airspeed,
                        'ClimbRate_fpm': climb_rate_fpm,
                        'Pitch_deg': pitch,
                        'Roll_deg': roll,
                        'Heading_deg': heading,
                        'Ctrl_Pitch_Ratio': actuator_pitch,
                        'Ctrl_Roll_Ratio': ctrl_roll,
                        'Ctrl_Yaw_Ratio': ctrl_yaw,
                        'Collective_deg': absolute_collective,
                        'Engine_Torque_Nm': engine_torque
                    }
                    csv_writer.writerow(data_row)
                    
                    next_sample_time += SAMPLE_PERIOD
                
                time.sleep(0.002)
                
        except KeyboardInterrupt:
            print("\n[!] Ensayo abortado por interrupción del operador.")
        except Exception as e:
            print(f"\n[!] Excepción en ejecución: {e}")
        finally:
            csv_file.close()
            print(f"[+] Archivos de telemetría guardados en disco de forma segura.")

if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        print("[*] Sincronizando e inyectando vector de estado inicial...")
        values = [LAT, LON, ALTITUDE, 0, 0, 0] 
        client.sendPOSI(values, 0)
        
        client.sendDREF("sim/operation/override/override_ias", 1)
        client.sendDREF("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", INITIAL_SPEED)
        
        time.sleep(3.0) # Ventana de estabilización aerodinámica
        
        client.sendDREF("sim/operation/override/override_ias", 0)
        print("[+] Inicialización cinemática completa.")
        
        controller = FlightController(client, log_filename="vuelo_crucero_estable_4500ft.csv")
        controller.run(target_altitude=ALTITUDE, duration_sec=120.0)