# ========================= 
# 1_F_1A_1_Vuelo Nivelado MEJORADO.py
# Crucero - Aumentar velocidad manteniendo altitud
# Validar el desempeño a velocidades por encima de la velocidad de máxima resistencia. 
# Determinar la performance durante un vuelo en crucero.

# Condiciones Iniciales: 
    # Altitud:      4500 ft
    # Velocidad:    50 kts

# Condiciones finales:       
    # Altitud:      4500 ft
    # Velocidad:    120 kts

# MEJORAS IMPLEMENTADAS:
# 1. Desacoplamiento de controles: Collective mantiene altitud, Pitch mantiene velocidad
# 2. PIDs más agresivos para seguimiento más rápido de velocidad
# 3. Limitadores de velocidad de cambio para evitar sobreville
# 4. Monitoreo de divergencia altitud-velocidad

# =========================

# Datarefs de interes para esta prueba:
# sim/flightmodel/engine/ENGN_TRQ			
# sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot			
# sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot			
# sim/cockpit2/controls/yoke_pitch_ratio			
# sim/cockpit2/controls/yoke_roll_ratio			
# sim/cockpit2/controls/yoke_heading_ratio			
# sim/cockpit2/engine/actuators/prop_ratio_all			

# Datarefs para parametros iniciales
# sim/flightmodel/weight/m_fixed		
# sim/flightmodel/weight/m_fuel1		
# sim/cockpit2/gauges/indicators/CG_indicator		

# =========================

from collections import deque
import math
import time
import csv
import os
import logging
from simple_pid import PID
from XPlaneConnect.Python3.src import xpc

#=========================================================
# PARAMETROS
#=========================================================

# Conversión de unidades
FT_TO_METERS = 0.3048
KTS_TO_METERS_PER_SEC = 0.514444    # 1 nudo = 0.514444 m/s
METERS_TO_FT = 1 / FT_TO_METERS

# =========================================================
# CONDICIONES INICIALES
# =========================================================

LAT = -34.554          # grados, se lo puede ajustar para iniciar en un lugar específico del mapa
LON = -58.425          # grados, se lo puede ajustar para iniciar en un lugar específico del mapa
ALTITUDE = 4500        # ft (CORREGIDO: era 10000)
INITIAL_SPEED = 50     # kts - airspeed

FINAL_SPEED = 120      # kts - airspeed
VERTICAL_SPEED = 0     # ft/min - vertical speed 
SPEED_ROLL = 0         # kts - velocidad a la que el helicóptero comienza a inclinarse 

# Parámetros de control de cambio gradual
SPEED_INCREMENT_RATE = 0.5  # kts/segundo - velocidad a la que aumentamos el setpoint
ALTITUDE_TOLERANCE = 50     # ft - tolerancia de altitud aceptable
CONVERGENCE_TIME = 300      # segundos - tiempo máximo para convergencia

# =========================================================
# CONTROLADOR 
# =========================================================

class FlightController:
    def __init__(self, client):
        self.client = client
        self.flight_phase = 'cruise'
        self.start_time = time.time()
        self.target_speed_setpoint = INITIAL_SPEED  # Setpoint que aumenta gradualmente

        # Inicializar PIDs con parámetros por defecto
        self._init_pids()

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

    # =========================================================
    # INICIALIZACIÓN DE PIDs
    # =========================================================

    def _init_pids(self):
        """
        ESTRATEGIA DE CONTROL DESACOPLADO:
        - Collective (Kp=2.0) → Mantiene altitud constante (control primario)
        - Pitch (Kp=0.08) → Aumenta velocidad (control secundario)
        - El collective responde agresivamente a cambios de altitud
        - El pitch responde suavemente para cambiar velocidad sin afectar altitud
        """
        
        # PID para velocidad vertical (climb rate)
        # FUNCIÓN PRINCIPAL: Mantener altitud constante (0 climb rate)
        # Mayor Kp (2.0) para respuesta rápida a desviaciones de altitud
        # Ki bajo para evitar sobrepaso (overshoot)
        self.pid_collective = PID(2.0, 0.05, 0.2, setpoint=0)
        self.pid_collective.output_limits = (-2.5, 2.5)  # Límites suave pero efectivos

        # -------------------------------------------- #
        # ÁNGULOS 

        # PID para pitch (theta) - cabeceo
        # FUNCIÓN PRINCIPAL: Controlar velocidad (airspeed)
        # Pitch UP = velocidad aumenta (nariz arriba acelera en helicópteros)
        # Pitch DOWN = velocidad disminuye
        # Kp moderado (0.08) para cambios suaves de velocidad
        # Kd para amortiguación y evitar oscilaciones
        self.pid_pitch = PID(0.08, 0.0002, 0.02, setpoint=INITIAL_SPEED)
        self.pid_pitch.output_limits = (-0.5, 0.5)  # Límites conservadores
        
        # PID para roll (phi) - Rolido
        # Evita que el helicóptero se incline lateralmente
        self.pid_roll = PID(0.008, 0.002, 0.0, setpoint=SPEED_ROLL)
        self.pid_roll.output_limits = (-0.7, 0.7)
        
        # PID de yaw (psi) - pedales
        # Mantiene el heading constante (contrarresta el torque del rotor)
        self.pid_yaw = PID(0.0008, 0.0, 0.001, setpoint=0)
        self.pid_yaw.output_limits = (-5, 5)

        # -------------------------------------------- #
    
    # =============================================
    # CONTROL COLLECTIVE - MANTIENE ALTITUD
    # =============================================
    def calculate_collective_control(self, target_altitude_m, current_altitude_m, climb_rate, current_collective):
        """
        Control del collective para mantener altitud constante.
        
        ESTRATEGIA:
        - El error de altitud se convierte directamente en setpoint de climb rate
        - El PID mantiene climb_rate = 0 para vuelo nivelado
        - El collective se ajusta hasta lograr climb_rate = 0
        
        Args:
            target_altitude_m: Altitud objetivo en metros
            current_altitude_m: Altitud actual en metros
            climb_rate: Velocidad vertical actual en m/s
            current_collective: Ángulo actual del collective en grados
        
        Returns:
            Nuevo comando de collective en grados
        """
        
        # Error de altitud en metros
        error_alt_m = target_altitude_m - current_altitude_m
        
        # Convertir error a setpoint de climb rate
        # Si estamos 10m abajo, queremos climb_rate positivo
        # Si estamos 10m arriba, queremos climb_rate negativo
        # Ganancia: 0.1 m/s por metro de error
        w_ref = 0.1 * error_alt_m
        
        # Limitar la referencia de velocidad vertical (máximo ±1 m/s)
        MAX_CLIMB_RATE = 1.0  # m/s
        w_ref = max(-MAX_CLIMB_RATE, min(MAX_CLIMB_RATE, w_ref))
        
        # Aplicar PID al climb rate actual
        self.pid_collective.setpoint = w_ref
        collective_change = self.pid_collective(climb_rate)
        
        # Calcular nuevo valor del collective
        new_collective = current_collective + collective_change
        
        # Limitar rango físico del collective (-4 a 11 grados)
        new_collective = max(-4.0, min(11.0, new_collective))
        
        return new_collective
    
    # =============================================
    # CONTROLES DE ÁNGULOS - VELOCIDAD Y ESTABILIDAD
    # =============================================
    
    def calculate_pitch_control(self, target_speed, current_speed):
        """Control de pitch para alcanzar velocidad objetivo"""
        self.pid_pitch.setpoint = target_speed
        return self.pid_pitch(current_speed)
    
    def calculate_roll_control(self, target_speed, current_speed):
        """Evita rolido lateral (mantiene alas niveladas)"""
        self.pid_roll.setpoint = target_speed
        return self.pid_roll(current_speed)
    
    def calculate_yaw_control(self, target_heading, current_heading):
        """Mantiene heading constante"""
        self.pid_yaw.setpoint = target_heading
        return self.pid_yaw(current_heading)
    
    def update_speed_setpoint(self, elapsed_time):
        """
        Aumenta el setpoint de velocidad gradualmente.
        Previene cambios abruptos que desestabilicen el vuelo.
        """
        speed_increase = SPEED_INCREMENT_RATE * elapsed_time
        new_setpoint = min(INITIAL_SPEED + speed_increase, FINAL_SPEED)
        self.target_speed_setpoint = new_setpoint
        return new_setpoint
    
    def run(self, target_altitude_m):
        """
        BUCLE PRINCIPAL DE CONTROL
        
        Estructura:
        1. Aumentar setpoint de velocidad gradualmente
        2. Usar collective para mantener altitud
        3. Usar pitch para alcanzar velocidad
        4. Mantener estabilidad (roll y yaw)
        """
        
        # Capturar heading de referencia al inicio del vuelo
        data = self.client.getDREFs(self.datarefs)
        heading_ref = data[1]  # Heading magnético inicial
        
        print("=" * 60)
        print("INICIANDO CONTROL DE VUELO")
        print("=" * 60)
        print(f"Altitud objetivo: {target_altitude_m * METERS_TO_FT:.1f} ft")
        print(f"Velocidad inicial: {INITIAL_SPEED} kts")
        print(f"Velocidad final: {FINAL_SPEED} kts")
        print(f"Tasa de cambio: {SPEED_INCREMENT_RATE} kts/seg")
        print("=" * 60)
        
        iteration = 0
        last_print_time = time.time()
        
        while True:
            iteration += 1
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            
            # Leer datos del simulador
            data = self.client.getDREFs(self.datarefs)
            altitude_agl_m = data[0]  # Ya está en metros
            altitude_agl_ft = altitude_agl_m * METERS_TO_FT
            heading = data[1]
            airspeed = data[2]
            climb_rate = data[4]
            current_collective = data[14]
            
            # Aumentar setpoint de velocidad gradualmente
            target_speed_now = self.update_speed_setpoint(elapsed_time)
            
            # CONTROLES DESACOPLADOS
            # 1. Collective mantiene altitud (es el control primario)
            collective_cmd = self.calculate_collective_control(
                target_altitude_m, 
                altitude_agl_m, 
                climb_rate, 
                current_collective
            )
            
            # 2. Pitch acelera/desacelera (control secundario)
            pitch_cmd = self.calculate_pitch_control(target_speed_now, airspeed)
            
            # 3. Roll mantiene alas niveladas
            roll_cmd = self.calculate_roll_control(SPEED_ROLL, airspeed)
            
            # 4. Yaw mantiene heading
            yaw_cmd = self.calculate_yaw_control(heading_ref, heading)
            
            # Enviar comandos al simulador
            self.client.sendDREF("sim/cockpit2/controls/yoke_pitch_ratio", pitch_cmd)
            self.client.sendDREF("sim/cockpit2/controls/yoke_roll_ratio", roll_cmd)
            self.client.sendDREF("sim/cockpit2/controls/yoke_heading_ratio", yaw_cmd)
            self.client.sendDREF("sim/cockpit2/engine/actuators/prop_angle_degrees", collective_cmd)
            
            # Imprime estado cada 2 segundos
            if current_time - last_print_time > 2.0:
                alt_error_ft = altitude_agl_ft - (target_altitude_m * METERS_TO_FT)
                speed_error_kts = airspeed - target_speed_now
                
                print(f"[{elapsed_time:6.1f}s] Alt: {altitude_agl_ft:7.1f}ft (error: {alt_error_ft:+6.1f}ft) | "
                      f"Vel: {airspeed:6.1f}kts (objetivo: {target_speed_now:6.1f}kts, error: {speed_error_kts:+5.1f}kts) | "
                      f"Collective: {collective_cmd:6.2f}° | Pitch: {pitch_cmd:+6.3f}")
                
                # Verificar convergencia
                if elapsed_time > CONVERGENCE_TIME:
                    if abs(alt_error_ft) < ALTITUDE_TOLERANCE and abs(airspeed - FINAL_SPEED) < 2:
                        print("\n" + "=" * 60)
                        print("✓ CONVERGENCIA ALCANZADA")
                        print(f"  Altitud: {altitude_agl_ft:.1f}ft (objetivo: {target_altitude_m * METERS_TO_FT:.1f}ft)")
                        print(f"  Velocidad: {airspeed:.1f}kts (objetivo: {FINAL_SPEED}kts)")
                        print("=" * 60)
                        break
                
                last_print_time = current_time
            
            # Seguridad: parar después de 10 minutos si no converge
            if elapsed_time > 600:
                print("\n⚠ TIMEOUT: No se alcanzó convergencia en 10 minutos")
                break


# =========================================================
# CONEXIÓN CON X-PLANE
# =========================================================
if __name__ == "__main__":
    with xpc.XPlaneConnect() as client:
        
        # Posición inicial 
        values = [LAT, LON, ALTITUDE, 0, 0, 0] # lat, lon, alt, pitch, roll, heading
        client.sendPOSI(values, 0)
        
        # Activar el override de IAS 
        client.sendDREF("sim/operation/override/override_ias", 1)
        
        # Forzar la velocidad inicial
        client.sendDREF("sim/cockpit2/gauges/indicators/airspeed_kts_pilot", INITIAL_SPEED)
        time.sleep(2) # Esperar a que el simulador estabilice la posición

        # Desactivar el override IAS 
        client.sendDREF("sim/operation/override/override_ias", 0)
        
        # Permitir que el helicóptero se estabilice antes de iniciar aceleración
        time.sleep(3)

        # Crear y ejecutar el controlador
        controller = FlightController(client)
        controller.run(ALTITUDE * FT_TO_METERS)
