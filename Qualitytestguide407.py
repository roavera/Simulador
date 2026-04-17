
import math
import time 
import csv
from simple_pid import PID
#from FMS.perfilofflight import get_perfil_vuelo    # agregamos el perfil de vuelo 
from XPlaneConnect.Python3.src import xpc

def get_perfil_vuelo():
    """Lee los puntos de trayectoria desde el archivo sabe_saez.csv"""
    points = []
    with open('trajectories/sabe_saez.csv', 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            try:
                point = {
                    'altitud': float(row['Altitud (ft)']) * 0.3048,  # Convertir pies a metros
                    'latitud': float(row['Lat']),
                    'longitud': float(row['Long']),
                    'airspeed': float(row['Airspeed (kias)']) * 0.514444  # Convertir nudos a m/s
                }
                points.append(point)
            except ValueError as e:
                print(f"Error al procesar línea: {row}. Error: {e}")
                continue
    return points

client = xpc.XPlaneConnect() # connect to x-plane

points = get_perfil_vuelo() # condition initial points 
current_point_index = 0    # indice actual en la ruta
start_time = time.time()  # Inicializar start_time

# Control constants
PROXIMITY_THRESHOLD = 50    # metros para distancia horizontal
ALTITUDE_THRESHOLD = 10     # metros para diferencia de altitud
TRANSITION_SPEED = 25       # velocidad de transición entre hover y vuelo forward
MIN_AIRSPEED = 5           # velocidad mínima para considerar movimiento
HOVER_HEIGHT = 10          # altura para considerar en fase de despegue
TRANSITION_DURATION = 10    # duración de la transición entre puntos (en segundos)

# variable of control initialization 

Kp_speed_vert , Ki_speed_vert , Kd_speed_vert   = 1  , 0.01   , 0.1      # climb rate 
Kp_pitch , Ki_pitch , Kd_pitch                  = 0.04 , 0.001  , 0.02         # pitch
Kp_roll , Ki_roll , Kd_roll                     = 0.07  , 0.002 , 0.02         # roll
Kp_yaw , Ki_yaw , Kd_yaw                        = 0.05  , 0.001 , 0.001      # yaw
Kp_yaw_hover , Ki_yaw_hover , Kd_yaw_hover      = 0.1  , 0.001 , 0.01      # yaw
Kp_yaw_cruise , Ki_yaw_cruise , Kd_yaw_cruise   = 0.0005  , 0.001 , 0.001         # yaw cruise
Kp_altitud, Ki_altitud , Kd_altitud             = 1     , 0.1   , 0.01      # altitud 
Kp_distnce, Ki_distance , Kd_distance             = 2     , 0.01   , 0.1      # altitud 

G_speed_long , G_pos_z = 0.7 , 0#0.5                                         # gain longitudinal
G_speed_lat , G_pos_x = 0.01 ,0# 0.02 , 0.1                           # gain lateral
G_speed_lat_cruise , G_speed_lat_another,correction_hdg = 0.5 ,0.3,0.3# 0.02 , 0.1                           # gain lateral

#init PID
pid_pitch       = PID(Kp_pitch,Ki_pitch,Kd_pitch, setpoint = 0)
pid_roll        = PID(Kp_roll,Ki_roll,Kd_roll, setpoint = 0)
pid_yaw         = PID(Kp_yaw,Ki_yaw,Kd_yaw, setpoint = 0)
pid_yaw_hover   = PID(Kp_yaw_hover,Ki_yaw_hover,Kd_yaw_hover, setpoint = 0)
pid_yaw_cruise  = PID(Kp_yaw_cruise,Ki_yaw_cruise,Kd_yaw_cruise, setpoint = 0)
pid_collective  = PID(Kp_speed_vert,Ki_speed_vert,Kd_speed_vert, setpoint = 0)
pid_distance    = PID(Kp_distnce, Ki_distance, Kd_distance, setpoint = 0) 
#limit output pid
pid_pitch.output_limits         = (-5,5)
pid_roll.output_limits          = (-5,5)
pid_yaw.output_limits           = (-15,15)
pid_yaw_hover.output_limits    = (-15,15)
pid_yaw_cruise.output_limits    = (-15,15)
pid_collective.output_limits    = (-4,4)
pid_distance.output_limits      = (-45,45)

def calculate_bearing(lat1,lon1,lat2,lon2): # Fuction by calculated bearing (lat1, lon1) y (lat2, lon2)
    lat1 = math.radians(lat1)   
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)
    
    delta_lon = lon2 - lon1
    x = math.sin(delta_lon) * math.cos(delta_lon)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)
    
    bearing = math.atan2(x,y)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    
    return bearing

def calculate_distance(lat1,lon1,lat2,lon2): # function by calculted distance (Haversine)
    
    R = 6371.0 # radius of the earth (kilometers)
    lat1 = math.radians(lat1)   
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)  
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    distance = R * c
    return distance * 1000 # convetir en meters 

def calculate_difference_heading(heading_actual,heading_deseado): # Calculate difference heading actual y el deseado 
    
    diff = (heading_deseado - heading_actual +360) % 360
    if diff > 180:
        diff -=360
    return diff

def saturar(valor, minimo, maximo): # Fuction by saturar
    """
    Limita el valor dentro del rango [minimo, maximo].
    
    Parameters:
    - valor: El valor que se va a saturar.
    - minimo: El valor mínimo permitido.
    - maximo: El valor máximo permitido.
    
    Returns:
    - El valor limitado dentro del rango [minimo, maximo].
    """
    if valor < minimo:
        return minimo
    elif valor > maximo:
        return maximo
    else:
        return valor

def get_flight_phase(altitude, airspeed): #fase flight actual
    """
    Determina la fase de vuelo actual basada en altitud y velocidad
    Retorna: 
    - 'hover': Durante despegue y hover
    - 'transition': Durante la transición
    - 'cruise': Durante vuelo de crucero
    """
    if altitude < HOVER_HEIGHT and airspeed < MIN_AIRSPEED:
        return 'hover'
    elif airspeed < TRANSITION_SPEED:
        return 'transition'
    else:
        return 'cruise'

def calculate_yaw_control(flight_phase, heading_error, airspeed):
    """
    Calcula el control de yaw basado en la fase de vuelo y el error de heading
    
    Parámetros:
    - flight_phase: Fase actual del vuelo ('hover', 'transition', 'cruise')
    - heading_error: Error entre el heading deseado y actual
    - airspeed: Velocidad actual del aire
    
    Retorna:
    - Valor de control para los pedales
    """
    # Factor de transición basado en la velocidad
    transition_factor = min(1.0, max(0.0, (airspeed - MIN_AIRSPEED) / (TRANSITION_SPEED - MIN_AIRSPEED)))
    
    if flight_phase == 'hover':
        # En hover, control agresivo del error de heading
        pid_yaw_hover.setpoint = 0  # El setpoint es 0 porque trabajamos directamente con el error
        pedal_hover = pid_yaw_hover(heading_error)
        return pedal_hover
    
    elif flight_phase == 'transition':
        # Mezclar entre control hover y crucero
        pid_yaw_hover.setpoint = 0
        pid_yaw_cruise.setpoint = 0
        
        pedal_hover = pid_yaw_hover(heading_error)
        pedal_cruise = pid_yaw_cruise(heading_error)
        
        # Interpolación lineal entre los dos controles
        return pedal_hover * (1 - transition_factor) + pedal_cruise * transition_factor
    
    else:  # 'cruise'
        # En crucero, control más suave del error
        pid_yaw_cruise.setpoint = 0
        pedal_cruise = 0#pid_yaw_cruise(heading_error)
        
        # Reducir la influencia de los pedales a alta velocidad
        speed_reduction_factor = max(0.2, 1 - min(1.0, (airspeed - TRANSITION_SPEED) / 50.0))
        return pedal_cruise * speed_reduction_factor

def goto_next_point(current_point_index, points):
    """
    Avanza al siguiente punto de la trayectoria si está disponible
    """
    if current_point_index < len(points) - 1:
        current_point_index += 1
        print(f"""
        Punto {current_point_index} alcanzado:
        - Latitud: {points[current_point_index]['latitud']} 
        - Longitud: {points[current_point_index]['longitud']}
        - Altitud objetivo: {points[current_point_index]['altitud']}
        - Velocidad objetivo: {points[current_point_index]['airspeed']}
        """)
        return current_point_index
    else:
        print("Se ha llegado al último punto de la trayectoria.")
        return current_point_index
   
#datares 

datarefs = [
                    'sim/flightmodel/position/y_agl',                                   #altitud
                    'sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot',    #heading
                    'sim/cockpit2/gauges/indicators/airspeed_kts_pilot',                #airspeed
                    'sim/flightmodel/position/local_vx',                                #Speed roll
                    'sim/flightmodel/position/local_vy',                                #climb rate
                    'sim/flightmodel/position/local_vz',                                #Speed longuitudinal - negative
                    'sim/flightmodel/position/local_x',                                 #position lateral 
                    'sim/flightmodel/position/local_y',                                 #position up
                    'sim/flightmodel/position/local_z',                                 #position longuitudinal - negative
                    'sim/flightmodel/position/theta',                                   #pitch
                    'sim/flightmodel/position/phi',                                     #Roll
                    'sim/flightmodel/position/psi',                                     #Yaw                
                    'sim/flightmodel/position/latitude',                                #latitude
                    'sim/flightmodel/position/longitude',                               #longuitude
                    'sim/cockpit2/engine/actuators/prop_angle_degrees',                 # angle of atack (AOA) -4,11 grades of blade (collective) 
                    'sim/joystick/yoke_pitch_ratio',                                    # position joke pitch ( cyclic )
                    'sim/joystick/yoke_roll_ratio',                                     # position joke roll  ( cyclic )
                    'sim/joystick/yoke_heading_ratio',                                  # position joke pedals
                    'sim/flightmodel/forces/g_nrml',  
                    'sim/flightmodel/engine/ENGN_ITT_c', 
                    'sim/flightmodel/engine/ENGN_N1_', 
                    'sim/flightmodel/engine/ENGN_N2_'
]


while True:
    
        dref_values = client.getDREFs(datarefs) # read datareft from X-plane

        altitude                = dref_values[0][0]
        heading                 = dref_values[1][0]
        airspeed                = dref_values[2][0]
        speed_lat               = dref_values[3][0]
        speed_vertical          = dref_values[4][0] #meter/second - climb_rate = speed_vertical * 196.85
        pos_x                   = dref_values[6][0]
        pos_y                   = dref_values[7][0]
        pos_z                   = dref_values[8][0]
        pitch                   = dref_values[9][0]
        roll                    = dref_values[10][0]
        yaw                     = dref_values[11][0]           
        latitud_Xplane          = dref_values[12][0]
        longitud_Xplane         = dref_values[13][0]
        collective_current      = dref_values[14][0]
        cyclic_long             = dref_values[15][0]
        cyclic_lat              = dref_values[16][0]
        ped                     = dref_values[17][0]
        mgt                     = dref_values[18][0]
        nr                      = dref_values[19][0]
        np                      = dref_values[20][0]
        nR                      = dref_values[21][0]
              
        
        flight_phase = get_flight_phase(altitude,airspeed) # flight phase
        target_point = points[current_point_index] #point actual

 
        # Definir un umbral de proximidad (por ejemplo, 50 metros)
        #PROXIMITY_THRESHOLD = 50
        pid_distance.setpoint = 0  
        
        bearing_to_next = calculate_bearing(latitud_Xplane,longitud_Xplane,
                                            target_point['latitud'],target_point['longitud']) #SAEZ
        distance_to_next = calculate_distance(latitud_Xplane,longitud_Xplane,
                                              target_point['latitud'],target_point['longitud'])
        heading_correction = pid_distance(distance_to_next) #error distance
        #print(heading_correction)
        
        heading_error= calculate_difference_heading(bearing_to_next - heading_correction*correction_hdg ,heading)
               
        # Actualizar setpoints de los PIDs
        #pid_yaw_hover.setpoint = bearing_to_next
        #pid_yaw_cruise.setpoint = bearing_to_next
                    
        #--------------------------------------verification next point-------------------------------------- 
        #altitude_error  = abs(target_point['altitud'] - altitude) 
        if distance_to_next <= PROXIMITY_THRESHOLD and abs(target_point['altitud'] - altitude) <= ALTITUDE_THRESHOLD :
                # Avanzar al siguiente punto
            current_point_index = goto_next_point(current_point_index, points)
            # Actualizar el punto objetivo
            target_point = points[current_point_index]
            bearing_to_next = calculate_bearing(latitud_Xplane, longitud_Xplane, 
                                            target_point['latitud'], target_point['longitud'])
            heading_error = calculate_difference_heading(bearing_to_next- heading_correction*correction_hdg ,heading)    
        #-------------------Ajuste de la referencia de la velocidad vertical basado en la altitud
        
        error_altitude = target_point['altitud'] - altitude                                                
        w_ref = saturar(error_altitude, -5, 5)

        #Control de Velocidad Vertical
        
        pid_collective.setpoint = w_ref                        # error = setpoint - valor_actual
        collective_finish = pid_collective (speed_vertical)
        collective = collective_current + collective_finish 
        collective = saturar(collective,-4,4)
        
        #---------------------------------------Control de Pitch --------------------------------------------
        
        vel_long_ref = target_point['airspeed'] #airspeed perfilvuelo
        error_long = -(vel_long_ref - airspeed)
        error_long = saturar(error_long, -5,5)

        pos_z_ref = 5 # position maximo desplazamiento perfilvuelo
        error_pos_z = pos_z_ref - pos_z
        error_pos_z = saturar(error_pos_z, -10, 10)

        pid_pitch.setpoint = (error_long * G_speed_long + error_pos_z * G_pos_z) 
        ciclico_long = pid_pitch(pitch)
        ciclico_long = saturar(ciclico_long, -0.7, 0.7)
        
        #---------------------------------------Control de roll --------------------------------------------
               
                # Ajustar los gains del control de roll según la fase de vuelo
        if flight_phase == 'cruise':
            G_heading = G_speed_lat_cruise  # Mayor ganancia para control con roll en cruise
        else:
            G_heading = G_speed_lat_another # Menor ganancia para control con roll en hoverr
         

        vel_lat_ref = 0 #speed_roll perfilvuelo
        error_lat = vel_lat_ref - speed_lat
        error_lat = saturar(error_lat, -10, 10)

        pos_x_ref = 5  # position maximo desplazamiento perfilvuel axis x ( roll)
        error_pos_x = pos_x_ref - pos_x
        error_pos_x = saturar(error_pos_x, -10, 10)
     
        pid_roll.setpoint = -(error_lat * G_speed_lat + error_pos_x * G_pos_x + heading_error * G_heading)
        ciclico_late = pid_roll(roll)
        ciclico_late = saturar(ciclico_late, -0.7, 0.7)
        #---------------------------------------Control de yaw --------------------------------------------       
        # Control de yaw mejorado usando el heading_error
        pedales = calculate_yaw_control(flight_phase, heading_error, airspeed)
        
        #---------------------------------------send controls --------------------------------------------   

        client.sendCTRL([ciclico_long, ciclico_late, pedales])
        client.sendDREF('sim/cockpit2/engine/actuators/prop_angle_degrees',collective)	
        # Dormir un intervalo corto para evitar sobrecargar el sistema
        time.sleep(0.05)