"""
    Variabili globali utilizzate nel progetto
    Definizione della velocità massima dei robot, del time_step, delle
    coordinate GPS riguardanti i vari angoli, i relativi orientamenti,
    il codice colore utilizzato dalle sfere e il numero delle stesse presenti nella stanza
"""

MAX_SPEED = 6.28
TIME_STEP = 64
GRIPPER_MOTOR_MAX_SPEED = 0.1
DEFAULT_LIMIT_SPEED = 0.4

WALL = 1000
MAX_SENSOR_VALUE = 930
MAX_SENSOR_VALUE_WALL_REACH = 970

yellow_corner = [-3, -0.2, -3.1]
red_corner = [1.1, -0.2, -3.1]
green_corner = [-2.9, -0.2, 0.7]
blue_corner = [1.1, -0.2, 0.7]

yellow_yaw = 0.4
red_yaw = -0.5
green_yaw = 2.5
blue_yaw = -2.5

yellow = [1.0, 1.0, 0.0]
red = [1.0, 0.0, 0.0]
green = [0.0, 1.0, 0.0]
blue = [0.0, 0.0, 1.0]

# orientamenti NORD, EST, OVEST, SUD
ORIENTAMENTI_left = [-0.1, 1.5, -1.7, -3.1]
ORIENTAMENTI_right = [0.1, 1.7, -1.5, -2.9]

total_boxes = 4