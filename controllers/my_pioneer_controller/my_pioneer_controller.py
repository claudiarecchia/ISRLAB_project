"""my_pioneer_controller controller."""

from controller import Robot, Motor, CameraRecognitionObject, Camera

MAX_SPEED = 6.28
TIME_STEP = 64
stop = False
count = 0
wall_reached_gps = None
wall_reached_compass = None
high_sensors = []
wall_reached = False
stopped = False
target_object = None
boxes_ids = []
prev_sensors = []
target_obj_initial_position = None
left = False
right = False
before_turning_left_compass = []
turn_left = False
before_action_gps = []
straight = False
box_placed = False
total_boxes = 2
check_for_box = False

ACTION = ""

# Robot instance.
robot = Robot()

inertial_unit = robot.getDevice('inertial unit')
inertial_unit.enable(TIME_STEP)

gps_sensor = robot.getDevice('gps')
gps_sensor.enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel')
rightMotor = robot.getDevice('right wheel')
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)
compass = robot.getDevice('compass')
compass.enable(TIME_STEP)
left_wheel_sensor = robot.getDevice('left wheel sensor')
right_wheel_sensor = robot.getDevice('right wheel sensor')
left_wheel_sensor.enable(TIME_STEP)
right_wheel_sensor.enable(TIME_STEP)

# da sinistra a destra
d0 = robot.getDevice('so0')
d1 = robot.getDevice('so1')
d2 = robot.getDevice('so2')
d3 = robot.getDevice('so3')
d4 = robot.getDevice('so4')
d5 = robot.getDevice('so5')
d6 = robot.getDevice('so6')
d7 = robot.getDevice('so7')

d8 = robot.getDevice('so8')
d9 = robot.getDevice('so9')
d10 = robot.getDevice('so10')
d11 = robot.getDevice('so11')
d12 = robot.getDevice('so12')
d13 = robot.getDevice('so13')
d14 = robot.getDevice('so14')
d15 = robot.getDevice('so15')

d0.enable(TIME_STEP)
d1.enable(TIME_STEP)
d2.enable(TIME_STEP)
d3.enable(TIME_STEP)
d4.enable(TIME_STEP)
d5.enable(TIME_STEP)
d6.enable(TIME_STEP)
d7.enable(TIME_STEP)

d8.enable(TIME_STEP)
d9.enable(TIME_STEP)
d10.enable(TIME_STEP)
d11.enable(TIME_STEP)
d12.enable(TIME_STEP)
d13.enable(TIME_STEP)
d14.enable(TIME_STEP)
d15.enable(TIME_STEP)

sensors = [d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15]

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.6 * MAX_SPEED)
rightMotor.setVelocity(0.6 * MAX_SPEED)


def check_wall_reached():
    global count, wall_reached_gps, wall_reached_compass, wall_reached, straight, before_action_gps
    # più di 1 sensore misura un valore elevato => parete
    # FRONT WALL REACHED => ho la parete davanti, allora devo andare indietro
    # if sum(s.getValue() > 950 for s in sensors) > 1:
    if d3.getValue() > 950 and d4.getValue() > 950:
        print("WALL REACHED")
        wall_reached_gps = gps_sensor.getValues()
        wall_reached_compass = compass.getValues()
        print("GO BACK")
        leftMotor.setVelocity(-0.6 * MAX_SPEED)
        rightMotor.setVelocity(-0.6 * MAX_SPEED)
        wall_reached = True
        # Se ci sono più sensori che misurano un valore elevato sono in un angolo
        # (posso aver portato un cubo in posizione)
        # quindi non posso avanzare più e devo tornare indietro

        # if sum(s.getValue() > 990 for s in sensors) >= 3:
        #     straight = False
        #     before_action_gps = gps_sensor.getValues()
        return wall_reached
    else:
        return False


def turn_after_wall_reached():
    global stopped, wall_reached_gps, wall_reached, before_turning_left_compass, turn_left, before_action_gps, straight
    # print(abs(wall_reached_gps[0] - gps_sensor.getValues()[0]))
    # print(abs(wall_reached_gps[1] - gps_sensor.getValues()[1]))
    # print(abs(wall_reached_gps[2] - gps_sensor.getValues()[2]))
    if abs(wall_reached_gps[0] - gps_sensor.getValues()[0]) > 0.3 or abs(
            wall_reached_gps[1] - gps_sensor.getValues()[1]) > 0.3 or abs(
            wall_reached_gps[2] - gps_sensor.getValues()[2]) > 0.3:
        print("STOP")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        stopped = True
        straight = False
    if stopped:
        # quando il robot è fermo, gli indico di girare a destra
        print("TURN RIGHT")
        leftMotor.setVelocity(0.2 * MAX_SPEED)
        rightMotor.setVelocity(0)
        # print(abs(round(wall_reached_compass[0]) - round(compass.getValues()[2])))
        # print(abs((wall_reached_compass[0]) - (compass.getValues()[2])))
        # if round(abs(wall_reached_compass[0]) - (compass.getValues()[2]), 3) == 1.999:

        # gira a destra finchè non raggiunge la posizione esatta verso destra (est)
        if round((inertial_unit.getRollPitchYaw())[1] >= 1.5):  # pi/2 = est
            print("saved")
            # before_turning_left_compass = compass.getValues()
            before_action_gps = gps_sensor.getValues()
            # quando raggiunge est, deve rigirare a sinistra
            turn_left = True
            wall_reached = False
            stopped = False
            go_straight()


def do_turn_left(before_action_gps):
    global turn_left, straight
    print("valori")
    print(abs(before_action_gps[0] - gps_sensor.getValues()[0]))
    print(abs(before_action_gps[1] - gps_sensor.getValues()[1]))
    print(abs(before_action_gps[2] - gps_sensor.getValues()[2]))
    if abs(before_action_gps[0] - gps_sensor.getValues()[0]) > 0.5 or \
            abs(before_action_gps[1] - gps_sensor.getValues()[1]) > 0.5 or \
            abs(before_action_gps[2] - gps_sensor.getValues()[2]) > 0.5:
        print("TURN LEFT")
        go_left()
        # print(abs(round(before_turning_left_compass[0]) - round(compass.getValues()[2])))
        # print(abs((before_turning_left_compass[0]) - (compass.getValues()[2])))
        # if round(abs(before_turning_left_compass[0] - (compass.getValues()[2])), 2) == 0.97 and d7.getValue() > 900:

        # TODO considerare la differenza con un valore salvato precedentemente
        if (inertial_unit.getRollPitchYaw())[1] <= -1.55:  # -pi/2 = ovest
            print("vai avanti")
            turn_left = False
            straight = True


def go_left():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0.2 * MAX_SPEED)


def go_straight():
    print("avanti")
    leftMotor.setVelocity(0.6 * MAX_SPEED)
    rightMotor.setVelocity(0.6 * MAX_SPEED)


def check_for_other_box():
    global before_action_gps, box_placed
    box_placed = False
    print("check for box")
    # se il robot ha percorso abbastanza spazio
    if abs(before_action_gps[0] - gps_sensor.getValues()[0]) > 0.5 or \
            abs(before_action_gps[1] - gps_sensor.getValues()[1]) > 0.5 or \
            abs(before_action_gps[2] - gps_sensor.getValues()[2]) > 0.5:
        if d7.getValue() > 900:
            # sensore 7 alto => parete a destra => giro a sinistra
            print("sinistra")
            go_left()
    else:
        # ho appena posizionato un box, quindi devo tornare indietro
        print("indietro")
        leftMotor.setVelocity(-0.6 * MAX_SPEED)
        rightMotor.setVelocity(-0.6 * MAX_SPEED)

        # TODO aggiungere sensore speculare a d7


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    print("Sensore 0:", d0.getValue())
    print("Sensore 1:", d1.getValue())
    print("Sensore 2:", d2.getValue())
    print("Sensore 3:", d3.getValue())
    print("Sensore 4:", d4.getValue())
    print("Sensore 5:", d5.getValue())
    print("Sensore 6:", d6.getValue())
    print("Sensore 7:", d7.getValue())
    print("Rolle - Yaw - Pitch: ", inertial_unit.getRollPitchYaw())
    for i in range(len(sensors)):
        prev_sensors.append(sensors[i].getValue())

    if target_object is None and camera.getRecognitionNumberOfObjects() >= 1:
        # prendo il primo della lista
        target_object = camera.getRecognitionObjects()[0]
        target_obj_initial_position = camera.getRecognitionObjects()[0].get_position()

    # print("target obj:", target_object)
    print("compass:", compass.getValues())

    if target_object is not None and not wall_reached and camera.getRecognitionNumberOfObjects() > 0:
        if left:
            left = False
            print("DRITTO")
            go_straight()
        if right:
            right = False
            print("DRITTO")
            go_straight()
        if camera.getRecognitionObjects()[0].get_position()[0] < -0.01:  # il target è a sinistra
            # giro leggermente a sinistra per centrarlo
            print("TARGET A SINISTRA")
            left = True
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0.6 * MAX_SPEED)
        if camera.getRecognitionObjects()[0].get_position()[0] > 0.01:  # il target è a destra
            # giro leggermente a destra per centrarlo
            print("TARGET A DESTRA")
            right = True
            leftMotor.setVelocity(0.6 * MAX_SPEED)
            rightMotor.setVelocity(0)

    check_wall_reached()
    if wall_reached: turn_after_wall_reached()
    if turn_left: do_turn_left(before_action_gps)
    if straight: go_straight()
    if camera.getRecognitionNumberOfObjects() == 1 and wall_reached and not check_for_box:
        objects = camera.getRecognitionObjects()
        obj = objects[0]
        if round(obj.get_position()[0], ) == 0 and \
                round(obj.get_position()[1], 0) == 0 and \
                round(obj.get_position()[2], 0) == 0 and \
                round(gps_sensor.getValues()[0], 1) == -3.1 and round(gps_sensor.getValues()[1], 1) == -0.2 and \
                -3.5 <= round(gps_sensor.getValues()[2], 1) <= -3.4:
            print("controlli superati")
            straight = False
            box_placed = True
            boxes_ids.append(obj.get_id())
            if len(boxes_ids) < total_boxes:
                check_for_box = True
                before_action_gps = gps_sensor.getValues()
    if check_for_box: check_for_other_box()

    # print("target obj pos on image", target_object.get_position_on_image())
    print("gps sensor", gps_sensor.getValues())
    #
    # print("n_objects = ", camera.getRecognitionNumberOfObjects())
    # objects = camera.getRecognitionObjects()
    # print("objects = ", objects)
    # for obj in objects:
    #     print("colore:", obj.get_colors())
    #     print("posizione:", obj.get_position())
    print("\n")

    pass

# Enter here exit cleanup code.
