"""my_pioneer_controller controller."""

from controller import Robot, Motor, CameraRecognitionObject, Camera
import time

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
objects = {}
prev_sensors = []
target_obj_initial_position = None
left = False
right = False
before_turning_left_compass = []
turn_left = False
before_turning_left_gps = []
straight = False
box_placed = False
placed_boxes = 0
total_boxes = 2

# Robot instance.
robot = Robot()

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
# print("tipologia:", d0.getType())

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.6 * MAX_SPEED)
rightMotor.setVelocity(0.6 * MAX_SPEED)


def check_wall_reached():
    global count, wall_reached_gps, wall_reached_compass, wall_reached
    # più di 1 sensore misura un valore elevato => parete
    if sum(s.getValue() > 950 for s in sensors) > 1:
        print("WALL REACHED")
        wall_reached_gps = gps_sensor.getValues()
        wall_reached_compass = compass.getValues()
        # tornare indietro per riprendere il box (in base al sensore gps)
        # if all(s.getValue() > 700 for s in high_sensors):
        print("GO BACK")
        leftMotor.setVelocity(-0.6 * MAX_SPEED)
        rightMotor.setVelocity(-0.6 * MAX_SPEED)
        wall_reached = True
        return wall_reached
    else:
        return False


def turn_after_wall_reached():
    global stopped, wall_reached_gps, wall_reached, before_turning_left_compass, turn_left, before_turning_left_gps
    print("QUIIIII")
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
    if stopped:
        print("TURN RIGHT")
        leftMotor.setVelocity(0.2 * MAX_SPEED)
        rightMotor.setVelocity(0)
        # print(abs(round(wall_reached_compass[0]) - round(compass.getValues()[2])))
        # print(abs((wall_reached_compass[0]) - (compass.getValues()[2])))
        if round(abs(wall_reached_compass[0]) - (compass.getValues()[2]), 3) == 1.999:
            print("saved")
            before_turning_left_compass = compass.getValues()
            before_turning_left_gps = gps_sensor.getValues()
            turn_left = True
            wall_reached = False
            stopped = False
            leftMotor.setVelocity(0.6 * MAX_SPEED)
            rightMotor.setVelocity(0.6 * MAX_SPEED)


def do_turn_left(before_turning_left_compass, before_turning_left_gps):
    global turn_left, straight
    print("valori")
    print(abs(before_turning_left_gps[0] - gps_sensor.getValues()[0]))
    print(abs(before_turning_left_gps[1] - gps_sensor.getValues()[1]))
    print(abs(before_turning_left_gps[2] - gps_sensor.getValues()[2]))
    if abs(before_turning_left_gps[0] - gps_sensor.getValues()[0]) > 0.5 or \
            abs(before_turning_left_gps[1] - gps_sensor.getValues()[1]) > 0.5 or \
            abs(before_turning_left_gps[2] - gps_sensor.getValues()[2]) > 0.5:
        print("TURN LEFT")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0.2 * MAX_SPEED)
        print(abs(round(before_turning_left_compass[0]) - round(compass.getValues()[2])))
        print(abs((before_turning_left_compass[0]) - (compass.getValues()[2])))
        if round(abs(before_turning_left_compass[0] - (compass.getValues()[2])), 2) == 0.97 and d7.getValue() > 900:
            print("vai avanti")
            turn_left = False
            straight = True


def go_straight():
    print("avanti")
    leftMotor.setVelocity(0.6 * MAX_SPEED)
    rightMotor.setVelocity(0.6 * MAX_SPEED)

def check_for_other_box():
    print("check for box")
    leftMotor.setVelocity(-0.6 * MAX_SPEED)
    rightMotor.setVelocity(-0.6 * MAX_SPEED)


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

    for i in range(len(sensors)):
        prev_sensors.append(sensors[i].getValue())

    if target_object is None and camera.getRecognitionNumberOfObjects() >= 1:
        # prendo il primo della lista
        target_object = camera.getRecognitionObjects()[0]
        target_obj_initial_position = camera.getRecognitionObjects()[0].get_position()

    print("target obj:", target_object)
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
    if turn_left: do_turn_left(before_turning_left_compass, before_turning_left_gps)
    if straight: go_straight()
    if camera.getRecognitionNumberOfObjects() == 1 and wall_reached:
        objects = camera.getRecognitionObjects()
        obj = objects[0]
        if round(obj.get_position()[0], ) == 0 and \
                round(obj.get_position()[1], 0) == 0 and \
                round(obj.get_position()[2], 0) == 0 and \
                round(gps_sensor.getValues()[0], 1) == -3.1 and round(gps_sensor.getValues()[1], 1) == -0.2 and \
                round(gps_sensor.getValues()[2], 1) == -3.5:
            straight = False
            box_placed = True
            placed_boxes += 1
            if placed_boxes < total_boxes:
                check_for_other_box()

    print("target obj pos on image", target_object.get_position_on_image())
    print("gps sensor", gps_sensor.getValues())

    print("n_objects = ", camera.getRecognitionNumberOfObjects())
    objects = camera.getRecognitionObjects()
    print("objects = ", objects)
    for obj in objects:
        print("colore:", obj.get_colors())
        print("posizione:", obj.get_position())
    print("\n")

    pass

# Enter here exit cleanup code.
