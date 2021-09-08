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

d0.enable(TIME_STEP)
d1.enable(TIME_STEP)
d2.enable(TIME_STEP)
d3.enable(TIME_STEP)
d4.enable(TIME_STEP)
d5.enable(TIME_STEP)
d6.enable(TIME_STEP)
d7.enable(TIME_STEP)

sensors = [d0, d1, d2, d3, d4, d5, d6, d7]
# print("tipologia:", d0.getType())

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.5 * MAX_SPEED)
rightMotor.setVelocity(0.5 * MAX_SPEED)

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

    for sensor in sensors:
        # se ho un target obj devo verificare che mi ci sto avvicinando
        # object_position deve avere valori decrescenti (e fermarmi a circa [X, X, 0.2] quando tocco l'oggetto)
        # devo minimizzare anche il primo parametro per portare il cubo al centro del robot

        # if target_object is not None:
        #     if camera.getRecognitionObjects()[0].get_position()[0] > target_obj_initial_position[0] or camera.getRecognitionObjects()[0].get_position()[1] > target_obj_initial_position[1] or camera.getRecognitionObjects()[0].get_position()[2] > target_obj_initial_position[2]:
        #         print("mi sto allontanando dall'oggetto target")

        if target_object is not None:
            if left is True:
                left = False
                print("DRITTO")
                leftMotor.setVelocity(0.5 * MAX_SPEED)
                rightMotor.setVelocity(0.5 * MAX_SPEED)
            if right is True:
                right = False
                print("DRITTO")
                leftMotor.setVelocity(0.5 * MAX_SPEED)
                rightMotor.setVelocity(0.5 * MAX_SPEED)
            if camera.getRecognitionObjects()[0].get_position()[0] < 0:  # il target è a sinistra
                # giro leggermente a sinistra per centrarlo
                print("TARGET A SINISTRA")
                left = True
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0.5 * MAX_SPEED)
            if camera.getRecognitionObjects()[0].get_position()[0] > 0:  # il target è a destra
                # giro leggermente a destra per centrarlo
                print("TARGET A DESTRA")
                right = True
                leftMotor.setVelocity(0.5 * MAX_SPEED)
                rightMotor.setVelocity(0)

        if sensor.getValue() > 900:  # wall reached
            count += 1
            high_sensors.append(sensor)
        if count > 1:
            count = 0
            print("WALL REACHED")
            wall_reached_gps = gps_sensor.getValues()
            wall_reached_compass = compass.getValues()
            wall_reached = True
            # tornare indietro per riprendere il box (in base al sensore gps)
            # if all(s.getValue() > 700 for s in high_sensors):
            print("GO BACK")
            leftMotor.setVelocity(-0.5 * MAX_SPEED)
            rightMotor.setVelocity(-0.5 * MAX_SPEED)
            high_sensors = []

    if wall_reached:
        # print(abs(wall_reached_gps[0] - gps_sensor.getValues()[0]))
        # print(abs(wall_reached_gps[1] - gps_sensor.getValues()[1]))
        # print(abs(wall_reached_gps[2] - gps_sensor.getValues()[2]))
        if abs(wall_reached_gps[0] - gps_sensor.getValues()[0]) > 0.3 or abs(wall_reached_gps[1] - gps_sensor.getValues()[1]) > 0.3 or abs(wall_reached_gps[2] - gps_sensor.getValues()[2]) > 0.3:
            print("STOP")
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            stopped = True
        if stopped:
            print("TURN RIGHT")
            leftMotor.setVelocity(0.2 * MAX_SPEED)
            rightMotor.setVelocity(0)
            print(abs(round(wall_reached_compass[0]) - round(compass.getValues()[2])))
            print(abs((wall_reached_compass[0]) - (compass.getValues()[2])))
            if round(abs(wall_reached_compass[0]) - (compass.getValues()[2]), 3) == 1.999:

                wall_reached = False
                stopped = False
                leftMotor.setVelocity(0.5 * MAX_SPEED)
                rightMotor.setVelocity(0.5 * MAX_SPEED)

    print("target obj position", camera.getRecognitionObjects()[0].get_position())
    print("target obj pos on image", target_object.get_position_on_image())
    print("gps sensor", gps_sensor.getValues())

    # for sensor in sensors:
    #     if sensor.getValue() > 100:
    #         count += 1
    #     if count > 4:  # wall reached
    #         print("STOP")
    #         if prev_compass is None:
    #             prev_compass = compass.getValues()
    #
    #         if prev_compass[2] - compass.getValues()[2] < 0.99:
    #             leftMotor.setVelocity(0.5 * MAX_SPEED)
    #             rightMotor.setVelocity(0)
    #         else:
    #             prev_compass = None
    #             leftMotor.setVelocity(0.5 * MAX_SPEED)
    #             rightMotor.setVelocity(0.5 * MAX_SPEED)

    # if d0.getValue() > 1000:
    #     # if d12.getValue() < d3.getValue():
    #     #     leftMotor.setVelocity(0.5 * MAX_SPEED)
    #     #     rightMotor.setVelocity(0)
    #     # else:
    #     #     leftMotor.setVelocity(0)
    #     #     rightMotor.setVelocity(0.5 * MAX_SPEED)
    #     print("STOP")
    #     stop = True
    #     leftMotor.setVelocity(0)
    #     rightMotor.setVelocity(0)


    print("n_objects = ", camera.getRecognitionNumberOfObjects())
    objects = camera.getRecognitionObjects()
    print("objects = ", objects)
    for obj in objects:
        print(obj.get_colors())
    print("\n")

    pass

# Enter here exit cleanup code.
