"""
    BODY
    Corpo del robot (non intelligente)
    Ha il compito di fornire le informazioni che legge dalla simulazione
    quali i valori dei sensori, informazioni derivate dalla fotocamera ecc.
    Gestisce le componenti interne del robot, come i moduli montati su di esso
    mediante l'extension-slot e controlla la velocità delle ruote.
"""
from controller import Robot
from controllers.my_pioneer_controller.global_variables import *

gripper_motors = [None, None, None]


class Body:
    def __init__(self):
        self.robot = Robot()  # robot instance.
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        self.inertial_unit = self.robot.getDevice('inertial unit')
        self.inertial_unit.enable(self.TIME_STEP)

        self.gps_sensor = self.robot.getDevice('gps')
        self.gps_sensor.enable(self.TIME_STEP)

        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.TIME_STEP)
        self.camera.recognitionEnable(self.TIME_STEP)

        self.left_wheel_sensor = self.robot.getDevice('left wheel sensor')
        self.right_wheel_sensor = self.robot.getDevice('right wheel sensor')

        # da sinistra a destra
        self.d0 = self.robot.getDevice('so0')
        self.d1 = self.robot.getDevice('so1')
        self.d2 = self.robot.getDevice('so2')
        self.d3 = self.robot.getDevice('so3')
        self.d4 = self.robot.getDevice('so4')
        self.d5 = self.robot.getDevice('so5')
        self.d6 = self.robot.getDevice('so6')
        self.d7 = self.robot.getDevice('so7')
        self.d8 = self.robot.getDevice('so8')
        self.d9 = self.robot.getDevice('so9')
        self.d10 = self.robot.getDevice('so10')
        self.d11 = self.robot.getDevice('so11')
        self.d12 = self.robot.getDevice('so12')
        self.d13 = self.robot.getDevice('so13')
        self.d14 = self.robot.getDevice('so14')
        self.d15 = self.robot.getDevice('so15')

        self.sensors = [self.d0, self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7,
                        self.d8, self.d9, self.d10, self.d11, self.d12, self.d13, self.d14, self.d15]

        for s in self.sensors:
            s.enable(self.TIME_STEP)

        gripper_motors[0] = self.robot.getDevice('lift motor')
        gripper_motors[1] = self.robot.getDevice('left finger motor')
        gripper_motors[2] = self.robot.getDevice('right finger motor')

        self.move_fingers(0.1)
        self.lift(0.05)

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(DEFAULT_LIMIT_SPEED * MAX_SPEED)
        self.right_motor.setVelocity(DEFAULT_LIMIT_SPEED * MAX_SPEED)

    def get_sensor_value(self, sensor_number):
        return self.sensors[sensor_number].getValue()

    def get_gps_values(self):
        return self.gps_sensor.getValues()

    def get_yaw(self):
        return self.inertial_unit.getRollPitchYaw()[2]

    def go_left(self):
        # print("sinistra")
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0.4 * MAX_SPEED)

    def go_straight(self):
        # print("avanti")
        self.left_motor.setVelocity(DEFAULT_LIMIT_SPEED * MAX_SPEED)
        self.right_motor.setVelocity(DEFAULT_LIMIT_SPEED * MAX_SPEED)

    def slow_down(self):
        # print("rallenta")
        self.left_motor.setVelocity(0.1 * MAX_SPEED)
        self.right_motor.setVelocity(0.1 * MAX_SPEED)

    def go_back(self):
        # print("back")
        self.left_motor.setVelocity(-DEFAULT_LIMIT_SPEED * MAX_SPEED)
        self.right_motor.setVelocity(-DEFAULT_LIMIT_SPEED * MAX_SPEED)

    def go_right(self):
        # print("destra")
        self.left_motor.setVelocity(0.4 * MAX_SPEED)
        self.right_motor.setVelocity(0)

    def stop(self):
        # print("stop")
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def get_camera_number_objects(self):
        return self.camera.getRecognitionNumberOfObjects()

    def get_camera_objects(self):
        return self.camera.getRecognitionObjects()

    def get_timestep(self):
        return self.TIME_STEP

    def get_robot(self):
        return self.robot

    def lift(self, position):
        gripper_motors[0].setVelocity(GRIPPER_MOTOR_MAX_SPEED)
        gripper_motors[0].setPosition(position)

    def move_fingers(self, position):
        gripper_motors[1].setVelocity(GRIPPER_MOTOR_MAX_SPEED)
        gripper_motors[2].setVelocity(GRIPPER_MOTOR_MAX_SPEED)
        gripper_motors[1].setPosition(position)
        gripper_motors[2].setPosition(position)

    def get_values_all_sensors(self):
        values = []
        for s in self.sensors:
            values.append(s.getValue())
        return values
