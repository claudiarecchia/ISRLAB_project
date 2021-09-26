MAX_SPEED = 6.28
from controller import Robot

class Body:
    def __init__(self):
        # robot instance.
        # self.TIME_STEP = 64
        self.robot = Robot()
        print(self.robot.getName())
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        print(self.TIME_STEP)
        self.inertial_unit = self.robot.getDevice('inertial unit')
        print(self.inertial_unit)
        self.inertial_unit.enable(self.TIME_STEP)

        self.gps_sensor = self.robot.getDevice('gps')
        self.gps_sensor.enable(self.TIME_STEP)
        print(self.gps_sensor)
        print(self.gps_sensor.getValues())

        # self.gps_sensor = self.gps_sensor_enable()

        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.TIME_STEP)
        self.camera.recognitionEnable(self.TIME_STEP)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.TIME_STEP)
        self.left_wheel_sensor = self.robot.getDevice('left wheel sensor')
        self.right_wheel_sensor = self.robot.getDevice('right wheel sensor')
        # self.left_wheel_sensor.enable(self.TIME_STEP)
        # self.right_wheel_sensor.enable(self.TIME_STEP)

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

        self.d0.enable(self.TIME_STEP)
        self.d1.enable(self.TIME_STEP)
        self.d2.enable(self.TIME_STEP)
        self.d3.enable(self.TIME_STEP)
        self.d4.enable(self.TIME_STEP)
        self.d5.enable(self.TIME_STEP)
        self.d6.enable(self.TIME_STEP)
        self.d7.enable(self.TIME_STEP)

        self.d8.enable(self.TIME_STEP)
        self.d9.enable(self.TIME_STEP)
        self.d10.enable(self.TIME_STEP)
        self.d11.enable(self.TIME_STEP)
        self.d12.enable(self.TIME_STEP)
        self.d13.enable(self.TIME_STEP)
        self.d14.enable(self.TIME_STEP)
        self.d15.enable(self.TIME_STEP)

        self.sensors = [self.d0, self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7,
                   self.d8, self.d9, self.d10, self.d11, self.d12, self.d13, self.d14, self.d15]

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.6 * MAX_SPEED)
        self.right_motor.setVelocity(0.6 * MAX_SPEED)
        print("body")

    def get_sensor_value(self, sensor_number):
        return self.sensors[sensor_number].getValue()

    def get_gps_values(self):
        return self.gps_sensor.getValues()

    def get_compass_values(self):
        return self.compass.getValues()

    def get_yaw(self):
        return self.inertial_unit.getRollPitchYaw()[1]

    def go_left(self):
        print("sinistra")
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0.4 * MAX_SPEED)

    def go_straight(self):
        print("avanti")
        self.left_motor.setVelocity(0.6 * MAX_SPEED)
        self.right_motor.setVelocity(0.6 * MAX_SPEED)

    def go_back(self):
        print("back")
        self.left_motor.setVelocity(-0.6 * MAX_SPEED)
        self.right_motor.setVelocity(-0.6 * MAX_SPEED)

    def go_right(self):
        print("destra")
        self.left_motor.setVelocity(0.4 * MAX_SPEED)
        self.right_motor.setVelocity(0)

    def stop_sim(self):
        print("stop simulazione. Massimo numero di box da riordinare raggiunto")
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        exit()

    def get_camera_number_objects(self):
        return self.camera.getRecognitionNumberOfObjects()

    def get_camera_objects(self):
        return self.camera.getRecognitionObjects()

    def get_timestep(self):
        return self.TIME_STEP

    # def gps_sensor_enable(self):
    #     gps_sensor = robot.getDevice('gps')
    #     gps_sensor.enable(64)
    #     print(gps_sensor)
    #     print(gps_sensor.getValues())
    #     return gps_sensor
    #
    def get_robot(self):
        return self.robot