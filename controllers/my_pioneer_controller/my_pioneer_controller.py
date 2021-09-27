"""my_pioneer_controller controller."""

"""
    BRAIN
    Controllore "intelligente" del robot disaccoppiato dal corpo 
    Leggere l'array dei sensori
    Scrivere l'azione definita dall'algoritmo di controllo
    Paradigma : SENSE-PLAN-ACT
"""

from controllers.my_pioneer_controller.RobotBody import Body

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
prev_sensors = []
target_obj_initial_position = None
left = False
right = False
before_turning_left_compass = []
turn_left = False
before_action_gps = []
straight = False
box_placed = False
total_boxes = 3
check_for_box = False
before_action_yaw = None
saved_yaw = False
turned = False

ACTION = ""
NEXT_ACTION = ""

yellow_corner = [-2.9, -0.2, -3.3]


class Brain:

    def __init__(self):
        self.robot = Body()
        self.target_object = None
        self.ACTION = ""
        self.NEXT_ACTION = ""
        self.PREVIOUS_ACTION = ""
        self.wall_reached = False
        self.check_for_box = False
        self.grabbed_box = False
        self.box_placed = False
        self.boxes_ids = []
        self.total_boxes = 3
        self.before_action_gps = []
        self.turned = False

    def get_robot(self):
        return self.robot.get_robot()

    def check_wall_reached(self):
        global count, wall_reached_gps, wall_reached_compass, straight, before_action_gps, ACTION
        # più di 1 sensore misura un valore elevato => parete
        # FRONT WALL REACHED => ho la parete davanti, allora devo andare indietro
        # if sum(s.getValue() > 950 for s in sensors) > 1:
        if self.robot.get_sensor_value(3) > 950 and self.robot.get_sensor_value(
                4) > 950 and ACTION != "go left" and ACTION != "go right":
            print("WALL REACHED")
            wall_reached_gps = self.robot.get_gps_values()
            wall_reached_compass = self.robot.get_compass_values()
            print("GO BACK")
            ACTION = "go back"

            self.wall_reached = True
            # Se ci sono più sensori che misurano un valore elevato sono in un angolo
            # (posso aver portato un cubo in posizione)
            # quindi non posso avanzare più e devo tornare indietro

            # if sum(s.getValue() > 990 for s in sensors) >= 3:
            #     straight = False
            #     before_action_gps = gps_sensor.getValues()
            return self.wall_reached
        else:
            return False

    def turn_after_wall_reached(self):
        global wall_reached_gps, before_action_yaw, saved_yaw

        if abs(wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.3 or abs(
                wall_reached_gps[1] - self.robot.get_gps_values()[1]) > 0.3 or abs(
            wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.3:

            # di default, vado a destra ma se sono in una situazione limite allora applico un'altra regola
            if self.robot.get_sensor_value(7) < 900:
                self.ACTION = "go right"
                self.NEXT_ACTION = "go straight"
                # salvo le informazioni per poter girare di 90 gradi
                # evito la sovrascrittura
                if not saved_yaw:
                    before_action_yaw = self.robot.get_yaw()
                    print("SALVATO")
                    saved_yaw = True

            else:
                # a destra ho un muro, quindi giro a sinistra (parte opposta)
                self.ACTION = "go left"
                self.NEXT_ACTION = "go straight"
                # evito la sovrascrittura
                if not saved_yaw:
                    before_action_yaw = self.robot.get_yaw()
                    print("SALVATO")
                    saved_yaw = True

    def do_turn_left(self, before_action_gps):
        global turn_left, straight
        print("valori")
        print(abs(before_action_gps[0] - self.robot.get_gps_values()[0]))
        print(abs(before_action_gps[1] - self.robot.get_gps_values()[1]))
        print(abs(before_action_gps[2] - self.robot.get_gps_values()[2]))
        if abs(before_action_gps[0] - self.robot.get_gps_values()[0]) > 0.5 or \
                abs(before_action_gps[1] - self.robot.get_gps_values()[1]) > 0.5 or \
                abs(before_action_gps[2] - self.robot.get_gps_values()[2]) > 0.5:
            print("TURN LEFT")
            self.ACTION = "go left"
            # go_left()
            # print(abs(round(before_turning_left_compass[0]) - round(compass.getValues()[2])))
            # print(abs((before_turning_left_compass[0]) - (compass.getValues()[2])))
            # if round(abs(before_turning_left_compass[0] - (compass.getValues()[2])), 2) == 0.97 and d7.getValue() > 900:

            # TODO considerare la differenza con un valore salvato precedentemente
            if self.robot.get_yaw() <= -1.55:  # -pi/2 = ovest
                print("vai avanti")
                self.ACTION = "go straight"
                # turn_left = False
                # straight = True

    def go_left(self):
        # print("sinistra")
        # leftMotor.setVelocity(0)
        # rightMotor.setVelocity(0.4 * MAX_SPEED)
        self.robot.go_left()

    def go_straight(self):
        # print("avanti")
        # leftMotor.setVelocity(0.6 * MAX_SPEED)
        # rightMotor.setVelocity(0.6 * MAX_SPEED)
        self.robot.go_straight()

    def go_back(self):
        # print("back")
        # leftMotor.setVelocity(-0.6 * MAX_SPEED)
        # rightMotor.setVelocity(-0.6 * MAX_SPEED)
        self.robot.go_back()

    def go_right(self):
        # print("destra")
        # leftMotor.setVelocity(0.4 * MAX_SPEED)
        # rightMotor.setVelocity(0)
        self.robot.go_right()

    def stop_sim(self):
        # print("stop simulazione. Massimo numero di box da riordinare raggiunto")
        # leftMotor.setVelocity(0)
        # rightMotor.setVelocity(0)
        # exit()
        self.robot.stop_sim()

    def check_for_other_box(self):
        global before_action_gps, box_placed, turned
        box_placed = False
        print("check for box")
        # # se il robot ha percorso abbastanza spazio
        # if abs(before_action_gps[0] - gps_sensor.getValues()[0]) > 0.26 or \
        #         abs(before_action_gps[1] - gps_sensor.getValues()[1]) > 0.26 or \
        #         abs(before_action_gps[2] - gps_sensor.getValues()[2]) > 0.26:
        #     print("qui go left")
        #     ACTION = "go left"
        #     turned = True
        #
        #     # if d7.getValue() > 900:
        #     #     # sensore 7 alto => parete a destra => giro a sinistra
        #     #     print("sinistra")
        #     #     go_left()
        #
        # else:
        #     if not turned:
        # ho appena posizionato un box, quindi devo tornare indietro
        # print("indietro")

        # print("qui go back")
        # ACTION = "go back"

        # TODO aggiungere sensore speculare a d7

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    # while self.robot.step(TIME_STEP) != -1:
    def controller(self):
        print("Sensore 0:", self.robot.get_sensor_value(0))
        print("Sensore 1:", self.robot.get_sensor_value(1))
        print("Sensore 2:", self.robot.get_sensor_value(2))
        print("Sensore 3:", self.robot.get_sensor_value(3))
        print("Sensore 4:", self.robot.get_sensor_value(4))
        print("Sensore 5:", self.robot.get_sensor_value(5))
        print("Sensore 6:", self.robot.get_sensor_value(6))
        print("Sensore 7:", self.robot.get_sensor_value(7))
        print("Rolle - Yaw - Pitch: ", self.robot.get_yaw())
        # for i in range(len(sensors)):
        #     prev_sensors.append(sensors[i].getValue())

        if self.target_object is None and self.robot.get_camera_number_objects() >= 1:
            # verifico che il target obj non sia già presente nella lista degli oggetti correttamente posizionati
            print("boxes ids: ")
            for element in self.boxes_ids:
                print("element in list:", element)
            for el in self.robot.get_camera_objects():
                print(el.get_id(), " ")
                if el.get_id() not in self.boxes_ids:
                    print("id: ", el.get_id())
                    self.target_object = el
                    target_obj_initial_position = el.get_position()

        # print("target obj:", target_object)
        print("compass:", self.robot.get_compass_values())

        if self.target_object is not None and not self.wall_reached and self.robot.get_camera_number_objects() > 0:
            self.ACTION, self.NEXT_ACTION = "", ""
            print(self.robot.get_camera_objects()[0].get_position()[0])
            print(self.robot.get_camera_objects()[0].get_position()[1])
            print(self.robot.get_camera_objects()[0].get_position()[2])

            print(round(self.robot.get_camera_objects()[0].get_position()[0], 2))
            print(round(self.robot.get_camera_objects()[0].get_position()[1], 2))
            print(round(self.robot.get_camera_objects()[0].get_position()[2], 2))
            for element in self.robot.get_camera_objects():
                if element.get_id() == self.target_object.get_id():
                    if element.get_position()[0] < -0.04:  # il target è a sinistra
                        # giro leggermente a sinistra per centrarlo
                        print("TARGET A SINISTRA")
                        # left = True
                        # leftMotor.setVelocity(0)
                        # rightMotor.setVelocity(0.6 * MAX_SPEED)
                        self.robot.go_left()
                    elif self.robot.get_camera_objects()[0].get_position()[0] > 0.04:  # il target è a destra
                        # giro leggermente a destra per centrarlo
                        print("TARGET A DESTRA")
                        # right = True
                        # leftMotor.setVelocity(0.6 * MAX_SPEED)
                        # rightMotor.setVelocity(0)
                        self.robot.go_right()

                    # se mi avvicino a un box rallento
                    elif not self.grabbed_box and not self.check_for_box and \
                            round(self.robot.get_camera_objects()[0].get_position()[0], 2) <= -0.01 and \
                            round(self.robot.get_camera_objects()[0].get_position()[1], 2) >= -0.05 and \
                            round(self.robot.get_camera_objects()[0].get_position()[2], 2) >= -0.3:
                        self.robot.move_fingers(0.03)
                        self.ACTION = "slow down"

                    # se davanti ho un box e lo afferro
                    elif self.PREVIOUS_ACTION != "stop" and round(self.robot.get_camera_objects()[0].get_position()[0], 1) >= -0.0 and \
                            round(self.robot.get_camera_objects()[0].get_position()[1], 2) >= -0.05 and \
                            round(self.robot.get_camera_objects()[0].get_position()[2], 2) >= -0.27:
                        self.ACTION = "stop"
                        self.PREVIOUS_ACTION = "stop"
                        self.grabbed_box = True
                        print("Box AFFERRATO")
                        self.robot.lift(0.0)

                    elif self.grabbed_box:
                        if element.get_colors() == [1.0, 1.0, 0.0]:  # giallo
                            print("set position")
                            if not self.wall_reached:
                                # vado avanti finchè posso
                                # quando mi accorgo che mi sto allontanando dal target modifico la direzione
                                if round(abs(self.robot.get_gps_values()[2] - yellow_corner[2]), 2) > 0.1:
                                    print("dritto")
                                    self.ACTION = "go straight"
                                elif round(abs(self.robot.get_gps_values()[0] - yellow_corner[0]), 2) > 0.05:
                                    print("sx")
                                    self.ACTION = "go left"
                                elif round(abs(self.robot.get_gps_values()[0] - yellow_corner[0]), 2) > 0.05:
                                    print("dx")
                                    self.ACTION = "go right"
                                else:
                                    print("-- arrivato")
                                    self.ACTION = "stop"
                                    self.robot.move_fingers(0.1)  # apro le braccia per lasciare il cubo
                                    self.robot.lift(0.05)  # e le abbasso
                                    self.box_placed = True
                                    self.grabbed_box = False
                                    self.boxes_ids.append(element.get_id())
                                    if len(self.boxes_ids) < self.total_boxes:
                                        self.check_for_box = True
                                        # devo prima andare indietro e poi girare a sinistra
                                        # indica che non ho ancora girato
                                        self.turned = False
                                        self.target_object = None
                                        self.before_action_gps = self.robot.get_gps_values()
                                    if len(self.boxes_ids) == self.total_boxes:
                                        print("raggiunto")
                                        exit()

                    elif self.PREVIOUS_ACTION != "stop":
                        print("DRITTO")
                        self.robot.go_straight()

        if self.NEXT_ACTION == "go straight" and self.ACTION == "go right":
            print("qui")
            # gira a destra finchè non raggiunge la posizione esatta verso destra (est)
            # if round((inertial_unit.getRollPitchYaw())[1] >= 1.5):  # pi/2 = est
            print("yaw:", abs(before_action_yaw - self.robot.get_yaw()))
            print("yaw:", abs(before_action_yaw - (self.robot.get_yaw())) >= 1.2)
            if abs(before_action_yaw - (self.robot.get_yaw())) >= 1.5:
                before_action_gps = self.robot.get_gps_values()
                # quando raggiunge est, deve rigirare a sinistra
                self.wall_reached = False
                saved_yaw = False
                self.ACTION = "go straight"
                self.NEXT_ACTION = "go left"

        if self.NEXT_ACTION == "go left" and self.ACTION == "go straight":
            if abs(before_action_gps[0] - self.robot.get_gps_values()[0]) > 0.5 or \
                    abs(before_action_gps[1] - self.robot.get_gps_values()[1]) > 0.5 or \
                    abs(before_action_gps[2] - self.robot.get_gps_values()[2]) > 0.5:
                self.wall_reached = False
                print("TURN LEFT")
                ACTION = "go left"
                NEXT_ACTION = "go straight"

        if self.NEXT_ACTION == "go straight" and self.ACTION == "go left":
            # if (inertial_unit.getRollPitchYaw())[1] <= -1.55:  # -pi/2 = ovest
            print("yaw:", abs(before_action_yaw - (self.robot.get_yaw())))
            # print("yaw:", abs(before_action_yaw - (inertial_unit.getRollPitchYaw())[1]) >= 1.2 )
            if abs(before_action_yaw - (self.robot.get_yaw()) >= 1.5):
                self.wall_reached = False
                saved_yaw = False
                print("vai avanti")
                ACTION = "go straight"

        if self.ACTION == "go back": self.robot.go_back()
        if self.ACTION == "go left": self.robot.go_left()
        if self.ACTION == "go straight": self.robot.go_straight()
        if self.ACTION == "go right": self.robot.go_right()
        if self.ACTION == "stop": self.robot.stop_sim()
        if self.ACTION == "slow down": self.robot.slow_down()


        # if turn_left: do_turn_left(before_action_gps)
        # if straight: go_straight()
        # if self.robot.get_camera_number_objects() == 1 and self.wall_reached and not self.check_for_box:
        #     objects = self.robot.get_camera_objects()
        #     obj = objects[0]
        #     print(round(obj.get_position()[0], ) == 0, round(obj.get_position()[1], 0) == 0,
        #           round(obj.get_position()[2], 0) == 0)
        #     print(1 <= round(self.robot.get_gps_values()[0], 1) <= 1.3,
        #           -0.3 <= round(self.robot.get_gps_values()[1], 1) <= -0.2,
        #           1 <= round(self.robot.get_gps_values()[2], 1) <= 1.1)
        #     if round(obj.get_position()[0], ) == 0 and \
        #             round(obj.get_position()[1], 0) == 0 and \
        #             round(obj.get_position()[2], 0) == 0 and \
        #             (-3.1 <= round(self.robot.get_gps_values()[0], 1) <= -2.9 and round(self.robot.get_gps_values()[1], 1) == -0.2 and
        #              -3.5 <= round(self.robot.get_gps_values()[2], 1) <= -3.3) \
        #             or \
        #             (1 <= round(self.robot.get_gps_values()[0], 1) <= 1.3 and -0.3 <= round(
        #                 self.robot.get_gps_values()[1], 1) <= -0.2 and 1 <= round(self.robot.get_gps_values()[2], 1) <= 1.1):
        #         print("controlli superati")
        #         box_placed = True
        #         boxes_ids.append(obj.get_id())
        #         if len(boxes_ids) < total_boxes:
        #             self.check_for_box = True
        #             # devo prima andare indietro e poi girare a sinistra
        #             # indica che non ho ancora girato
        #             turned = False
        #             self.target_object = None
        #             before_action_gps = self.robot.get_gps_values()
        #         if len(boxes_ids) == total_boxes:
        #             print("raggiunto")
        #             exit()
        #             # ACTION = "stop"

        if self.target_object is not None: self.check_for_box = False
        if self.check_for_box: self.check_for_other_box()

        self.check_wall_reached()
        if self.wall_reached: self.turn_after_wall_reached()

        # print("target obj pos on image", target_object.get_position_on_image())
        print("gps sensor", self.robot.get_gps_values())

        # print("n_objects = ", self.robot.get_camera_number_objects())
        # objects = self.robot.get_camera_objects()
        # print("objects = ", objects)
        # for obj in objects:
        #     print("colore:", obj.get_colors())
        #     print("posizione:", obj.get_position())
        # print("\n")

        pass

    # Enter here exit cleanup code.


if __name__ == "__main__":
    brain = Brain()
    while brain.get_robot().step(TIME_STEP) != -1:
        brain.controller()
