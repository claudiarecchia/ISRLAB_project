"""my_pioneer_controller controller."""

"""
    BRAIN
    Controllore "intelligente" del robot disaccoppiato dal corpo 
    Leggere l'array dei sensori
    Scrivere l'azione definita dall'algoritmo di controllo
    Paradigma : SENSE-PLAN-ACT
"""

from controllers.my_pioneer_controller.RobotBody import Body
import time

MAX_SPEED = 6.28
TIME_STEP = 64

turn_left = False
# before_action_gps = []
straight = False
box_placed = False
turned = False

ACTION = ""
NEXT_ACTION = ""

yellow_corner = [-2.9, -0.2, -3.3]
red_corner = [1.19, -0.2, -3.3]
green_corner = [-3.15, -0.2, 0.86]


def current_milli_time():
    return round(time.time() * 1000)


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
        # self.target_obj_initial_position = []
        self.wall_reached_gps = []
        self.wall_reached_compass = []
        self.saved_yaw = False
        self.before_action_yaw = None
        self.orientation_set = False
        self.turn_other_side = False
        self.saved_millis = None
        self.first_saving = True

    def get_robot(self):
        return self.robot.get_robot()

    def check_wall_reached(self):
        # più di 1 sensore misura un valore elevato => parete
        # FRONT WALL REACHED => ho la parete davanti, allora devo andare indietro
        # if sum(s.getValue() > 950 for s in sensors) > 1:

        # if self.robot.get_sensor_value(3) > 950 and self.robot.get_sensor_value(
        #         4) > 950 and ACTION != "go left" and ACTION != "go right":
        # if self.robot.get_number_wall_sensors() > 2:
        if self.robot.get_sensor_value(4) > 950 and self.robot.get_sensor_value(5) > 950 or \
                self.robot.get_sensor_value(2) > 950 and self.robot.get_sensor_value(3) > 950 or \
                self.robot.get_sensor_value(3) > 950 and self.robot.get_sensor_value(4) > 950:
            print("WALL REACHED")
            self.wall_reached_gps = self.robot.get_gps_values()
            self.wall_reached_compass = self.robot.get_compass_values()

            if self.grabbed_box: self.leave_box()

            print("GO BACK")
            self.ACTION = "go back"
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

    # def turn_after_wall_reached(self):
    #     print("turn after wall reached")
    #     # vado indietro perchè ho raggiunto la parete
    #     if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.3 or abs(
    #             self.wall_reached_gps[1] - self.robot.get_gps_values()[1]) > 0.3 or abs(
    #         self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.3:
    #
    #         # di default, vado a destra ma se sono in una situazione limite allora applico un'altra regola
    #         if self.robot.get_sensor_value(7) < 900:
    #             # print("gira a destra - turn")
    #             self.ACTION = "go right"
    #             self.NEXT_ACTION = "go straight"
    #             # salvo le informazioni per poter girare di 90 gradi
    #             # evito la sovrascrittura
    #             if not self.saved_yaw:
    #                 self.before_action_yaw = self.robot.get_yaw()
    #                 print("SALVATO")
    #                 self.saved_yaw = True
    #
    #         else:
    #             # print("gira a sinistra - turn")
    #             # a destra ho un muro, quindi giro a sinistra (parte opposta)
    #             self.ACTION = "go left"
    #             self.NEXT_ACTION = "go straight"
    #             # evito la sovrascrittura
    #             if not self.saved_yaw:
    #                 self.before_action_yaw = self.robot.get_yaw()
    #                 print("SALVATO")
    #                 self.saved_yaw = True

    def turn_after_wall_reached(self):
        print("turn after wall reached")
        # vado indietro perchè ho raggiunto la parete
        if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.3 or abs(
                self.wall_reached_gps[1] - self.robot.get_gps_values()[1]) > 0.3 or abs(
            self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.3:
            print("ho percorso abbastanza spazio")
            # se sto cercando un nuovo box, giro finchè non ne trovo uno nella camera
            if not self.check_for_box:
                # di default, vado a destra ma se sono in una situazione limite allora applico un'altra regola
                if self.robot.get_sensor_value(7) < 900:
                    # print("gira a destra - turn")
                    self.ACTION = "go right"
                    self.NEXT_ACTION = "go straight"
                    # salvo le informazioni per poter girare di 90 gradi
                    # evito la sovrascrittura
                    if not self.saved_yaw:
                        self.before_action_yaw = self.robot.get_yaw()
                        print("SALVATO")
                        self.saved_yaw = True

                else:
                    # print("gira a sinistra - turn")
                    # a destra ho un muro, quindi giro a sinistra (parte opposta)
                    self.ACTION = "go left"
                    self.NEXT_ACTION = "go straight"
                    # evito la sovrascrittura
                    if not self.saved_yaw:
                        self.before_action_yaw = self.robot.get_yaw()
                        print("SALVATO")
                        self.saved_yaw = True

            # se sto cercando un nuovo box, giro finchè non ne trovo uno nella camera
            else:
                # se il valore misurato sui sensori a sinistra è maggiore di quello misurato a destra
                # allora giro a destra
                print("sto cercando un box")
                self.wall_reached = False
                if self.robot.get_sensor_value(0) + self.robot.get_sensor_value(1) > self.robot.get_sensor_value(
                        6) + self.robot.get_sensor_value(7):
                    self.ACTION = "go right"
                else:
                    self.ACTION = "go left"

    # def do_turn_left(self, before_action_gps):
    #     print("valori")
    #     print(abs(self.before_action_gps[0] - self.robot.get_gps_values()[0]))
    #     print(abs(self.before_action_gps[1] - self.robot.get_gps_values()[1]))
    #     print(abs(self.before_action_gps[2] - self.robot.get_gps_values()[2]))
    #     if abs(self.before_action_gps[0] - self.robot.get_gps_values()[0]) > 0.5 or \
    #             abs(self.before_action_gps[1] - self.robot.get_gps_values()[1]) > 0.5 or \
    #             abs(self.before_action_gps[2] - self.robot.get_gps_values()[2]) > 0.5:
    #         print("TURN LEFT")
    #         self.ACTION = "go left"
    #
    #         # TODO considerare la differenza con un valore salvato precedentemente
    #         if self.robot.get_yaw() <= -1.55:  # -pi/2 = ovest
    #             print("vai avanti")
    #             self.ACTION = "go straight"
    #             # turn_left = False
    #             # straight = True

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

    def leave_box(self):
        print("-- arrivato")
        self.ACTION = "stop"
        self.robot.move_fingers(0.1)  # apro le braccia per lasciare il cubo
        self.robot.lift(0.05)  # e le abbasso
        self.box_placed = True
        self.grabbed_box = False
        self.first_saving = True
        self.PREVIOUS_ACTION = ""
        objects = self.robot.get_camera_objects()
        for obj in objects:
            if obj.get_id() == self.target_object.get_id():
                self.boxes_ids.append(obj.get_id())
        if len(self.boxes_ids) < self.total_boxes:
            self.check_for_box = True
            # devo prima andare indietro e poi girare a sinistra
            # indica che non ho ancora girato
            self.turned = False
            self.target_object = None
            self.before_action_gps = self.robot.get_gps_values()
            # self.ACTION = "go back"
        if len(self.boxes_ids) == self.total_boxes:
            print("raggiunto")
            exit()

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
                    # self.target_obj_initial_position = el.get_position()

        # print("compass:", self.robot.get_compass_values())

        if self.target_object is not None and not self.wall_reached and self.robot.get_camera_number_objects() > 0:
            self.ACTION, self.NEXT_ACTION = "", ""
            # print(self.robot.get_camera_objects()[0].get_position()[0])
            # print(self.robot.get_camera_objects()[0].get_position()[1])
            # print(self.robot.get_camera_objects()[0].get_position()[2])
            print(round(self.robot.get_camera_objects()[0].get_position()[0], 2))
            print(round(self.robot.get_camera_objects()[0].get_position()[1], 2))
            print(round(self.robot.get_camera_objects()[0].get_position()[2], 2))
            for element in self.robot.get_camera_objects():
                if element.get_id() == self.target_object.get_id():
                    if element.get_position()[0] < -0.04:  # il target è a sinistra
                        # giro leggermente a sinistra per centrarlo
                        print("TARGET A SINISTRA")
                        self.robot.go_left()
                    # elif self.robot.get_camera_objects()[0].get_position()[0] > 0.04:  # il target è a destra
                    elif element.get_position()[0] > 0.04:  # il target è a destra
                        # giro leggermente a destra per centrarlo
                        print("TARGET A DESTRA")
                        self.robot.go_right()

                    # se davanti ho un box e lo afferro
                    # elif self.PREVIOUS_ACTION != "stop" and round(self.robot.get_camera_objects()[0].get_position()[0], 1) >= -0.0 and \
                    #         round(self.robot.get_camera_objects()[0].get_position()[1], 2) >= -0.05 and \
                    #         round(self.robot.get_camera_objects()[0].get_position()[2], 2) >= -0.27:
                    # elif not self.grabbed_box and round(element.get_position()[0], 1) >= -0.0 and \
                    #         round(element.get_position()[1], 2) >= -0.05 and \
                    #         round(element.get_position()[2], 2) >= -0.27:
                    # elif not self.grabbed_box and \
                    #      round(element.get_position()[1], 2) >= -0.05 and \
                    #      round(element.get_position()[2], 2) >= -0.30:
                    #     self.ACTION = "stop"
                    #     self.PREVIOUS_ACTION = "stop"
                    #     self.grabbed_box = True
                    #     self.orientation_set = False
                    #     print("Box AFFERRATO")
                    #     self.robot.lift(0.0)

                    # se mi avvicino a un box rallento
                    elif not self.grabbed_box and not self.check_for_box and \
                            round(element.get_position()[1], 2) >= -0.05 and \
                            round(element.get_position()[2],
                                  2) >= -0.36:  # asse z => più è maggiore, più mi sto avvicinando al box
                        self.robot.move_fingers(0.025)
                        self.ACTION = "slow down"
                        if self.first_saving:
                            self.saved_millis = current_milli_time()
                            self.first_saving = False
                        # if round(element.get_position()[2], 2) >= -0.28:
                        if self.first_saving is not None and current_milli_time() - self.saved_millis >= 0.5*1000 and \
                                round(element.get_position()[2], 2) >= -0.30:
                            self.ACTION = "stop"
                            self.PREVIOUS_ACTION = "stop"
                            self.grabbed_box = True
                            self.orientation_set = False
                            print("Box AFFERRATO")
                            self.robot.lift(0.0)

                    elif self.grabbed_box:
                        print(element.get_colors())

                        # if element.get_colors() == [1.0, 1.0, 0.0]:  # giallo
                        #     print("set position")
                        #     # if not self.wall_reached:
                        #     # vado avanti finchè posso
                        #     # quando mi accorgo che mi sto allontanando dal target modifico la direzione
                        #     if round(abs(self.robot.get_gps_values()[2] - yellow_corner[2]), 2) > 0.1:
                        #         print("dritto")
                        #         self.ACTION = "go straight"
                        #     elif round(abs(self.robot.get_gps_values()[0] - yellow_corner[0]), 2) > 0.05:
                        #         print("sx")
                        #         self.ACTION = "go left"
                        #     elif round(abs(self.robot.get_gps_values()[0] - yellow_corner[0]), 2) > 0.05:
                        #         print("dx")
                        #         self.ACTION = "go right"

                        if element.get_colors() == [1.0, 1.0, 0.0]:  # giallo
                            print("giallo set position")
                            # giro il robot per puntare nella corretta direzione rispetto all'angolo di yaw
                            if round(self.robot.get_yaw(), 1) > -0.5 and not self.orientation_set:
                                print("uno")
                                self.ACTION = "go left"

                            elif round(self.robot.get_yaw(), 1) < -0.5 and not self.orientation_set:
                                print("due")
                                self.ACTION = "go right"

                            else:
                                print("tre")
                                self.orientation_set = True
                                print("dritto")
                                self.ACTION = "go straight"

                        elif element.get_colors() == [1.0, 0.0, 0.0]:  # rosso
                            print("rosso set position")

                            # giro il robot per puntare nella corretta direzione rispetto all'angolo di yaw
                            if round(self.robot.get_yaw(), 1) > 0.5 and not self.orientation_set:
                                print("uno")
                                self.ACTION = "go left"

                            elif round(self.robot.get_yaw(), 1) < 0.5 and not self.orientation_set:
                                print("due")
                                self.ACTION = "go right"

                            else:
                                print("tre")
                                self.orientation_set = True
                                print("dritto")
                                self.ACTION = "go straight"

                        elif element.get_colors() == [0.0, 1.0, 0.0]:  # verde
                            print("verde set position")

                            # giro il robot per puntare nella corretta direzione rispetto all'angolo di yaw
                            if round(self.robot.get_yaw(), 1) > -1.5 and not self.orientation_set:
                                if self.robot.get_number_wall_sensors('sx') == 0 and not self.turn_other_side:
                                    print("uno_sinistra")
                                    self.ACTION = "go left"
                                else:
                                    self.turn_other_side = True
                                    # raggiungo la posizione girando dall'altra parte
                                    print("uno_destra")
                                    self.ACTION = "go right"

                            elif round(self.robot.get_yaw(), 1) < -1.5 and not self.orientation_set:
                                print("due")
                                self.ACTION = "go right"

                            else:
                                print("tre")
                                self.orientation_set = True
                                print("dritto")
                                self.ACTION = "go straight"

                    # elif self.PREVIOUS_ACTION != "stop":
                    else:
                        print("DRITTO")
                        self.robot.go_straight()

        if self.NEXT_ACTION == "go straight" and self.ACTION == "go right":
            print("qui")
            # gira a destra finchè non raggiunge la posizione esatta verso destra (est)
            # if round((inertial_unit.getRollPitchYaw())[1] >= 1.5):  # pi/2 = est
            print("yaw:", abs(self.before_action_yaw - self.robot.get_yaw()))
            print("yaw:", abs(self.before_action_yaw - (self.robot.get_yaw())) >= 1.2)

            if not self.check_for_box:
                if abs(self.before_action_yaw - (self.robot.get_yaw())) >= 1.5:
                    self.before_action_gps = self.robot.get_gps_values()
                    # quando raggiunge est, deve rigirare a sinistra
                    self.wall_reached = False
                    self.saved_yaw = False
                    self.ACTION = "go straight"
                    self.NEXT_ACTION = "go left"
            # se devo cercare un nuovo box, allora smetto di girare quando ne trovo uno
            else:
                self.wall_reached = False
                self.saved_yaw = False
                # l'azione rimane la stessa
                if self.target_object is not None:
                    self.ACTION = "go straight"

        if self.NEXT_ACTION == "go left" and self.ACTION == "go straight":
            if abs(self.before_action_gps[0] - self.robot.get_gps_values()[0]) > 0.5 or \
                    abs(self.before_action_gps[1] - self.robot.get_gps_values()[1]) > 0.5 or \
                    abs(self.before_action_gps[2] - self.robot.get_gps_values()[2]) > 0.5:
                self.wall_reached = False
                print("TURN LEFT")
                self.ACTION = "go left"
                self.NEXT_ACTION = "go straight"

        if self.NEXT_ACTION == "go straight" and self.ACTION == "go left":
            # if (inertial_unit.getRollPitchYaw())[1] <= -1.55:  # -pi/2 = ovest
            print("yaw:", abs(self.before_action_yaw - (self.robot.get_yaw())))
            # print("yaw:", abs(before_action_yaw - (inertial_unit.getRollPitchYaw())[1]) >= 1.2 )
            if not self.check_for_box:
                if abs(self.before_action_yaw - (self.robot.get_yaw()) >= 1.5):
                    self.wall_reached = False
                    self.saved_yaw = False
                    print("vai avanti")
                    self.ACTION = "go straight"
            # se devo cercare un nuovo box, allora smetto di girare quando ne trovo uno
            else:
                self.wall_reached = False
                self.saved_yaw = False
                # l'azione rimane la stessa
                if self.target_object is not None:
                    self.ACTION = "go straight"

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
        if self.wall_reached and not self.grabbed_box: self.turn_after_wall_reached()

        # print("target obj pos on image", target_object.get_position_on_image())
        print("gps sensor", self.robot.get_gps_values())

        # print("n_objects = ", self.robot.get_camera_number_objects())
        # objects = self.robot.get_camera_objects()
        # print("objects = ", objects)
        # for obj in objects:
        #     print("colore:", obj.get_colors())
        #     print("posizione:", obj.get_position())
        print("\n")

        pass

    # Enter here exit cleanup code.


if __name__ == "__main__":
    brain = Brain()
    while brain.get_robot().step(TIME_STEP) != -1:
        brain.controller()
