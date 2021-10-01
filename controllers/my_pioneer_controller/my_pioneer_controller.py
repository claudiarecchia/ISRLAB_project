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
blue_corner = [1.19, -0.2, 0.86]

yellow_yaw = 0.4
red_yaw = -0.5
green_yaw = 2.5
blue_yaw = -2.5


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
        self.total_boxes = 4
        self.before_action_gps = []
        self.turned = False
        self.wall_reached_gps = []
        self.wall_reached_compass = []
        self.saved_yaw = False
        self.before_action_yaw = None
        self.orientation_set = False
        self.turn_other_side = False
        self.saved_millis = None
        self.first_saving = True
        self.balance = False
        self.first_saving_gps = True
        self.left_sequence = False
        self.turn_reached = False
        self.first_saving_yaw = True
        self.final_step = False
        self.right_sequence = False
        self.destination = None
        self.angle = None
        self.modify_yaw = False
        self.first_saving_angle = True

    def get_robot(self):
        return self.robot.get_robot()

    def check_wall_reached(self):

        if self.robot.get_number_wall_sensors() >= 2 or self.robot.wall():
            self.ACTION = "stop"
            print("WALL REACHED")
            self.wall_reached_gps = self.robot.get_gps_values()

            if self.grabbed_box:
                if self.check_if_destination_reached():
                    self.leave_box()

            print("GO BACK")
            self.ACTION = "go back"
            self.wall_reached = True
            return self.wall_reached
        else:
            return False

    def turn_to_destination(self):
        print("turn to destination")
        if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.5 or \
                abs(self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.5 or self.modify_yaw:
            print("ho percorso abbastanza spazio")
            # ho la parete a sinistra
            # quindi devo andare più a destra
            if self.left_sequence and self.first_saving_angle:
                if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                    self.angle = self.angle - 0.3
                    print("modificato:", self.angle)
                    self.first_saving_yaw = True
                    self.first_saving_gps = True
                    self.left_sequence = False
                    self.turn_reached = False
                    self.balance = False
                    self.final_step = False

            if self.right_sequence and self.first_saving_angle:
                if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                    self.angle = self.angle + 0.4
                    print("modificato:", self.angle)
                    self.first_saving_yaw = True
                    self.first_saving_gps = True
                    self.right_sequence = False
                    self.turn_reached = False
                    self.balance = False
                    self.final_step = False

            self.first_saving_angle = False

            if round(self.robot.get_yaw(), 1) != round(self.angle, 1) and not self.robot.wall():
                self.modify_yaw = True
                print(round(self.robot.get_yaw(), 1))

                if round(self.robot.get_yaw(), 1) >= round(self.angle, 1):
                    print("maggiore")
                    if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                        print("turn to destination_uno")
                        self.ACTION = "go right"
                    else:
                        print("turn to destination_due")
                        self.ACTION = "go left"

                elif round(self.robot.get_yaw(), 1) < round(self.angle, 1):
                    print("minore")
                    if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                        print("turn to destination_uno")
                        self.ACTION = "go left"
                    else:
                        print("turn to destination_due")
                        self.ACTION = "go right"

            else:
                print("uguale")
                self.ACTION = "go straight"
                self.modify_yaw = False
                self.wall_reached = False

    def turn_after_wall_reached(self):
        print("turn after wall reached")

        # controllo se mentre vado indietro arrivo a una parete
        if not self.robot.wall():
            # vado indietro fino a raggiungere un determinato spazio percorso
            if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.3 or \
                    abs(self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.3:
                self.turn_right_or_left()

        # se mentre vado indietro sono arrivato a una parete (anche dietro) mi fermo prima
        else:
            print("parete raggiunta")
            self.ACTION = "stop"
            self.turn_right_or_left()

    def turn_right_or_left(self):
        print("ho percorso abbastanza spazio")
        if not self.check_for_box:
            # di default, vado a destra ma se sono in una situazione limite allora applico un'altra regola
            if self.robot.get_sensor_value(7) < 900:
                self.ACTION = "go right"
                self.NEXT_ACTION = "go straight"
                # salvo le informazioni per poter girare di 90 gradi
                # evito la sovrascrittura
                if not self.saved_yaw:
                    self.before_action_yaw = self.robot.get_yaw()
                    print("SALVATO")
                    self.saved_yaw = True

            else:
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

    def check_for_other_box(self):
        global before_action_gps, box_placed, turned
        box_placed = False
        print("check for box")

        # TODO aggiungere sensore speculare a d7

    def check_if_destination_reached(self):
        reached = False
        if round(self.robot.get_gps_values()[0], 0) - round(self.destination[0], 0) == 0 and \
                round(self.robot.get_gps_values()[2], 0) - round(self.destination[2], 0) == 0:
            reached = True
        return reached

    def leave_box(self):
        print("-- arrivato")
        self.ACTION = "stop"
        self.robot.move_fingers(0.1)  # apro le braccia per lasciare il cubo
        self.robot.lift(0.05)  # e le abbasso
        self.destination = None
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
        if self.left_sequence or self.right_sequence:
            print("reimposto variabili")
            self.first_saving_yaw = True
            self.first_saving_gps = True
            self.left_sequence = False
            self.turn_reached = False
            self.balance = False
            self.final_step = False
            self.right_sequence = False
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

                    if el.get_colors() == [1.0, 1.0, 0.0]:  # giallo
                        self.destination = yellow_corner
                        self.angle = yellow_yaw
                    elif el.get_colors() == [1.0, 0.0, 0.0]:  # rosso
                        self.destination = red_corner
                        self.angle = red_yaw
                    elif el.get_colors() == [0.0, 1.0, 0.0]:  # verde
                        self.destination = green_corner
                        self.angle = green_yaw
                    elif el.get_colors() == [0.0, 0.0, 1.0]:  # blu
                        self.destination = blue_corner
                        self.angle = blue_yaw

        if self.target_object is not None and not self.wall_reached and self.robot.get_camera_number_objects() > 0:
            self.ACTION, self.NEXT_ACTION = "", ""

            print(round(self.robot.get_camera_objects()[0].get_position()[0], 2))
            print(round(self.robot.get_camera_objects()[0].get_position()[1], 2))
            print(round(self.robot.get_camera_objects()[0].get_position()[2], 2))
            for element in self.robot.get_camera_objects():
                if element.get_id() == self.target_object.get_id():
                    if element.get_position()[0] < -0.1 and abs(element.get_position()[2]) > 1 or \
                            element.get_position()[0] < -0.02 and abs(element.get_position()[2]) < 1:
                        # if element.get_position()[0] < -0.04:  # il target è a sinistra
                        # giro leggermente a sinistra per centrarlo
                        print("TARGET A SINISTRA")
                        # self.robot.go_left()
                        self.ACTION = "go left"
                    # elif self.robot.get_camera_objects()[0].get_position()[0] > 0.04:  # il target è a destra
                    # elif element.get_position()[0] > 0.04:  # il target è a destra
                    elif element.get_position()[0] > 0.1 and abs(element.get_position()[2]) > 1 or \
                            element.get_position()[0] > 0.02 and abs(element.get_position()[2]) < 1:
                        # giro leggermente a destra per centrarlo
                        print("TARGET A DESTRA")
                        # self.robot.go_right()
                        self.ACTION = "go right"

                    # se mi avvicino a un box rallento
                    elif not self.grabbed_box and not self.check_for_box and \
                            round(element.get_position()[2],
                                  2) >= -0.30:  # asse z => più è maggiore, più mi sto avvicinando al box
                        self.robot.move_fingers(0.03)
                        self.ACTION = "slow down"
                        if self.first_saving:
                            self.saved_millis = current_milli_time()
                            self.first_saving = False

                        if self.first_saving is not None and current_milli_time() - self.saved_millis >= 1 * 1000:
                            # and \
                            # round(element.get_position()[2], 2) >= -0.29:
                            self.ACTION = "stop"
                            self.PREVIOUS_ACTION = "stop"
                            self.grabbed_box = True
                            self.orientation_set = False
                            print("Box AFFERRATO")
                            self.robot.lift(0.0)

                    elif self.grabbed_box:
                        print(element.get_colors())

                        # if element.get_colors() == [1.0, 1.0, 0.0]:  # giallo
                        #     self.destination = yellow_corner
                        #     self.angle = yellow_yaw
                        # elif element.get_colors() == [1.0, 0.0, 0.0]:  # rosso
                        #     self.destination = red_corner
                        #     self.angle = red_yaw
                        # elif element.get_colors() == [0.0, 1.0, 0.0]:  # verde
                        #     self.destination = green_corner
                        #     self.angle = green_yaw
                        # elif element.get_colors() == [0.0, 0.0, 1.0]:  # blu
                        #     self.destination = blue_corner
                        #     self.angle = blue_yaw

                        print("set position")
                        # giro il robot per puntare nella corretta direzione rispetto all'angolo di yaw
                        if round(self.robot.get_yaw(), 1) > self.angle and not self.orientation_set and not self.balance:
                            if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                                print("due_due")
                                self.ACTION = "go right"
                            else:
                                print("uno")
                                self.ACTION = "go left"

                        elif round(self.robot.get_yaw(), 1) < self.angle and not self.orientation_set and not self.balance:
                            if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                                print("due_uno")
                                self.ACTION = "go left"
                            else:
                                print("due_due")
                                self.ACTION = "go right"

                        else:
                            print("tre")
                            self.orientation_set = True

                            # controllo la vicinanza alle pareti di destra e sinistra
                            if self.robot.get_number_wall_sensors('sx') > self.robot.get_number_wall_sensors(
                                    'dx') or self.left_sequence:
                                if self.first_saving_yaw:
                                    self.balance = True
                                    self.left_sequence = True
                                    print("left sequence:", self.left_sequence)
                                    self.saved_yaw = self.robot.get_yaw()
                                    self.first_saving_angle = True  # per quando incontro la parete davanti
                                self.first_saving_yaw = False

                                print("sinistra più vicina")
                                print(abs(self.saved_yaw) - abs(self.robot.get_yaw()))

                                if not self.turn_reached:
                                    print("< 20 destra")
                                    self.ACTION = "go right"

                                if abs(self.saved_yaw) - abs(
                                        self.robot.get_yaw()) >= 0.1 or self.final_step:  # giro di 20 gradi
                                    self.turn_reached = True

                                    if self.first_saving_gps:
                                        self.before_action_gps = self.robot.get_gps_values()
                                    self.first_saving_gps = False

                                    # se ho finito di percorrere un determinato spazio

                                    # e se posso andare avanti (controllo sensori)
                                    if (abs(self.robot.get_gps_values()[0] - self.before_action_gps[0]) >= 0.2 or \
                                            abs(self.robot.get_gps_values()[2] - self.before_action_gps[2]) >= 0.2) and \
                                            round(self.robot.get_yaw(), 1) != round(self.angle, 1) and \
                                            self.robot.get_wall_proximity('dx') >= 2:  # cioè sono abbastanza vicino all'angolo
                                        print("angolo yaw")
                                        self.ACTION = "go left"

                                        # e reimposto le variabili quando finisco di girare il robot
                                        if round(self.robot.get_yaw(), 1) == self.angle:
                                            print("reimposto variabili")
                                            self.first_saving_yaw = True
                                            self.first_saving_gps = True
                                            self.left_sequence = False
                                            self.turn_reached = False
                                            self.balance = False
                                            self.final_step = False

                                    # altrimenti devo ancora finire di percorrerlo e devo andare dritto
                                    else:
                                        print("dritto----")
                                        self.ACTION = "go straight"
                                        self.final_step = True

                            elif self.robot.get_number_wall_sensors('dx') > self.robot.get_number_wall_sensors(
                                    'sx') or self.right_sequence:
                                if self.first_saving_yaw:
                                    self.balance = True
                                    self.right_sequence = True
                                    print("right sequence:", self.right_sequence)
                                    self.saved_yaw = self.robot.get_yaw()
                                    self.first_saving_angle = True  # per quando incontro la parete davanti
                                self.first_saving_yaw = False

                                print("destra più vicina")
                                print(abs(self.saved_yaw) - abs(self.robot.get_yaw()))

                                if not self.turn_reached:
                                    print("< 20 sinistra")
                                    self.ACTION = "go left"

                                if abs(self.saved_yaw) - abs(
                                        self.robot.get_yaw()) >= 0.1 or self.final_step:  # giro di 20 gradi
                                    self.turn_reached = True

                                    if self.first_saving_gps:
                                        self.before_action_gps = self.robot.get_gps_values()
                                    self.first_saving_gps = False

                                    # se ho finito di percorrere un determinato spazio
                                    if abs(self.robot.get_gps_values()[0] - self.before_action_gps[0]) >= 0.2 or \
                                            abs(self.robot.get_gps_values()[2] - self.before_action_gps[2]) >= 0.2 and \
                                            round(self.robot.get_yaw(), 1) != self.angle and \
                                            self.robot.get_wall_proximity('sx') >= 2:  # cioè sono abbastanza vicino all'angolo
                                        print("angolo yaw")
                                        self.ACTION = "go right"

                                        # e reimposto le variabili quando finisco di girare il robot
                                        if round(self.robot.get_yaw(), 1) == self.angle:
                                            print("reimposto variabili")
                                            self.first_saving_yaw = True
                                            self.first_saving_gps = True
                                            self.right_sequence = False
                                            self.turn_reached = False
                                            self.balance = False
                                            self.final_step = False

                                    # altrimenti devo ancora finire di percorrerlo e devo andare dritto
                                    else:
                                        print("dritto----")
                                        self.ACTION = "go straight"
                                        self.final_step = True


                            else:
                                print("else")
                                self.ACTION = "go straight"

                    else:
                        print("DRITTO")
                        self.ACTION = "go straight"

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
            print("yaw:", abs(self.before_action_yaw - (self.robot.get_yaw())))
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

        if self.target_object is not None: self.check_for_box = False
        if self.check_for_box:
            self.box_placed = False
            print("check for box")

        self.check_wall_reached()
        if self.wall_reached and self.grabbed_box: self.turn_to_destination()
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
