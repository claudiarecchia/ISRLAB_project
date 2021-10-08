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
import redis
import json

MAX_SPEED = 6.28
TIME_STEP = 64

turn_left = False
straight = False
box_placed = False
turned = False

ACTION = ""
NEXT_ACTION = ""

yellow_corner = [-2.9, -0.2, -3.3]
red_corner = [1.49, -0.2, -3.3]  # 1.19 x
green_corner = [-3.15, -0.2, 0.66]  # 0.86 z
blue_corner = [1.19, -0.2, 0.86]

yellow_yaw = 0.4
red_yaw = -0.5
green_yaw = 2.5
blue_yaw = -2.5

yellow = [1.0, 1.0, 0.0]
red = [1.0, 0.0, 0.0]
green = [0.0, 1.0, 0.0]
blue = [0.0, 0.0, 1.0]


def current_milli_time():
    return round(time.time() * 1000)


class Brain:

    def __init__(self, channel_in_spheres, channel_out_spheres, channel_in_current_sphere, channel_out_current_sphere,
                 ch_in_current_gps, ch_out_current_gps):
        self._myR = redis.Redis(decode_responses=True)
        print(self._myR.info())

        # sfere totali
        self._ch_in_spheres = channel_in_spheres
        self._ch_out_spheres = channel_out_spheres

        # sfera attuale e colore
        self._ch_in_current_sphere = channel_in_current_sphere
        self._ch_out_current_sphere = channel_out_current_sphere

        # coordinate gps
        self._ch_in_current_gps = ch_in_current_gps
        self._ch_out_current_gps = ch_out_current_gps

        # pubsub to "in" channels
        self._pubsub_spheres = self._myR.pubsub()
        self._pubsub_spheres.subscribe(self._ch_in_spheres)

        self._pubsub_current_sphere = self._myR.pubsub()
        self._pubsub_current_sphere.subscribe(self._ch_in_current_sphere)

        self._pubsub_current_gps = self._myR.pubsub()
        self._pubsub_current_gps.subscribe(self._ch_in_current_gps)

        self.current_sphere_other_robot = [None, None]
        self.current_coordinates_other_robot = []

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
        self.total_boxes = 9
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
        self.time_left_box = None
        self.last_box = False
        self.backup_angle = None
        self.stop_time = None
        self.first_save_stop_time = True
        self.stopped = False
        self.modified_angle = 0
        self.color = None

    def get_robot(self):
        return self.robot.get_robot()

    def check_wall_behind_if_back(self):
        if self.robot.wall('back') and self.ACTION == "go back":
            return True
        else:
            return False

    def check_wall_reached(self):
        print("check wall reached; num sensors: ", self.robot.get_number_wall_sensors())

        if self.robot.get_number_wall_sensors() >= 2 or self.check_wall_behind_if_back() or self.robot.wall('front'):
            print(self.robot.get_number_wall_sensors())

            if not self.stopped:
                print("mi fermo")
                self.ACTION = "stop"
                self.stopped = True
                self.stop_time = current_milli_time()

            print("WALL REACHED")

            self.wall_reached_gps = self.robot.get_gps_values()

            if self.grabbed_box:
                if self.check_if_destination_reached():
                    self.leave_box()

            self.wall_reached = True
            return self.wall_reached
        else:
            return False

    def turn_to_destination(self):
        print("turn to destination")
        if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.5 or \
                abs(self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.5 or self.modify_yaw:

            self.stopped = False

            print("ho percorso abbastanza spazio")
            # ho la parete a sinistra
            # quindi devo andare più a destra
            if self.left_sequence and self.first_saving_angle:
                if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                    if self.modified_angle <= 4:  # modifico l'angolo non più di 4 volte
                        if abs(self.backup_angle - self.angle) >= 0.6:
                            self.angle = self.angle - 0.1
                            self.modified_angle = self.modified_angle + 1
                        else:
                            self.angle = self.angle - 0.3
                            self.modified_angle = self.modified_angle + 1
                    print("modificato:", self.angle)
                    self.first_saving_yaw = True
                    self.first_saving_gps = True
                    self.left_sequence = False
                    self.turn_reached = False
                    self.balance = False
                    self.final_step = False

            if self.right_sequence and self.first_saving_angle:
                if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                    if self.modified_angle <= 4:  # modifico l'angolo non più di 4 volte
                        if abs(self.backup_angle - self.angle) >= 0.6:
                            self.angle = self.angle + 0.1
                        else:
                            self.angle = self.angle + 0.3
                    print("modificato:", self.angle)
                    self.first_saving_yaw = True
                    self.first_saving_gps = True
                    self.right_sequence = False
                    self.turn_reached = False
                    self.balance = False
                    self.final_step = False

            self.first_saving_angle = False

            if round(self.robot.get_yaw(), 1) != round(self.angle, 1) and not self.robot.wall('back'):
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

        # if not (self.robot.get_number_wall_sensors() >= 2 or self.check_wall_behind_if_back() or self.robot.wall('front')):
        #     self.stopped = False

        # controllo se mentre vado indietro arrivo a una parete
        if not self.robot.wall('back'):
            # vado indietro fino a raggiungere un determinato spazio percorso
            if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.3 or \
                    abs(self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.3:
                # self.stopped = False
                self.turn_right_or_left()

        # se mentre vado indietro sono arrivato a una parete (anche dietro) mi fermo prima
        else:
            self.first_save_stop_time = True  # per la prossima volta che arriverò a una parete
            print("parete raggiunta")
            # self.ACTION = "stop"

            # self.stopped = False
            self.turn_right_or_left()

    def turn_right_or_left(self):
        print("ho percorso abbastanza spazio")

        if not (self.robot.get_number_wall_sensors() >= 2 or self.check_wall_behind_if_back() or self.robot.wall(
                'front')):
            self.stopped = False

        if not self.check_for_box:
            print("sono qui")
            # di default, vado a destra ma se sono in una situazione limite allora applico un'altra regola
            if self.robot.get_sensor_value(7) < 900:
                self.ACTION = "go right"
                self.NEXT_ACTION = "go straight"
                self.wall_reached = False
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

    def back_after_last_box(self):
        # dopo 3 secondi in cui stavo andando indietro, termino l'esecuzione
        if current_milli_time() - self.time_left_box >= 3 * 1000:
            if len(self.boxes_ids) == self.total_boxes:
                exit()
            else:
                self.ACTION = "stop"

    def check_for_other_box(self):
        global before_action_gps, box_placed, turned
        box_placed = False
        print("check for box")

    def check_if_destination_reached(self):
        reached = False
        if round(self.robot.get_gps_values()[0], 0) - round(self.destination[0], 0) == 0 and \
                round(self.robot.get_gps_values()[2], 0) - round(self.destination[2], 0) == 0:
            reached = True
        return reached

    def leave_box(self):
        self.write_placed_spheres(self.target_object.get_id())
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
        self.time_left_box = current_milli_time()
        for obj in objects:
            if obj.get_id() == self.target_object.get_id():
                self.boxes_ids.append(obj.get_id())

        if len(self.boxes_ids) == self.total_boxes or self.check_if_last_box_other_robot():
            print("raggiunto")
            self.ACTION = "go back"
            self.last_box = True
            self.check_for_box = False

        elif len(self.boxes_ids) < self.total_boxes:
            self.check_for_box = True
            # devo prima andare indietro e poi girare a sinistra
            # indica che non ho ancora girato
            self.turned = False
            self.target_object = None
            self.before_action_gps = self.robot.get_gps_values()

            if self.left_sequence or self.right_sequence:
                print("reimposto variabili")
                self.first_saving_yaw = True
                self.first_saving_gps = True
                self.left_sequence = False
                self.turn_reached = False
                self.balance = False
                self.final_step = False
                self.right_sequence = False

    def check_if_last_box_other_robot(self):
        if len(self.boxes_ids) == self.total_boxes - 1 and self.current_sphere_other_robot[0] is not None:
            return True
        else:
            return False

    def not_same_color_other_robot_unless_last_sphere(self, el):
        # controllo che il colore della sfera visualizzata sia diverso dal colore della sfera che sta posizionando l'altro robot
        # in modo da non collidere in fase di posizionamento della sfera
        # altrimenti, se si tratta dell'ultima sfera da posizionare, non tengo conto del colore
        response = False
        if el.get_colors() != self.current_sphere_other_robot[1] and len(
                self.boxes_ids) < self.total_boxes - 1: response = True
        if el.get_colors() == self.current_sphere_other_robot[1] and len(
                self.boxes_ids) == self.total_boxes - 1: response = True
        return response

        # Main loop:

    def near_other_robot(self):
        if self.current_coordinates_other_robot:
            # print(round(self.robot.get_gps_values()[0], 0))
            # print(round(self.current_coordinates_other_robot[0], 0))
            # print(round(self.robot.get_gps_values()[1], 0))
            # print(round(self.current_coordinates_other_robot[1], 0))
            if (round(self.robot.get_gps_values()[0], 0) == round(self.current_coordinates_other_robot[0], 0) and \
                    round(self.robot.get_gps_values()[2], 1) == round(self.current_coordinates_other_robot[2], 1)) and \
                    self.robot.close_to_other_robot() \
                    or \
                    round(self.robot.get_gps_values()[0], 1) == round(self.current_coordinates_other_robot[0], 1) and \
                    round(self.robot.get_gps_values()[2], 0) == round(self.current_coordinates_other_robot[2], 0) and \
                    self.robot.close_to_other_robot():
                return True
            else:
                return False

    def robot_collision(self):
        if self.current_coordinates_other_robot != []:
            print(round(self.robot.get_gps_values()[0], 0))
            print(round(self.current_coordinates_other_robot[0], 0))
            print(round(self.robot.get_gps_values()[1], 0))
            print(round(self.current_coordinates_other_robot[1], 0))
        # if (self.robot.get_number_wall_sensors() >= 1 or self.check_wall_behind_if_back() or self.robot.wall(
        #        'front')) and self.near_other_robot():
        if self.near_other_robot():
            # lo slave si ferma sempre in questa situazione
            print("Scontro con il master. Mi fermo")
            self.wall_reached = False
            self.ACTION = "stop"
            return True
        else: return False

    # - perform simulation steps until Webots is stopping the controller
    def controller(self):
        print("Sensore 0:", self.robot.get_sensor_value(0))
        print("Sensore 1:", self.robot.get_sensor_value(1))
        print("Sensore 2:", self.robot.get_sensor_value(2))
        print("Sensore 3:", self.robot.get_sensor_value(3))
        print("Sensore 4:", self.robot.get_sensor_value(4))
        print("Sensore 5:", self.robot.get_sensor_value(5))
        print("Sensore 6:", self.robot.get_sensor_value(6))
        print("Sensore 7:", self.robot.get_sensor_value(7))
        print("Sensore 8:", self.robot.get_sensor_value(8))
        print("Sensore 9:", self.robot.get_sensor_value(9))
        print("Sensore 10:", self.robot.get_sensor_value(10))
        print("Sensore 11:", self.robot.get_sensor_value(11))
        print("Sensore 12:", self.robot.get_sensor_value(12))
        print("Sensore 13:", self.robot.get_sensor_value(13))
        print("Sensore 14:", self.robot.get_sensor_value(14))
        print("Sensore 15:", self.robot.get_sensor_value(15))
        print("Rolle - Yaw - Pitch: ", self.robot.get_yaw())

        if self.target_object is None and self.robot.get_camera_number_objects() >= 1:
            # verifico che il target obj non sia già presente nella lista degli oggetti correttamente posizionati
            print("boxes ids: ")
            for element in self.boxes_ids:
                print("element in list:", element)
            for el in self.robot.get_camera_objects():
                print(el.get_id(), " ")
                # lo slave aspetta che il master abbia deciso il colore della propria sfera
                if el.get_id() not in self.boxes_ids and el.get_id() != self.current_sphere_other_robot[0] and \
                        self.not_same_color_other_robot_unless_last_sphere(el) and self.current_sphere_other_robot[0] is not None:
                    print("id: ", el.get_id())
                    self.target_object = el
                    self.modified_angle = 0  # inizializzo il numero di volte in cui si è modificato l'angolo del nuovo oggetto

                    if el.get_colors() == yellow:  # giallo
                        print("GIALLO")
                        self.destination = yellow_corner
                        self.backup_angle = yellow_yaw
                        self.angle = yellow_yaw
                        self.color = yellow
                    elif el.get_colors() == red:  # rosso
                        print("ROSSO")
                        self.destination = red_corner
                        self.backup_angle = red_yaw
                        self.angle = red_yaw
                        self.color = red
                    elif el.get_colors() == green:  # verde
                        print("VERDE")
                        self.destination = green_corner
                        self.backup_angle = green_yaw
                        self.angle = green_yaw
                        self.color = green
                    elif el.get_colors() == blue:  # blu
                        print("BLU")
                        self.destination = blue_corner
                        self.backup_angle = blue_yaw
                        self.angle = blue_yaw
                        self.color = blue

        if self.target_object is not None and not self.wall_reached and self.robot.get_camera_number_objects() > 0:
            self.ACTION, self.NEXT_ACTION = "", ""

            # print(round(self.robot.get_camera_objects()[0].get_position()[0], 2))
            # print(round(self.robot.get_camera_objects()[0].get_position()[1], 2))
            # print(round(self.robot.get_camera_objects()[0].get_position()[2], 2))
            for element in self.robot.get_camera_objects():
                if element.get_id() == self.target_object.get_id():
                    # if not self.robot_collision():
                    if element.get_position()[0] < -0.1 and abs(element.get_position()[2]) > 1 or \
                            element.get_position()[0] < -0.02 and abs(element.get_position()[2]) < 1:
                        # if element.get_position()[0] < -0.04:  # il target è a sinistra
                        # giro leggermente a sinistra per centrarlo
                        print("TARGET A SINISTRA")
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
                            self.ACTION = "stop"
                            self.PREVIOUS_ACTION = "stop"
                            self.grabbed_box = True
                            self.orientation_set = False
                            print("Box AFFERRATO")
                            self.robot.lift(0.0)

                    elif self.grabbed_box:
                        print(element.get_colors())
                        print("set position")

                        # giro il robot per puntare nella corretta direzione rispetto all'angolo di yaw
                        if round(self.robot.get_yaw(),
                                 1) > self.angle and not self.orientation_set and not self.balance:
                            if self.robot.get_yaw() > 0 and self.angle > 0 or self.robot.get_yaw() < 0 and self.angle < 0:
                                print("due_due")
                                self.ACTION = "go right"
                            else:
                                print("uno")
                                self.ACTION = "go left"

                        elif round(self.robot.get_yaw(),
                                   1) < self.angle and not self.orientation_set and not self.balance:
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

                                if abs(
                                        self.saved_yaw - self.robot.get_yaw()) >= 0.1 or self.final_step:  # giro di 20 gradi
                                    print("ho girato abbastanza")
                                    self.turn_reached = True

                                    if self.first_saving_gps:
                                        self.before_action_gps = self.robot.get_gps_values()
                                    self.first_saving_gps = False

                                    # se ho finito di percorrere un determinato spazio

                                    # e se posso andare avanti (controllo sensori)
                                    if (abs(self.robot.get_gps_values()[0] - self.before_action_gps[0]) >= 0.2 or \
                                        abs(self.robot.get_gps_values()[2] - self.before_action_gps[2]) >= 0.2) and \
                                            round(self.robot.get_yaw(), 1) != round(self.angle, 1) and \
                                            self.robot.get_wall_proximity(
                                                'dx') >= 1:  # cioè sono abbastanza vicino all'angolo
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

                                if abs(
                                        self.saved_yaw - self.robot.get_yaw()) >= 0.1 or self.final_step:  # giro di 20 gradi
                                    self.turn_reached = True

                                    if self.first_saving_gps:
                                        self.before_action_gps = self.robot.get_gps_values()
                                    self.first_saving_gps = False

                                    # se ho finito di percorrere un determinato spazio
                                    if abs(self.robot.get_gps_values()[0] - self.before_action_gps[0]) >= 0.2 or \
                                            abs(self.robot.get_gps_values()[2] - self.before_action_gps[2]) >= 0.2 and \
                                            round(self.robot.get_yaw(), 1) != self.angle and \
                                            self.robot.get_wall_proximity(
                                                'sx') >= 2:  # cioè sono abbastanza vicino all'angolo
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

                    # # altrimenti sono troppo vicino al master. mi devo fermare
                    # else:
                    #     print("troppo vicino al master. Mi fermo")
                    #     self.ACTION = "stop"

        # if self.NEXT_ACTION == "go straight" and self.ACTION == "go right":
        #     print("qui")
        #     # gira a destra finchè non raggiunge la posizione esatta verso destra (est)
        #     print("yaw:", abs(self.before_action_yaw - self.robot.get_yaw()))
        #     print("yaw:", abs(self.before_action_yaw - (self.robot.get_yaw())) >= 1.2)
        #
        #     if not self.check_for_box:
        #         if abs(self.before_action_yaw - (self.robot.get_yaw())) >= 1.5:
        #             self.before_action_gps = self.robot.get_gps_values()
        #             # quando raggiunge est, deve rigirare a sinistra
        #             self.wall_reached = False
        #             self.saved_yaw = False
        #             self.ACTION = "go straight"
        #             self.NEXT_ACTION = "go left"
        #     # se devo cercare un nuovo box, allora smetto di girare quando ne trovo uno
        #     else:
        #         self.wall_reached = False
        #         self.saved_yaw = False
        #         # l'azione rimane la stessa
        #         if self.target_object is not None:
        #             self.ACTION = "go straight"
        #
        # if self.NEXT_ACTION == "go left" and self.ACTION == "go straight":
        #     if abs(self.before_action_gps[0] - self.robot.get_gps_values()[0]) > 0.5 or \
        #             abs(self.before_action_gps[1] - self.robot.get_gps_values()[1]) > 0.5 or \
        #             abs(self.before_action_gps[2] - self.robot.get_gps_values()[2]) > 0.5:
        #         self.wall_reached = False
        #         print("TURN LEFT")
        #         self.ACTION = "go left"
        #         self.NEXT_ACTION = "go straight"
        #
        # if self.NEXT_ACTION == "go straight" and self.ACTION == "go left":
        #     print("yaw:", abs(self.before_action_yaw - (self.robot.get_yaw())))
        #     if not self.check_for_box:
        #         if abs(self.before_action_yaw - (self.robot.get_yaw()) >= 1.5):
        #             self.wall_reached = False
        #             self.saved_yaw = False
        #             print("vai avanti")
        #             self.ACTION = "go straight"
        #     # se devo cercare un nuovo box, allora smetto di girare quando ne trovo uno
        #     else:
        #         self.wall_reached = False
        #         self.saved_yaw = False
        #         # l'azione rimane la stessa
        #         if self.target_object is not None:
        #             self.ACTION = "go straight"

        if self.ACTION == "go back": self.robot.go_back()
        if self.ACTION == "go left": self.robot.go_left()
        if self.ACTION == "go straight": self.robot.go_straight()
        if self.ACTION == "go right": self.robot.go_right()
        if self.ACTION == "stop": self.robot.stop()
        if self.ACTION == "slow down": self.robot.slow_down()

        if self.target_object is not None: self.check_for_box = False
        if self.check_for_box:
            self.box_placed = False
            print("check for box")

        if self.last_box: self.back_after_last_box()

        # robot_collision = self.robot_collision()
        # if not robot_collision:
        self.check_wall_reached()
        if self.stop_time is not None:
            if self.ACTION == "stop" and self.stopped and (
                    current_milli_time() - self.stop_time >= 1000) and not self.last_box:
                # if self.ACTION == "stop" and self.stopped and (current_milli_time() - self.stop_time >= 1000):
                print("vado indietro")
                self.ACTION = "go back"
        if self.wall_reached and self.grabbed_box: self.turn_to_destination()
        # if self.wall_reached and self.target_object is not None: self.turn_to_target()
        if self.wall_reached and not self.grabbed_box and not self.last_box: self.turn_after_wall_reached()

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

    def turn_to_target(self):
        print("turn to target")
        if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.5 or \
                abs(self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.5:
            self.stopped = False

            print("ho percorso abbastanza spazio")
            if self.robot.get_sensor_value(7) < 900:
                self.ACTION = "turn right"
            else:
                self.ACTION = "turn left"

    # Enter here exit cleanup code.

    def read_placed_spheres(self):
        now = time.time()
        timeout = now + 0.1
        while now < timeout:
            message = self._pubsub_spheres.get_message(ignore_subscribe_messages=True)
            if message is not None:
                print("PRINT PLACED SPHERES:")
                print(message)
                data = message['data']
                data = json.loads(data)
                print("DATI")
                print(data)
                self.boxes_ids.append(data)
            else:
                break

    def read_current_sphere_master(self):
        now = time.time()
        timeout = now + 0.1
        while now < timeout:
            message = self._pubsub_current_sphere.get_message(ignore_subscribe_messages=True)
            if message is not None:
                print("PRINT CURRENT MASTER SPHERE:")
                print(message)
                data = message['data']
                data = json.loads(data)
                self.current_sphere_other_robot = data
            else:
                break

    def read_current_gps_other_robot(self):
        now = time.time()
        timeout = now + 0.1
        while now < timeout:
            message = self._pubsub_current_gps.get_message(ignore_subscribe_messages=True)
            if message is not None:
                print("PRINT CURRENT MASTER GPS:")
                print(message)
                data = message['data']
                data = json.loads(data)
                self.current_coordinates_other_robot = data
                print("coordinate altro robot:", self.current_coordinates_other_robot)
            else:
                break

    def write_placed_spheres(self, id_placed_sphere):
        print("scrivo i box posizionati")
        self._myR.publish(self._ch_out_spheres, json.dumps(id_placed_sphere))

    def write_current_sphere(self):
        print("scrivo il target object")
        if self.target_object is not None:
            self._myR.publish(self._ch_out_current_sphere, json.dumps([self.target_object.get_id(), self.color]))

    def write_current_gps(self):
        self._myR.publish(self._ch_out_current_gps, json.dumps(self.robot.get_gps_values()))


if __name__ == "__main__":

    brain = Brain("CH_placed_spheres_M2S", "CH_placed_spheres_S2M", "CH_current_sphere_M2S", "CH_current_sphere_S2M",
                  "CH_current_coord_M2S", "CH_current_coord_S2M")
    while brain.get_robot().step(TIME_STEP) != -1:
        brain.read_placed_spheres()
        brain.read_current_sphere_master()
        brain.read_current_gps_other_robot()
        brain.controller()
        # brain.write_placed_spheres()
        brain.write_current_sphere()
        # brain.write_current_gps()    # lo slave non deve scrivere, deve adeguarsi al master
