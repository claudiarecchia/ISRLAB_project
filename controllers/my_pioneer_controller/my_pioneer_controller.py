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

yellow_corner = [-2.9, -0.2, -3.2]  # -3.3 z (anche red)
red_corner = [0.9, -0.2, -3.3]  # 1.19 x
green_corner = [-2.9, -0.2, 0.7]  #0.86 z
blue_corner = [1.1, -0.2, 0.7]

yellow_yaw = 0.4
red_yaw = -0.5
green_yaw = 2.5
blue_yaw = -2.5

yellow = [1.0, 1.0, 0.0]
red = [1.0, 0.0, 0.0]
green = [0.0, 1.0, 0.0]
blue = [0.0, 0.0, 1.0]

# orientamenti NORD, EST, OVERS, SUD
ORIENTAMENTI_left = [-0.1, 1.5, -1.7, -3.1]
ORIENTAMENTI_right = [0.1, 1.7, -1.4, -2.9]


def current_milli_time():
    return round(time.time() * 1000)


class Brain:

    def __init__(self, channel_in_spheres, channel_out_spheres, channel_in_current_sphere, channel_out_current_sphere, ch_in_current_gps, ch_out_current_gps):
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

        #pubsub to in channels
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

        self.wall_reached = False
        self.check_for_box = False
        self.grabbed_sphere = False
        self.box_placed = False
        self.boxes_ids = []
        self.total_boxes = 5
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
        self.left = False
        self.turn_reached = False
        self.first_saving_yaw = True
        self.final_step = False
        self.right = False
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
        self.reaching_destination = False
        self.orientation_reached = False
        self.turn_time = None
        self.near_the_spot = False
        self.touching_other_robot = False
        self.last_color_placed = None
        self.last_box_other_robot = False

    def get_robot(self):
        return self.robot.get_robot()

    def check_wall_behind_if_back(self):
        if self.robot.wall('back') and self.ACTION == "go back": return True
        else: return False

    def check_collision_other_robot(self):
        if round(self.robot.get_gps_values()[0], 0) == round(self.current_coordinates_other_robot[0]) and \
                round(self.robot.get_gps_values()[2], 0) == round(self.current_coordinates_other_robot[2]):
            return True
        else: return False

    def reaching_sphere(self):
        if not self.grabbed_sphere and self.target_object is not None:
            return True
        else: return False

    def reach_angle(self):
        print("reach angle")

        if self.robot.too_close_wall():
            print("front wall")
            self.stopped = False
            self.stop_time = current_milli_time()
            self.near_the_spot = False

        if round(self.robot.get_yaw(), 1) > self.angle:
            if (self.robot.get_yaw() > 0 and self.angle > 0) or (self.robot.get_yaw() < 0 and self.angle < 0):
                print("due_due")
                self.ACTION = "go right"
            elif self.robot.get_yaw() > 0 and self.angle < 0 and self.robot.get_gps_values()[2] > 0:
                print("uno")
                self.ACTION = "go left"
            elif self.robot.get_yaw() > 0 and self.angle < 0 and self.robot.get_gps_values()[2] < 0:
                print("tre_")
                self.ACTION = "go right"
        elif round(self.robot.get_yaw(), 1) < self.angle:
            if (self.robot.get_yaw() > 0 and self.angle > 0) or (self.robot.get_yaw() < 0 and self.angle < 0):
                print("due_uno")
                self.ACTION = "go left"
            elif self.robot.get_yaw() < 0 and self.angle > 0 and self.robot.get_gps_values()[2] > 0:
                print("due_due")
                self.ACTION = "go right"
            elif self.robot.get_yaw() < 0 and self.angle > 0 and self.robot.get_gps_values()[2] < 0:
                print("tre_")
                self.ACTION = "go left"

        else:
            print("not near the spot")
            self.near_the_spot = False
            self.stopped = False
            return

    def check_wall_reached(self):
        print("check wall reached; num sensors: ", self.robot.get_number_wall_sensors())

        if self.robot.get_number_wall_sensors() >= 2 or self.check_wall_behind_if_back() or self.robot.wall('front') \
                or (self.grabbed_sphere and self.robot.check_near_wall_front("any")):
            print(self.robot.get_number_wall_sensors())

            if not self.stopped:
                self.wall_reached = True
                print("mi fermo")
                self.ACTION = "stop"
                self.stopped = True
                self.stop_time = current_milli_time()

            print("WALL REACHED_")
            if not self.reaching_destination:
                self.wall_reached_gps = self.robot.get_gps_values()
            if self.grabbed_sphere:
                if self.check_if_place_reached():
                    print("place reached")
                    if self.robot.get_yaw() != self.angle and self.stopped:
                        print("angolo diverso")
                        self.reach_angle()
                        if not self.stopped:
                            print("devo lasciare il box")
                            self.leave_box()



            return self.wall_reached
        else:
            return False

    def turn_to_destination(self):
        print("turn to destination")
        # se i sensori misurano valori maggiori a destra allora devo costeggiare la destra
        # altrimenti la sinistra

        # se l'orientamento è quello dell'angolo dove portare il box
        if self.orientation_set and not self.check_collision_other_robot() and not self.touching_other_robot:
            self.reaching_destination = True
            if self.robot.get_number_wall_sensors('sx') > self.robot.get_number_wall_sensors('dx') and not self.orientation_reached:
                # finchè non arrivo ad avere un orientamento orizzontale o verticale per raggiungere la destinazione
                self.left = True
                if not (ORIENTAMENTI_left[0] < round(self.robot.get_yaw(), 1) < ORIENTAMENTI_right[0] or \
                        ORIENTAMENTI_left[1] < round(self.robot.get_yaw(), 1) < ORIENTAMENTI_right[1] or \
                        ORIENTAMENTI_left[2] < round(self.robot.get_yaw(), 1) < ORIENTAMENTI_right[2] or \
                        ORIENTAMENTI_left[3] < round(self.robot.get_yaw(), 1) < ORIENTAMENTI_right[3]):
                    print("costeggio parete a sinistra")
                    self.ACTION = "go right"
                else:
                    print("QUI")
                    self.orientation_reached = True
                    self.ACTION = "go straight"
            elif self.robot.get_number_wall_sensors('dx') > self.robot.get_number_wall_sensors('sx') and not self.orientation_reached:
                # finchè non arrivo ad avere un orientamento orizzontale o verticale per raggiungere la destinazione
                self.right = True
                if not (ORIENTAMENTI_left[0] < round(self.robot.get_yaw(), 1) < ORIENTAMENTI_right[0] or \
                        ORIENTAMENTI_left[1] < round(self.robot.get_yaw(), 1) < ORIENTAMENTI_right[1] or \
                        ORIENTAMENTI_left[2] < round(self.robot.get_yaw(), 1) < ORIENTAMENTI_right[2] or \
                        ORIENTAMENTI_left[3] < round(self.robot.get_yaw(), 1) < ORIENTAMENTI_right[3]):
                    print("costeggio parete a destra")
                    self.ACTION = "go left"
                else:
                    self.orientation_reached = True
                    self.ACTION = "go straight"
            else:
                print("else")
                if self.orientation_reached:
                    # ho raggiunto la posizione corretta per raggiungere l'angolo
                    self.ACTION = "go straight"

        # altrimenti significa che ho sbattuto all'altro robot
        # ho in mano la sfera ma non ho ancora raggiunto l'angolo corretto
        else:
            self.touching_other_robot = True
            # self.wall_reached = False  # devo continuare ad eseguire il codice che mi porta ad arrivare al corretto orientamento
            print("else - turn to dest")
            self.turn_after_wall_reached()

    def turn_after_wall_reached(self):
        print("turn after wall reached")
        # controllo se mentre vado indietro arrivo a una parete

        if not self.robot.wall('back'):
            # vado indietro fino a raggiungere un determinato spazio percorso
            if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.3 or \
                    abs(self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.3:
                self.turn_right_or_left()
                self.touching_other_robot = False  # dopo aver percorso un certo spazio, non sto più toccando l'altro robot se ci avevo sbattuto

        # se mentre vado indietro sono arrivato a una parete (anche dietro) mi fermo prima
        else:
            self.first_save_stop_time = True  # per la prossima volta che arriverò a una parete
            self.touching_other_robot = False
            print("parete raggiunta")
            self.turn_right_or_left()

    def turn_right_or_left(self):
        print("turn right or left")
        if not (self.robot.get_number_wall_sensors() >= 2 or self.check_wall_behind_if_back() or self.robot.wall('front')):
            self.stopped = False
        if not self.check_for_box:
            # devo riorientare il robot verso l'angolo corretto
            self.wall_reached = False
            return

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
        # vado indietro di un po' e poi termino l'esecuzione
        if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.3 or \
                abs(self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.3:
            if len(self.boxes_ids) == self.total_boxes:
                # exit()
                print("ultima sfera. mi fermo")
                self.ACTION = "stop"
                # self.terminate_execution()
            else:
                self.ACTION = "stop"

    def terminate_execution(self):
        exit()

    def check_for_other_box(self):
        global before_action_gps, box_placed, turned
        box_placed = False
        print("check for box")

    def check_if_place_reached(self):
        if round(self.robot.get_gps_values()[0], 1) - round(self.destination[0], 1) == 0 and \
                round(self.robot.get_gps_values()[2], 0) - round(self.destination[2], 0) == 0 \
            or round(self.robot.get_gps_values()[0], 0) - round(self.destination[0], 0) == 0 and \
                round(self.robot.get_gps_values()[2], 1) - round(self.destination[2], 1) == 0:
            if self.robot.check_near_wall_front("num") >= 2:  # se almeno due sensori superano un determinato valore
                if abs(self.robot.get_gps_values()[0] - self.destination[0]) <= 0.3:  # verifico ulteriormente la vicinanza all'angolo
                    print("near the spot")
                    self.near_the_spot = True
        return self.near_the_spot

    def leave_box(self):
        self.wall_reached_gps = self.robot.get_gps_values()  # sovrascrivo l'informazione con i dati in cui poggio il box
        print("-- arrivato")
        self.ACTION = "stop"
        self.robot.move_fingers(0.1)  # apro le braccia per lasciare il cubo
        self.robot.lift(0.05)  # e le abbasso
        self.destination = None
        self.box_placed = True
        self.grabbed_sphere = False
        self.first_saving = True
        self.time_left_box = current_milli_time()
        objects = self.robot.get_camera_objects()

        for obj in objects:
            if obj.get_id() == self.target_object.get_id():
                self.boxes_ids.append(obj.get_id())
                self.last_color_placed = obj.get_colors()
                self.write_placed_spheres(obj.get_id())

        if len(self.boxes_ids) == self.total_boxes or self.check_if_last_box_other_robot():
            print("raggiunto")
            if self.last_box_other_robot: self.last_box = False
            elif not self.last_box_other_robot: self.last_box = True
            # self.last_box = True
            self.check_for_box = False
            self.target_object = None

        elif len(self.boxes_ids) < self.total_boxes:
            self.target_object = None
            self.check_for_box = True
            # devo prima andare indietro e poi girare a sinistra
            # indica che non ho ancora girato
            self.turned = False

            self.before_action_gps = self.robot.get_gps_values()

            if self.left or self.right:
                print("reimposto variabili")
                self.first_saving_yaw = True
                self.first_saving_gps = True
                self.left = False
                self.turn_reached = False
                self.balance = False
                self.final_step = False
                self.right = False

                self.orientation_reached = False
                self.reaching_destination = False

    def check_if_last_box_other_robot(self):
        if len(self.boxes_ids) == self.total_boxes - 1 and self.current_sphere_other_robot[0] not in self.boxes_ids:
            self.last_box_other_robot = True
            self.destination = [-1.1, 0.0, -1.1]  # imposto luogo destinazione da raggiungere (centro della stanza)
        else:
            self.last_box_other_robot = False
        return self.last_box_other_robot

    def not_same_color_other_robot_unless_last_sphere(self, el):
        # controllo che il colore della sfera visualizzata sia diverso dal colore della sfera che sta posizionando l'altro robot
        # in modo da non collidere in fase di posizionamento della sfera
        # altrimenti, se si tratta dell'ultima sfera da posizionare, non tengo conto del colore
        response = False
        print(self.current_sphere_other_robot[0], self.current_sphere_other_robot[1])
        print(len(self.boxes_ids), self.total_boxes -1 )
        if el.get_colors() != self.current_sphere_other_robot[1] and len(self.boxes_ids) < self.total_boxes - 1:
            print("uno")
            response = True
        elif len(self.boxes_ids) == self.total_boxes - 1:
            print("due")
            response = True
        return response

    def stop_or_move_robot(self):
        print("stop or move robot")
        if self.last_color_placed != self.current_sphere_other_robot[1]:  # se i colori sono diversi, posso rimanere nella posizione in cui mi trovo
            self.back_after_last_box()
        else:  # se invece i colori sono uguali mi devo spostare per far passare l'altro robot
            # vado verso il centro
            if 0 < round(self.robot.get_yaw(), 1) < 1.6:
                angle = blue_yaw
            elif 1.6 < round(self.robot.get_yaw(), 1) < 3.1:
                angle = red_yaw
            elif -3.1 < round(self.robot.get_yaw(), 1) < -1.6:
                angle = yellow_yaw
            elif -1.6 < round(self.robot.get_yaw(), 1) < 0:
                angle = green_yaw

            if round(self.robot.get_yaw(), 1) != angle:
                print("giro per mettermi in direzione del centro")
                self.ACTION = "go right"
            else:  # ho girato abbastanza
                self.ACTION = "go straight"
                print("vado dritto")
                # quando mi avvicino al centro mi fermo
                self.check_if_place_reached()
                if self.near_the_spot:
                    print("arrivato al centro")
                    self.ACTION = "stop"

    def placed_all_spheres(self):
        if self.last_box or self.last_box_other_robot: return True
        else: return False

    # Main loop:
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

        if self.target_object is None and self.robot.get_camera_number_objects() >= 1 and not self.placed_all_spheres():
            # verifico che il target obj non sia già presente nella lista degli oggetti correttamente posizionati
            print("boxes ids: ")
            for element in self.boxes_ids:
                print("element in list:", element)
            for el in self.robot.get_camera_objects():
                print(el.get_id(), " ")
                print(el.get_id(), "diverso dall'altro robot", el.get_id() != self.current_sphere_other_robot[0])
                print(self.not_same_color_other_robot_unless_last_sphere(el))
                if el.get_id() not in self.boxes_ids and el.get_id() != self.current_sphere_other_robot[0] and \
                        self.not_same_color_other_robot_unless_last_sphere(el):
                    if "slave" in self.robot.get_robot().getName():
                        if self.current_sphere_other_robot[0] is not None:
                            print("id: ", el.get_id())
                            self.target_object = el
                            self.modified_angle = 0  # inizializzo il numero di volte in cui si è modificato l'angolo del nuovo oggetto
                            self.near_the_spot = False  # inizializzazione variabile "vicinanza all'angolo"
                            self.time_left_box = None

                    else:
                        print("id: ", el.get_id())
                        self.target_object = el
                        self.modified_angle = 0  # inizializzo il numero di volte in cui si è modificato l'angolo del nuovo oggetto
                        self.near_the_spot = False  # inizializzazione variabile "vicinanza all'angolo"
                        self.time_left_box = None

                    if el.get_colors() == yellow:  # giallo
                        self.destination = yellow_corner
                        self.backup_angle = yellow_yaw
                        self.angle = yellow_yaw
                        self.color = yellow
                    elif el.get_colors() == red:  # rosso
                        self.destination = red_corner
                        self.backup_angle = red_yaw
                        self.angle = red_yaw
                        self.color = red
                    elif el.get_colors() == green:  # verde
                        self.destination = green_corner
                        self.backup_angle = green_yaw
                        self.angle = green_yaw
                        self.color = green
                    elif el.get_colors() == blue:  # blu
                        self.destination = blue_corner
                        self.backup_angle = blue_yaw
                        self.angle = blue_yaw
                        self.color = blue

                if len(self.boxes_ids) == self.total_boxes - 1 and el.get_id() not in self.boxes_ids and el.get_colors() == self.current_sphere_other_robot[1]:
                    self.ACTION = "go back"
                    self.last_box_other_robot = True
                    self.check_for_box = False
                    self.destination = [-1.1, 0.0, -1.1]

        if self.target_object is not None and not self.wall_reached and self.robot.get_camera_number_objects() > 0:
            self.ACTION, self.NEXT_ACTION = "", ""

            for element in self.robot.get_camera_objects():
                if element.get_id() == self.target_object.get_id():
                    if element.get_position()[0] < -0.1 and abs(element.get_position()[2]) > 1 or \
                            element.get_position()[0] < -0.02 and abs(element.get_position()[2]) < 1:
                        print("TARGET A SINISTRA")
                        self.ACTION = "go left"
                    elif element.get_position()[0] > 0.1 and abs(element.get_position()[2]) > 1 or \
                            element.get_position()[0] > 0.02 and abs(element.get_position()[2]) < 1:
                        # giro leggermente a destra per centrarlo
                        print("TARGET A DESTRA")
                        self.ACTION = "go right"

                    # se mi avvicino a un box rallento
                    elif not self.grabbed_sphere and not self.check_for_box and \
                            round(element.get_position()[2],
                                  2) >= -0.30:  # asse z => più è grande, più mi sto avvicinando alla sfera
                        self.robot.move_fingers(0.03)
                        self.ACTION = "slow down"
                        if self.first_saving:
                            self.saved_millis = current_milli_time()
                            self.first_saving = False

                        if self.first_saving is not None and current_milli_time() - self.saved_millis >= 1 * 1000:
                            self.ACTION = "stop"
                            self.grabbed_sphere = True
                            self.orientation_set = False
                            print("Box AFFERRATO")
                            self.robot.lift(0.0)

                    elif self.grabbed_sphere:
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

                        # ho raggiunto l'orientamento corretto per raggiungere l'angolo
                        # ora quando raggiungerò una parete dovrò seguirla per raggiungere l'angolo
                        else:
                            if not self.reaching_destination:
                                print("tre")
                                self.orientation_set = True
                                self.ACTION = "go straight"

                    else:
                        print("DRITTO")
                        self.ACTION = "go straight"

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

        # if self.check_if_last_box_other_robot():
        #     print("last box other robot")
        #     print("stop time:", self.stop_time)
        #     # self.ACTION = "go back"
        #     self.last_box_other_robot = True
        #     self.check_for_box = False
        #     self.destination = [-1.1, 0.0, -1.1]

        # if self.last_box_other_robot: self.stop_or_move_robot()
        # if self.last_box: self.back_after_last_box()

        if not self.reaching_sphere() and not self.last_box:
            self.check_wall_reached()

        if self.stop_time is not None:
            print("stop time not none")
            if self.ACTION == "stop" and self.stopped and not self.last_box and not self.near_the_spot:
                if not self.touching_other_robot:
                    if current_milli_time() - self.stop_time >= 1000:
                        print("vado indietro")
                        self.ACTION = "go back"
                # se sto toccando l'altro robot, aspetto di meno per andare indietro
                else:
                    if current_milli_time() - self.stop_time >= 500:
                        print("vado indietro")
                        self.ACTION = "go back"
            if self.ACTION == "stop" and self.last_box:
                print("a")
                if current_milli_time() - self.stop_time >= 1000:
                    print("vado indietro")
                    self.ACTION = "go back"
                    self.stop_time = None
                    # self.back_after_last_box()
            if self.ACTION == "stop" and self.last_box_other_robot:
                print("b")
                if current_milli_time() - self.stop_time >= 1000:
                    print("vado indietro")
                    self.ACTION = "go back"
                    self.stop_time = None
                    # self.stop_or_move_robot()

        if self.ACTION == "go back" and self.last_box:
            print("BACK")
            self.back_after_last_box()
        if self.ACTION == "go back" and self.last_box_other_robot:
            print("STOP OR MOVE")
            self.stop_or_move_robot()

        print("near:", self.near_the_spot)
        if self.wall_reached and self.grabbed_sphere and not self.check_if_place_reached(): self.turn_to_destination()
        if self.wall_reached and self.target_object is not None and not self.grabbed_sphere: self.turn_to_target()
        if self.wall_reached and not self.grabbed_sphere and not self.last_box: self.turn_after_wall_reached()

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

    def turn_to_target(self):
        print("turn to target")
        if abs(self.wall_reached_gps[0] - self.robot.get_gps_values()[0]) > 0.3 or \
                abs(self.wall_reached_gps[2] - self.robot.get_gps_values()[2]) > 0.3:
            self.stopped = False

            print("ho percorso abbastanza spazio")
            if self.robot.get_sensor_value(7) < 900:
                self.ACTION = "turn right"
            else:
                self.ACTION = "turn left"

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
                self.boxes_ids.append(data)
            else:
                break

    def read_current_sphere_other_robot(self):
        now = time.time()
        timeout = now + 0.1
        while now < timeout:
            message = self._pubsub_current_sphere.get_message(ignore_subscribe_messages=True)
            if message is not None:
                print("PRINT CURRENT SLAVE SPHERE:")
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
                # print("PRINT CURRENT MASTER GPS:")
                # print(message)
                data = message['data']
                data = json.loads(data)
                self.current_coordinates_other_robot = data
            else:
                break

    def write_placed_spheres(self, id_placed_sphere):
        self._myR.publish(self._ch_out_spheres, json.dumps(id_placed_sphere))

    def write_current_sphere(self):
        if self.target_object is not None:
            self._myR.publish(self._ch_out_current_sphere, json.dumps([self.target_object.get_id(), self.color]))

    def write_current_gps(self):
        self._myR.publish(self._ch_out_current_gps, json.dumps(self.robot.get_gps_values()))


if __name__ == "__main__":
    brain_master = Brain("CH_placed_spheres_S2M", "CH_placed_spheres_M2S", "CH_current_sphere_S2M", "CH_current_sphere_M2S", "CH_current_coord_S2M", "CH_current_coord_M2S")

    while brain_master.get_robot().step(TIME_STEP) != -1:
        brain_master.read_placed_spheres()
        brain_master.read_current_sphere_other_robot()
        brain_master.read_current_gps_other_robot()
        brain_master.controller()
        brain_master.write_current_sphere()
        brain_master.write_current_gps()
