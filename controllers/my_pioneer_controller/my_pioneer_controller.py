"""my_pioneer_controller controller."""

"""
    BRAIN
    Controllore "intelligente" del robot disaccoppiato dal corpo 
    Leggere l'array dei sensori
    Scrivere l'azione definita dall'algoritmo di controllo
    Paradigma : SENSE-PLAN-ACT
"""
from math import sqrt
from controllers.my_pioneer_controller.RobotBody import Body
from controllers.my_pioneer_controller.global_variables import *
import time
import redis
import json

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

        self.wall_reached = False
        self.check_for_box = False
        self.grabbed_sphere = False
        self.boxes_ids = []
        self.wall_reached_gps = []
        self.orientation_set = False
        self.saved_millis = None
        self.first_saving = True
        self.balance = False
        self.left = False
        self.right = False
        self.destination = None
        self.angle = None
        self.last_box = False
        self.stop_time = None
        self.stopped = False
        self.color = None
        self.reaching_destination = False
        self.orientation_reached = False
        self.near_the_spot = False
        self.touching_other_robot = False
        self.last_color_placed = None
        self.last_box_other_robot = False
        self.recognized_objects = []
        self.stopped_in_the_middle = False
        self.nearest_objects_to_place = []
        self.nearest_objects_to_place = []

        # SENSE
        self.gps = None
        self.yaw = None
        self.sensors = None
        self.objects = []
        self.number_objects = None

    def get_robot(self):
        return self.robot.get_robot()

    def check_wall_behind_if_back(self):
        if self.wall('back') and self.ACTION == "go back": return True
        else: return False

    def check_collision_other_robot(self):
        if round(self.gps[0], 0) == round(self.current_coordinates_other_robot[0]) and \
                round(self.gps[2], 0) == round(self.current_coordinates_other_robot[2]):
            return True
        else: return False

    def reaching_sphere(self):
        if not self.grabbed_sphere and self.target_object is not None:
            return True
        else: return False

    def reach_angle(self):
        # print("reach angle")
        if self.too_close_wall():
            # print("front wall")
            self.stopped = False
            self.near_the_spot = False

        if round(self.yaw, 1) > self.angle:
            if (self.yaw > 0 and self.angle > 0) or (self.yaw < 0 and self.angle < 0):
                self.ACTION = "go right"
            elif self.yaw > 0 and self.angle < 0 and self.gps[2] > 0:
                self.ACTION = "go left"
            elif self.yaw > 0 and self.angle < 0 and self.gps[2] < 0:
                self.ACTION = "go right"
        elif round(self.yaw, 1) < self.angle:
            if (self.yaw > 0 and self.angle > 0) or (self.yaw < 0 and self.angle < 0):
                self.ACTION = "go left"
            elif self.yaw < 0 and self.angle > 0 and self.gps[2] > 0:
                self.ACTION = "go right"
            elif self.yaw < 0 and self.angle > 0 and self.gps[2] < 0:
                self.ACTION = "go left"
        else:
            # print("not near the spot")
            self.near_the_spot = False
            self.stopped = False
            return

    def check_wall_reached(self):
        if self.get_number_wall_sensors() >= 2 or self.check_wall_behind_if_back() or self.wall('front') \
                or (self.grabbed_sphere and self.check_near_wall_front("any")):
            # print("WALL REACHED")
            if not self.reaching_destination:
                self.wall_reached_gps = self.gps
            if self.grabbed_sphere:
                if self.check_if_place_reached():
                    if self.yaw != self.angle and self.stopped:
                        self.reach_angle()
                        if not self.stopped:
                            self.leave_box()

            if not self.stopped:
                self.wall_reached = True
                self.ACTION = "stop"
                self.stopped = True
                self.stop_time = current_milli_time()
            return self.wall_reached
        else:
            return False

    def turn_to_destination(self):
        # se l'orientamento è quello dell'angolo dove portare il box
        if self.orientation_set and not self.check_collision_other_robot() and not self.touching_other_robot:
            self.reaching_destination = True
            if self.get_number_wall_sensors('sx') > self.get_number_wall_sensors('dx') and not self.orientation_reached:
                # finchè non arrivo ad avere un orientamento orizzontale o verticale per raggiungere la destinazione
                self.left = True
                if not (ORIENTAMENTI_left[0] < round(self.yaw, 1) < ORIENTAMENTI_right[0] or
                        ORIENTAMENTI_left[1] < round(self.yaw, 1) < ORIENTAMENTI_right[1] or
                        ORIENTAMENTI_left[2] < round(self.yaw, 1) < ORIENTAMENTI_right[2] or
                        ORIENTAMENTI_left[3] < round(self.yaw, 1) < ORIENTAMENTI_right[3]):
                    # print("costeggio parete a sinistra")
                    self.ACTION = "go right"
                else:
                    self.orientation_reached = True
                    self.ACTION = "go straight"
            elif self.get_number_wall_sensors('dx') > self.get_number_wall_sensors('sx') and not self.orientation_reached:
                # finchè non arrivo ad avere un orientamento orizzontale o verticale per raggiungere la destinazione
                self.right = True
                if not (ORIENTAMENTI_left[0] < round(self.yaw, 1) < ORIENTAMENTI_right[0] or
                        ORIENTAMENTI_left[1] < round(self.yaw, 1) < ORIENTAMENTI_right[1] or
                        ORIENTAMENTI_left[2] < round(self.yaw, 1) < ORIENTAMENTI_right[2] or
                        ORIENTAMENTI_left[3] < round(self.yaw, 1) < ORIENTAMENTI_right[3]):
                    # print("costeggio parete a destra")
                    self.ACTION = "go left"
                else:
                    self.orientation_reached = True
                    self.ACTION = "go straight"
            else:
                if self.orientation_reached:
                    # ho raggiunto la posizione corretta per raggiungere l'angolo
                    self.ACTION = "go straight"

        # altrimenti significa che ho sbattuto all'altro robot
        # ho in mano la sfera ma non ho ancora raggiunto l'angolo corretto
        else:
            self.touching_other_robot = True
            self.turn_after_wall_reached()

    def turn_after_wall_reached(self):
        print("turn after wall reached")
        # controllo se mentre vado indietro arrivo a una parete
        if not self.wall('back'):
            # vado indietro fino a raggiungere un determinato spazio percorso
            if abs(self.wall_reached_gps[0] - self.gps[0]) > 0.3 or \
                    abs(self.wall_reached_gps[2] - self.gps[2]) > 0.3:
                self.turn_right_or_left()
                self.touching_other_robot = False  # dopo aver percorso un certo spazio, non sto più toccando l'altro robot se ci avevo sbattuto

        # se mentre vado indietro sono arrivato a una parete (anche dietro) mi fermo prima
        else:
            self.touching_other_robot = False
            self.turn_right_or_left()

    def turn_right_or_left(self):
        if not (self.get_number_wall_sensors() >= 2 or self.check_wall_behind_if_back() or self.wall('front')):
            self.stopped = False
        if not self.check_for_box:
            # devo riorientare il robot verso l'angolo corretto
            self.wall_reached = False
            return
        # se sto cercando un nuovo box, giro finchè non ne trovo uno nella camera
        else:
            # se il valore misurato sui sensori a sinistra è maggiore di quello misurato a destra
            # allora giro a destra
            self.wall_reached = False
            self.choose_right_or_left()

    def choose_right_or_left(self):
        if self.sensors[0] + self.sensors[1] > self.sensors[6] + self.sensors[7]:
            self.ACTION = "go right"
        else:
            self.ACTION = "go left"

    def back_after_last_box(self):
        # vado indietro di un po' e poi termino l'esecuzione
        if abs(self.wall_reached_gps[0] - self.gps[0]) > 0.3 or \
                abs(self.wall_reached_gps[2] - self.gps[2]) > 0.3:
            if len(self.boxes_ids) == total_boxes:
                print("ultima sfera. mi fermo")
                self.ACTION = "stop"
            else:
                self.ACTION = "stop"

    def check_if_place_reached(self):
        if round(self.gps[0], 1) - round(self.destination[0], 1) == 0 and \
                round(self.gps[2], 0) - round(self.destination[2], 0) == 0 \
            or round(self.gps[0], 0) - round(self.destination[0], 0) == 0 and \
                round(self.gps[2], 1) - round(self.destination[2], 1) == 0:
            if self.check_near_wall_front("num") >= 2:  # se almeno due sensori superano un determinato valore
                if abs(self.gps[0] - self.destination[0]) <= 0.4:  # verifico ulteriormente la vicinanza all'angolo
                    self.near_the_spot = True
        return self.near_the_spot

    def leave_box(self):
        self.wall_reached_gps = self.gps  # sovrascrivo l'informazione con i dati in cui poggio il box
        self.ACTION = "stop"
        self.robot.move_fingers(0.1)  # apro le braccia per lasciare il cubo
        self.robot.lift(0.05)  # e le abbasso
        self.destination = None
        self.grabbed_sphere = False
        self.first_saving = True
        objects = self.objects

        for obj in objects:
            if obj.get_id() == self.target_object.get_id():
                self.boxes_ids.append(obj.get_id())
                self.last_color_placed = obj.get_colors()
                self.write_placed_spheres(obj.get_id())

        if len(self.boxes_ids) == total_boxes or self.check_if_last_box_other_robot():
            if self.last_box_other_robot: self.last_box = False
            elif not self.last_box_other_robot: self.last_box = True
            self.check_for_box = False
            self.target_object = None

        elif len(self.boxes_ids) < total_boxes:
            self.target_object = None
            self.check_for_box = True

            if self.left or self.right:
                self.left = False
                self.balance = False
                self.right = False
                self.orientation_reached = False
                self.reaching_destination = False

    def check_if_last_box_other_robot(self):
        if len(self.boxes_ids) == total_boxes - 1 and self.current_sphere_other_robot[0] not in self.boxes_ids:
            self.last_box_other_robot = True
        else:
            self.last_box_other_robot = False
        return self.last_box_other_robot

    def not_same_color_other_robot(self, el):
        # controllo che il colore della sfera visualizzata sia diverso dal colore della sfera che sta posizionando l'altro robot
        # in modo da non collidere in fase di posizionamento della sfera
        response = False
        if el.get_colors() != self.current_sphere_other_robot[1]:
            response = True
        return response

    def stop_or_move_robot(self):
        if self.last_color_placed != self.current_sphere_other_robot[1]:  # se i colori sono diversi, posso rimanere nella posizione in cui mi trovo
            self.back_after_last_box()

    def placed_all_spheres(self):
        if self.last_box or self.last_box_other_robot: return True
        else: return False

    def recognized_all_objects(self):
        # ritorna un valore positivo se ho visto tutte le sfere o se me ne manca una
        # e quella che mi manca è quella che ha in mano l'altro robot (che può essere girato dall'altra parte
        # e quindi fuori dalla mia visuale)
        response = False
        if len(self.recognized_objects) == total_boxes: response = True
        if len(self.recognized_objects) == total_boxes - 1 and self.current_sphere_other_robot[0] not in self.recognized_objects: response = True
        return response

    def controller(self):
        # informazioni (id, colore) di tutti gli oggetti che incontro
        if self.number_objects > 0:
            for obj in self.objects:
                if [obj.get_id(), obj.get_colors()] not in self.recognized_objects:
                    self.recognized_objects.append([obj.get_id(), obj.get_colors()])

        if self.target_object is None and self.number_objects == 0 and len(self.boxes_ids) == 0:
            self.choose_right_or_left()

        if self.target_object is None and self.recognized_all_objects() and len(self.boxes_ids) != total_boxes and not self.check_if_last_box_other_robot() and len(self.recognized_objects) == total_boxes:
            # verifico che le palline rimanenti non siano tutte dello stesso colore del target obj dell'altro robot
            # se sono tutte di quel colore, io mi fermo
            count = 0
            objects_to_be_placed = [obj for obj in self.recognized_objects if obj[0] not in self.boxes_ids]
            number = len(objects_to_be_placed)
            for obj in objects_to_be_placed:
                if obj[1] == self.current_sphere_other_robot[1]:
                   count += 1
            if count == number:
                # mi devo fermare perchè tutte le sfere che rimangono sono del colore della sfera target dell'altro robot
                # quindi sarà lui a posizionarla
                self.last_box = True
                self.stopped = True
                self.stop_time = None
                self.stopped_in_the_middle = True

        if self.target_object is None and self.number_objects >= 1 and not self.placed_all_spheres():
            self.nearest_objects_to_place = []
            # verifico che il target obj non sia già presente nella lista degli oggetti correttamente posizionati
            for el in self.objects:
                print(el.get_id(), " ")
                if el.get_id() not in self.boxes_ids and el.get_id() != self.current_sphere_other_robot[0] and \
                        self.not_same_color_other_robot(el):
                    if "slave" in self.robot.get_robot().getName():
                        if self.current_sphere_other_robot[0] is not None or (len(self.boxes_ids) == total_boxes - 1\
                                and el.get_colors() == self.last_color_placed):
                            self.near_the_spot = False  # inizializzazione variabile "vicinanza all'angolo"
                            self.nearest_objects_to_place.append(el)

                    else:
                        self.near_the_spot = False  # inizializzazione variabile "vicinanza all'angolo"
                        self.nearest_objects_to_place.append(el)

                    if len(self.boxes_ids) == total_boxes - 1 and el.get_id() not in self.boxes_ids and el.get_colors() == self.current_sphere_other_robot[1]:
                        # devo rimanere dove mi trovo
                        self.last_box_other_robot = True
                        self.check_for_box = False
                        self.ACTION = "stop"

            distances = []
            for obj in self.nearest_objects_to_place:
                distances.append(
                    [abs((obj.get_position()[0]) + (obj.get_position()[2])),
                      obj, obj.get_colors()])

            self.sort_distances(distances)
            if distances:
                element = distances[0]
                self.target_object = element[1]
                self.set_element_color(element[1])

        if self.target_object is not None and not self.wall_reached and self.number_objects > 0:
            self.ACTION = ""

            for element in self.objects:
                if element.get_id() == self.target_object.get_id():
                    if element.get_position()[0] < -0.1 and abs(element.get_position()[2]) > 1 or \
                            element.get_position()[0] < -0.02 and abs(element.get_position()[2]) < 1:
                        # print("TARGET A SINISTRA")
                        self.ACTION = "go left"
                    elif element.get_position()[0] > 0.1 and abs(element.get_position()[2]) > 1 or \
                            element.get_position()[0] > 0.02 and abs(element.get_position()[2]) < 1:
                        # giro leggermente a destra per centrarlo
                        # print("TARGET A DESTRA")
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
                            # print("Box AFFERRATO")
                            self.robot.lift(0.0)

                    elif self.grabbed_sphere:
                        # giro il robot per puntare nella corretta direzione rispetto all'angolo di yaw
                        if round(self.yaw, 1) > self.angle and not self.orientation_set and not self.balance:
                            if self.yaw > 0 and self.angle > 0 or self.yaw < 0 and self.angle < 0:
                                self.ACTION = "go right"
                            else:
                                print("uno")
                                self.ACTION = "go left"

                        elif round(self.yaw, 1) < self.angle and not self.orientation_set and not self.balance:
                            if self.yaw > 0 and self.angle > 0 or self.yaw < 0 and self.angle < 0:
                                self.ACTION = "go left"
                            else:
                                self.ACTION = "go right"

                        # ho raggiunto l'orientamento corretto per raggiungere l'angolo
                        # ora quando raggiungerò una parete dovrò seguirla per raggiungere l'angolo
                        else:
                            if not self.reaching_destination:
                                self.orientation_set = True
                                self.ACTION = "go straight"
                    else:
                        self.ACTION = "go straight"

        if self.target_object is not None: self.check_for_box = False

        if not self.reaching_sphere() and not self.last_box:
            self.check_wall_reached()

        if self.stop_time is not None:
            if self.ACTION == "stop" and self.stopped and not self.last_box and not self.near_the_spot:
                if not self.touching_other_robot:
                    if current_milli_time() - self.stop_time >= 1000:
                        self.ACTION = "go back"
                # se sto toccando l'altro robot, aspetto di meno per andare indietro
                else:
                    if current_milli_time() - self.stop_time >= 500:
                        self.ACTION = "go back"

            if self.ACTION == "stop" and self.last_box:
                if current_milli_time() - self.stop_time >= 1000:
                    self.ACTION = "go back"
                    self.stop_time = None

            if self.ACTION == "stop" and self.last_box_other_robot:
                if current_milli_time() - self.stop_time >= 1000:
                    self.ACTION = "go back"
                    self.stop_time = None

        if self.stop_time is None and self.stopped and self.last_box and self.stopped_in_the_middle:
            self.ACTION = "stop"

        if self.ACTION == "go back" and self.last_box:
            self.back_after_last_box()
        if self.ACTION == "go back" and self.last_box_other_robot:
            self.stop_or_move_robot()

        if self.wall_reached and self.grabbed_sphere and not self.check_if_place_reached(): self.turn_to_destination()
        if self.wall_reached and self.target_object is not None and not self.grabbed_sphere: self.turn_to_target()
        if self.wall_reached and not self.grabbed_sphere and not self.last_box: self.turn_after_wall_reached()

    def sort_distances(self, distances):
        for i in range(0, len(distances) - 1):
            if distances[i][0] > distances[i + 1][0]:
                copy = distances[i]
                distances[i] = distances[i + 1]
                distances[i + 1] = copy

    def set_element_color(self, el):
        if el.get_colors() == yellow:  # giallo
            self.destination = yellow_corner
            self.angle = yellow_yaw
            self.color = yellow
        elif el.get_colors() == red:  # rosso
            self.destination = red_corner
            self.angle = red_yaw
            self.color = red
        elif el.get_colors() == green:  # verde
            self.destination = green_corner
            self.angle = green_yaw
            self.color = green
        elif el.get_colors() == blue:  # blu
            self.destination = blue_corner
            self.angle = blue_yaw
            self.color = blue

    def turn_to_target(self):
        if abs(self.wall_reached_gps[0] - self.gps[0]) > 0.3 or \
                abs(self.wall_reached_gps[2] - self.gps[2]) > 0.3:
            self.stopped = False
            if self.sensors[7] < 900:
                self.ACTION = "turn right"
            else:
                self.ACTION = "turn left"

    def read_placed_spheres(self):
        now = time.time()
        timeout = now + 0.1
        while now < timeout:
            message = self._pubsub_spheres.get_message(ignore_subscribe_messages=True)
            if message is not None:
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
        self._myR.publish(self._ch_out_current_gps, json.dumps(self.gps))

    def wall(self, side):
        if side == "front":
            sublist = self.sensors[0:8]
        elif side == "back":
            sublist = self.sensors[8:]
        if any(s >= WALL for s in sublist):
            return True
        else:
            return False

    def too_close_wall(self):
        sublist = self.sensors[0:8]
        if sum(s >= 985 for s in sublist) >= 2:
            return True
        else:
            return False

    def get_number_wall_sensors(self, side=None):
        sublist = []
        value = 0
        if side == "sx":
            sublist = self.sensors[0:4]
            value = sum(MAX_SENSOR_VALUE < s for s in sublist)
        elif side == "dx":
            sublist = self.sensors[4:8]
            value = sum(MAX_SENSOR_VALUE < s for s in sublist)
        else:
            sublist = self.sensors[0:8]
            value = sum(s > MAX_SENSOR_VALUE for s in sublist)
        return value

    def check_near_wall_front(self, mode):
        sublist = self.sensors[0:8]
        return_value = None
        count = 0
        if mode == "any":
            if any(s >= MAX_SENSOR_VALUE_WALL_REACH for s in sublist):
                return_value = True
            else:
                return_value = False
        elif mode == "num":
            count = sum(s >= 930 for s in sublist)
            return_value = count
        return return_value

    def sense(self):
        # ottengo le informazioni dalla simulazione (sensori)
        self.gps = self.robot.get_gps_values()
        self.yaw = self.robot.get_yaw()
        self.sensors = self.robot.get_values_all_sensors()
        self.objects = self.robot.get_camera_objects()
        self.number_objects = self.robot.get_camera_number_objects()

        # ottengo le informazioni dal canale di comunicazione con l'altro robot
        self.read_placed_spheres()
        self.read_current_sphere_other_robot()
        self.read_current_gps_other_robot()

    def plan(self):
        self.controller()
        self.write_current_sphere()
        self.write_current_gps()

    def act(self):
        if self.ACTION == "go back": self.robot.go_back()
        if self.ACTION == "go left": self.robot.go_left()
        if self.ACTION == "go straight": self.robot.go_straight()
        if self.ACTION == "go right": self.robot.go_right()
        if self.ACTION == "stop": self.robot.stop()
        if self.ACTION == "slow down": self.robot.slow_down()


