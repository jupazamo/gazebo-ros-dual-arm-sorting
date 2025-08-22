#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import sys
import os
import rospy
import math
import random
import threading
from distutils.version import LooseVersion
import numpy as np
import moveit_commander
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from math import hypot

if sys.version_info < (3, 0):
    PY3 = False
    import Queue as queue
else:
    PY3 = True
    import queue

boxPoints = cv2.boxPoints if LooseVersion(cv2.__version__) >= LooseVersion('3.0') else cv2.cv.BoxPoints

COLOR_DICT = {
    'red': {'lower': np.array([0, 43, 46]), 'upper': np.array([10, 255, 255])},
    'blue': {'lower': np.array([90, 100, 100]), 'upper': np.array([130, 255, 255])},
    'green': {'lower': np.array([50, 60, 60]), 'upper': np.array([77, 255, 255])},
    'yellow': {'lower': np.array([27, 200, 120]), 'upper': np.array([31, 255, 160])},
}

COLOR_TARGET_ZONE = {
    'red':    {'x': 660, 'y': -250},
    'blue':   {'x': 340, 'y': -250},
    'green':  {'x': 340, 'y': 250},
    'yellow': {'x': 660, 'y': 250},
}

DROP_ZONE_PX = {
    'red':    (260, 400),
    'blue':   (160, 400),
    'green':  (380, 400),
    'yellow': (480, 400),
}

classified_centers = {
    'red': [],
    'blue': [],
    'green': [],
    'yellow': []
}

# ------ UI helpers ------

COLOR_BGR = {
    'red':    (0,   0, 255),
    'blue':   (255, 0,   0),
    'green':  (0, 255,   0),
    'yellow': (0, 255, 255),
    'pick':   (0, 255, 255),
}

def draw_minarearect(frame, rect, bgr=(0,255,255), thickness=2, label=None):
    box = boxPoints(rect)
    pts = np.array(box, dtype=np.intp)
    cv2.drawContours(frame, [pts], -1, bgr, thickness)
    if label:
        cx, cy = int(rect[0][0]), int(rect[0][1])
        cv2.putText(frame, label, (cx+6, cy-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(frame, label, (cx+6, cy-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

def rect_center(rect):
    return (float(rect[0][0]), float(rect[0][1]))

def nearest_rect(rects, target_xy):
    if not rects: return None
    tx, ty = target_xy
    best, bestd = None, 1e18
    for r in rects:
        cx, cy = rect_center(r)
        d = (cx - tx)*(cx - tx) + (cy - ty)*(cy - ty)
        if d < bestd: bestd, best = d, r
    return best

class Display:
    def __init__(self, window_name="Sorter Camera", record_path=None, fps=10, size=(960,540)):
        self.enabled = True
        if os.environ.get('DISPLAY','') == '':
            rospy.logwarn("No DISPLAY found. Disabling OpenCV windows.")
            self.enabled = False
        self.window = window_name
        self.writer = None
        self.size = size
        self.fps  = fps
        if self.enabled:
            try:
                cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
                cv2.startWindowThread()
            except Exception as e:
                rospy.logwarn("OpenCV GUI init warning: {}".format(e))
                self.enabled = False
        if record_path:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(record_path, fourcc, fps, size)
            if not self.writer.isOpened():
                rospy.logwarn("Could not open VideoWriter, disabling recording.")
                self.writer = None
        rospy.on_shutdown(self.cleanup)

    @staticmethod
    def overlay(frame, text):
        cv2.putText(frame, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1, cv2.LINE_AA)
        return frame

    def show(self, frame, status_text=None):
        if frame is None: return
        if status_text: frame = self.overlay(frame, status_text)
        if self.enabled:
            disp = cv2.resize(frame, self.size)
            cv2.imshow(self.window, disp)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                rospy.signal_shutdown("User quit")
        if self.writer is not None:
            self.writer.write(cv2.resize(frame, self.size))

    def cleanup(self):
        try:
            if self.writer is not None: self.writer.release()
            if self.enabled: cv2.destroyAllWindows()
        except Exception:
            pass

# ------ Robot control ------

class GripperCtrl(object):
    def __init__(self, robot_type='xarm'):
        self._commander = moveit_commander.move_group.MoveGroupCommander('{}_gripper'.format(robot_type))
        self._init()
    def _init(self):
        self._commander.set_max_acceleration_scaling_factor(1.0)
        self._commander.set_max_velocity_scaling_factor(1.0)
    def open(self, wait=True):
        try:
            self._commander.set_named_target('open')
            ret = self._commander.go(wait=wait)
            print('gripper_open, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] gripper open exception, {}'.format(e))
        return False
    def close(self, wait=True):
        try:
            self._commander.set_named_target('close')
            ret = self._commander.go(wait=wait)
            print('gripper_close, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] gripper close exception, {}'.format(e))
        return False

class XArmCtrl(object):
    def __init__(self, dof, robot_type='xarm'):
        self._commander = moveit_commander.move_group.MoveGroupCommander('{}{}'.format(robot_type, dof))
        self.dof = int(dof)
        self._init()
    def _init(self):
        self._commander.set_max_acceleration_scaling_factor(1.0)
        self._commander.set_max_velocity_scaling_factor(1.0)
    def set_joints(self, angles, wait=True):
        try:
            joint_target = self._commander.get_current_joint_values()
            for i in range(joint_target):
                if i >= len(angles):
                    break
                if angles[i] is not None:
                    joint_target[i] = math.radians(angles[i])
            print('set_joints, joints={}'.format(joint_target))
            self._commander.set_joint_value_target(joint_target)
            ret = self._commander.go(wait=wait)
            print('move to finish, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] set_joints exception, ex={}'.format(e))
    def set_joint(self, angle, inx=-1, wait=True):
        try:
            joint_target = self._commander.get_current_joint_values()
            joint_target[inx] = math.radians(angle)
            print('set_joints, joints={}'.format(joint_target))
            self._commander.set_joint_value_target(joint_target)
            ret = self._commander.go(wait=wait)
            print('move to finish, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] set_joint exception, ex={}'.format(e))
        return False
    def moveto(self, x=None, y=None, z=None, ox=None, oy=None, oz=None, relative=False, wait=True):
        if x == 0 and y == 0 and z == 0 and ox == 0 and oy == 0 and oz == 0 and relative:
            return True
        try:
            pose_target = self._commander.get_current_pose().pose
            if relative:
                pose_target.position.x += x / 1000.0 if x is not None else 0
                pose_target.position.y += y / 1000.0 if y is not None else 0
                pose_target.position.z += z / 1000.0 if z is not None else 0
                pose_target.orientation.x += ox if ox is not None else 0
                pose_target.orientation.y += oy if oy is not None else 0
                pose_target.orientation.z += oz if oz is not None else 0
            else:
                pose_target.position.x = x / 1000.0 if x is not None else pose_target.position.x
                pose_target.position.y = y / 1000.0 if y is not None else pose_target.position.y
                pose_target.position.z = z / 1000.0 if z is not None else pose_target.position.z
                pose_target.orientation.x = ox if ox is not None else pose_target.orientation.x
                pose_target.orientation.y = oy if oy is not None else pose_target.orientation.y
                pose_target.orientation.z = oz if oz is not None else pose_target.orientation.z
            if self.dof == 7:
                path, fraction = self._commander.compute_cartesian_path([pose_target], 0.005, 0.0)
                if fraction < 0.9:
                    ret = False
                else:
                    ret = self._commander.execute(path, wait=wait)
            else:
                self._commander.set_pose_target(pose_target)
                ret = self._commander.go(wait=wait)
            return ret
        except Exception as e:
            print('[Ex] moveto exception: {}'.format(e))
        return False
    def go_home(self, wait=True):
        try:
            print("Moviendo el brazo a la posición de reposo 'home' para finalizar.")
            self._commander.set_named_target('home')
            return self._commander.go(wait=wait)
        except Exception as e:
            print('[Ex] go_home exception: {}'.format(e))
            return False

class GazeboMotionThread(threading.Thread):
    def __init__(self, que, **kwargs):
        if PY3: super().__init__()
        else:   super(GazeboMotionThread, self).__init__()
        self.que = que
        self.daemon = True
        self.in_motion = True
        dof = kwargs.get('dof', 6)
        robot_type = kwargs.get('robot_type', 'xarm')
        self._xarm_ctrl = XArmCtrl(dof, robot_type=robot_type)
        self._gripper_ctrl = GripperCtrl(robot_type=robot_type)
        self._grab_z = kwargs.get('grab_z', 10)
        self._safe_z = kwargs.get('safe_z', 100)
    @staticmethod
    def _rect_to_move_params(rect):
        return int((466 - rect[0][1]) * 900.0 / 460.0 + 253.3), int((552 - rect[0][0]) * 900.0 / 460.0 - 450), rect[2] - 90
    def run(self):
        while True:
            self._gripper_ctrl.open()
            if not self._xarm_ctrl.moveto(z=self._safe_z): continue
            if not self._xarm_ctrl.moveto(x=300, y=0, z=self._safe_z): continue
            self.in_motion = False
            data = self.que.get()
            rects = data['rects']
            color_name = data['color']
            dest_xy = COLOR_TARGET_ZONE[color_name]
            self.in_motion = True
            rect = rects[random.randint(0, 100) % len(rects)]
            x, y, angle = self._rect_to_move_params(rect)
            if not self._xarm_ctrl.moveto(z=self._safe_z): continue
            if not self._xarm_ctrl.moveto(x=x, y=y, z=self._safe_z): continue
            if not self._xarm_ctrl.moveto(x=x, y=y, z=self._grab_z): continue
            self._gripper_ctrl.close()
            if not self._xarm_ctrl.moveto(x=x, y=y, z=self._safe_z): continue
            if not self._xarm_ctrl.moveto(x=dest_xy['x'], y=dest_xy['y'], z=self._safe_z): continue
            self._gripper_ctrl.open()

# ------ Camera & vision ------

class GazeboCamera(object):
    def __init__(self, topic_name='/camera/image_raw/compressed'):
        self._frame_que = queue.Queue(10)
        self._bridge = CvBridge()
        self._img_sub = rospy.Subscriber(topic_name, CompressedImage, self._img_callback)
    def _img_callback(self, data):
        if self._frame_que.full(): self._frame_que.get()
        self._frame_que.put(self._bridge.compressed_imgmsg_to_cv2(data))
    def get_frame(self):
        if self._frame_que.empty(): return None
        return self._frame_que.get()

def get_recognition_rect(frame, lower, upper):
    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    erode_hsv = cv2.erode(hsv, None, iterations=2)
    inRange_hsv = cv2.inRange(erode_hsv, lower, upper)
    contours = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    rects = []
    for c in contours:
        rect = cv2.minAreaRect(c)
        if rect[1][0] < 8 or rect[1][1] < 8: continue
        rects.append(rect)
    return rects, inRange_hsv

def rect_to_xy_mm(rect):
    x = int((466 - rect[0][1]) * 900.0 / 460.0 + 253.3)
    y = int((552 - rect[0][0]) * 900.0 / 460.0 - 450)
    return x, y

def is_new_block(center, known_centers, drop_zone_px=None, threshold=15):
    for known in known_centers:
        dist = ((center[0] - known[0])**2 + (center[1] - known[1])**2) ** 0.5
        if dist < threshold:
            return False
    if drop_zone_px:
        drop_x, drop_y = drop_zone_px
        dist_to_drop = ((center[0] - drop_x)**2 + (center[1] - drop_y)**2) ** 0.5
        if dist_to_drop < 40:
            return False
    return True

# ------ MAIN ------

if __name__ == '__main__':
    rospy.init_node('color_recognition_node', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    dof = rospy.get_param('/xarm/DOF', default=6)
    robot_type = rospy.get_param('~robot_type', 'xarm')
    rate = rospy.Rate(10.0)

    motion_que = queue.Queue(1)
    motion = GazeboMotionThread(motion_que, dof=dof, robot_type=robot_type)
    motion.start()

    cam = GazeboCamera(topic_name='camera/image_raw/compressed')
    display = Display(window_name="Sorter Camera",
                      record_path=None,   # o '/tmp/sorter.mp4' si quieres grabar
                      fps=10, size=(960,540))
    cv2.namedWindow("Sorter Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Sorter Camera", 640, 480)
    # Orden por color aleatorio (para que sea uno por uno, en secuencia)
    colors = list(COLOR_DICT.items())
    random.shuffle(colors)

    # Estado para tracking visual del bloque en curso
    current_color = None
    current_center_px = None

    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.get_frame()
        if frame is None:
            continue

        # Si el brazo está ejecutando un movimiento, intenta seguir SOLO el bloque actual
        if motion.in_motion and current_color is not None and current_center_px is not None:
            rects_now, _ = get_recognition_rect(frame.copy(),
                                                COLOR_DICT[current_color]['lower'],
                                                COLOR_DICT[current_color]['upper'])
            tracked = nearest_rect(rects_now, current_center_px)
            vis = frame.copy()
            if tracked is not None:
                draw_minarearect(vis, tracked, bgr=COLOR_BGR[current_color], thickness=3,
                                 label="{} (moving)".format(current_color))
            display.show(vis, status_text="Sorting '{}' (in motion)".format(current_color))
            continue

        # Si el brazo está libre, busca el próximo color y muestra SOLO ese
        found_task_this_scan = False

        for color_name, color_val in colors:
            rects, mask = get_recognition_rect(frame.copy(), lower=color_val['lower'], upper=color_val['upper'])

            if len(rects) == 0:
                continue  # Nada detectado para este color

            drop_px = DROP_ZONE_PX[color_name]
            new_rects = []
            for rect in rects:
                center = rect[0]
                x_mm, y_mm = rect_to_xy_mm(rect)
                drop = COLOR_TARGET_ZONE[color_name]
                if hypot(x_mm - drop['x'], y_mm - drop['y']) < 40:
                    continue  # Ya está en la zona de destino, ignorar
                if is_new_block(center, classified_centers[color_name]):
                    new_rects.append(rect)

            if len(new_rects) == 0:
                continue  # Todos ya fueron clasificados para ese color

            # Mostrar SOLO el bloque que se va a mover para este color
            vis0 = frame.copy()
            draw_minarearect(vis0, new_rects[0], bgr=COLOR_BGR[color_name],
                             thickness=3, label="{} → drop zone".format(color_name))
            display.show(vis0, status_text="Dispatching '{}'".format(color_name))

            # Enviar tarea y fijar estado de tracking
            if motion.in_motion or motion_que.qsize() != 0:
                break  # si por algún motivo se ocupó, sal del for y reintenta en la próxima iteración

            classified_centers[color_name].append(new_rects[0][0])
            motion_que.put({'rects': [new_rects[0]], 'color': color_name})

            current_color = color_name
            current_center_px = rect_center(new_rects[0])

            found_task_this_scan = True
            break  # Solo procesa un color por iteración

        # Si en esta pasada no se encontró nada y el brazo está libre → terminar
        if not found_task_this_scan and not motion.in_motion:
            rospy.loginfo("✓ Escaneo completo. No se encontraron más bloques por ordenar.")
            rospy.signal_shutdown("Todos los bloques fueron ordenados.")
            break
