#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zone_checker.py  – Fase 0 únicamente
• Detecta bloques de cada color en las imágenes de la cámara.
• Si un bloque NO está sobre su tapete correspondiente, lo
  envía a la cola de movimiento para que el brazo lo corrija.
• Cuando ya no quedan errores, publica /zones_clear=True y finaliza.
"""

import sys, math, random, threading, rospy, cv2, numpy as np
from math import hypot
import os
from distutils.version import LooseVersion
import moveit_commander
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

# ───────────────────────── Constantes ────────────────────────── #
PY3   = sys.version_info.major >= 3
Queue = __import__("queue" if PY3 else "Queue")

COLOR_DICT = {
    'red':    {'lower': np.array([0, 43, 46]),  'upper': np.array([10, 255,255])},
    'blue':   {'lower': np.array([90,100,100]), 'upper': np.array([130,255,255])},
    'green':  {'lower': np.array([50,60,60]),   'upper': np.array([77, 255,255])},
    'yellow': {'lower': np.array([27,200,120]), 'upper': np.array([31, 255,160])},
}

COLOR_TARGET_ZONE = {
    'red':    {'x': 660, 'y': -250},
    'blue':   {'x': 340, 'y': -250},
    'green':  {'x': 340, 'y':  250},
    'yellow': {'x': 660, 'y':  250},
}

COLOR_BGR = {
    'red':    (0,   0, 255),
    'blue':   (255, 0,   0),
    'green':  (0, 255,   0),
    'yellow': (0, 255, 255),
    'gray':   (180,180,180),
    'ok':     (0, 200,   0),
    'bad':    (0,   0, 200),
    'pick':   (0, 255, 255),
}

ZONE_HALF = 60          # radio (mm) del tapete (10×10 cm)

SHOW_CAM = False         # Pon en False si no tienes entorno gráfico

# ─────────────────── Utilidades de visión ────────────────────── #
boxPoints = (cv2.boxPoints
             if LooseVersion(cv2.__version__) >= LooseVersion('3.0')
             else cv2.cv.BoxPoints)

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
    def __init__(self, window_name="Zone Checker", record_path=None, fps=10, size=(960,540)):
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

def rect_to_xy_mm(rect):
    """Convierte centro del rectángulo (px) → coordenadas mesa (mm)"""
    x = int((466 - rect[0][1]) * 900.0 / 460.0 + 253.3)
    y = int((552 - rect[0][0]) * 900.0 / 460.0 - 450)
    return x, y

def point_in_zone(x_mm, y_mm, color):
    z = COLOR_TARGET_ZONE[color]
    return (abs(x_mm - z['x']) <= ZONE_HALF and
            abs(y_mm - z['y']) <= ZONE_HALF)

def get_rects(frame, lower, upper):
    gs  = cv2.GaussianBlur(frame, (5,5), 0)
    hsv = cv2.cvtColor(gs, cv2.COLOR_BGR2HSV)
    mask= cv2.inRange(cv2.erode(hsv,None,iterations=2), lower, upper)
    cnts= cv2.findContours(mask, cv2.RETR_EXTERNAL,
                           cv2.CHAIN_APPROX_SIMPLE)[-2]
    rects=[]
    for c in cnts:
        r=cv2.minAreaRect(c)
        if r[1][0] < 8 or r[1][1] < 8: continue
        rects.append(r)
    return rects

def zone_of_point(x_mm, y_mm):
    """Devuelve el color del tapete en el que cae (x,y), o None si está fuera de todos."""
    for zc, z in COLOR_TARGET_ZONE.items():
        if abs(x_mm - z['x']) <= ZONE_HALF and abs(y_mm - z['y']) <= ZONE_HALF:
            return zc
    return None


# ───────────── Control del robot (brazo + pinza) ─────────────── #
class GripperCtrl:
    def __init__(self, robot_type='xarm'): # <-- Añade robot_type
        self.g = moveit_commander.MoveGroupCommander(f'{robot_type}_gripper')
        self.g.set_max_velocity_scaling_factor(1.0)
    def open(self):  self.g.set_named_target('open');  self.g.go()
    def close(self): self.g.set_named_target('close'); self.g.go()

class ArmCtrl:
    def __init__(self, dof, robot_type='xarm'):
        self.g = moveit_commander.MoveGroupCommander(f'{robot_type}{dof}')
        self.g.set_max_velocity_scaling_factor(1.0)
    def move_xyz(self, x,y,z):
        p = self.g.get_current_pose().pose
        p.position.x, p.position.y, p.position.z = x/1000.0, y/1000.0, z/1000.0
        self.g.set_pose_target(p); self.g.go()
    def get_xyz(self):
        """Devuelve (x,y,z) actuales en mm."""
        p = self.g.get_current_pose().pose
        return (p.position.x * 1000.0, p.position.y * 1000.0, p.position.z * 1000.0)

    def move_z(self, z):
        """Movimiento vertical puro hasta z (mm), manteniendo x,y actuales."""
        x, y, _ = self.get_xyz()
        self.move_xyz(x, y, z)

    

class MotionThread(threading.Thread):
    def __init__(self, q, dof=6, safe_z=100, grab_z=10, robot_type='xarm'):
        super().__init__(daemon=True)
        self.q, self.in_motion = q, True
        self.arm, self.grip   = ArmCtrl(dof, robot_type=robot_type), GripperCtrl(robot_type=robot_type)
        self.safe_z, self.grab_z = safe_z, grab_z
    def run(self):
        while not rospy.is_shutdown():
            # Abrir pinza y elevar primero en Z (vertical) para evitar colisión con la mesa
            self.grip.open()
            self.arm.move_z(self.safe_z)                # ← primero elevar
            self.arm.move_xyz(290, 0, self.safe_z)      # ← luego moverse en XY a la altura segura
            self.in_motion=False
            data = self.q.get()
            self.in_motion=True
            rect = random.choice(data['rects'])
            x,y = rect_to_xy_mm(rect)
            # recoger
            self.arm.move_xyz(x,y,self.safe_z)
            self.arm.move_xyz(x,y,self.grab_z)
            self.grip.close()
            self.arm.move_xyz(x,y,self.safe_z)
            # soltar
            dest = COLOR_TARGET_ZONE[data['color']]
            self.arm.move_xyz(dest['x'],dest['y'],self.safe_z)
            self.grip.open()

# ───────────── Subscripción a la cámara ───────────── #
class Cam:
    def __init__(self, topic='camera/image_raw/compressed'):
        self.q = Queue.Queue(10); self.bridge= CvBridge()
        rospy.Subscriber(topic, CompressedImage, self.cb)
    def cb(self,msg):
        if self.q.full(): self.q.get()
        self.q.put(self.bridge.compressed_imgmsg_to_cv2(msg))
    def frame(self):
        return None if self.q.empty() else self.q.get()

# ───────────────────── Nodo principal ─────────────────────── #
def main():
    rospy.init_node('zone_verifier')
    moveit_commander.roscpp_initialize(sys.argv)
    dof  = rospy.get_param('/xarm/DOF', 6)
    robot_type = rospy.get_param('~robot_type', 'xarm')
    cam  = Cam()
    display = Display(window_name="Zone Checker Camera",
                  record_path=None,   # o '/tmp/zone_checker.mp4'
                  fps=10, size=(960,540))
    cv2.namedWindow("Zone Checker Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Zone Checker Camera", 640, 480)
    qmov = Queue.Queue(1)
    mot  = MotionThread(qmov,dof=dof, robot_type=robot_type); mot.start()
    pub  = rospy.Publisher('/zones_clear', Bool, latch=True, queue_size=1)
    rate = rospy.Rate(8)

    processed = {c:[] for c in COLOR_DICT}   # evita reprocesar mismos bloques

    last_target_center = None
    last_target_color  = None

    while not rospy.is_shutdown():
        frame = cam.frame()
        if frame is None: rate.sleep(); continue

        composite = frame.copy()
        misplacements = []
        ok_count, wrong_count, center_count = 0, 0, 0

        # 1) Analiza y DIBUJA todos los bloques por color
        for cname, cval in COLOR_DICT.items():
            rects = get_rects(frame.copy(), cval['lower'], cval['upper'])
            for r in rects:
                x_mm, y_mm = rect_to_xy_mm(r)
                zone_color = zone_of_point(x_mm, y_mm)

                if zone_color is None:
                    # montón central (fuera de tapetes)
                    draw_minarearect(composite, r, bgr=COLOR_BGR['gray'], thickness=1, label=None)
                    center_count += 1
                    continue

                if zone_color == cname:
                    # correcto en su propio tapete
                    draw_minarearect(composite, r, bgr=COLOR_BGR['ok'], thickness=2, label=cname)
                    ok_count += 1
                    continue

                # mal ubicado (está en un tapete de otro color)
                draw_minarearect(composite, r, bgr=COLOR_BGR['bad'], thickness=2, label=f"{cname} @ {zone_color}")
                # Evita reprocesar el mismo bloque
                if any(hypot(x_mm - x, y_mm - y) < 15 for x, y in processed[cname]):
                    continue
                misplacements.append({'rect': r, 'color': cname})

        status = f"OK:{ok_count}  WRONG:{wrong_count+len(misplacements)}  CENTER:{center_count}"
        display.show(composite, status_text=f"Zone check (phase 0) – {status}")

        # 2) Si hay errores y el brazo está libre => corrige el primero
        if misplacements and not mot.in_motion and qmov.empty():
            target = misplacements[0]
            processed[target['color']].append(rect_to_xy_mm(target['rect']))
            qmov.put({'rects':[target['rect']], 'color':target['color']})

            # Guarda centro para tracking visual
            last_target_center = rect_center(target['rect'])
            last_target_color  = target['color']

            # Resalta el seleccionado en amarillo una vez
            highlight = composite.copy()
            draw_minarearect(highlight, target['rect'],
                            bgr=COLOR_BGR['pick'], thickness=3,
                            label=f"Fix: {last_target_color}")
            display.show(highlight, status_text="Dispatching correction")

            # Mientras se mueve el brazo, intenta seguir SOLO ese bloque por cercanía
            while mot.in_motion and not rospy.is_shutdown():
                fr = cam.frame()
                if fr is None: rate.sleep(); continue
                vis = fr.copy()
                rects_now = get_rects(fr.copy(),
                                    COLOR_DICT[last_target_color]['lower'],
                                    COLOR_DICT[last_target_color]['upper'])
                tracked = nearest_rect(rects_now, last_target_center)
                if tracked is not None:
                    draw_minarearect(vis, tracked, bgr=COLOR_BGR['pick'], thickness=3,
                                    label=f"{last_target_color} (moving)")
                display.show(vis, status_text="Correcting misplacement")
                rate.sleep()

        # 3) Si ya no hay errores y el brazo está libre: terminar
        if not misplacements and not mot.in_motion:
            pub.publish(True)
            rospy.loginfo("✓ Todos los tapetes tienen bloques correctos.")
            rospy.loginfo("✓ Tarea completada...")
            rospy.signal_shutdown("Plan finalizado.")
            break

        rate.sleep()

if __name__ == '__main__':
    main()
