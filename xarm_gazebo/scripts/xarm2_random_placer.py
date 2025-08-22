#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xarm2_random_placer.py
Un nodo de ROS para que un brazo robótico redistribuya 4 bloques de forma aleatoria.

FASE 1: ESCANEO
  - Utiliza la cámara para localizar la posición inicial de los 4 bloques.
FASE 2: EJECUCIÓN
  - Genera un plan aleatorio para mover cada bloque detectado a una zona única.
  - Ejecuta el plan de movimiento.

✓ Combina la visión por computadora con una lógica de planificación aleatoria.
✓ Reutiliza la arquitectura multi-robot para máxima flexibilidad.
"""

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
from math import hypot, cos, sin, radians

try:
    import queue
except ImportError:
    import Queue as queue

# ───────────────────────── Constantes y Configuración ────────────────────────── #
boxPoints = cv2.boxPoints if LooseVersion(cv2.__version__) >= LooseVersion('3.0') else cv2.cv.BoxPoints

COLOR_DICT = {
    'red': {'lower': np.array([0, 43, 46]), 'upper': np.array([10, 255, 255])},
    'blue': {'lower': np.array([90, 100, 100]), 'upper': np.array([130, 255, 255])},
    'green': {'lower': np.array([50, 60, 60]), 'upper': np.array([77, 255, 255])},
    'yellow': {'lower': np.array([27, 200, 120]), 'upper': np.array([31, 255, 160])},
}

TARGET_ZONES = {
    'zone1': {'x': 635, 'y': -245},
    'zone2': {'x': 370, 'y': -245},
    'zone3': {'x': 370, 'y':  245},
    'zone4': {'x': 635, 'y':  245},
}

# Radio para colocar aleatoriamente dentro de la zona
ZONE_RANDOM_OFFSET = 40 

# --- ¡NUEVA CONSTANTE PARA FILTRAR POR TAMAÑO! ---
# Área máxima en píxeles. Si un contorno es más grande que esto,
# se considera un tapete y se ignora.
# Este valor puede requerir ajuste.
MAX_BLOCK_AREA_PX = 5000

# ─────────────────── Funciones y Clases (Reutilizadas y Adaptadas) ───────────────── #

# Las clases GripperCtrl, XArmCtrl, Camera y las funciones de utilidad
# mesa_to_robot, rect_to_xy_mm, get_recognition_rects son idénticas
# a las del script color_recognition_multi.py funcional.
# Las incluyo aquí para que el script sea autocontenido.

def mesa_to_robot(x_mm, y_mm, dx=0, dy=0, dtheta_deg=0):
    theta_rad = radians(dtheta_deg)
    xr = x_mm * cos(theta_rad) + y_mm * sin(theta_rad) + dx
    yr = -x_mm * sin(theta_rad) + y_mm * cos(theta_rad) + dy
    return xr, yr

def rect_to_xy_mm(rect):
    x_calculado = int((466 - rect[0][1]) * 900.0 / 460.0 + 253.3)
    y_calculado = int((552 - rect[0][0]) * 900.0 / 460.0 - 450)
    offset_x_mm = -17.5
    x_corregido = x_calculado + offset_x_mm
    y_corregido = y_calculado
    return x_corregido, y_corregido

class GripperCtrl(object):
    def __init__(self, ns='', robot_type='xarm'):
        group_name = f'{robot_type}_gripper' # <-- Usa robot_type
        ns_prefix = f'/{ns}' if ns else ''
        self._commander = moveit_commander.move_group.MoveGroupCommander(group_name, ns=ns_prefix)
        self._commander.set_max_acceleration_scaling_factor(1.0)
        self._commander.set_max_velocity_scaling_factor(1.0)
    def open(self, wait=True):
        self._commander.set_named_target('open'); return self._commander.go(wait=wait)
    def close(self, wait=True):
        self._commander.set_named_target('close'); return self._commander.go(wait=wait)

class XArmCtrl(object):
    def __init__(self, dof, ns='', robot_type='xarm'):
        group_name = f'{robot_type}{dof}' # <-- Usa robot_type
        ns_prefix = f'/{ns}' if ns else ''
        self._commander = moveit_commander.move_group.MoveGroupCommander(group_name, ns=ns_prefix)
        self._commander.set_max_acceleration_scaling_factor(1.0)
        self._commander.set_max_velocity_scaling_factor(1.0)
    def moveto(self, x=None, y=None, z=None, wait=True):
        pose_target = self._commander.get_current_pose().pose
        pose_target.position.x = x / 1000.0 if x is not None else pose_target.position.x
        pose_target.position.y = y / 1000.0 if y is not None else pose_target.position.y
        pose_target.position.z = z / 1000.0 if z is not None else pose_target.position.z
        self._commander.set_pose_target(pose_target)
        return self._commander.go(wait=wait)
    # --- MÉTODO NUEVO AÑADIDO ---
    def go_home(self, wait=True):
        """
        Mueve el brazo a su posición de reposo predefinida en MoveIt!.
        El nombre común es 'home'.
        """
        rospy.loginfo("Moviendo el brazo a la posición de reposo 'home'.")
        try:
            self._commander.set_named_target('home')
            return self._commander.go(wait=wait)
        except Exception as e:
            rospy.logerr(f"No se pudo mover a 'home': {e}")
            return False

class MotionThread(threading.Thread):
    def __init__(self, que, dof, ns, base_tf, safe_z, grab_z, robot_type='xarm'):
        super().__init__(daemon=True)
        self.que = que
        self.in_motion = False
        self._xarm_ctrl = XArmCtrl(dof, ns, robot_type=robot_type)
        self._gripper_ctrl = GripperCtrl(ns, robot_type=robot_type)
        self.dx, self.dy, self.dth = base_tf
        self._grab_z = grab_z
        self._safe_z = safe_z
    def mesa2robot(self, x, y):
        return mesa_to_robot(x, y, self.dx, self.dy, self.dth)
    def run(self):
        while not rospy.is_shutdown():
            task = self.que.get()
            self.in_motion = True
            source_pos, dest_pos = task['source'], task['dest']
            
            # Recoger
            x_source_robot, y_source_robot = self.mesa2robot(source_pos['x'], source_pos['y'])
            rospy.loginfo(f"Recogiendo de (Mesa: {source_pos['x']:.1f}, {source_pos['y']:.1f})")
            self._xarm_ctrl.moveto(x=x_source_robot, y=y_source_robot, z=self._safe_z)
            self._xarm_ctrl.moveto(x=x_source_robot, y=y_source_robot, z=self._grab_z)
            self._gripper_ctrl.close(); rospy.sleep(0.5)
            self._xarm_ctrl.moveto(x=x_source_robot, y=y_source_robot, z=self._safe_z)

            # Soltar
            xd_robot, yd_robot = self.mesa2robot(dest_pos['x'], dest_pos['y'])
            rospy.loginfo(f"Soltando en (Mesa: {dest_pos['x']:.1f}, {dest_pos['y']:.1f})")
            self._xarm_ctrl.moveto(x=xd_robot, y=yd_robot, z=self._safe_z)
            self._gripper_ctrl.open(); rospy.sleep(0.5)

            # Regresar a Home
            x_home, y_home = self.mesa2robot(300, 0)
            self._xarm_ctrl.moveto(x=x_home, y=y_home, z=self._safe_z)
            self.in_motion = False

class Camera(object):
    def __init__(self, topic_name):
        self._frame_que = queue.Queue(10)
        self._bridge = CvBridge()
        rospy.Subscriber(topic_name, CompressedImage, self._img_callback, queue_size=1)
    def _img_callback(self, data):
        if self._frame_que.full(): self._frame_que.get()
        self._frame_que.put(self._bridge.compressed_imgmsg_to_cv2(data))
    def get_frame(self):
        return None if self._frame_que.empty() else self._frame_que.get()

def get_recognition_rects(frame, lower, upper):
    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    erode_hsv = cv2.erode(hsv, None, iterations=2)
    inRange_hsv = cv2.inRange(erode_hsv, lower, upper)
    contours = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    rects = []
    vis = frame.copy()
    for c in contours:
        area = cv2.contourArea(c)
        if area > MAX_BLOCK_AREA_PX:
            continue
        rect = cv2.minAreaRect(c)
        if rect[1][0] < 8 or rect[1][1] < 8: 
            continue
        rects.append(rect)
        box = boxPoints(rect)
        cv2.drawContours(vis, [np.int0(box)], -1, (0,255,255), 2)
    return rects, vis

class Display:
    def __init__(self, window_name="Camera", record_path=None, fps=10, size=None):
        self.enabled = True
        # Desactiva GUI si no hay DISPLAY (por SSH o headless)
        if os.environ.get('DISPLAY', '') == '':
            rospy.logwarn("No DISPLAY found. Disabling OpenCV windows.")
            self.enabled = False
        self.window = window_name
        self.writer = None
        self.size = size   # (w,h) deseado para el video
        self.fps  = fps

        if self.enabled:
            try:
                cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
                cv2.startWindowThread()
            except Exception as e:
                rospy.logwarn("OpenCV GUI init warning: {}".format(e))
                self.enabled = False

        # Grabador de video opcional (MP4 H.264 o MJPG según tu OpenCV)
        if record_path is not None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # o 'MJPG'
            self.writer = cv2.VideoWriter(record_path, fourcc, fps, self.size if self.size else (960, 540))
            if not self.writer.isOpened():
                rospy.logwarn("Could not open VideoWriter, disabling recording.")
                self.writer = None

        rospy.on_shutdown(self.cleanup)

    @staticmethod
    def overlay(frame, text):
        cv2.putText(frame, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, cv2.LINE_AA)
        return frame

    def show(self, frame, status_text=None, draw_size=(960, 540)):
        if frame is None:
            return
        if status_text:
            frame = self.overlay(frame, status_text)
        if self.enabled:
            disp = cv2.resize(frame, draw_size)
            cv2.imshow(self.window, disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.signal_shutdown("User quit")
        if self.writer is not None:
            # Aseguramos tamaño fijo para el archivo
            w, h = self.size if self.size else (960, 540)
            out = cv2.resize(frame, (w, h))
            self.writer.write(out)

    def cleanup(self):
        try:
            if self.writer is not None:
                self.writer.release()
            if self.enabled:
                cv2.destroyAllWindows()
        except Exception:
            pass

COLOR_BGR = {
    'red':    (0,   0, 255),
    'blue':   (255, 0,   0),
    'green':  (0, 255,   0),
    'yellow': (0, 255, 255),
}

def draw_minarearect(frame, rect, bgr=(0,255,255), label=None):
    """Dibuja un minAreaRect sobre frame."""
    box = boxPoints(rect)
    pts = np.array(box, dtype=np.intp)
    cv2.drawContours(frame, [pts], -1, bgr, 2)
    if label:
        cx, cy = int(rect[0][0]), int(rect[0][1])
        cv2.putText(frame, label, (cx+6, cy-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(frame, label, (cx+6, cy-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

def rect_center(rect):
    """Centro (x,y) en píxeles de un minAreaRect."""
    return (float(rect[0][0]), float(rect[0][1]))

def nearest_rect(rects, target_xy):
    """Devuelve el rect de rects cuyo centro está más cerca de target_xy."""
    if not rects:
        return None
    tx, ty = target_xy
    best = None
    bestd = 1e18
    for r in rects:
        cx, cy = rect_center(r)
        d = (cx - tx)*(cx - tx) + (cy - ty)*(cy - ty)
        if d < bestd:
            bestd = d
            best = r
    return best

# ───────────────────────── Nodo Principal ─────────────────────────── #
def main():
    rospy.init_node('block_scrambler_node')
    moveit_commander.roscpp_initialize(sys.argv)

    ns          = rospy.get_param('~namespace', '')
    dof         = rospy.get_param('~dof', 6)
    cam_topic   = rospy.get_param('~camera_topic', 'camera/image_raw/compressed')
    base_tf     = rospy.get_param('~base_tf', [0, 0, 0])
    safe_z      = rospy.get_param('~safe_z', 50)
    grab_z      = rospy.get_param('~grab_z', -67.5)
    robot_type  = rospy.get_param('~robot_type2', 'xarm')


    motion_que = queue.Queue()
    motion = MotionThread(motion_que, dof, ns, base_tf, safe_z, grab_z, robot_type=robot_type)
    motion.start()
    cam = Camera(topic_name=cam_topic)
    # === inicializa UI/Grabación ===
    # Si quieres solo mostrar: deja record_path=None.
    # Si quieres grabar evidencia: pon una ruta, p. ej. '/tmp/evidencia_random_placer.mp4'
    display = Display(window_name="Random Placer Camera",
                    record_path=None,     # o '/tmp/evidencia_random_placer.mp4'
                    fps=10,
                    size=(960, 540))      # tamaño del video de salida
    
    cv2.namedWindow("Random Placer Camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Random Placer Camera", 640, 480)

    rate = rospy.Rate(5)
    
    # --- FASE 1: ESCANEO Y DETECCIÓN ---
    rospy.loginfo("FASE 1: Escaneando la mesa para localizar los 4 bloques...")
    detected_blocks = {}
    while len(detected_blocks) < 4 and not rospy.is_shutdown():
        frame = cam.get_frame()
        if frame is None:
            rate.sleep()
            continue

        display.show(frame, status_text="Scanning for 4 blocks (phase 1)")
        
        for color_name, color_val in COLOR_DICT.items():
            if color_name in detected_blocks:
                continue # Ya hemos encontrado este color
            
            rects, vis = get_recognition_rects(frame, color_val['lower'], color_val['upper'])
            if rects:
                detected_blocks[color_name] = rects[0]
                rospy.loginfo(f"  ✓ Bloque '{color_name}' localizado.")

        rate.sleep()
    
    if rospy.is_shutdown(): return
    rospy.loginfo("¡Escaneo completo! Los 4 bloques han sido localizados.")

    # --- FASE 2: PLANIFICACIÓN Y EJECUCIÓN ---
    rospy.loginfo("FASE 2: Generando y ejecutando el plan de colocación aleatorio...")

    block_colors = list(detected_blocks.keys())
    destination_zones = list(TARGET_ZONES.values())
    random.shuffle(destination_zones)

    for i, color in enumerate(block_colors):
        if rospy.is_shutdown(): break

        # Origen: el que se detectó en la FASE 1
        source_rect = detected_blocks[color]
        source_x, source_y = rect_to_xy_mm(source_rect)

        # Destino: uno de la lista aleatoria de zonas
        target_zone_center = destination_zones[i]
        dest_x = target_zone_center['x']
        dest_y = target_zone_center['y']
        
        rospy.loginfo(f"Movimiento {i+1}/4: Bloque '{color}' → Zona en ({dest_x:.0f}, {dest_y:.0f})")

        # Visual: enmarcar SOLO el bloque que se va a mover
        frame0 = cam.get_frame()
        if frame0 is not None:
            vis0 = frame0.copy()
            draw_minarearect(vis0, source_rect, bgr=COLOR_BGR[color], label=f"{color} → ({dest_x:.0f},{dest_y:.0f})")
            display.show(vis0, status_text=f"Dispatching move for '{color}'")

        # Encola la tarea
        task = {'source': {'x': source_x, 'y': source_y}, 'dest': {'x': dest_x, 'y': dest_y}}
        motion_que.put(task)

        # Guarda el centro en píxeles para hacer tracking simple durante el movimiento
        selected_center_px = rect_center(source_rect)
        selected_color = color

        # Esperar a que el movimiento se complete
        while motion_que.qsize() > 0 or motion.in_motion:
            if rospy.is_shutdown(): break
            fr = cam.get_frame()
            if fr is not None:
                vis = fr.copy()
                # Busca SOLO el color en movimiento y el rect más cercano al centro original
                rects_now, _ = get_recognition_rects(fr, COLOR_DICT[selected_color]['lower'], COLOR_DICT[selected_color]['upper'])
                tracked = nearest_rect(rects_now, selected_center_px)
                if tracked is not None:
                    draw_minarearect(vis, tracked, bgr=COLOR_BGR[selected_color], label=f"{selected_color} (moving)")
                display.show(vis, status_text=f"Moving '{selected_color}' to zone (phase 2)")
            rate.sleep()
    
    if not rospy.is_shutdown():
        rospy.loginfo("✓ Tarea completada: 4 bloques redistribuidos aleatoriamente.")
        rate.sleep() 
        # Llama directamente al método go_home() del controlador del brazo dentro del hilo.
        motion._xarm_ctrl.go_home()
        motion._xarm_ctrl.moveto(0, 250)
        rospy.loginfo("✓ Brazo en posición de reposo. Finalizando el nodo.")
    
    rospy.signal_shutdown("Plan de colocación finalizado.")


if __name__ == '__main__':
    main()