import sys
import time
import math
import numpy as np
import cv2
from enum import Enum, auto

# 1) Ruta a la API ZMQ de CoppeliaSim
sys.path.append(r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src")
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 2) Conexión y arranque de simulación
client = RemoteAPIClient()
sim    = client.getObject('sim')
sim.startSimulation()
time.sleep(0.5)

# 3) Handles de robot, motores, sensor y esfera verde
robot         = sim.getObject('/PioneerP3DX')
left_motor    = sim.getObject('/PioneerP3DX/leftMotor')
right_motor   = sim.getObject('/PioneerP3DX/rightMotor')
vision_sensor = sim.getObject('/Vision_sensor')
sphere_green  = sim.getObject('/esfera_verde')

# 4) Parámetros de control y PID
MAX_SPEED   = 2.0
WHEEL_BASE  = 0.325
PID_KP, PID_KI, PID_KD = 2.0, 0.0, 0.3

class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
    def update(self, error):
        now = time.time()
        dt  = now - self.prev_time if now!=self.prev_time else 1e-3
        self.integral += error*dt
        derivative = (error-self.prev_error)/dt
        out = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.prev_error, self.prev_time = error, now
        return out

# 5) FSM
class State(Enum):
    STOP       = auto()
    FORWARD    = auto()
    TURN_RIGHT = auto()
    TURN_LEFT  = auto()
    PAUSE      = auto()   # pausar → WAIT

class RobotController:
    def __init__(self):
        self.state = State.STOP
        self.pid = PID(PID_KP, PID_KI, PID_KD)
        self.pause_start = None
        self.pause_duration = 5.0

    def set_wheel_speeds(self, vl, vr):
        sim.setJointTargetVelocity(left_motor, vl)
        sim.setJointTargetVelocity(right_motor, vr)

    def enter_stop(self):
        self.state = State.STOP
        self.set_wheel_speeds(0,0)
    def enter_forward(self):
        self.state = State.FORWARD
    def enter_turn_right(self):
        self.state = State.TURN_RIGHT
        self.set_wheel_speeds(MAX_SPEED, -MAX_SPEED)
    def enter_turn_left(self):
        self.state = State.TURN_LEFT
        self.set_wheel_speeds(-MAX_SPEED, MAX_SPEED)
    def enter_pause(self):
        self.state = State.PAUSE
        self.set_wheel_speeds(0,0)
        self.pause_start = time.time()

    def update(self, event):
        if   self.state == State.STOP:
            if   event == 'esfera_verde':      self.enter_forward()
            elif event == 'esfera_naranja':    self.enter_turn_right()
            elif event == 'esfera_azul':       self.enter_turn_left()
            elif event == 'esfera_amarilla':   self.enter_pause()
        elif self.state == State.FORWARD:
            if   event == 'esfera_naranja':    self.enter_turn_right()
            elif event == 'esfera_azul':       self.enter_turn_left()
            elif event == 'esfera_amarilla':   self.enter_pause()
            elif event == 'ninguna_esfera':    self.enter_stop()
        elif self.state == State.TURN_RIGHT:
            if   event == 'esfera_verde':      self.enter_forward()
            elif event == 'esfera_amarilla':   self.enter_pause()
            elif event == 'ninguna_esfera':    self.enter_stop()
        elif self.state == State.TURN_LEFT:
            if   event == 'esfera_verde':      self.enter_forward()
            elif event == 'esfera_amarilla':   self.enter_pause()
            elif event == 'ninguna_esfera':    self.enter_stop()
        elif self.state == State.PAUSE:
            if time.time() - self.pause_start >= self.pause_duration:
                self.enter_stop()

    def follow_target(self):
        pr = sim.getObjectPosition(robot, -1)
        ps = sim.getObjectPosition(sphere_green, -1)
        dx, dy = ps[0]-pr[0], ps[1]-pr[1]
        desired = math.atan2(dy, dx)
        yaw = sim.getObjectOrientation(robot, -1)[2]
        error = (desired - yaw + math.pi)%(2*math.pi)-math.pi
        omega = self.pid.update(error)
        v     = MAX_SPEED * max(0, 1-abs(error)/math.pi)
        vl    = (2*v - omega*WHEEL_BASE)/2
        vr    = (2*v + omega*WHEEL_BASE)/2
        vl = np.clip(vl, -MAX_SPEED, MAX_SPEED)
        vr = np.clip(vr, -MAX_SPEED, MAX_SPEED)
        self.set_wheel_speeds(vl, vr)

# 6) Detección de color (ahora key 'AMARILLA')
color_ranges = {
  'VERDE':    ((50,100,100),(70,255,255)),
  'NARANJA':  ((10,100,100),(25,255,255)),
  'AZUL':     ((100,100,100),(130,255,255)),
  'AMARILLA': ((20,100,100),(30,255,255)),
}
PIXEL_THRESHOLD = 0.005

def get_color_event():
    img_bytes,(w,h) = sim.getVisionSensorImg(vision_sensor)
    buf = np.frombuffer(img_bytes, dtype=np.uint8).reshape(h,w,3)
    bgr = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    total = w*h
    for name,(low,high) in color_ranges.items():
        if cv2.countNonZero(cv2.inRange(hsv, np.array(low), np.array(high))) > total*PIXEL_THRESHOLD:
            return f'esfera_{name.lower()}'
    return 'ninguna_esfera'

# 7) Ventana grande + texto pequeño
win = 'PID+FSM Vision'
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
res = sim.getVisionSensorResolution(vision_sensor)
cv2.resizeWindow(win, res[0]+200, res[1]+150)

ctrl = RobotController()

try:
    while True:
        evt = get_color_event()
        ctrl.update(evt)
        if ctrl.state == State.FORWARD:
            ctrl.follow_target()

        # captura
        img_b,(w,h)=sim.getVisionSensorImg(vision_sensor)
        frame=np.frombuffer(img_b,dtype=np.uint8).reshape(h,w,3)
        vis=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)

        # preparar canvas con margen fijo
        margin = 100
        canvas = np.zeros((h+margin, w, 3), dtype=np.uint8)
        canvas[:h] = vis

        # decide label de estado
        state_label = 'WAIT' if ctrl.state==State.PAUSE else ctrl.state.name

        # líneas de texto
        font, scale, th = cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1
        lines = [
            f'Estado: {state_label}',
            f'Evento: {evt}'
        ]
        if ctrl.state == State.PAUSE:
            rem = int(ctrl.pause_duration - (time.time() - ctrl.pause_start)) + 1
            rem = max(0, min(rem, int(ctrl.pause_duration)))
            lines.append(f'Timer: {rem}s')

        # dibujar
        y = h + 20
        for l in lines:
            cv2.putText(canvas, l, (10,y), font, scale, (0,255,0), th)
            y += int(30*scale) + 10

        cv2.imshow(win, canvas)
        if cv2.waitKey(1)&0xFF == ord('q'):
            break
        time.sleep(0.02)

finally:
    ctrl.enter_stop()
    sim.stopSimulation()
    cv2.destroyAllWindows()
