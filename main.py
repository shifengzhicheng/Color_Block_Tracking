# main function entry
# 导入必须库依赖
import sensor
import image
import time
from machine import I2C, Pin
from pid import PID
from ClassServo import Servos
from enum import Enum, auto
# 定义一些基本的量
# define servo
i2c = I2C(sda=Pin('P5'), scl=Pin('P4'))
servo = Servos(i2c, address=0x40, freq=50,
               min_us=500, max_us=2500, degrees=180)

# define PID
# pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
# tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
pan_pid = PID(p=0.1, i=0, imax=90)  # 在线调试使用这个PID
tilt_pid = PID(p=0.1, i=0, imax=90)  # 在线调试使用这个PID
pan_current = 0
tilt_current = 90

# define sensor
sensor.reset()  # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565)  # use RGB565.
sensor.set_framesize(sensor.QQVGA)  # use QQVGA for speed.
sensor.skip_frames(10)  # Let new settings take affect.
sensor.set_auto_whitebal(False)  # turn this off.

# define color
red_threshold = (13, 49, 18, 61, 6, 47)

clock = time.clock()  # Tracks FPS.

# define state
class MachineState(Enum):
    RESET = auto()
    TRACING = auto()
    TRACKING = auto()

# initial stare
state = MachineState.RESET
# 默认非暂停
INpause = False

# 根据button读入进行state跳转
def button_read():
    new_state = MachineState.TRACKING
    # 在这里插入按键的读入程序
    
    return new_state, False

# 这里是不同的函数入口
def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob = blob
            max_size = blob[2]*blob[3]
    return max_blob

def doReset():
    state = MachineState.RESET
    return

def doTraceBlackLine():
    img = sensor.snapshot()  # Take a picture and return the image.
    start_flag = find_angle()
    if start_flag:
        finish = doCircle()
    if finish:
        Alarm()
        doReset()
    return

def find_angle():
    return

def doCircle():
    return

def Alarm():
    return
def doTrackRedPoint():
    img = sensor.snapshot()  # Take a picture and return the image.
    blobs = img.find_blobs([red_threshold])
    if blobs and not INpause:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2

        print("pan_error: ", pan_error)

        img.draw_rectangle(max_blob.rect())  # rect
        img.draw_cross(max_blob.cx(), max_blob.cy())  # cx, cy

        pan_output = pan_pid.get_pid(pan_error, 1)/2
        tilt_output = tilt_pid.get_pid(tilt_error, 1)
        print("pan_output", pan_output)
        print("tilt_output", pan_output)

        servo.position(0, pan_current + pan_output)
        pan_current = servo.get_angle()
        servo.position(1, tilt_current - tilt_output)
        tilt_current = servo.get_angle()
    return

# 在这里进入loop
while True:
    clock.tick()  # Track elapsed milliseconds between snapshots().
    state, INpause = button_read() # change if button signal comes
    
# 这里开始进行状态跳转
    match state:
        case MachineState.RESET:
            print("Reset")
            doReset()
        case MachineState.TRACING:
            print("Tracing")
            doTraceBlackLine()
        case MachineState.TRACKING:
            print("Tracking")
            doTrackRedPoint()
