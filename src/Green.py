# main function entry
# 导入必须库依赖
import utime
import sensor
import image
import time
from machine import I2C, Pin
from pyb import Pin
from pid import PID
from ClassServo import Servos
from enum import Enum, auto
# 定义一些基本的量
# define servo and pin
i2c = I2C(sda=Pin('P5'), scl=Pin('P4'))
pin1 = Pin('P1', Pin.IN, Pin.PULL_UP)
pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)
pin3 = Pin('P3', Pin.IN, Pin.PULL_UP)
servo = Servos(i2c, address=0x40, freq=50,
               min_us=500, max_us=2500, degrees=180)

# define PID
# pan_pid = PID(p=0.07, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
# tilt_pid = PID(p=0.05, i=0, imax=90) #脱机运行或者禁用图像传输，使用这个PID
pan_pid = PID(p=0.1, i=0, imax=90)  # 在线调试使用这个PID
tilt_pid = PID(p=0.1, i=0, imax=90)  # 在线调试使用这个PID
pan_current = 90
tilt_current = 90

# define sensor
sensor.reset()  # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565)  # use RGB565.
sensor.set_framesize(sensor.QQVGA)  # use QQVGA for speed.
sensor.skip_frames(20)  # Let new settings take affect.
sensor.set_auto_whitebal(False)  # turn this off.

# define alarm

# define color
red_threshold = (13, 49, 18, 61, 6, 47)
green_threshold = (90, 150, 30, 100, 30, 100)
black_threshold = (0, 180, 0, 30, 0, 30)

clock = time.clock()  # Tracks FPS.

# define state


class MachineState(Enum):
    RESET = auto()
    TRACING = auto()
    TRACKING = auto()

class TraceState(Enum):
    RESET = auto()
    FindAngle = auto()
    Travel = auto()
    Finish = auto()
    
# initial stare
state = MachineState.RESET
## state of Travel rect
state_Trace = TraceState.RESET
# 默认非暂停
INpause = False

# 根据button读入进行state跳转


def button_read():
    # 在这里插入按键的读入程序
    pause = INpause
    key1 = pin1.value()
    key2 = pin2.value()
    key3 = pin3.value()

    # 等待一小段时间，例如20毫秒，以进行按键消抖
    utime.sleep_ms(20)

    # 再次读取按键状态
    key1_debounced = pin1.value()
    key2_debounced = pin2.value()
    key3_debounced = pin3.value()

    # 检测按键状态是否稳定，如果稳定则进行状态切换
    if key1 == key1_debounced == 1:
        new_state = MachineState.RESET
    elif key2 == key2_debounced == 1:
        new_state = MachineState.TRACKING
    else:
        new_state = MachineState.TRACKING  # 默认状态

    # 检测第三个按键，并进行暂停/继续操作
    if key3 == key3_debounced == 1:
        pause = ~pause

    return new_state, pause


# 这里是不同的函数入口
def find_max(blobs):
    max_size = 0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob = blob
            max_size = blob[2]*blob[3]
    return max_blob


def doReset():
    # 初始化
    servoturn(0, 0, 90, 90)
    return

def servoturn(pan_error, tilt_error, pan_current, tilt_current):
    
    pan_output = pan_pid.get_pid(pan_error, 1)/2
    tilt_output = tilt_pid.get_pid(tilt_error, 1)
    
    # 水平目标
    pan_target = pan_current + pan_output
    if pan_target < 75 or pan_target > 105:
        if pan_target < 75:
            pan_target = 75
        if pan_target > 105:
            pan_target = 105
    servo.position(0, pan_target)
    pan_current = servo.get_angle()
    
    # 垂直目标
    tilt_target = tilt_current + tilt_output
    if tilt_target < 75 or tilt_target > 105:
        if tilt_target < 75:
            tilt_target = 75
        if tilt_target > 105:
            tilt_target = 105
    servo.position(1, tilt_target)
    tilt_current = servo.get_angle()
    return pan_current, tilt_current


def Alarm():
    print("Alarm rings!")
    return


def doTrackRedPoint():
    img = sensor.snapshot()  # Take a picture and return the image.
    blobs = img.find_blobs([green_threshold])
    if blobs and not INpause:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2

        print("pan_error: ", pan_error)

        img.draw_rectangle(max_blob.rect())  # rect
        img.draw_cross(max_blob.cx(), max_blob.cy())  # cx, cy
        
        pan_current,tilt_current = servoturn(pan_error,tilt_error,pan_current,tilt_current)
        
        print("pan_current: ", pan_current)
        print("tilt_current: ", tilt_current)
        if abs(pan_error) + abs(tilt_error) < 1:
            Alarm()
    return


# 在这里进入loop
while True:
    clock.tick()  # Track elapsed milliseconds between snapshots().
    state, INpause = button_read()  # change if button signal comes

# 这里开始进行状态跳转执行
    match state:
        case MachineState.RESET:
            print("Reseting")
            doReset()
        case MachineState.TRACKING:
            print("Tracking")
            doTrackRedPoint()
