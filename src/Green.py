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


class MachineState():
    RESET = 0
    TRACING = 1
    TRACKING = 2


class TraceState():
    RESET = 0
    FindAngle = 1
    Travel = 2
    Finish = 3


# initial stare
state = MachineState.RESET
# state of Travel rect
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
    if key1 == key1_debounced == 0:
        new_state = MachineState.RESET
    elif key2 == key2_debounced == 0:
        new_state = MachineState.TRACKING
    else:
        new_state = MachineState.RESET  # 默认状态

    # 检测第三个按键，并进行暂停/继续操作
    if key3 == key3_debounced == 0:
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
    img = sensor.snapshot()
    pan_current, tilt_current, pan_out, tilt_out = servoturn(0, 0, 90, 90)
    return pan_current, tilt_current


def constraint(target):
    if target < 75 or target > 105:
        if target < 75:
            target = 75
        if target > 105:
            target = 105
    return target


def servoturn(pan_error, tilt_error, pan, tilt):

    pan_output = pan_pid.get_pid(pan_error, 1)/2
    tilt_output = tilt_pid.get_pid(tilt_error, 1)

    # 水平目标
    pan_target = pan + pan_output
    pan_target = constraint(pan_target)
    servo.position(0, pan_target)
    pan = servo.get_angle()

    # 垂直目标
    tilt_target = tilt + tilt_output
    tilt_target = constraint(tilt_target)
    servo.position(1, tilt_target)
    tilt = servo.get_angle()
    return pan, tilt,pan_output,tilt_output


def Alarm():
    print("Alarm rings!")
    return


def doTrackRedPoint(pan_current, tilt_current):
    img = sensor.snapshot()  # Take a picture and return the image.
    blobs = img.find_blobs([red_threshold])
    if blobs and not INpause:
        max_blob = find_max(blobs)
        pan_error = max_blob.cx()-img.width()/2
        tilt_error = max_blob.cy()-img.height()/2
        img.draw_rectangle(max_blob.rect())  # rect
        img.draw_cross(max_blob.cx(), max_blob.cy())  # cx, cy

        pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)

        print("pan_current: ", pan_current)
        print("tilt_current: ", tilt_current)
        print(pan_output,tilt_output)

        if abs(pan_output) + abs(tilt_output) < 1:
            Alarm()
    return pan_current, tilt_current


# 在这里进入loop
while True:
    clock.tick()  # Track elapsed milliseconds between snapshots().
    state, INpause = button_read()  # change if button signal comes

# 这里开始进行状态跳转执行
    if state == MachineState.RESET:
        print("Reseting")
        pan_current, tilt_current = doReset()
    elif state == MachineState.TRACKING:
        print("Tracking")
        pan_current, tilt_current = doTrackRedPoint(pan_current, tilt_current)
    else:
        print("Unknown state")
