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
pin6 = Pin('P6', Pin.IN, Pin.PULL_UP)
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
green_threshold = (5, 76, -44, 13, -1, 34)
black_threshold = (0, 180, 0, 30, 0, 30)

# define global const
total_error = 1
area = (35,20,95,95)
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

class NowPosition():
    UL = 0
    UR = 1
    DR = 2
    DL = 3
    Finish = 4

# initial stare
state = MachineState.RESET
## state of Travel rect
state_Trace = TraceState.RESET
### state of position
position = NowPosition.UL
# 默认非暂停
INpause = False

# 根据button读入进行state跳转

max_step = 5
def doTask2(pan_current, tilt_current):
    img = sensor.snapshot()
    global position
    ul = (0,0)
    ur = (100,0)
    dr = (100,-100)
    dl = (0,-100)
    con_rect = [ul,ur,dr,dl]
    if position == NowPosition.UL:
        pan_error = con_rect[1][0]-img.width()/2
        tilt_error = con_rect[1][1]-img.height()/2
        ratio = pan_error/tilt_error
        pan_current, tilt_current, pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
        if abs(pan_output) + abs(tilt_output) < total_error:
            position = NowPosition.UR
    elif position == NowPosition.UR:
        pan_error = min(max_step, con_rect[2][0]-img.width()/2)
        tilt_error = min(max_step, con_rect[2][1]-img.height()/2)
        pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
        if abs(pan_output) + abs(tilt_output) < total_error:
            position = NowPosition.DR
    elif position == NowPosition.DR:
        pan_error = min(max_step, con_rect[3][0]-img.width()/2)
        tilt_error = con_rect[3][1]-img.height()/2
        pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
        if abs(pan_output) + abs(tilt_output) < total_error:
            position = NowPosition.DL
    elif position == NowPosition.DL:
        pan_error = con_rect[0][0]-img.width()/2
        tilt_error = con_rect[0][1]-img.height()/2
        pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
        if abs(pan_output) + abs(tilt_output) < total_error:
            position = NowPosition.UL
            Alarm()
            state = MachineState.RESET
    return pan_current, tilt_current

def button_read():
    # 在这里插入按键的读入程序
    pause = INpause
    key1 = pin1.value()
    key2 = pin2.value()
    key3 = pin3.value()
    key4 = pin6.value()

    # 等待一小段时间，例如20毫秒，以进行按键消抖
    utime.sleep_ms(20)

    # 再次读取按键状态A
    key1_debounced = pin1.value()
    key2_debounced = pin2.value()
    key3_debounced = pin3.value()
    key4_debounced = pin6.value()

    # 检测按键状态是否稳定，如果稳定则进行状态切换
    if key1 == key1_debounced == 0:
        new_state = MachineState.RESET
    elif key2 == key2_debounced == 0:
        new_state = MachineState.TRACING
    elif key3 == key3_debounced == 0:
        new_state = MachineState.TRACKING
    else:
        new_state = state
    # 检测第四个按键，并进行暂停/继续操作
    if key4 == key4_debounced == 0:
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
    img = sensor.snapshot()  # Take a picture and return the image.
    # initial stare
    state = MachineState.RESET
    ## state of Travel rect
    state_Trace = TraceState.RESET
    ### state of position
    position = NowPosition.UL
    # 默认非暂停
    INpause = False
    pan_current, tilt_current, pan_out, tilt_out = servoturn(0, 0, 90, 90)
    return pan_current, tilt_current

LowLimit = 60
LargeLimit = 120

def constraint(target):
    if target < LowLimit or target > LargeLimit:
        if target < LowLimit:
            target = LowLimit
        if target > LargeLimit:
            target = LargeLimit
    return target

def servoturn(pan_error, tilt_error, pan, tilt):

    pan_output = pan_pid.get_pid(pan_error, 1)/2
    tilt_output = tilt_pid.get_pid(tilt_error, 1)/2

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


def doTraceBlackLine(pan_current, tilt_current):
    global state_Trace
    global position
    img = sensor.snapshot()  # Take a picture and return the image.
    # Find the UpLeft angle of the black rectangle
    black_rect = img.find_rects(threshold=15000, roi=area)
    if black_rect:
        black_rect = find_max(black_rect)
        # 找到黑色矩形，设定标志为True
        # 在图像上绘制矩形及中心点
        img.draw_rectangle(black_rect.rect(), color = (255,255,255))
        if state_Trace == TraceState.RESET:
            state_Trace = TraceState.FindAngle
        elif state_Trace == TraceState.FindAngle:
            pan_error = black_rect.x()-img.width()/2
            tilt_error = black_rect.y()-img.height()/2

            pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
            if abs(pan_output) + abs(tilt_output) < total_error:
                state_Trace = TraceState.Travel
                position = NowPosition.UL
        elif state_Trace == TraceState.Travel:
            if position == NowPosition.UL:
                # 目标点的距离
                pan_error = black_rect.corners()[1][0]-img.width()/2
                tilt_error = black_rect.corners()[1][1]-img.height()/2

                pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
                if abs(pan_output) + abs(tilt_output) < total_error:
                    position = NowPosition.UR
            elif position == NowPosition.UR:
                pan_error = black_rect.corners()[2][0]-img.width()/2
                tilt_error = black_rect.corners()[2][1]-img.height()/2

                pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
                if abs(pan_output) + abs(tilt_output) < total_error:
                    position = NowPosition.DR
            elif position == NowPosition.DR:
                # 目标点的距离
                pan_error = black_rect.corners()[3][0]-img.width()/2
                tilt_error = black_rect.corners()[3][1]-img.height()/2

                pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
                if abs(pan_output) + abs(tilt_output) < total_error:
                    position = NowPosition.DL
            elif position == NowPosition.DL:
                # 目标点的距离
                pan_error = black_rect.corners()[0][0]-img.width()/2
                tilt_error = black_rect.corners()[0][1]-img.height()/2

                pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
                if abs(pan_output) + abs(tilt_output) < total_error:
                    position = NowPosition.UL
        elif state_Trace == TraceState.Finish:
            Alarm()
            state = MachineState.RESET
    return pan_current, tilt_current


def Alarm():
    print("Alarm rings!")
    return


def doTrackGreenPoint(pan_current, tilt_current):
    img = sensor.snapshot()  # Take a picture and return the image.
    blobs = img.find_blobs([green_threshold])
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
        print("Mission 1 Reseting")
        pan_current, tilt_current = doReset()
    elif state == MachineState.TRACING:
        print("Mission 2 squre")
        pan_current, tilt_current = doTask2(pan_current, tilt_current)
    elif state == MachineState.TRACKING:
        print("Mission 3 rectangle")
        pan_current, tilt_current = doTraceBlackLine(pan_current, tilt_current)
#        pan_current, tilt_current = doTrackGreenPoint(pan_current, tilt_current)
