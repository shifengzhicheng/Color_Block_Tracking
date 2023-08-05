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
pan_pid = PID(p=0.08, i=0.00015, d=0.11,imax=90)  # 在线调试使用这个PID
tilt_pid = PID(p=0.08, i=0.00015,d=0.11, imax=90)  # 在线调试使用这个PID
pan_current = 90
tilt_current = 90
# define sensor
sensor.reset()  # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565)  # use RGB565.
sensor.set_framesize(sensor.QQVGA)  # use QQVGA for speed.
sensor.skip_frames(10)  # Let new settings take affect.
sensor.set_auto_whitebal(False)  # turn this off.
sensor.set_brightness(3)   #设置亮度
sensor.set_contrast(3) #对比度
#sensor.set_auto_exposure(False, 10000)
# define alarm

# define color
red_threshold2 = (62, 72, 19, 36, -7, 127)
red_threshold3 = (65, 76, 20, 57, -10, 8)
red_threshold1 = (41, 43, 5, 6, -8, 10)
red_threshold = [red_threshold3,red_threshold2,red_threshold1]
green_threshold1 = (29, 76, -48, -11, -14, 43)
green_threshold2 = (65, 84, -21, -5, 0, 8)
green_threshold3 = (68, 75, -17, -3, -3, 8)

green_threshold = [green_threshold1,green_threshold2,green_threshold3]
black_threshold = (0, 50, 0, 30, 0, 30)

# define global const

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
    CE = 5

# initial stare
state = MachineState.RESET
## state of Travel rect
state_Trace = TraceState.RESET
### state of position
position = NowPosition.UL
# 默认非暂停
INpause = False

# 根据button读入进行state跳转

iterator = 1
cx = 90
cy = 90

ul_x = 104
ul_y = 104

dx = (ul_x-cx)/100
dy = (ul_y-cy)/100
i_1 =96.5
i_2 = 96
i_3 = 92
i_4 = 97

def doTask2():
    global position
    global state
    global iterator
    if position == NowPosition.UL:
        servoturn(0, 0, cx+5*iterator*dx, cy+5*iterator*dy)    
        iterator += 1
        time.sleep(0.1)
        if iterator == 20:
            position = NowPosition.UR
            iterator = 1
    elif position == NowPosition.UR:
        servoturn(0, 0, ul_x-2*iterator*dx, ul_y-0.18*iterator*dy)
        iterator += 1
        time.sleep(0.05)
        if iterator == i_1:
            position = NowPosition.DR
            iterator = 1
    elif position == NowPosition.DR:
        servoturn(0, 0, ul_x-2*i_1*dx, ul_y-2*iterator*dy)
        iterator += 1
        time.sleep(0.05)
        if iterator == i_2:
            position = NowPosition.DL
            iterator = 1
    elif position == NowPosition.DL:
        servoturn(0, 0, ul_x-i_1*2*dx+2*iterator*dx, ul_y-2*i_2*dy-0.03*iterator*dy)
        iterator += 1
        time.sleep(0.05)
        if iterator == i_3:
            iterator = 1
            position = NowPosition.Finish
    elif position == NowPosition.Finish:
        servoturn(0, 0, ul_x-0.2-0.1*iterator*dx, ul_y-2*i_2*dy+2*iterator*dy)
        iterator += 1
        time.sleep(0.05)
        if iterator == i_4:
            position == NowPosition.CE
            Alarm()
            iterator = 1
            state = MachineState.RESET

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


def find_r(rects):
    max_s = 120000
    min_s = 40000
    for r in rects:
        if r[4] < max_s and r[4] > min_s:
            ans = r
            return ans

def doReset():
    # 初始化
    global state,state_Trace,position,INpause,iterator
    img, red_point, black_rect = findtwo()

    # 找到黑色矩形和红点
    # 在图像上绘制矩形及中心点
    # initial stare
    iterator = 1
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

max_step = 6
def constrait_error(pan_error, tilt_error):
    if tilt_error == 0 and pan_error != 0:
        pan_error = pan_error/abs(pan_error)*min(abs(pan_error), max_step)
    elif pan_error != 0 and tilt_error != 0:
        ratio = pan_error/tilt_error
        if abs(pan_error) > abs(tilt_error):
            pan_error = pan_error/abs(pan_error)*min(abs(pan_error), max_step)
            tilt_error = pan_error/ratio
        else:
            tilt_error = tilt_error/abs(tilt_error)*min(max_step, abs(tilt_error))
            pan_error =  tilt_error*ratio
    elif tilt_error != 0 and pan_error == 0:
        tilt_error = tilt_error/abs(tilt_error)*min(abs(tilt_error), max_step)
    return pan_error, tilt_error

def servoturn(pan_error, tilt_error, pan, tilt):

    pan_error, tilt_error = constrait_error(pan_error, tilt_error)

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

def findtwo():
    img = sensor.snapshot()  # Take a picture and return the image.
    img.lens_corr(2)
    black_rects = img.find_rects(threshold=5000)
    #red_points = img.find_blobs(red_threshold, merge = 1)
    # 找到黑色矩形和红点
    # 在图像上绘制矩形及中心点
    '''
    if black_rects:
        max_r = find_r(black_rects)
        if max_r:
            img.draw_rectangle(max_r.rect(),color = (255,255,255))
            area = (round(max_r.x()+max_r.w()/2)+30,round(max_r.y()+max_r.h()/2)+20,280,240)
            sensor.set_framesize(sensor.VGA)
            sensor.set_windowing(area)
    '''
    if black_rects:
        black_rect = find_r(black_rects)
        if  black_rect:

            #img.draw_cross(red_point.cx(),red_point.cy(),color = (255,0,0))
            img.draw_rectangle(black_rect.rect())
            #print("red_point", red_point)
            print("black_rect", black_rect.magnitude())           
            #area = (round(black_rect.x()),round(black_rect.y()),round(black_rect.w()),round(black_rect.h()))
            #print(area[0],area[1],area[2],area[3])
            area = (30,10,100,100)                       
            sensor.set_windowing(area)
            #print(img.width()/2,img.height()/2)
        return img, None, black_rect
    else:
        return img, None, None


total_error = 3
red_x = 48
red_y = 42
def doTraceBlackLine(pan_current, tilt_current):
    global state_Trace
    global position
    #img = sensor.snapshot()  # Take a picture and return the image.
    #img.lens_corr(1.8)
    ## Find the UpLeft angle of the black rectangle
    #black_rect = img.find_rects(threshold=15000)
    #red_points = img.find_blobs([green_threshold,green_threshold2], merge = 1)
    img, red_point, black_rect = findtwo()
    # 找到黑色矩形和红点
    # 在图像上绘制矩形及中心点


    if  black_rect and not INpause:
        img.draw_cross(black_rect.corners()[0][0],black_rect.corners()[0][1],color = (255,0,0))#red
        img.draw_cross(black_rect.corners()[1][0],black_rect.corners()[1][1],color = (0,255,0))#green
        img.draw_cross(black_rect.corners()[2][0],black_rect.corners()[2][1],color = (0,0,255))#blue
        img.draw_cross(black_rect.corners()[3][0],black_rect.corners()[3][1],color = (255,255,255))#white
        black_rect_cx = (black_rect.corners()[0][0]+black_rect.corners()[1][0]+black_rect.corners()[2][0]+black_rect.corners()[3][0])/4
        black_rect_cy = (black_rect.corners()[0][1]+black_rect.corners()[1][1]+black_rect.corners()[2][1]+black_rect.corners()[3][1])/4

        if state_Trace == TraceState.RESET:
            state_Trace = TraceState.FindAngle
        elif state_Trace == TraceState.FindAngle:
            pan_error = black_rect.corners()[0][0]-red_x
            tilt_error = black_rect.corners()[0][1]-red_y

            pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)

            if abs(pan_error) + abs(tilt_error) < total_error:
                state_Trace = TraceState.Travel
                position = NowPosition.UL
        elif state_Trace == TraceState.Travel:
            if position == NowPosition.UL:
                # 目标点的距离

                pan_error = black_rect.corners()[3][0]-red_x
                tilt_error = black_rect.corners()[3][1]-red_y
                pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
                if abs(pan_error) + abs(tilt_error) < total_error:
                    position = NowPosition.UR
            elif position == NowPosition.UR:

                pan_error = black_rect.corners()[2][0]-red_x
                tilt_error = black_rect.corners()[2][1]-red_y

                pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
                if abs(pan_error) + abs(tilt_error) < total_error:
                    position = NowPosition.DR
            elif position == NowPosition.DR:

                # 目标点的距离
                pan_error = black_rect.corners()[1][0]-red_x
                tilt_error = black_rect.corners()[1][1]-red_y

                pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
                if abs(pan_error) + abs(tilt_error) < total_error:
                    position = NowPosition.DL
            elif position == NowPosition.DL:

                # 目标点的距离
                pan_error = black_rect.corners()[0][0]-red_x
                tilt_error = black_rect.corners()[0][1]-red_y

                pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)
                if abs(pan_error) + abs(tilt_error) < total_error:
                    position = NowPosition.UL

        elif state_Trace == TraceState.Finish:
            Alarm()
            state = MachineState.RESET

    return pan_current, tilt_current


def Alarm():
    print("Alarm rings!")
    return


#def doTrackGreenPoint(pan_current, tilt_current):
#    img = sensor.snapshot()  # Take a picture and return the image.
#    blobs = img.find_blobs([green_threshold])
#    if blobs and not INpause:
#        max_blob = find_max(blobs)
#        pan_error = max_blob.cx()-img.width()/2
#        tilt_error = max_blob.cy()-img.height()/2
#        img.draw_rectangle(max_blob.rect())  # rect
#        img.draw_cross(max_blob.cx(), max_blob.cy())  # cx, cy

#        pan_current, tilt_current,pan_output,tilt_output = servoturn(pan_error, tilt_error, pan_current, tilt_current)

#        print("pan_current: ", pan_current)
#        print("tilt_current: ", tilt_current)
#        print(pan_output,tilt_output)

#        if abs(pan_output) + abs(tilt_output) < 0.1:
#            Alarm()
#    return pan_current, tilt_current


# 在这里进入loop
while True:
    clock.tick()  # Track elapsed milliseconds between snapshots().
    state, INpause = button_read()  # change if button signal comes

# 这里开始进行状态跳转执行
    if state == MachineState.RESET:
        print("RESET")
        pan_current, tilt_current = doReset()
    elif state == MachineState.TRACING:
        print("doTask2")
        doTask2()
    elif state == MachineState.TRACKING:
        print("TRACE BLACK")
        pan_current, tilt_current = doTraceBlackLine(pan_current, tilt_current)
        #pan_current, tilt_current = doTrackGreenPoint(pan_current, tilt_current)
