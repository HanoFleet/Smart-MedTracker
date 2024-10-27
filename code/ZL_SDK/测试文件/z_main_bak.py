'''
z_main是开机程序，即树梅派开机后会自动运行此程序，正常开机后不能直接运行，先关闭程序（运行killmain.sh脚本）
在打开程序，修改socket端口，将1314修改为其他值。
在终端输入 sudo gedit /etc/init.d/init-app 可以设置开机自启脚本

'''

# -*- coding:utf-8 -*-
#导入模块
import sys
sys.path.append('/home/pi/Desktop/ZL-PI/factory_code/')
import cv2   
import numpy as np
import time
import threading

# 导入flask框架所需要的包，模块
from flask import Flask, request, render_template, Response
import time

# 固定写法
app = Flask(__name__)

import ZL_SDK.z_led as myLed
import ZL_SDK.z_beep as myBeep
import ZL_SDK.Z_UartServer as myUart
import ZL_SDK.ActionGroupControl as AGC
import ZL_SDK.Z_SocketServer as mySocket

import Camera

import AI_Functions.z_kinematics as kms
import AI_Functions.Color_Follow as ColorF
import AI_Functions.Face_Follow as FaceF
import AI_Functions.Color_Sort as ColorSort
import AI_Functions.QRD_Sort as QRDSort
import AI_Functions.Wood_Stack as WoodStack
import AI_Functions.Waste_Sort as WasteSort

width = 320
hight = 240

#全局变量定义
systick_ms_bak = 0
ai_mode = 0
clamp_flag = True
webpage_get_ok = 0                             # 用于判断网页发送过来的数据格式    
webpage_receive_buf = ''                       # 用于接收保存网页发送过来的数据
response_mode_flag = 0                         # 渲染网页时要用到的机器人模式：0是遥控模式，1是智能模式

voice_dist = {           
            '$RESET!':'#012P1507T1000!','$AI-FOLLOW!':'#012P1511T1000!','$AI-QRDRUN!':'#012P1517T1000!',
            '$AI-FACE!':'#012P1514T1000!','$AI-SORT!':'#012P1515T1000!','$AI-STACK!':'#012P1516T1000!','$AI-GarSORT!':'#012P1518T1000!',
            '$red!':'#012P1523T1000!','$green!':'#012P1524T1000!','$blue!':'#012P1525T1000!',
            '$LR!':'#012P1500T1000!','$LG!':'#012P1500T1000!','$LB!':'#012P1500T1000!',
            '$R1!':'#012P1500T1000!','$R2!':'#012P1500T1000!','$R3!':'#012P1500T1000!',        
            }

ai_dist = {'$RESET!':0,
           '$AI-FOLLOW!':1,         
           '$AI-FACE!':2,
           '$AI-QRDRUN!':3,
           '$AI-SORT!':4,
           '$AI-STACK!':5,
           '$AI-GarSORT!':6          
            }

move_dist = ['$LR!','$LG!','$LB!','$R1!','$R2!','$R3!']

color_distss = ['red','green','blue']

color_str = color_distss[1]

webpage_func_mode_dict = {
    'webpage_mode':{
                    'remote_mode': '遥控模式',
                    'ai_mode': '智能模式'
                    },
    'webpage_remote_button_dict':{
                                   'title': '树莓派机械臂',            
                                   'setting': '停止',
                                   'mode1':  '遥控模式',
                                   'mode2':  '智能模式',
                                   'self_cmd_text':  '自定义指令',
                                   'l1': '张开爪子',
                                   'l2': '爪子左转',
                                   'l3': '暂无功能',
                                   'r1': '闭合爪子',
                                   'r2': '爪子右转',
                                   'r3': '暂无功能',
                                   'lu': '1号舵机前',
                                   'ld': '1号舵机后',
                                   'll': '云台左转',
                                   'lr': '云台右转',
                                   'lst': '停止',
                                   'ru': '2号舵机前',
                                   'rd': '2号舵机后',
                                   'rl': '3号舵机前',
                                   'rr': '3号舵机后',
                                   'rst': '停止',
                                  },
    'webpage_ai_button_dict': {
                                   'title': '树莓派机械臂',
                                   'setting': '蜷缩/复位',
                                   'mode1':  '智能模式',
                                   'mode2':  '遥控模式',
                                   'self_cmd_text': '自定义指令',
                                   'l1': '人脸跟随',
                                   'l2': '颜色跟随',
                                   'l3': '颜色分拣',
                                   'r1': '垃圾分类',
                                   'r2': '智能码垛',
                                   'r3': '二维码分拣',
                                   'lu': '1号舵机前',
                                   'ld': '1号舵机后',
                                   'll': '云台左转',
                                   'lr': '云台右转',
                                   'lst': '停止',
                                   'ru': '张开爪子',
                                   'rd': '闭合爪子',
                                   'rl': '爪子左转',
                                   'rr': '爪子右转',
                                   'rst': '停止',
                              },
    'webpage_remote_cmd_dict':{
                                   'title': None,            
                                   'setting': '$DST!',
                                   'self_cmd_text': None,
                                   'l1': '#005P1200T6000!',
                                   'l2': '#004P0600T6000!',
                                   'l3': '',
                                   'r1': '#005P1780T6000!',
                                   'r2': '#004P2400T6000!',
                                   'r3': '',
                                   'lu': '#001P0600T6000!',
                                   'ld': '#001P2400T6000!',
                                   'll': '#000P2400T6000!',
                                   'lr': '#000P0600T6000!',
                                   'lst': '$DST!',
                                   'ru': '#002P2400T6000!',
                                   'rd': '#002P0600T6000!',
                                   'rl': '#003P0600T6000!',
                                   'rr': '#003P2400T6000!',
                                   'rst': '$DST!',
                                  },
    'webpage_ai_cmd_dict': {
                                   'title': None,
                                   'setting': '$RESET!',
                                   'self_cmd_text': None,
                                   'l1': '$AI-FACE!',
                                   'l2': '$AI-FOLLOW!',
                                   'l3': '$AI-SORT!',
                                   'r1': '$AI-GarSORT!',
                                   'r2': '$AI-STACK!',
                                   'r3': '$AI-QRDRUN!',
                                   'lu': '#001P0600T6000!',
                                   'ld': '#001P2400T6000!',
                                   'll': '#000P2400T6000!',
                                   'lr': '#000P0600T6000!',
                                   'lst': '$DST!',
                                   'ru': '#005P1200T6000!',
                                   'rd': '#005P1780T6000!',
                                   'rl': '#004P0600T6000!',
                                   'rr': '#004P2400T6000!',
                                   'rst': '$DST!',
                              }
}


# 用于接收网页发送过来的按钮对应数据
webpage_button_dict = {
                       'setting': {'recv_cmd': None},
                       'l1': {'recv_cmd': None},
                       'l2': {'recv_cmd': None},
                       'l3': {'recv_cmd': None},
                       'r1': {'recv_cmd': None},
                       'r2': {'recv_cmd': None},
                       'r3': {'recv_cmd': None},
                       'lu': {'recv_cmd': None},
                       'ld': {'recv_cmd': None},
                       'll': {'recv_cmd': None},
                       'lr': {'recv_cmd': None},
                       'lst': {'recv_cmd': None},
                       'ru': {'recv_cmd': None},
                       'rd': {'recv_cmd': None},
                       'rl': {'recv_cmd': None},
                       'rr': {'recv_cmd': None},
                       'rst': {'recv_cmd': None},
                       'self_cmd_text': {'recv_cmd': None},
                      }

# 进入的首个网页
@app.route('/', methods=['GET','POST'])
def webpage_main():
    global webpage_get_ok, webpage_receive_buf, response_mode_flag
    webpage_mode = 0
    for k in webpage_button_dict:
        if request.form.get(k):
            if request.form.get(k) == webpage_func_mode_dict['webpage_remote_button_dict'][k]:
                webpage_button_dict[k]['recv_cmd'] = webpage_func_mode_dict['webpage_remote_cmd_dict'][k]
            elif request.form.get(k) == webpage_func_mode_dict['webpage_ai_button_dict'][k]:
                webpage_button_dict[k]['recv_cmd'] = webpage_func_mode_dict['webpage_ai_cmd_dict'][k]
            if webpage_button_dict[k]['recv_cmd']:
                if k == 'self_cmd_text':
                    webpage_button_dict[k]['recv_cmd'] = request.form.get(k)
                if webpage_get_ok == 0:
                    webpage_receive_buf = webpage_button_dict[k]['recv_cmd']
                    uart_get_ok = 0
                    if webpage_mode == 0:
                        if webpage_receive_buf.find('{') >= 0:
                            webpage_mode = 1
                        elif webpage_receive_buf.find('$') >= 0:
                            webpage_mode = 2
                        elif webpage_receive_buf.find('#') >= 0:
                            webpage_mode = 3
                    if webpage_mode == 1:
                        if webpage_receive_buf.find('}') >= 0:
                            uart_get_ok = 1
                            webpage_mode = 0
                    elif webpage_mode == 2:
                        if webpage_receive_buf.find('!') >= 0:
                            uart_get_ok = 2
                            webpage_mode = 0
                    elif webpage_mode == 3:
                        if webpage_receive_buf.find('!') >= 0:
                            uart_get_ok = 3
                            webpage_mode = 0
                    webpage_get_ok = uart_get_ok
    if request.form.get('webpage_func_mode') == webpage_func_mode_dict['webpage_mode']['remote_mode']:
        response_mode_flag = 0
    elif request.form.get('webpage_func_mode') == webpage_func_mode_dict['webpage_mode']['ai_mode']:
        response_mode_flag = 1
        
    if response_mode_flag:
        response_dict = webpage_func_mode_dict['webpage_ai_button_dict']
    else:
        response_dict = webpage_func_mode_dict['webpage_remote_button_dict']
    return render_template("index.html",response_dict=response_dict)   # 返回网页

# 要执行的flask线程函数
def flask_thread():
    app.run(host='192.168.12.1', port=8090)   # 该地址为树莓派的IP地址，端口任意设（也可以用本机的IP地址:127.0.0.2,port:8081）

# 对网页发过来的数据进行处理
def loop_webpage():
    global webpage_get_ok, webpage_receive_buf
    # 如果网页发送过来了数据
    if webpage_get_ok:
        myUart.uart_get_ok = webpage_get_ok
        # 将网页数据赋值给串口数据处进行处理
        myUart.uart_receive_buf = webpage_receive_buf
        # 然后将网页接收数据赋值为空
        webpage_receive_buf = ''
        webpage_get_ok = 0

#LED灯循坏
def loop_led():
    global systick_ms_bak
    if(int((time.time() * 1000))- systick_ms_bak > 500):
        systick_ms_bak = int((time.time() * 1000))
        myLed.flip()

#串口检测
def loop_uart():
    global voice_flag   
    if myUart.uart_get_ok == 2:
        myBeep.beep(1,0.1)
        parse_cmd(myUart.uart_receive_buf)
        AGC.groups_parse_cmd(myUart.uart_receive_buf)
        myUart.uart_receive_buf = ''
        myUart.uart_get_ok = 0
    elif myUart.uart_get_ok == 1 or myUart.uart_get_ok == 3:        
        print(myUart.uart_receive_buf)                 
        myUart.uart_send_str(myUart.uart_receive_buf)
        myUart.uart_receive_buf = ''
        myUart.uart_get_ok = 0

#网络连接服务
def loop_socket():  
    if mySocket.socket_get_ok:
        myUart.uart_get_ok =  mySocket.socket_get_ok
        myUart.uart_receive_buf = mySocket.socket_receive_buf
        mySocket.socket_receive_buf = ''
        mySocket.socket_get_ok = 0

#串口指令解析,智能转功能       
def parse_cmd(myStr):
    global color_str,clamp_flag
    try:
        WoodStack.stack_count = 0
        voice_broadcast(myStr)
        
        time.sleep(0.1)
        if myStr in move_dist:            
            move_wood(myStr)
        if myStr in ai_dist:
            clamp_flag = True            
            ai_mdoe(myStr)
        Str = myStr[1:-1]
        if Str in color_distss:
            color_str = Str
            #print('color_str:',color_str)
    except Exception as e:
        print(e)
        pass
   
#MP3播报
def voice_broadcast(myStr):
    mpStr = voice_dist[myStr]
    print(mpStr)
    myUart.uart_send_str(mpStr)   

#智能模式解析,姿态切换
def ai_mdoe(myStr):
    global ai_mode
    ai_mode = ai_dist[myStr]
    if  ai_mode == 0 or ai_mode == 1 or ai_mode == 2:
        myUart.uart_send_str('{#000P1500T1000!#001P2150T1000!#002P2300T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!}')
        time.sleep(1)
    elif ai_mode == 3 or ai_mode == 4 or ai_mode == 5 or ai_mode == 6:
        myUart.uart_send_str('{#000P1500T1000!#001P1500T1000!#002P1800T1000!#003P0610T1000!#004P1500T1000!#005P1500T1000!}')
        time.sleep(1)
   
def move_wood(myStr):
    global clamp_flag
    moveStr = myStr
    print(moveStr)
    if AGC.group_ok:
        if moveStr == '$LB!':
            if clamp_flag:
                AGC.groups_parse_cmd('$DGT:25-30,1!')
            else:
                AGC.groups_parse_cmd('$DGT:8-12,1!')
            clamp_flag = not clamp_flag
        elif moveStr == '$LG!':
            if clamp_flag:
                AGC.groups_parse_cmd('$DGT:31-36,1!')
            else:
                AGC.groups_parse_cmd('$DGT:13-18,1!')
            clamp_flag = not clamp_flag
        elif moveStr == '$LR!':
            if clamp_flag:
                AGC.groups_parse_cmd('$DGT:37-42,1!')
            else:
                AGC.groups_parse_cmd('$DGT:19-24,1!')
            clamp_flag = not clamp_flag
            
        elif moveStr == '$R3!':
            if clamp_flag:
                AGC.groups_parse_cmd('$DGT:61-66,1!')
            else:
                AGC.groups_parse_cmd('$DGT:43-48,1!')
            clamp_flag = not clamp_flag
        
        elif moveStr == '$R2!':
            if clamp_flag:
                AGC.groups_parse_cmd('$DGT:67-72,1!')
            else:
                AGC.groups_parse_cmd('$DGT:49-54,1!')
            clamp_flag = not clamp_flag
            
        elif moveStr == '$R1!':
            if clamp_flag:
                AGC.groups_parse_cmd('$DGT:73-78,1!')
            else:
                AGC.groups_parse_cmd('$DGT:55-60,1!')
            clamp_flag = not clamp_flag

#ai模式
def loop_ai_run(frame):
    global ai_mode,color_str
    if ai_mode == 0:
        pass
    elif ai_mode == 1:
        ColorF.frame = frame
        ColorF.color_detect(color_str)
    elif ai_mode == 2:
        FaceF.frame = frame
        FaceF.face_detect()        
    elif ai_mode == 3:
        QRDSort.frame = frame
        QRDSort.qrd_detect()        
    elif ai_mode == 4:
        ColorSort.frame = frame
        ColorSort.color_sort()
    elif ai_mode == 5:
        WoodStack.frame = frame
        WoodStack.wood_stack()
    elif ai_mode == 6:
        WasteSort.frame = frame
        WasteSort.waste_sort()

# 创建flask线程
flask_task = threading.Thread(target=flask_thread, args=())

#大循环
if __name__ == '__main__':
    # 开启flask线程
    flask_task.start()
    time.sleep(5)
    myLed.setup_led()        #led初始化
    myBeep.setup_beep()      #蜂鸣器初始化
    kms.setup_kinematics(110,105,75,180)
    myUart.setup_uart(115200) #设置串口
    mySocket.setup_socket(1314)
    cam = Camera.Camera()  #摄像头库实例化    
    cam.camera_open()      #打开摄像头
    #读取动作组文件。输入参数：动作组文件位置   
    AGC.Group_read('/home/pi/Desktop/ZL-PI/factory_code/Jibot3_Group/Jibot3_Group-pi.ini')
    frame = None
    ResetStr = AGC.myList[1] #蜷缩姿态
    print('dongzuozu',ResetStr)
    myUart.uart_send_str(ResetStr)
    myBeep.beep(3,0.1)
    myUart.uart_send_str('#012P1598T1000!')
    try:
        while 1:
            frame = cam.frame.copy() 
            loop_led()
            loop_uart()
            loop_socket()
            loop_webpage()      # 对网页发送过来的数据进行处理    
            AGC.loop_group()
            if frame is not None:
                loop_ai_run(frame)
                
    except KeyboardInterrupt:
        pass
        
        
        