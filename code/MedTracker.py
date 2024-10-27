#位置  软膏   风油精  滴眼液   
import sys
sys.path.append('/home/pi/Desktop/ZL-PI/factory_code/')
import cv2
import time
import threading
import numpy as np
from math import *
import Camera
import socket
import ZL_SDK.Z_UartServer as myUart
import ZL_SDK.z_beep as myBeep
import AI_Functions.z_kinematics as kms

frame = None

#画面宽度和高度
img_w = 640
img_h = 480

#定义全局变量
systick_ms_bak = 0
systick_ms_bak_zhuan = 0
carry_step = 0
next_time = 50
zhuan_flag = True
servo_yuntai_bias = 0
run_flag = True
testStr = ''

#图像处理参数
Running = True
c_x = 0
c_y = 0
c_h = 0
c_w = 0
pos_y = 0


grab_success = False  # 标记是否抓取成功

init_position_commands = []
init_servo_yuntai_bias = []
init_tag=[]

current_position_index = 0
total_positions = 0
'''
color_dist = {
             'red':   {'Lower': np.array([0, 80, 80]), 'Upper': np.array([10, 255, 255])},
             'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
             'blue':  {'Lower': np.array([90, 50, 50]), 'Upper': np.array([120, 255, 255])},
             'gray':  {'Lower': np.array([0,0,30]),'Upper':np.array([255,40,80])}
             }
'''

#记录识别到颜色的坐标，记录两个值，
tag_point_cur = [[0,0],[0,0],[0,0]]
tag_point_lst = [[0,0],[0,0],[0,0]]
#判断两次值的偏差确定稳定性,记录稳定性的次数
tag_count = [0,0,0]
#记录放置点的y坐标
tag_pos_y = 1
ls_zhua = -1
tag_mb=-1

init_position_commands = []
init_servo_yuntai_bias = []

def get_user_choice():
    global init_position_commands, init_servo_yuntai_bias
    
    # 提示用户输入0、1、2中的任意多个数字
    #user_input = input("请输入0, 1, 2中的任意多个数字，用逗号分隔: ")
    
    # 将输入的字符串分割成列表，并将每个元素转换为整数
    #choices = list(map(int, user_input.split(',')))
    choices = [1]
    # 遍历用户输入的数字
    for choice in choices:
        if choice == 0:
            # 如果选择的是0，向数组添加相应内容
            init_position_commands.append("#000P1500T1000!#001P1900T1000!#002P2000T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!")
            init_servo_yuntai_bias.append(20)
            init_tag.append(0)
        elif choice == 1:
            # 如果选择的是1，向数组添加相应内容
            init_position_commands.append("#000P1500T1000!#001P1900T1000!#002P2000T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!")
            init_servo_yuntai_bias.append(0)
            init_tag.append(1)
        elif choice == 2:
            # 如果选择的是2，向数组添加相应内容
            init_position_commands.append("#000P1500T1000!#001P1900T1000!#002P2000T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!")
            init_servo_yuntai_bias.append(-20)
            init_tag.append(2)
        else:
            print(f"无效输入: {choice}")
    
    # 打印结果
    

def move_to_next_init_position():
    global current_position_index,Running,run_flag,servo_yuntai_bias,tag_mb
    if current_position_index >= total_positions:
        return False  # 所有位置已遍历完，返回 False，表示结束

    # 获取当前的初始位置的指令
    position_command = init_position_commands[current_position_index]
    print(f"移动到初始位置 {current_position_index + 1}")
    myUart.uart_send_str(position_command)  # 通过串口发送移动指令
    time.sleep(2)  # 等待机械臂到位
    
    servo_yuntai_bias = init_servo_yuntai_bias[current_position_index]
    tag_mb = init_tag[current_position_index]
    testStr ='#000P{0:0>4d}T{1:0>4d}!'.format(int(1500-2000 * servo_yuntai_bias / 270), 200)
    myUart.uart_send_str(testStr)

    time.sleep(20)  # 等待机械臂到位
    current_position_index += 1  # 更新到下一个初始位置
    print("Running = True")
    Running = True
    run_flag = True
    return True  # 表示还有位置可遍历

#颜色识别，获取木块坐标位置
def receive_data_from_pc():
    global c_x, c_y, c_w, c_h, c_angle ,pos_y, grab_success,run_flag,Running, testStr,ls_zhua,tag_mb
    # 创建一个 socket 服务器，监听端口 9998
    #time.sleep(0.2)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('', 9996))  # '' 表示监听所有可用的网络接口
    server_socket.listen(1)
    print("等待接收从电脑发送的中心坐标和边界框尺寸数据...")

    conn, addr = server_socket.accept()
    print(f"接收到来自 {addr} 的连接")

    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            #print(f"接收到数据, Running = {Running}")
            if Running == True:

                # 解码并处理接收到的数据
                message = data.decode()
                print(f"shuju:{message}")
                center_x, center_y, width, height, tag= map(int, message.split(','))
                print(f"接收到的中心坐标：x={center_x}, y={center_y}, 宽度={width}, 高度={height} , 类别={tag}")

                tag_point_cur[tag][0] = c_x
                tag_point_cur[tag][1] = c_y
                if tag_count[tag]:
                    if abs(tag_point_cur[tag][0]-tag_point_lst[tag][0]) < 10 and abs(tag_point_cur[tag][1]-tag_point_lst[tag][1]) < 10:
                        tag_count[tag] = tag_count[tag] + 1
                    else:
                        tag_count[tag] = 0
                else:
                    tag_count[tag] = tag_count[tag] + 1
                print(tag)
                tag_point_lst[tag][0] = c_x
                tag_point_lst[tag][1] = c_y
                
                if tag_count[tag] == 4 and tag == tag_mb and Running == True and tag != ls_zhua:
                    ls_zhua = tag
                    myBeep.beep(1, 0.1)
                    pos_y = tag_pos_y  
                    c_x, c_y, c_w, c_h = center_x, center_y, width, height
                    c_angle = 0
                    for j in range(len(tag_count)):
                        tag_count[j] = 0
                    run_flag = False
                    Running = False
                    process_data()
                    
                    for j in range(len(tag_count)):
                        tag_count[j] = 0

    except Exception as e:
        print("接收数据时发生错误：", e)
    finally:
        conn.close()
        server_socket.close()


#处理画面坐标，得到的逆运动学坐标位置
def process_data():
    print("进入 process_data")
    global kms_x, kms_y, kms_z, c_angle, servo_yuntai, servo_zhuazi, servo_yuntai_bias, c_x, c_y, c_h, c_w, Running, systick_ms_bak, next_time
    # 控制执行下列程序的时间
    if int((time.time() * 1000)) - systick_ms_bak > next_time:
        systick_ms_bak = int((time.time() * 1000))
        # 摄像头画面中心点和机械臂实际中心点偏差
        c_bias_x = 25
        c_bias_y = 0
        # 将画面坐标换算为逆运动学坐标，由于机械臂在旋转，这里计算的是机械臂坐标系旋转后的坐标位置
        kms_x_c = (c_x - img_w / 2 + c_bias_x) * 30 / 160
        kms_z_c = 280 - ((c_y - img_h / 2 + c_bias_y) * 30 / 160)  
        # 180 要改成 初始状态的z坐标
        
        # 计算实际坐标，机械臂坐标系旋转之前的坐标位置
        
        theta = servo_yuntai_bias / (180.0 / pi)            
        kms_y = 200
        kms_z = kms_z_c                  
        kms_y_c = (kms_y + kms_x_c * sin(theta)) / cos(theta)
        kms_x = kms_x_c * cos(servo_yuntai_bias / (180.0 / pi)) + kms_y_c * sin(servo_yuntai_bias / (180.0 / pi))
        
        print("机械臂坐标",kms_x,kms_y,kms_z)


        # 计算云台旋转角度
        if kms_x == 0 and kms_y != 0:
            theta6 = 0.0
        elif kms_y == 0 and kms_x > 0:
            theta6 = 90
        elif kms_y == 0 and kms_x < 0:
            theta6 = -90
        else:
            theta6 = atan(kms_x / kms_y) * 180.0 / pi
                  
        servo_yuntai = int(1500 - 2000.0 * theta6 / 270.0)
                          
        #c_angle = abs(c_angle)
        # c_angle = c_angle - servo_yuntai_bias                                  
        
        c_angle = 0

        if theta6 > 0:                        
            if c_angle < 45:
                servo_zhuazi = int(1500 - 2000.0 * (c_angle ) / 270.0)
            else:
                angle = 90 - (c_angle )
                servo_zhuazi = int(1500 + 2000.0 * (angle) / 270.0)
            print("++++++++++++++++++++++")
        else:
            if c_angle < 45:
                servo_zhuazi = int(1500 - 2000.0 * (c_angle ) / 270.0)
            else:
                angle = 90 - (c_angle )
                servo_zhuazi = int(1500 + 2000.0 * (angle) / 270.0)
            print("------------------------")
        debug = False
        if debug:
            print('kms_x:', kms_x, 'kms_y:', kms_y, 'kms_z:', kms_z)
            print('theta6', theta6)
            print('servo_yuntai_bias', servo_yuntai_bias)
            print('c_angle', c_angle)
            print('servo_yuntai:', servo_yuntai)
            print('servo_zhuazi:', servo_zhuazi)                       
            Str = "#000P{0:0>4d}T{0:0>4d}!#004P{1:0>4d}T{2:0>4d}!#005P1200T{2:0>4d}!".format(servo_yuntai, servo_zhuazi, 1000)
            myUart.uart_send_str(Str)
            time.sleep(3)
            kinematics_move(kms_x, kms_y, kms_z, 1000)
        else:
            next_time = 8000
            Running = False
            eEvent.set()
        c_x = 0
        c_y = 0
                    
#夹取木块
def scarry_wood():
    global grab_success,carry_step, Running, servo_yuntai, servo_zhuazi, index, servo_yuntai_bias, pos_y, run_flag
    while 1:
        eEvent.wait()             
        if carry_step == 0:
            print("夹取0")      
            myUart.uart_send_str("#005P1200T0500!")
            time.sleep(0.5)
            carry_step = 1
        elif carry_step == 1:
            print("夹取1")      
            
            Str = "#000P{0:0>4d}T{0:0>4d}!#004P{1:0>4d}T{2:0>4d}!#005P1200T{2:0>4d}!".format(servo_yuntai, servo_zhuazi, 1000)
            myUart.uart_send_str(Str)  
            time.sleep(0.5)
            carry_step = 2
                    
        #机械臂运行到目标位置
        elif carry_step == 2:
            print("夹取2")      
            
            kinematics_move(kms_x, kms_y + 10, kms_z, 2000)  # Z 是高度
            print(kms_x, kms_y, kms_z)
            time.sleep(2)
            carry_step = 3
        #闭合爪子   
        elif carry_step == 3:
            print("夹取3")      
            
            myUart.uart_send_str("#005P2000T1000!")
            time.sleep(1)
            carry_step = 4     
        #抬起机械臂 
        elif carry_step == 4:
            print("夹取4")      
            myUart.uart_send_str('#000P{0:0>4d}T1000!#001P1900T1000!#002P2000T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!'.format(int(1500+2000 * 0/ 270)))
            #kinematics_move(kms_x, kms_y - 100, kms_z, 1000)
            time.sleep(2)  
            carry_step = 5
        #旋转机械臂
        elif carry_step == 5:
            print("夹取5")      
            
            print("pos_y",pos_y)
            theta = atan(-200 / pos_y) * 180.0 / pi                                       
            servo_yuntai = int(1500 - 2000.0 * theta / 270.0)
            print('servo_yuntai:', servo_yuntai)
            myUart.uart_send_str('#000P{0:0>4d}T{1:0>4d}!#004P1500T{1:0>4d}!'.format(servo_yuntai, 1000))  
            time.sleep(1)
            carry_step = 6       
        #放置木块
        elif carry_step == 6:
            print("夹取6")      
            
            kinematics_move(-210,pos_y,10,2000)
            time.sleep(2)
            carry_step = 7
        #张开抓手
        elif carry_step == 7:
            print("夹取7")      
            
            myUart.uart_send_str('#005P1200T0500!')
            time.sleep(0.5)
            carry_step = 8        
        #抬起机械臂爪子，回到初始位置    
        elif carry_step == 8:
            print("夹取8")      
            
            kinematics_move(-210, pos_y, 50, 1000)  # Z 轴高度改变
            time.sleep(1)
            myUart.uart_send_str('#000P{0:0>4d}T1000!#001P1900T1000!#002P2000T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!'.format(int(1500+2000 * 0/ 270)))
            time.sleep(1)           
            carry_step = 9            
        elif carry_step == 9:    
            print("夹取9")      
                
            #myUart.uart_send_str('#000P{0:0>4d}T{1:0>4d}!'.format(int(1500-2000*(servo_yuntai_bias/270)),1000))
            time.sleep(2)
            print("抓完") 
            servo_yuntai_bias = 0      
            carry_step = 0
            eEvent.clear()
            grab_success = True
            
#逆运动学算法
def kinematics_move(x,y,z,mytime): 
    global servo_angle,servo_pwm
    if y < 0 :
        return 0
    #寻找最佳角度
    flag = 0
    my_min = 0
    for i in range(-135,0) :
        if 0 == kms.kinematics_analysis(x,y,z,i):
            if(i < my_min):
                my_min = i
            flag = 1
        
    #用3号舵机与水平最大的夹角作为最佳值
    if(flag) :
        kms.kinematics_analysis(x,y,z,my_min);
        testStr = '{'
        for j in range(0,4) :
            testStr += "#%03dP%04dT%04d!" % (j,kms.servo_pwm[j],mytime)
        testStr += '}'
        print(testStr)
        myUart.uart_send_str(testStr)
        return 1
    
    return 0

#搬运线程
eEvent = threading.Event()
th1 = threading.Thread(target = scarry_wood)
th1.setDaemon(True)    
th1.start()
receive_thread = threading.Thread(target=receive_data_from_pc, daemon=True)
receive_thread.start()


if __name__ == '__main__':
    

    myBeep.setup_beep()   
    myUart.setup_uart(115200)
    
    get_user_choice()
    print("init_position_commands:", init_position_commands)
    print("init_servo_yuntai_bias:", init_servo_yuntai_bias)
    total_positions =len(init_position_commands)

    kms.setup_kinematics(110,105,75,185)
    
    myUart.uart_send_str("#255P1500T1000!")
    time.sleep(2)
    myBeep.beep(3,0.1)
    #myUart.uart_send_str('#000P{0:0>4d}T1000!#001P1900T1000!#002P2000T1000!#003P1000T1000!#004P1500T1000!#005P1500T1000!'.format(int(1500+2000 * 0/ 270)))
     
    time.sleep(2)

  
    # 调整机械臂和摄像头朝向前方（初始位置）
    #myUart.uart_send_str('#000P1500T1000!#001P1500T1000!#002P1800T1000!#003P1500T1000!#004P1500T1000!#005P1500T1000!')

    #cam = Camera.Camera()  # 摄像头库实例化    
    #cam.camera_open()      # 打开摄像头
    #frame = None
    # 发出哔哔哔作为开机声音
    myBeep.beep(3, 0.1)
    
    start_time = int((time.time() * 1000))
    
    move_to_next_init_position()
    while 1:
        time.sleep(0.2)
        '''
        if int((time.time() * 1000)) - systick_ms_bak_zhuan > 100 and run_flag:
            systick_ms_bak_zhuan = int((time.time() * 1000))
            if zhuan_flag:
                servo_yuntai_bias += 3          
                if servo_yuntai_bias >= 30:
                    zhuan_flag = not zhuan_flag
            else:
                servo_yuntai_bias -= 3
                if servo_yuntai_bias <= -30:
                    zhuan_flag = not zhuan_flag            
            testStr ='#000P{0:0>4d}T{1:0>4d}!'.format(int(1500-2000 * servo_yuntai_bias / 270), 200)
                #print(testStr)
            myUart.uart_send_str(testStr)
        '''

        if grab_success:
            if move_to_next_init_position() :
                grab_success = False
            else:
                break 
    '''
    while 1:
        if cam.frame is not None:
            frame = cam.frame.copy()
            color_sort()  # 执行颜色识别和坐标获取
            cv2.imshow('frame', frame)
        if cv2.waitKey(5) & 0xFF == 27:  # 如果按了ESC就退出，当然也可以自己设置
           break
        if grab_success:
            if move_to_next_init_position() :
                grab_success = False
            else:
                break          
         
            if grab_success:
                print("抓取成功，准备前往下一个位置")
                break
            #print(grab_success)
        
    '''    
            
    
    myUart.uart_send_str("#255P1500T1000!")
    
    #cam.camera_close()
    cv2.destroyAllWindows()
