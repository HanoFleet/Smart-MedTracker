# -*- coding:utf-8 -*-
#导入模块
import sys
sys.path.append('/home/pi/Desktop/ZL-PI/factory_code/')
import ActionGroupControl as AGC
import ZL_SDK.Z_UartServer as myUart

#读取动作组文件。输入参数：动作组文件位置   
AGC.Group_read('/home/pi/Desktop/ZL-PI/factory_code/Jibot3_Group/Jibot3_Group-pi.ini')
myUart.setup_uart(115200)
while 1:
    #判断动作组是否执行完成
    if AGC.group_ok:
        AGC.groups_parse_cmd("$DGT:73-78,1!")
    ##循环发送动作组指令   
    AGC.loop_group()
