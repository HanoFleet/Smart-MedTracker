# -*- coding:utf-8 -*-
#导入模块
import sys
sys.path.append('/home/pi/Desktop/ZL-PI/factory_code/')
import time
import re
import configparser
import ZL_SDK.Z_UartServer as myUart

#全局变量定义
systick_ms_group_bak = 0
group_next_time = 0
group_start = 0
group_end = 0
group_times = 0
group_start_bak = 0
group_end_bak = 0
group_times_bak = 0
group_ok = 1

systick_ms_start = time.time()

myList = []

config = configparser.ConfigParser() # 类实例化
#myUart.setup_uart(115200)

def Group_read(path):
    index = 0
    config.read(path)
    while True:
        tmpStr = "G%04d" % index
        if config.has_option('group',tmpStr):
            value = config.get('group',tmpStr)
            myList.append(value)
            #print("read:")
            #print(value)
            index = index+1
        else:
            print("groups_read_ok")
            break

#指令解析
def groups_parse_cmd(myStr):
    global group_start, group_end, group_times, group_start_bak, group_end_bak, group_times_bak, group_ok    
    if myStr.find('$DST!') >= 0:
        myUart.uart_send_str("#255PDST!")
        group_ok = 1      
    elif myStr.find('$DGT:') >= 0:
        parseStr = myStr[1:]
        parseStr = '>' + parseStr
        print(parseStr)
        pattern = re.compile(r".DGT:(\d+)-(\d+),(\d+)!")
        matched = pattern.match(parseStr)
        if matched:
            group_start = int(matched.groups()[0])
            group_end = int(matched.groups()[1])
            group_times = int(matched.groups()[2])
            if(group_start < len(myList) and group_end < len(myList)):
                group_start_bak = group_start
                group_end_bak = group_end
                group_times_bak = group_times
                print(group_start,group_end,group_times)
                group_ok = 0
#获取动作组中最大时间
def get_max_time(group_str):
    timeList = re.findall(r"T\d+!", group_str)
    #print(timeList)
    numList = []
    for i in range(0, len(timeList)):
        numList.append(re.findall(r"\d+", timeList[i])[0])
    #print(numList)
    numList = map(int,numList)
    maxNum = max(numList)
    #print(maxNum)
    return maxNum
#动作组解析
def loop_group():
    global group_ok, systick_ms_group_bak, group_next_time, group_start, group_start_bak, group_end, group_end_bak, group_times, group_times_bak  
    if group_ok == 0 and len(myList):
        
        if group_start_bak == group_end_bak:
            myUart.uart_send_str(myList[group_start])
            #print(myList[group_start])
            group_ok = 1
            return
        
        if int((time.time() * 1000))- systick_ms_group_bak < group_next_time:
            return
        
        systick_ms_group_bak = time.time() * 1000
        group_next_time = get_max_time(myList[group_start])
        if group_start < group_end:
            myUart.uart_send_str(myList[group_start])
            print(myList[group_start])
            group_start = group_start+1
            if group_start > group_end:
                if group_times_bak == 0:
                    pass
                else:
                    group_times = group_times - 1
                    if group_times == 0:
                        group_ok = 1
                        return
                    else:
                        pass
                group_start = group_start_bak
                group_end = group_end_bak
        
        else:
            myUart.uart_send_str(myList[group_end])
            group_end = group_end-1
            print("group_end",group_end)
            if group_end < group_start:
                if group_times_bak == 0:
                    pass
                else:
                    group_times = group_times - 1
                    if group_times == 0:
                        group_ok = 1
                        return
                    else:
                        pass
                group_start = group_start_bak
                group_end = group_end_bak
