#导包
import sys
sys.path.append('/home/pi/Desktop/ZL-PI/factory_code/')
import RPi.GPIO as GPIO
import ZL_SDK.z_led as myLed
import ZL_SDK.z_beep as myBeep
import time

#端口模式设置
GPIO.setmode(GPIO.BCM)   #编码模式设置
GPIO.setwarnings(False)  #关闭警告

#引脚定义 
KEY1_PIN = 24
KEY2_PIN = 25

#读取KEY1的值,按键按下为低电平
def key1():
    return GPIO.input(KEY1_PIN)

#读取KEY2的值,按键按下为低电平
def key2():
    return GPIO.input(KEY2_PIN)

#初始化按键引脚
def setup_key():
    GPIO.setup(KEY1_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)  #设置引脚上拉输入模式
    GPIO.setup(KEY2_PIN, GPIO.IN, pull_up_down = GPIO.PUD_UP)  #设置引脚上拉输入模式

#循环检测按键引脚，key1控制led1的亮灭，key2控制蜂鸣器的响声
def loop_key():
    if(key1() == 0):
        time.sleep(0.02)
        if(key1() == 0):
            myLed.on()
            while(key1() == 0):
                pass
            myLed.off()
            
    if(key2() == 0):
        time.sleep(0.02)
        if(key2() == 0):
            myBeep.on()
            while(key2() == 0):
                pass
            myBeep.off()
            
#程序反复执行处
if __name__ == "__main__":
    setup_key()
    myBeep.setup_beep()
    myLed.setup_led()
    try:
        while True:
            loop_key()
    except KeyboardInterrupt:
        myBeep.off()
        myLed.off()
    