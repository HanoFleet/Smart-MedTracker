import threading
import socket

socket_port = 1314
socket_get_ok = 0
socket_receive_buf = ""

def socketEvent():
    global socket_receive_buf,socket_get_ok, socket_port
    serverSocket = socket.socket()
    #ip addr cannot conver to str!!! TODO 
    #hostIP = str(os.popen('ifconfig wlan0 | grep "inet " | awk \'{print $2}\'').read())
    hostIP = "192.168.12.1"
    print('hostIP:', hostIP)
    #data port
    dataPort = socket_port
    address = (hostIP,dataPort)
    serverSocket.bind(address)
    
    serverSocket.listen(5)
    print('socket waitting......')
    serverConnect,addr = serverSocket.accept()
    print('socket connected!')

    mode = 0
    while True:
        data = str(serverConnect.recv(1024))
        data = data.split('\'')[1]
        if len(data):
            print(data)
            socket_receive_buf = data
            if socket_get_ok == 0:
                uart_receive_buf = socket_receive_buf
                uart_get_ok = 0
                if mode == 0:
                    if uart_receive_buf.find('{') >= 0:
                        mode = 1
                        #print('mode1 start')
                    elif uart_receive_buf.find('$') >= 0:
                        mode = 2
                        #print('mode2 start')
                    elif uart_receive_buf.find('#') >= 0:
                        mode = 3
                        #print('mode3 start')
                
                if mode == 1:
                    if uart_receive_buf.find('}') >= 0:
                        uart_get_ok = 1
                        mode = 0
                        #print('{}:',uart_receive_buf, " len:", len(uart_receive_buf))
                        #print('mode1 end')
                elif mode == 2:
                    if uart_receive_buf.find('!') >= 0:
                        uart_get_ok = 2
                        mode = 0
                        #print('$!:',uart_receive_buf, " len:", len(uart_receive_buf))
                        #print('mode2 end')
                elif mode == 3:
                    if uart_receive_buf.find('!') >= 0:
                        uart_get_ok = 3
                        mode = 0
                        #print('#!:', uart_receive_buf, " len:", len(uart_receive_buf))
                        #print('mode3 end')
                socket_get_ok = uart_get_ok
                #print('get2:',uart_receive_buf, " len:", len(uart_receive_buf), " mode:", mode, " getok:", uart_get_ok)
    
        else:
            print('socket disconnected.')
            serverSocket.listen(5)
            print('socket waitting......')
            serverConnect,addr = serverSocket.accept()
            print('socket connected!')
            
socket_thread = threading.Thread(target=socketEvent)

def setup_socket(port):
    global socket_thread, socket_port
    socket_port = port
    #socket接收线程
    socket_thread.start()

def loop_socket():
    global socket_get_ok, socket_receive_buf
    if socket_get_ok:
        print(socket_receive_buf)
        socket_get_ok = 0

#大循环
if __name__ == '__main__':
    setup_socket(2020)
    try:
        while True:
            loop_socket()
            
    except KeyboardInterrupt:
        if serverSocket != None:
            serverSocket.close()
