# # import socket
# # import xml.dom.minidom
# # # create a socket of a client
# # client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


# # # set the ip of the  server
# # host = '10.15.198.151'
# # # set the port of the server
# # port = 59152
# # # connect the server
# # connected = False
# # while not connected:
# #     try:
# #         client.connect((host, port))
# #         connected = True
# #     except socket.error as e:
# #         print(e)

# # # keep the connection
# # loop = 0
# # sign = 1
# # while True:
# #     # input the message which need be sent
# #     if loop < 10:
# #         sendmsg = "hello,server"
# #     # encode the message into  binary before being sent
# #     if sendmsg == 'q':
# #         break
# #     client.send(sendmsg.encode("utf-8"))
# #     # if client input the character q, break the connection and quit

# #     msg = client.recv(1024)
# #     # deconde the msg received
# #     print(msg.decode("utf-8"))
# #     loop += 1
# #     if loop > 10:
# #         sendmsg = 'q'

# # # close the client
# # client.close()


# import xml.dom.minidom as minidom

# with open('sendmsg.xml', 'r') as f:
#     dom = minidom.parse(f)

#     data = dom.toxml(encoding='utf8')
#     client.send(str(data.toxml(encoding='utf8')))

#     root = dom.documentElement
#     root.getElementsByTagName('Pose')[0].setAttribute('X', "255.00")
#     # pose.setAttribute('X', 255)
#     a = float(root.getElementsByTagName('Pose')[0].getAttribute('X'))
#     b = dom.toxml(encoding='utf8')

# with open('recvmsg.xml', 'w') as w:
#     dom.writexml(w, indent='', addindent='\t', newl='\n', encoding='UTF-8')


import socket
import time
import struct
import os
import time
#导入外部插件
#定义客户端，地址
sc=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#实例化socket
address=('10.15.198.151',9999)
#定义address
sc.connect(address)
#发起连接
while True:
    file_path="/home/nimpng/kuka_ros/src/sendmsg.xml"
    #定义要传输的文件地址
    file_info_size=struct.calcsize('128sl')
    #定义打包规则
    fhead=struct.pack('128sl',os.path.basename(file_path),os.stat(file_path).st_size)
    #定义头信息
    sc.send(fhead)
    #首先发送头部信息
    print("send file:",file_path)
    #打印文件信息
    if True:
        #一切正常，开始处理文件
        fo=open(file_path, 'rb')
        fileData=fo.read(3000000)
        #读取文件数据，1024切分
        if not fileData:
            break
            #文件为空break
        else:
             sc.send(fileData)
            #发送数据，1024切分的数据
        fo.close()
        #发送结束，关闭文件。
        print("finish send..")
    # time.sleep(0.2)
    sc.close()
    exit()