import socket
import xml.dom.minidom
# create the object of the socket's server
socketserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = 'myAddr'
port = 59152

# bind IP and Port
socketserver.bind((host, port))
# set listening
socketserver.listen(5)
# wait for client to connect
clientsocket, addr = socketserver.accept()
# while loop keep connection
while True:
    # receive the request from the client
    recvmsg = clientsocket.recv(1024)
    with open('recvmsg.xml', 'w') as f:
        f.write(recvmsg)
    recvmsg = xml.dom.minidom.parseString(recvmsg)
    # decode the recvmsg
    strData = recvmsg.decode("utf-8")
    # check whether the client has sent the quit commander 'q'
    if strData == 'q':
        break
    print("recvmsg: " + strData)
    # msg = str(input("reply: "))
    msg = 'hello client'
    # encode the msg 
    clientsocket.send(msg.encode("utf-8"))


socketserver.close()