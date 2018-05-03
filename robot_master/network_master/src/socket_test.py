# import time
# from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
#
# SERVER_IP = '192.168.1.5'
# PORT_NUMBER = 5000
#
# mySocket = socket( AF_INET, SOCK_DGRAM )
# mySocket.connect((SERVER_IP,PORT_NUMBER))
#
# while True:
#         mySocket.send('cool')
#         time.sleep(.5)

# from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
# import sys
# PORT_NUMBER = 5000
# SIZE = 1024
#
# hostName = gethostbyname( '0.0.0.0' )
#
# mySocket = socket( AF_INET, SOCK_DGRAM )
# mySocket.bind( (hostName, PORT_NUMBER) )
#
# print ("Test server listening on port {0}\n".format(PORT_NUMBER))
#
# while True:
#         (data,addr) = mySocket.recvfrom(SIZE)
#         print data

import socket

HOST = '192.168.1.5'
PORT = 9876
ADDR = (HOST,PORT)
BUFSIZE = 8192

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

print client.recv(BUFSIZE)

client.close()

# import jsocket
#
# client = jsocket.JsonClient(address="192.168.1.5", port=21151)
# client.connect()
# client.send_obj({"message": "new connection"})