# import time
# from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
#
# SERVER_IP = '192.168.1.14'
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

# import jsocket
# import logging
# import time
#
# logger = logging.getLogger("jsocket.example_servers")
#
# class MyFactoryThread(jsocket.ServerFactoryThread):
#     def __init__(self):
#         super(MyFactoryThread, self).__init__()
#         self.address = "0.0.0.0"
#         self.port = 21151
#
#     ##
#     #  virtual method - Implementer must define protocol
#     def _process_message(self, obj):
#         if obj != '':
#             if obj['message'] == "new connection":
#                 logger.info("new connection.")
#             else:
#                 logger.info(obj)
#
#
# if __name__ == "__main__":
#     server = jsocket.ServerFactory(MyFactoryThread)
#     server.timeout = 2.0
#     server.start()
#
#     while True:
#         time.sleep(0.1)

import socket

HOST = ''
PORT = 9876
ADDR = (HOST,PORT)
BUFSIZE = 4096

serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

serv.bind(ADDR)
serv.listen(5)

print 'listening ...'

while True:
  conn, addr = serv.accept()
  print 'client connected ... ', addr
  myfile = open('testfile.mov', 'w')

  while True:
    data = conn.recv(BUFSIZE)
    if not data: break
    print data
    print 'writing file ....'

  myfile.close()
  print 'finished writing file'
  conn.close()
  print 'client disconnected'