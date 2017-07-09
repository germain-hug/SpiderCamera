import socket
req = "DESCRIBE rtsp://192.168.0.1/livePreviewStream RTSP/1.0\r\nCSeq: 2\r\n\r\n"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("192.168.0.1", 554))
print("connected")
s.sendall(req)
data = s.recv(512)
print data