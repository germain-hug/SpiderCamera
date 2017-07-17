import subprocess as sp
import numpy 
command = [ 'ffmpeg',
		'-i', 'rtsp://192.168.0.1/livePreviewStream',
		'-f', 'image2pipe',
		'-preset', 'ultrafast',
		'-ar', '11025', 
		'-maxrate', '960k', 
		'-bufsize', '960k',
		'-']

command = ['ffplay', '-i', '/home/hugo/stream.flv']


pipe = sp.Popen(command, stdout = sp.PIPE, bufsize=10**8)
print('OKK')
while true:
	raw_image = pipe.stdout.read(1280*720*3)
	image =  numpy.fromstring(raw_image, dtype='uint8')
	image = image.reshape((360,420,3))
	pipe.stdout.flush()
	print('Hello')

