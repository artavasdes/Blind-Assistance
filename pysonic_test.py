import pySonic
import time

def finished_stream(source):
print('Stream finished playing')

# initialize the audio environment
w = pySonic.World()

# create two sources
src1 = pySonic.Source()
src2 = pySonic.Source()

# load a sound entirely from disk, stream another from disk
src1.Sound = pySonic.FileSample('short.wav')
src2.Sound = pySonic.FileStream('long.mp3')

# position the sources in 3D space
src1.Position = (-0.5, 0.0, 0.5)
src2.Position = (0.5, 0.0, 0.5)

# register a callback for when the stream finishes
src2.SetEndStreamCallback(finished_stream)

# register a callback for when the stream finishes
src1.Play()
src2.Play()

# just block while we're playing in this example
while src1.IsPlaying() or src2.IsPlaying():
time.sleep(1)