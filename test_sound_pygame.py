import winsound
#import pygame
import time
from pygame import mixer


# frequency is set to 500Hz
freq = 50
# duration is set to 100 milliseconds            
dur = 1000
#winsound.Beep(freq, dur)


mixer.pre_init(44100, -16, 2, 4096) #frequency, size, channels, buffersize

mixer.init()
sound = mixer.Sound("audio/bleep-long.wav")

#plays audio on left
channel1 = mixer.Channel(0)
channel1.play(sound)
channel1.set_volume(1,0)

time.sleep(5)

#plays audio on right
channel2 = mixer.Channel(1)
channel2.play(sound)
channel2.set_volume(0,1)

time.sleep(5)




