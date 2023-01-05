# from openal import al, alc

# source = al.ALuint()
# device = alc.alcOpenDevice(None)

from pyglet.media.drivers.openal import lib_openal as al
from pyglet.media.drivers.openal import lib_alc as alc
from synthesizer import Player, Synthesizer, Waveform, Writer


import time
from openal import oalOpen, Listener
#from pyglet.media.drivers.openal import *

if __name__ == "__main__":
    x_pos = 3
    y_pos = 10
    pitch = 10
    sleep_time = 1

    writer = Writer()
    player = Player()
    player.open_stream()
    synthesizer = Synthesizer(osc1_waveform=Waveform.sine, osc1_volume=1, use_osc2=False)
    wave = synthesizer.generate_constant_wave(440.0, 1.0)

    writer.write_waves("temp/test_synthesis.wav", wave)
    

    source = oalOpen("temp/test_synthesis.wav")

    #source.set_position([x_pos, 0, 0])
    source.set_looping(True)
    source.play()
   
    
    listener = Listener()
    listener.set_gain(-100)
    

    while True:
        #works with x, y, z
        source.set_position([x_pos, y_pos, 0])
        print("Playing at: {0}".format(source.position))
        time.sleep(sleep_time)
        source.set_pitch(pitch)
        #source.set_gain(0)
        print(source.gain)
        x_pos *= -1
        pitch -= 1
    
    oalQuit()


