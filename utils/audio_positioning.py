from synthesizer import Player, Synthesizer, Waveform, Writer
from openal import oalOpen, Listener
import time
import multiprocessing

source = oalOpen("temp/test_synthesis.wav")
listener = Listener()

def setup():
    source.set_looping(True)
    listener.set_gain(2)
    source.play()

def set_pitch(value):
    source.set_pitch(value)
    
def adjust_position(coordinate):
    source.set_position(coordinate)
    
def mute():
    source.set_gain(0)
    
def unmute():
    source.set_gain(1)
    
    
# x_pos = 3
# y_pos = 10
# sleep_time = 5

# writer = Writer()
# player = Player()
# player.open_stream()
# synthesizer = Synthesizer(osc1_waveform=Waveform.sine, osc1_volume=1, use_osc2=False)

# wave = synthesizer.generate_constant_wave(440.0, 1.0)

# writer.write_waves("temp/test_synthesis.wav", wave)

#source.set_position([x_pos, 0, 0])
    


