from synthesizer import Player, Synthesizer, Waveform, Writer

writer = Writer()

player = Player()
player.open_stream()
synthesizer = Synthesizer(osc1_waveform=Waveform.sine, osc1_volume=0.2, use_osc2=False)
# Play A4
#player.play_wave(synthesizer.generate_constant_wave(440.0, 1.0))

wave = synthesizer.generate_constant_wave(440.0, 1.0)




import time
import sounddevice as sd

# fs = 44100
# sd.play(wave, fs)
# time.sleep(5)





writer.write_waves("temp/test_synthesis.wav", wave)


from pygame import mixer
import pygame
import numpy as np
# mixer.init()
# channel1 = mixer.Channel(0)
# channel1.play("temp/test_synthesis.wav")

size = 44100

mixer.init()
mixer.pre_init(44100, -16, 2, 4096) 


audio = np.repeat(wave.reshape(size, 1), 2, axis = 1)

print(audio)
audio = pygame.sndarray.make_sound(audio)
channel1 = mixer.Channel(0)
channel1.play(audio)

time.sleep(5)


