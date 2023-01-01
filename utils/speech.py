from gtts import gTTS
import os
from mutagen.mp3 import MP3
from pydub.playback import play
from pydub import AudioSegment
import multiprocessing
from playsound import playsound
import soundfile as sf
import pyrubberband as pyrb
import librosa
import numpy as np
import scipy
import wave

language = 'en'

def direction_distance(filename, distance, direction, label, body_rel_direction):
    text = str (label + " " + str(distance) + " meters at " + str(direction) + " degrees " + body_rel_direction + ".")
    audio = gTTS(text=text, lang=language, slow=False)
    audio.save("temp/" + filename)

def speed_setter(filename, speed):
    audio = AudioSegment.from_mp3("temp/" + filename)
    audio.export("temp/" + os.path.splitext(filename)[0] + ".wav", format = "wav")
    
    data, samplerate = sf.read("temp/" + os.path.splitext(filename)[0] + ".wav")
    y_stretch = pyrb.time_stretch(data, samplerate, speed)
    y_shift = pyrb.pitch_shift(y_stretch, samplerate, speed)
    sf.write("temp/" + os.path.splitext(filename)[0] + ".wav", y_shift, samplerate, format='wav')

    audio = AudioSegment.from_mp3("temp/" + os.path.splitext(filename)[0] + ".wav")
    audio.export("temp/" + filename, format = "mp3")
    os.remove("temp/" + os.path.splitext(filename)[0] + ".wav")

# def speed_setter(filename, speed):
#     song, fs = librosa.load("temp/" + filename)

#     song_2_times_faster = librosa.effects.time_stretch(song, rate = speed)

#     scipy.io.wavfile.write("temp/" + filename, fs, song_2_times_faster)

def volume_setter(filename, volume):
    audio = AudioSegment.from_mp3("temp/" + filename)
    volume_changed = (audio + volume)
    volume_changed.export("temp/" + filename, format = "mp3")

def get_duration(filename):
    audio = MP3("temp/" + filename)
    return audio.info.length

def play_audio(filename):
    #p = multiprocessing.Process(target=playsound, args=("temp/" + filename, True))
    #p.start()
    playsound("temp/" + filename, True)
    #p.terminate()
    os.remove("temp/" + filename)