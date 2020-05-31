from time import sleep
from queue import Queue
from threading import Thread

from pyaudio import PyAudio, paInt16
from scipy.fftpack import sfft
from scipy.io import wavfile

from numpy import arange, sinc, blackman, convolve, sum as np_sum, ceil as np_ceil
from wave import open as wave_open
from pathlib import Path
from os.path import join as os_join

#
# Constants
#

COLLECTION_WAIT_TIME_S = 0.1

AUDIO_FRAME_FORMAT      = paInt16
AUDIO_FRAME_TRACK_BITS  = 16
AUDIO_FRAME_CHANNELS    = 2
AUDIO_FRAME_RATE        = 44100
AUDIO_FRAME_SIZE_FRAMES = 1024

FFT_CUTOFF_FREQUENCY = 0.3625
FFT_TRANSITION_BAND = 0.08

#
# Class definitions
#

## A class designed to handle capturing audio output, as well as perform any audio analysis.
class AudioHandler():
	
	## The constructor. Initializes the queue of audio frames to be processed and starts the
	#  thread dedicated to collecting outgoing audio frames.
	#
	#  @param self The object pointer.
	def __init__(self):
		self.keep_audio_collection_thread_alive = False
		self.audio_collection_thread = Thread(target=self._do_collection)
		
		self.collect_audio = None
		self.audio_collected = Queue()
		self.audio_stream_parent = PyAudio()
		self.audio_stream = None
	
	## The main routine for the audio collection thread. Simply gathers outgoing audio frame
	#  and passes them to the queue to be processed.
	#
	#  @param self The object pointer.
	def _do_collection(self):
		while self.keep_audio_collection_thread_alive:
			audio_frame = self.audio_stream.read(AUDIO_FRAME_SIZE_FRAMES)
			if self.collect_audio and audio_frame:
				self.audio_collected.put(audio_frame)
	
	## Starts the audio collection thread.
	#
	#  @param self The object pointer.
	def start(self):
		self.audio_stream = self.audio_stream_parent.open(
			format=AUDIO_FRAME_FORMAT,
			channels=AUDIO_FRAME_CHANNELS,
			rate=AUDIO_FRAME_RATE,
			input=True,
			frames_per_buffer=AUDIO_FRAME_SIZE_FRAMES
		)
		
		self.keep_audio_collection_thread_alive = True
		self.collect_audio = False
		self.audio_collection_thread.start()
	
	## This function will block until it has successfully joined the thread. Furthermore, this
	#  function handles cleanly closing the audio stream.
	#
	#  @param self The object pointer.
	def join(self):
		self.audio_collection_thread.join()
		self.audio_collected.join()
		
		if self.audio_stream:
			self.audio_stream.stop_stream()
			self.audio_stream.close()
		
		self.audio_stream_parent.terminate()
	
	## Sets a flag to inform the audio collection thread that it should be shut down.
	#
	#  @param self The object pointer.
	def stop(self):
		self.set_collection_status(False)
		self.keep_audio_collection_thread_alive = False
	
	## Set whether or not audio data should be collected. If this is disabled, this will also
	#  purge existing frames that went unprocessed.
	#
	#  @param self The object pointer.
	#  @param collect Whether or not this object should collect outgoing audio packets.
	def set_collection_status(self, collect):
		self.collect_audio = collect
		
		if not self.collect_audio:
			while not self.audio_collected.empty():
				self.audio_collected.get()
				self.audio_collected.task_done()
	
	## Checks to see if there is any new audio coming from the speakers' output to analyze.
	#
	#  @param self The object pointer.
	#  @return Whether or not there is audio data that can be analyzed.
	def can_do_work(self, required_size=0):
		return self.audio_collected.qsize() >= required_size
	
	## .
	#
	#  @param self The object pointer.
	def fft(self):
		frames = []
		
		while not self.audio_collected.empty():
			frames.append(self.audio_collected.get())
			self.audio_collected.task_done()
		
		return sfft(frames)
		
		#normalized = []
		#for frame in frames:
		#	normalized.extend([(((b / (2 ** AUDIO_FRAME_TRACK_BITS)) * 2) - 1) for b in frame])
		#
		##fs, data = wavfile.read("/home/nick/test_wav.wav")
		##normalized = [(((b / (2 ** AUDIO_FRAME_TRACK_BITS)) * 2) - 1) for b in data.T[0]]
		#
		#raw_fft_output = sfft(normalized)
		#
		#N = int(np_ceil(4 / FFT_TRANSITION_BAND))
		#if N % 2 == 1:
		#	N = N + 1
		#
		#sinc_func = sinc(2 * FFT_CUTOFF_FREQUENCY * (arange(N) - (N - 1) / 2.)) * blackman(N)
		#sinc_func = -(sinc_func / np_sum(sinc_func))
		#sinc_func[int((N - 1) / 2)] = sinc_func[int((N - 1) / 2)] + 1
		#
		#return convolve(raw_fft_output, sinc_func)
	
	## A testing function to save raw audio bytes to a .wav file.
	#
	#  @param self The object pointer.
	def save_to_wav(self, frames, url=os_join(str(Path.home()), "test_wav.wav")):
		wf = wave_open(url, 'wb')
		wf.setnchannels(AUDIO_FRAME_CHANNELS)
		wf.setsampwidth(self.audio_stream_parent.get_sample_size(AUDIO_FRAME_FORMAT))
		wf.setframerate(AUDIO_FRAME_RATE)
		wf.writeframes(b''.join(frames))
		wf.close()