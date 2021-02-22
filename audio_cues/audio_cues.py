 #!/usr/bin/python
# audio_cues.py used during EEG acquisition experiments. User subjects were instructed to close their
# eyelids upon hearing the characteristic noise

from playsound import playsound
import time
import sys
import random

#1st argument: file to write
t=time.time()
log_name=sys.argv[1] #"audio_cues.txt"


if __name__=="__main__":
	log=open(log_name,"w")

	while(True):
		period=random.uniform(2.8,5.2)
		if time.time()-t>60:
			break

		time.sleep(period)
		#print(time.time()-t)
		log.write("{:f}\n".format(time.time()))

		playsound('/media/neurorobotics/E03C08763C084A4C/python')
	log.close()
