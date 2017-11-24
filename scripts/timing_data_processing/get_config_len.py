"""
Small demonstration of the hlines and vlines plots.
"""

import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd
import urllib2

data = urllib2.urlopen("http://192.168.137.100/tftpboot/upload/sum.txt");
with open("sum.txt","wb") as file:
	file.write(data.read())



def read_time(string):
	t = string.split('_')
	try:
		return float(t[0])+float(t[1])/1000000000
	except:
		print "Cannot convert number: " + string
		raise

class ThreadData:
	def __init__(self):
		pass

# list for all the thread infos
thread_datas = list()
acc_name2id = dict()


#read up the input file
with open("sum.txt", 'r') as data:
	while True:
		td = ThreadData()

		line = data.readline()
		if len(line) < 3:
			print "File end"
			# file ended
			break;

		if line.startswith("#pid"):
			td.pid  = int(line[4:])
		else:
			raise Exception("Invalid pid line" + line)

		line = data.readline()
		if line.startswith("#name"):
			td.name = line[5:]
		else:
			raise Exception("Invalid name line" + line)


		td.open_start = read_time(data.readline())
		td.open_end = read_time(data.readline())
		td.mmap_start = read_time(data.readline())
		td.mmap_end = read_time(data.readline())
		
		td.get_start = list()
		get_start_pairs = data.readline().split()
		for pair in get_start_pairs:
			td.get_start.append(read_time(pair))
	
		td.get_end = list()
		get_end_pairs = data.readline().split()
		for pair in get_end_pairs:
			td.get_end.append(read_time(pair))

		td.dma_start = list()
		dma_start_pairs = data.readline().split()
		for pair in dma_start_pairs:
			td.dma_start.append(read_time(pair))

		td.dma_end = list()
		dma_end_pairs = data.readline().split()
		for pair in dma_end_pairs:
			td.dma_end.append(read_time(pair))

		td.release_start = list()
		release_start_pairs = data.readline().split()
		for pair in release_start_pairs:
			td.release_start.append(read_time(pair))
	
		td.release_end = list()
		release_end_pairs = data.readline().split()
		for pair in release_end_pairs:
			td.release_end.append(read_time(pair))

		#thread ready save data
		thread_datas.append(td)
		print "Thread ready "+ str(td.pid)

#calculate config length
configs = np.array([])

for td in thread_datas:
	start = np.array(td.get_start)
	end = np.array(td.get_end)
	configs = np.concatenate((configs,end-start))

	print configs

# show config lenth distributionl
'''
resolution = 0.01
figh = plt.figure(figsize=(12,6))
h_c = figh.add_subplot(111)
h_c.hist(configs, bins = np.arange(min(configs),max(configs)+resolution,resolution))
h_c.axvline(np.mean(configs), color = 'red', linestyle='dashed', linewidth=2)
h_c.set_title('Config length')
plt.show()
'''
#############################
fig = plt.figure(figsize=(12, 6))
hax = fig.add_subplot(111)

#figh = plt.figure(figsize=(12,6))
#h_w = figh.add_subplot(131)
#h_c = figh.add_subplot(132)
#h_o = figh.add_subplot(133)

for td in thread_datas:
	hax.hlines(len(td.get_start) * [td.pid], td.get_start, td.get_end, lw=3,color='yellow')
	hax.hlines(len(td.get_start) * [td.pid], td.dma_start, td.dma_end, lw=5,color='red')
	hax.hlines(len(td.get_start) * [td.pid], td.get_end, td.release_end, lw=1,color='blue')

	#hax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
	hax.set_xlabel('time (s)')
	hax.set_ylabel('Process number')
	hax.set_title('Process timing')

plt.show()