"""
Small demonstration of the hlines and vlines plots.
"""

import matplotlib.pyplot as plt
import numpy as np
import numpy.random as rnd
import urllib2

download = False
filename= 'sum15.txt'

	#downloading the data file
	
if download:
	data = urllib2.urlopen("http://192.168.137.100/tftpboot/sum21.txt");
	with open("data.txt","wb") as file:
		file.write(data.read())

print "Data gathered"

wait_id = list()
wait_start = list()
wait_end = list()

operate_id = list()
operate_start = list()
operate_end = list()


actual_process = 0
actual_name = ""

pid_map = dict()
next_num = 1

name_acc_id_map = dict()
id_acc_id_map = dict()
next_acc_id = 1

with open(filename,'r') as data:
	for line in data:
		if line.startswith("#pid"):
			pid  = int(line[4:])
			pid_map[pid] = next_num
			actual_process = next_num
			next_num = next_num+1
		elif line.startswith("#name"):
			actual_name = line[5:]
			if not actual_name in name_acc_id_map.keys():
				name_acc_id_map[actual_name] = next_acc_id
				next_acc_id = next_acc_id + 1
				
			if not actual_process in id_acc_id_map.keys():
					id_acc_id_map[actual_process] = name_acc_id_map[actual_name];			
		elif len(line)>20:
			#ordinary data record
			fields = line.split()
			w = int(fields[4])+float(fields[5])/1000000000
			o = int(fields[6])+float(fields[7])/1000000000
			f = int(fields[8])+float(fields[9])/1000000000
			
			wait_id.append(actual_process)
			wait_start.append(w)
			wait_end.append(o)
			
			
			operate_id.append(actual_process)
			operate_start.append(o)
			operate_end.append(f)

		
## post processing
process_number = next_num-1
accel_number =  next_acc_id -1

start_time = min(wait_start)
end_time = max([max(wait_end),max(operate_end)]) - start_time

# substracting start time
w_s = np.array(wait_start)-start_time
w_e = np.array(wait_end) - start_time
o_s = np.array(operate_start)-start_time
o_e = np.array(operate_end) - start_time

w_l = w_e-w_s
o_l = o_e-o_s

o_s_sort = np.sort(o_s)
o_e_sort = np.sort(o_e)

c_l = o_s_sort[1:]-o_e_sort[0:-1]
print c_l

avg_wait_len = np.mean(w_l)
avg_config_len = np.mean(c_l[c_l>0.04])
avg_operating_len = np.mean(o_l)

sum_wait_time = np.sum(w_l)
sum_operating_time = np.sum(o_l)

usage_conf_rat = sum_operating_time/sum_wait_time
ideal_usage_conf_rat = sum_operating_time/5/(accel_number*avg_wait_len)


# substract starting time
wait_start_corr = list()
for time in wait_start:
	wait_start_corr.append(time-start_time)


#plotting

fig = plt.figure(figsize=(12, 6))
hax = fig.add_subplot(111)

figh = plt.figure(figsize=(12,6))
h_w = figh.add_subplot(131)
h_c = figh.add_subplot(132)
h_o = figh.add_subplot(133)

hax.hlines(wait_id, w_s, w_e, lw=2,color='yellow', label='Waiting for fpga')
hax.hlines(operate_id, o_s, o_e, lw=2,color='blue',label='Using accelerator')

hax.hlines([0 for x in range(len(wait_id))], o_s, o_e, lw=2,color='blue')

hax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
hax.set_xlabel('time (s)')
hax.set_ylabel('Process number')
hax.set_ylim([-1,next_num])
hax.set_xlim([-0.1,end_time+0.01])
hax.set_title('Process timing')
# print accel id
for id in id_acc_id_map:
	hax.text(-0.08,id,str(id_acc_id_map[id]))
hax.text(-0.08,0,'Sum')

resolution = 0.002

h_w.hist(w_l,bins = np.arange(min(w_l),max(w_l)+0.05,0.05))
h_w.axvline(avg_wait_len, color = 'red', linestyle='dashed', linewidth=2)
h_w.set_title('Waiting times')
h_c.hist(c_l, bins = np.arange(min(c_l),max(c_l)+resolution,resolution))
h_c.axvline(avg_config_len, color = 'red', linestyle='dashed', linewidth=2)
h_c.set_title('Config length')
h_o.hist(o_l,bins = np.arange(min(o_l),max(o_l)+0.02,0.02))
h_o.axvline(avg_operating_len, color = 'red', linestyle='dashed', linewidth=2)
h_o.set_title('Operating times')




# printing datas
print(name_acc_id_map)
print(id_acc_id_map)
print "Starting time: " + str(start_time) +"sec"

print "Average waiting len: " + str(avg_wait_len*1000) + "msec"
print "Average operating len: " + str(avg_operating_len*1000) + "msec"

print "Usage/config ratio: " + str(usage_conf_rat*100) + ' %'
print "Ideal usage/config ratio: " + str(ideal_usage_conf_rat*100) + ' %'
print "Efficiency ratio: " + str(usage_conf_rat/ideal_usage_conf_rat*100) + ' %'
print ""

#storing it to the collected file

plt.show()