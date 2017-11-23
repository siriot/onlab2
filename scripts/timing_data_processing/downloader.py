import urllib2

data = urllib2.urlopen("http://192.168.137.100/tftpboot/sum.txt");
with open("sum.txt","wb") as file:
	file.write(data.read())