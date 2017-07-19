#!/usr/bin/python
from graphics import *
import time
import math
import random
import paho.mqtt.client as mqtt
import config as cfg
lines={}
lineR_old=0
############
def on_message(MQclient, userdata, message):
	global lineR_old
	#messages is like index:byte4:byte3:byte2:byte1 
	MSG=message.payload.decode("utf-8")
	print("message received " ,str(MSG))
	print("message topic=",message.topic)
	print("message qos=",message.qos)
	print("message retain flag=",message.retain)
	#angle=int (MSG[0:3])
	#distance=int(MSG[4:7])
	#direction=int(MSG[8:9])
	
	index,b4,b3,b2,b1=MSG.split(":")
	indexold=index

	if (int(index)<23):
		direction=1
		angle=float(index)*(90/11)
	else:
		direction=0
		angle=360 - (float(index)*(90/11))
	if (int(index)==23)or (int(index)==22):
		return
		
	distance=(int(b4)<<24)+(int(b3)<<16)+(int(b2)<<8)+int(b1)
	distance=distance/5

	print " A %f Dis %f d Dir %d I %d" % (angle , distance,  direction, int(index))
    #n=n+1
    #Hypo= random.randint(0,280)
    #Xn=math.cos(math.radians(n))*Hypo
    #Yn=math.sin(math.radians(n))*Hypo

	if lineR_old :
		lineR_old.undraw()
		
	print "LINE OLD %s" % lineR_old
	Xn=math.cos(math.radians(angle))*300
	Yn=math.sin(math.radians(angle))*300
	lineR = Line(centerPoint,Point (RX0+Xn, 300-Yn) ) # set center and radius	
	lineR.setFill("red")
	lineR.setWidth(6)
	lineR.draw(win)
	lineR_old=lineR	
	if angle in lines.keys():
		lines[int(angle)].undraw()
		print "O %d %s " %(angle, lines[angle])

	#if direction==1:
	Xn=math.cos(math.radians(angle))*distance
	Yn=math.sin(math.radians(angle))*distance
	line = Line(centerPoint,Point (RX0+Xn, 300-Yn) ) # set center and radius
	lines[angle]=line
	#print "L %d %s" %  (len(lines),lines[int(angle)])
	print "L %d %s " %(angle, lines[angle])
	line.setFill("black")
	line.setWidth(3)
	line.draw(win)
	update(0)
########################################

broker_address=cfg.host

MQclient = mqtt.Client("P1") #create new instance
MQclient.username_pw_set(cfg.usr,cfg.password)
MQclient.on_message=on_message #attach function to callback
MQclient.connect(broker_address) #connect to broker

topic=cfg.queue

MQclient.subscribe(topic, qos=0)
#time.sleep(4) # wait


#def main():					
win = GraphWin('Face', 600, 300,autoflush=True) # give title and dimensions  (with,height)
    #win.yUp() # make right side up coordinates!

    #head = Circle(Point(40,100), 25) # set center and radius
    #head.setFill("yellow")
    #head.draw(win)

    #eye1 = Circle(Point(30, 105), 5)
    #eye1.setFill('blue')
    #eye1.draw(win)
#m1 = Text(Point(290, 10), 'x=290 y=10')
#m1.draw(win)   
#m1 = Text(Point(20, 77), 'x=20 y=77')
#m1.draw(win)   
#m1 = Text(Point(550, 290), 'x=550 y=290')
#m1.draw(win)  
RX0=win.getWidth()/2
RY0=295
P0=Point(RX0,RY0)
P0.draw(win)
update()
centerPoint=Point(RX0,RY0)
print "cos(3) : ",  math.cos(math.radians(45))
for x in range (0,10):
	circle = Circle(Point(RX0,RY0), x*30) # set center and radius
	circle.draw(win)
	update(0)
Hypo=300
n=0
for x in range (0,11):
	n=n+15
	Xn=math.cos(math.radians(n))*Hypo
	Yn=math.sin(math.radians(n))*Hypo
	print "Xn=",Xn+RX0
	print "Yn=",Yn
	print "n=",n,"\n\r"
	line = Line(centerPoint,Point (RX0+Xn, 300-Yn) ) # set center and radius
	line.draw(win)
	Mx= Text(Point (RX0+Xn, 300-Yn), n)
	Mx.draw(win) 
	update(0)

count = 0
#while (count < 9):
#	print 'The count is:', count
#	count = count + 1
#	#time.sleep( 5 )
#	P3 = Point(50,count);
#	P3.draw(win)
#	update(0)

    #mouth = Oval(Point(30, 90), Point(50, 85)) # set corners of bounding box
    #mouth.setFill("red")
    #mouth.draw(win)

    #label = Text(Point(100, 120), 'A face')
    #label.draw(win)
MQclient.loop_forever() #start the loop
message = Text(Point(win.getWidth()/2, 20), 'Click anywhere to quit.')
message.draw(win)
win.getMouse()
win.close()

#main()

