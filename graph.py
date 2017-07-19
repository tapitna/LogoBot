#!/usr/bin/python
from graphics import *
import time
import math

#def main():					
win = GraphWin('Face', 600, 300,autoflush=False) # give title and dimensions  (with,height)
    #win.yUp() # make right side up coordinates!

    #head = Circle(Point(40,100), 25) # set center and radius
    #head.setFill("yellow")
    #head.draw(win)

    #eye1 = Circle(Point(30, 105), 5)
    #eye1.setFill('blue')
    #eye1.draw(win)
m1 = Text(Point(290, 10), 'x=290 y=10')
m1.draw(win)   
m1 = Text(Point(20, 77), 'x=20 y=77')
m1.draw(win)   
m1 = Text(Point(550, 290), 'x=550 y=290')
m1.draw(win)  
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
while (count < 9):
	print 'The count is:', count
	count = count + 10
	time.sleep( 5 )
	P3 = Point(50,count);
	P3.draw(win)
	update(0)
P1 = Point(1,1);

    #P1.setWidth(10)
P1.draw(win)   
P2 = Point(1,10);
P2.draw(win)
P3 = Point(1,50);
P3.draw(win)
P3 = Point(1,60);
P3.draw(win)

    #mouth = Oval(Point(30, 90), Point(50, 85)) # set corners of bounding box
    #mouth.setFill("red")
    #mouth.draw(win)

    #label = Text(Point(100, 120), 'A face')
    #label.draw(win)

message = Text(Point(win.getWidth()/2, 20), 'Click anywhere to quit.')
message.draw(win)
win.getMouse()
win.close()

#main()

