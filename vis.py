import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.widgets import Slider, Button


line1 = [[0,-1], [1,-2]] # s = [[x1,y1],[x2,y2]]
line2 = [[2,-2],[3,1]] # s'= [[x1,y1],[x2,y2]]
line3 = [[-2,1],[0,3]] # s"= [[x1,y1],[x2,y2]]

def vector(line):  # Find vector given two points
    return (line[1][0] - line[0][0], line[1][1] - line[0][1])

def length(v): # Find length given a vector
    return math.sqrt(v[0]**2 + v[1]**2)

def slope(line):
    return ((line[1][1] - line[0][1]) / (line[1][0] - line[0][0]))

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.35)

length1 = length(vector(line1))
length2 = length(vector(line2))
length3 = length(vector(line3))

m1 = slope(line1)
m2 = slope(line2)
m3 = slope(line3)

a1 = line1[1][1]-(m1*line1[1][0])
a2 = line2[1][1]-(m2*line2[1][0])
a3 = line3[1][1]-(m3*line3[1][0])

u1 = [vector(line1)[0]/length(vector(line1)), vector(line1)[1]/length(vector(line1))]
u2 = [vector(line2)[0]/length(vector(line2)), vector(line2)[1]/length(vector(line2))]
u3 = [vector(line3)[0]/length(vector(line3)), vector(line3)[1]/length(vector(line3))]

plt.plot([line1[0][0], line1[1][0]], [line1[0][1], line1[1][1]])
plt.plot([line2[0][0], line2[1][0]], [line2[0][1], line2[1][1]])
plt.plot([line3[0][0], line3[1][0]], [line3[0][1], line3[1][1]])

temp = ((m1-1)/(m2-1))*line1[1][0]-(a1+a2)/(m2-1)
#l1 = [[line1[1][0],line1[1][1]],[temp,m2*temp+a2]]
#midpoint = [(line1[1][0]+temp)/2,(line1[1][1]+m2*temp+a2)/2]
#l2 = [[],[]]
b = max(min(line1[1][1]-line1[1][0],line1[0][1]-line1[0][0]),min(line2[1][1]-line2[1][0],line2[0][1]-line2[0][0]),min(((m1-1)*(m2-1)*(line3[0][0]+line3[0][1])+(m2-1)*a1+(m1-1)*a2)/(m1*m2-1),((m1-1)*(m2-1)*(line3[1][0]+line3[1][1])+(m2-1)*a1+(m1-1)*a2)/(m1*m2-1)))

l1, = ax.plot([(b-a1)/(m1-1),(b-a2)/(m2-1)],[((b-a1)/(m1-1))+b,((b-a2)/(m2-1))+b])
l2, = ax.plot([(b-a1)/(m1-1), ((b-a1)/(m1-1)+((b-a2)/(m2-1))+b-a3)/(m3+1)],[((b-a2)/(m2-1))+b,((b-a2)/(m2-1))+b+(b-a1)/(m1-1)-((b-a1)/(m1-1)+((b-a2)/(m2-1))+b-a3)/(m3+1)])

AX = plt.axes([0.25, 0.15, 0.65, 0.03])

b_start = max(min(line1[1][1]-line1[1][0],line1[0][1]-line1[0][0]),min(line2[1][1]-line2[1][0],line2[0][1]-line2[0][0]),min(((m1-1)*(m2-1)*(line3[0][0]+line3[0][1])+(m2-1)*a1+(m1-1)*a2)/(m1*m2-1),((m1-1)*(m2-1)*(line3[1][0]+line3[1][1])+(m2-1)*a1+(m1-1)*a2)/(m1*m2-1)))
b_end = min(max(line1[1][1]-line1[1][0],line1[0][1]-line1[0][0]),max(line2[1][1]-line2[1][0],line2[0][1]-line2[0][0]),max(((m1-1)*(m2-1)*(line3[0][0]+line3[0][1])+(m2-1)*a1+(m1-1)*a2)/(m1*m2-1),((m1-1)*(m2-1)*(line3[1][0]+line3[1][1])+(m2-1)*a1+(m1-1)*a2)/(m1*m2-1)))
freq = Slider(AX, 'Segment1\'s Width from the origin', b_start, b_end, b)


ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_title('Line Segment Plot')
ax.set_aspect(1.0/ax.get_data_ratio(), adjustable='box')

distance = 0

d = ax.text(0, -5, "Length covered: "+str(abs(temp-line1[1][0])+abs((line1[1][0]+m2*temp+a2-a3)/(m3+1)-line1[1][0])), ha="center", va="center")

def update(val):
    f = freq.val
    l1.set_data([(val-a1)/(m1-1),(val-a2)/(m2-1)],[((val-a1)/(m1-1))+val,((val-a2)/(m2-1))+val])
    #print((((val-a2)/(m2-1))+val - (((val-a1)/(m1-1))+val))/((val-a2)/(m2-1)-(val-a1)/(m1-1)))
    l2.set_data([(val-a1)/(m1-1), ((val-a1)/(m1-1)+((val-a2)/(m2-1))+val-a3)/(m3+1)],[((val-a2)/(m2-1))+val,((val-a2)/(m2-1))+val+(val-a1)/(m1-1)-((val-a1)/(m1-1)+((val-a2)/(m2-1))+val-a3)/(m3+1)])
    #print((((val-a2)/(m2-1))+val+(val-a1)/(m1-1)-((val-a1)/(m1-1)+((val-a2)/(m2-1))+val-a3)/(m3+1) - (((val-a2)/(m2-1))+val))/(((val-a1)/(m1-1)+((val-a2)/(m2-1))+val-a3)/(m3+1) - (val-a1)/(m1-1)))
    d.set_text("Length covered: "+str(abs(((val-a1)/(m1-1))-((val-a2)/(m2-1)))+abs((((val-a1)/(m1-1)+((val-a2)/(m2-1))+val-a3)/(m3+1)) - ((val-a1)/(m1-1)))))
    fig.canvas.draw_idle()
 
freq.on_changed(update)

plt.show()

