#!/usr/bin/env python
import rospy
from sender_receiver.msg import BrickArray, Brick
#import vision_util

class VisionNodeXY():
	def __init__(self):
		self.publisher = rospy.Publisher("/imgXY", BrickArray, queue_size=1)
		self.timer = rospy.Timer(rospy.Duration(0.1), self.onTimer)

	def onTimer(self, event):
    		msg = BrickArray()
#		img = vision_util.capture_img()
#    		redBricks, xArr, yArr, wArr, hArr = vision_util.get_red_bricks(img)    		
#		for x in range(redBricks):
#			data = Brick()
#			data.color = "red"
#			data.xPos = xArr[x]
#			data.yPos = yArr[x]
#			data.width = wArr[x]
#			dara.height = hArr[x]
#			msg.bricks.append(data)
#		blueBricks, xArr, yArr, wArr, hArr = vision_util.get_red_bricks(img)
#		for x in range(blueBricks):
#			data = Brick()
#			data.color = "blue"
#			data.xPos = xArr[x]
#			data.yPos = yArr[x]
#			data.width = wArr[x]
#			dara.height = hArr[x]
#			msg.bricks.append(data)
		
		x = 40
		y = 50
		size = 200
		for x in range (0,3):
			data = Brick()
			data.color = "red"
			data.xPos = x
			data.yPos = y
			data.height = size
			data.width = size
			x += 10
			y += 20
			size += 100
			msg.bricks.append(data)
		
		self.publisher.publish(msg)

if __name__ == '__main__':
	rospy.init_node("visionXY")
	node = VisionNodeXY()
	rospy.spin()
