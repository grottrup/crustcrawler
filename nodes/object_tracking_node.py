#!/usr/bin/env python
import rospy
from sender_receiver.msg import BrickArray
#import message_filters

class ObjectTrackingNode():
    def __init__(self):
        rospy.Subscriber("/vision_top_topic", BrickArray, self.onReceive)

    def onReceive(self, top_bricks: BrickArray):
        for brick_object in range (len(top_bricks.bricks)):
            print(brick_object,top_bricks.bricks[brick_object].xPos,top_bricks.bricks[brick_object].yPos)			

if __name__ == '__main__':
	rospy.init_node("objectTracking")
	node = ObjectTrackingNode()
	rospy.spin()
