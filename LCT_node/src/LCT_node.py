#!/usr/bin/env python3
import numpy as np
import rospy
import csv
import time
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import (LanePose,Twist2DStamped,FSMState)
from std_msgs.msg import String

class LCTnode(DTROS):
    def __init__(self, node_name):

        super(LCTnode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Initialize variables
        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.compute_gain)
        self.velocity_reading= rospy.Subscriber("~car_cmd", Twist2DStamped, self.callback)
        self.sub_lane_reading = rospy.Subscriber("~fsm_state", FSMState, self.callback_fsm)
        rospy.set_param("/woifi/kinematics_node/gain", 0.7)
        self.pub_LCT_out=rospy.Publisher("~LCT_out",String, queue_size=1)

        self.state=0
        self.log("Initialized!")
        self.count=0
        self.start_time=time.time()
    def callback(self,data):
        #rospy.loginfo("Heard: %s", data)
        #self.data_collect(data.d,data.phi)
        pass
    def publishCmd(self, LCT_output):
        self.pub_LCT_out.publish(LCT_output)
    def callback_fsm(self,fsm_state):
        if fsm_state.state=="LANE_FOLLOWING":
            self.state=0
        elif fsm_state.state=="NORMAL_JOYSTICK_CONTROL" and self.state==0:
            self.state=1

    def compute_gain(self, LanePose):
        d=LanePose.d
        phi=LanePose.phi
        frame_rate='0000'
        self.publishCmd(frame_rate)
        if self.state==0:
            print(str(round(time.time(),2)) + "," +str(round(d,2)) + ","+ str(round(phi,2)) )
    def data_collect(self,d,phi):
        print("here")
     
        next_row = pd.DataFrame({"time" : time.time(), 
                                    "disp" : d, 
                                    "phi" : phi, 
                                    }, index=[0])
        self.database=pd.concat([next_row, self.database[:]])
        

        if self.state == 1:
            self.database.plot(x='time', y='disp', kind='scatter')	
        """
        with open('data.csv','w',newline='') as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=' ',quotechar='|', quoting=csv.QUOTE_MINIMAL)
            spamwriter.writerow([d+phi])"""
            
if __name__ == "__main__":
    # Initialize the node
    LCT_node = LCTnode(node_name="LCTnode")
    
    # Keep it spinning
    rospy.spin()
    
