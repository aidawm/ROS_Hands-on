#!/usr/bin/env python

from hw0.srv import proximities,proximitiesResponse
import rospy
import json 
import os 


class Sensor: 
    def __init__(self) -> None:
        
        data_address = "src/hw0/distances.json"
        f = open(data_address)
        self.sensor_data = json.load(f)
        self.i = 1

    def read_data(self,req):
        i = self.i
        d = self.sensor_data[str(i)] 
        d = d.split(" ")
        self.i = (i+1)%10+1
        d = [int(d[s]) for s in range(len(d))]
        return proximitiesResponse(d[1],d[2],d[3],d[0])

    def update_proximity_values(self):
        rospy.init_node('sensor')
        s = rospy.Service('distances',proximities,self.read_data)
        print(s)
        print("service is up!")
        rospy.spin()

if __name__ == "__main__":
    s = Sensor()
    s.update_proximity_values()