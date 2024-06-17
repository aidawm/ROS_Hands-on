import rospy
from hw2.srv import nextDestination, nextDestinationResponse
import os

class Mission:
    def __init__(self) -> None:
        # print(os.getcwd())

        working_directory = os.getcwd()
        self.data_address = "/home/aida/Desktop/robotics_homeworks/src/hw2/obstacles.txt"
        self.obstacle_list = open(self.data_address)

    def extract_x_y(self):
        d = self.obstacle_list.readline()
        l= str(d).replace(",","").replace("\n","").split(" ")
        if "" in l : 
            l=list(filter(("").__ne__, l))
        return float(l[1]),float(l[3])
    
    def next(self,req):
        try:
            x,y = self.extract_x_y()
            return nextDestinationResponse(x,y)
        except:
            self.obstacle_list = open(self.data_address)
            x,y = self.extract_x_y()
            return nextDestinationResponse(x,y)

    def init_node(self):
        rospy.init_node('mission')
        s = rospy.Service('getNexyDestination',nextDestination,self.next)
        print(s)
        print("service is up!")
        rospy.spin()


if __name__ == "__main__":
    m = Mission()
    m.init_node()
