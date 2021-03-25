#!/usr/bin/env python3
import rospy
import rospkg 
import sys
from std_msgs.msg import String, Time
from geometry_msgs.msg import Pose
#from ktmpb.msg import fVector
#from ktmpb.srv import *
rospack =rospkg.RosPack()
import xml.etree.ElementTree as ET 
from collections import defaultdict
#Import python module with functions necessary for interfacing with kautham
import kautham_py.kautham_python_interface as kautham


# Global variables
directory=''
Robot_move_control= ''
Robot_pos=[]
taskfile=''
graspedobject= False

#Function to write to xml file in Conf tag
def writePath(taskfile,tex):
    taskfile.write("\t\t<Conf> %s </Conf>\n" % tex)
    return True

def main():
    # Initialise code
    #check for arguments
    if len(sys.argv)<2:
        print("Number of parameters is not correct")
        print (" Should be: $./kautham_client_python_node.py kthconfig.xml")

    print("Using kautham---------------------")
    
    rospy.loginfo ("Starting Kautham Python Client")
    rospy.init_node("kautham_python_client")

    #Open config file
    ROSpackage_path= rospack.get_path("kautham")
    config_tree = ET.parse(ROSpackage_path+sys.argv[1])
    config_root = config_tree.getroot()
    #Get data from config file
    #Get data for Problem files
    kauthamproblem = config_root.find('Problemfiles').find('kautham').get('name')
    DIRECTORY =config_root.find('Problemfiles').find('directory').get('name')
    print("Using kautham problem",kauthamproblem)

    #Setting problem files
    modelFolder = ROSpackage_path + "/demos/models/"
    global directory
    directory=ROSpackage_path + DIRECTORY#"/demos/OMPL_geo_demos/Table_Rooms_R2/" 
    kauthamProblemFile= directory + kauthamproblem

    rvizconfigfile = directory + config_root.find('Problemfiles').find('rviz').get('name')

    rospy.loginfo_once(kauthamProblemFile)

    ##Solving the motion planning problem
    #Open kautham problem
    print("***************************************")
    print("   Opening problem                     ")
    print("***************************************")
    kautham.kOpenProblem(modelFolder,kauthamProblemFile)

    #Solve query
    # print("***************************************")
    # print("   Solving Query                       ")
    # print("***************************************")
    # path=kautham.kGetPath(1)

    #Write path to taskfile
    # if path:
    #     #Save to file
    #     global taskfile
    #     kthconfig= ROSpackage_path+sys.argv[1]
    #     kthconfig= kthconfig.replace(directory,'')
    #     tfile =directory+'taskfile_'+kthconfig
    #     taskfile = open(tfile, "w+")
    #     taskfile.write("<?xml version=\"1.0\"?>\n")
    #     taskfile.write("<Task name= \"%s\" >\n" % kauthamproblem)
    #     taskfile.write("\t<Transit>\n")
    #     k= sorted(list(path.keys()))[-1][1]+1
    #     for i in range(int(len(path.keys())/k)-1):
    #       tex=''
    #       for j in range(0,k):
    #           tex=tex + str(path[i,j]) + " "  
    #       writePath(taskfile,tex)
    #     taskfile.write("\t</Transit>\n")

    #Close and save XML document
    # taskfile.write("</Task>")
    # taskfile.close()
    # print("Results saved in ", taskfile)

    use_joint_state_publisher_gui = False
    kautham.kVisualizeScene(use_joint_state_publisher_gui, rvizconfigfile)

    controls = (0.1,0.2,0.3,0.4,0.5,0.6,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5)
    kautham.kMoveRobot(controls)


    poseobject = (0.025,0.025,0.02,0,0,0,1)
    kautham.kSetObstaclePos("pawnB1",poseobject)

    
    input("Press Enter to Finalize...")

    #Close kautham problem
    kautham.kCloseProblem()


if __name__ == '__main__':
    try:
        #Run the main function 
        main()
    except rospy.ROSInterruptException:
        pass
