#!/usr/bin/env python3
import rospy
import rospkg 
import sys
from std_msgs.msg import String, Time
from geometry_msgs.msg import Pose
from kautham.msg import fVector
from kautham.srv import * 
import random
rospack =rospkg.RosPack()
#Function that wraps the call to the kautham service that opens a problem
def kOpenProblem(modelFolder, problemFile):
    print(modelFolder)
    rospy.wait_for_service("/kautham_node/OpenProblem")
    kthopenproblem_srv = OpenProblem()
    kthopenproblem_srv.problem = problemFile
    kthopenproblem_srv.dir=[1]
    kthopenproblem_srv.dir[0] = modelFolder

    kthopenproblem_client =rospy.ServiceProxy("/kautham_node/OpenProblem", OpenProblem)
    k=kthopenproblem_client(kthopenproblem_srv.problem, kthopenproblem_srv.dir)
    if k.response is True:
        rospy.loginfo( "Kautham Problem opened correctly" )
    else:
        rospy.logerr( "ERROR Opening Kautham Problem" )
        rospy.logerr( "models folpder: %s", kthopenproblem_srv.dir[0] )
        rospy.logerr( "problem file: %s", kthopenproblem_srv.problem)

#Function that wraps the call to the kautham service that solves a problem
def kGetPath (printpath=0):
    #define server and client
    rospy.wait_for_service("/kautham_node/GetPath")
    getpath_srv=GetPath()
    getpath_client=rospy.ServiceProxy("/kautham_node/GetPath", GetPath)
    #Call the getpath service
    getpath_srv= getpath_client()

    #listen response
    if len(getpath_srv.response) is 0:
        print ("No path found")
        return False
    else:
        print ("Path Found")
        path = { (i,j):0 for i in range(len(getpath_srv.response)) for j in range(len(getpath_srv.response[0].v)) }
        #path=[[0]*7]*(len(getpath_srv.response))
        if printpath:
            for i in range(len(getpath_srv.response)):
                for j in range(len(getpath_srv.response[i].v)):
                    path[i,j]=getpath_srv.response[i].v[j]
                    print(path[i,j], end=" ")
                print()
        return path
    
# Function that wraps the call to the kautham service that closes a problem
def kCloseProblem():
    rospy.wait_for_service("/kautham_node/CloseProblem")
    kthcloseproblrm_srv = CloseProblem()
    kthcloseproblrm_client = rospy.ServiceProxy("/kautham_node/CloseProblem", CloseProblem)
    kthcloseproblrm_client()
    print( "Kautham Problem closed")

# Function that wraps the call to the kautham service that gets the computational time spent solving a query
def kGetPlannerComputationTime():
    #define server client
    rospy.wait_for_service("/kautham_node/GetLastPlanComputationTime")
    getlastplancomputationtime_srv=GetLastPlanComputationTime()
    getlastplancomputationtime_client= rospy.ServiceProxy("/kautham_node/GetLastPlanComputationTime",GetLastPlanComputationTime)
    getlastplancomputationtime_srv= getlastplancomputationtime_client()
    print("Computation Time= ",end=" ")
    print(getlastplancomputationtime_srv.time)
    return getlastplancomputationtime_srv.time

# Function that wraps the call to the kautham service that checks for collision
def kIsCollisionFree (controls,index):
    rospy.wait_for_service("/kautham_node/CheckCollision")
    checkcollision_srv= CheckCollision()
    checkcollision_client=rospy.ServiceProxy("/kautham_node/CheckCollision",CheckCollision)
    checkcollision_srv.config= controls
    checkcollision_srv.index=index
    r= checkcollision_client(checkcollision_srv.config,checkcollision_srv.index)
    return r.collisionFree

# Function that wraps the call to the kautham service that sets the planner parameters
def kSetPlannerParameter(parametername, paramatervalue):
    rospy.wait_for_service("/kautham_node/SetPlannerParameter")
    setplannerparameter_srv= SetPlannerParameter()
    setplannerparameter_client=rospy.ServiceProxy("/kautham_node/SetPlannerParameter",SetPlannerParameter)
    
    setplannerparameter_srv.parameter= parametername
    setplannerparameter_srv.value= paramatervalue
    r= setplannerparameter_client(setplannerparameter_srv.parameter, setplannerparameter_srv.value)
    return r.response

#Function that wraps the call to the kautham service that sets a planner to solve the opened problem
def kSetPlannerByName(plannername):
    rospy.wait_for_service("/kautham_node/SetPlannerByName")
    setplannerbyname_srv = SetPlannerByName()
    setplannerbyname_client= rospy.ServiceProxy("/kautham_node/SetPlannerByName",SetPlannerByName)
    setplannerbyname_srv.name=plannername
    r=setplannerbyname_client(setplannerbyname_srv.name)
    return r.response

# Function that wraps the call to the kautham service that sets a new query to be solved
def kSetQuery(init,goal):
    rospy.wait_for_service("/kautham_node/SetQuery")
    setquery_srv=SetQuery()
    setquery_client=rospy.ServiceProxy("/kautham_node/SetQuery",SetQuery)
    #load the query request
    setquery_srv.init = init
    setquery_srv.goal = goal
    #call the query service 
    r= setquery_client(setquery_srv.init, setquery_srv.goal)
    if r.response is False:
        print("Query has not been set")
    return r.response

# Function that wraps the call to the kautham service that moves the robot
def kMoveRobot(controls):
    #set robot config
    #define derver and client
    rospy.wait_for_service("/kautham_node/SetRobotsConfig")
    setrobotconfig_srv= SetRobotsConfig()
    setrobotconfig_cleint=rospy.ServiceProxy("/kautham_node/SetRobotsConfig",SetRobotsConfig)
    setrobotconfig_srv.config=controls
    r=setrobotconfig_cleint(setrobotconfig_srv.config)
    if r.response:
        print("Robot Moved correctly")
    else:
        print("ERROR failed moveRobot")
    return r.response

# Function that wraps the call to the kautham service that attaches an object to a given link
def kAttachObject(robotnumber, linknumber, objectnumber):
    rospy.wait_for_service("/kautham_node/AttachObstacle2RobotLink")
    attachobstacle2robotlink_srv = AttachObstacle2RobotLink()
    attachobstacle2robotlink_client= rospy.ServiceProxy("/kautham_node/AttachObstacle2RobotLink",AttachObstacle2RobotLink)
    attachobstacle2robotlink_srv.robot = robotnumber
    attachobstacle2robotlink_srv.link = linknumber
    attachobstacle2robotlink_srv.obs = objectnumber
    r=attachobstacle2robotlink_client(attachobstacle2robotlink_srv.robot,attachobstacle2robotlink_srv.link,attachobstacle2robotlink_srv.obs)
    print("Attached object",end=" ")
    print(objectnumber,end=" ")
    print("to link",end=" ")
    print(linknumber,end=" ")
    print("of robot",end=" ")
    print(robotnumber)
    return r.response

# Function that wraps the call to the kautham service that dettaches an attached object
def kDetachObject(objectnumber):
    rospy.wait_for_service("/kautham_node/DetachObstacle")
    detachobstacle_srv= DetachObstacle()
    detachobstacle_client= rospy.ServiceProxy("/kautham_node/DetachObstacle",DetachObstacle)
    detachobstacle_srv.obs=objectnumber
    r= detachobstacle_client(detachobstacle_srv.obs)
    print ("Detached object",end=" ")
    print(objectnumber)
    return r.response

#Function that wraps the call to the kautham service that gets the pose of an obstacle
def kObstaclePos(indexobs):
    rospy.wait_for_service("/kautham_node/GetObstaclePos")
    kauthamobstaclepos_srv = ObsPos()
    kauthamobstaclepos_srv.index= indexobs
    kauthamobstaclepos_srv.setPos =( 20 ,70, 40, 0, 0, 0, 1 )#Dummy for python error fix
    kauthamobstaclepos_client = rospy.ServiceProxy("/kautham_node/GetObstaclePos",ObsPos)
    r=kauthamobstaclepos_client(kauthamobstaclepos_srv.index,kauthamobstaclepos_srv.setPos)
    if len(r.getPos)>0:
        print("obstacle position = ", r.getPos)
    else:
        print("No position returned")
        return False
    return (r.getPos)

#Function that wraps the call to the kautham service that sets the position of obstcle
def kSetObstaclePos(index,pose):
    rospy.wait_for_service("/kautham_node/SetObstaclePos")
    kauthamobstaclepos_srv = ObsPos()
    kauthamobstaclepos_srv.index= index
    kauthamobstaclepos_srv.setPos=pose
    kauthamobstaclepos_client = rospy.ServiceProxy("/kautham_node/SetObstaclePos",ObsPos)
    r=kauthamobstaclepos_client(kauthamobstaclepos_srv.index,kauthamobstaclepos_srv.setPos)
    if r.response:
        print("Obstacle Set")
    else:
        print("Set obstacle failed")
    return (r.response)

#Function that wraps the call to the kautham service that gets the pose of an obstacle
def kGetRobotPos(indexobs):
    rospy.wait_for_service("/kautham_node/GetRobotPos")
    kauthamrobotpos_srv = ObsPos()
    kauthamrobotpos_srv.index= indexobs
    kauthamrobotpos_srv.setPos =( 20 ,70, 40, 0, 0, 0, 1 )#Dummy for python error fix
    kauthamrobotpos_client = rospy.ServiceProxy("/kautham_node/GetRobotPos",ObsPos)
    r=kauthamrobotpos_client(kauthamrobotpos_srv.index,kauthamrobotpos_srv.setPos)
    if len(r.getPos)>0:
        print("Robot position = ", r.getPos)
    else:
        print("No position returned")
        return False
    return (r.getPos)

#Function that wraps the call to the kautham service that gets the home pose of an obstacle
def kGetRobotHomePos(indexobs):
    rospy.wait_for_service("/kautham_node/GetRobotHomePos")
    kauthamrobothomepos_srv = ObsPos()
    kauthamrobothomepos_srv.index= indexobs
    kauthamrobothomepos_srv.setPos =( 20 ,70, 40, 0, 0, 0, 1 )#Dummy for python error fix
    kauthamrobothomepos_client = rospy.ServiceProxy("/kautham_node/GetRobotHomePos",ObsPos)
    r=kauthamrobothomepos_client(kauthamrobothomepos_srv.index,kauthamrobothomepos_srv.setPos)
    if len(r.getPos)>0:
        print("Robot home position = ", r.getPos)
    else:
        print("No position returned")
        return False
    return (r.getPos)

#Function that wraps the call to the kautham service that sets the position of Robot
def kSetRobotPos(index,pose):
    rospy.wait_for_service("/kautham_node/SetRobotPos")
    kauthamrobotspos_srv = ObsPos()
    kauthamrobotspos_srv.index= index
    kauthamrobotspos_srv.setPos=pose
    kauthamrobotspos_client = rospy.ServiceProxy("/kautham_node/SetRobotPos",ObsPos())
    r=kauthamrobotspos_client(kauthamrobotspos_srv.index,kauthamrobotspos_srv.setPos)
    if r.response:
        print("Robot Set")
    else:
        print("Set robot failed")
    return (r.response)

#Function that wraps the call to the kautham service that sets the control file for the robot and sets the query to kautham
def kSetRobControls(controls, init, goal):
    rospy.wait_for_service("/kautham_node/SetRobControls")
    kauthamsetrobcontrols_srv= SetRobControls()
    kauthamsetrobcontrols_srv.controls=controls
    kauthamsetrobcontrols_srv.init=init
    kauthamsetrobcontrols_srv.goal= goal
    kauthamsetrobcontrols_client = rospy.ServiceProxy("/kautham_node/SetRobControls", SetRobControls())
    r=kauthamsetrobcontrols_client(kauthamsetrobcontrols_srv.controls,kauthamsetrobcontrols_srv.init,kauthamsetrobcontrols_srv.goal)
    if r.response:
        print("Control Set")
    else:
        print("Set Robot control failed")
    return (r.response)

#Function that wraps the call to the kautham service that sets the control file for the robot
def kSetRobControlsNoQuery(controls):
    rospy.wait_for_service("/kautham_node/SetRobControlsNoQuery")
    kauthamsetrobcontrolsnoquery_srv= SetRobControlsNoQuery()
    kauthamsetrobcontrolsnoquery_srv.controls=controls
    kauthamsetrobcontrolsnoquery_client = rospy.ServiceProxy("/kautham_node/SetRobControlsNoQuery", SetRobControlsNoQuery())
    r=kauthamsetrobcontrolsnoquery_client(kauthamsetrobcontrolsnoquery_srv.controls)
    if r.response:
        print("Control Set")
    else:
        print("Set Robot control failed")
    return (r.response)

#Function that wraps the call to the kautham service that gets the pose of an obstacle
def kRemoveObstacle(indexobs):
    rospy.wait_for_service("/kautham_node/RemoveObstaclePos")
    kauthamremoveobstacle_srv= RemoveObstacle()
    kauthamremoveobstacle_srv.index= indexobs
    kauthamremoveobstacle_client = rospy.ServiceProxy("/kautham_node/RemoveObstaclePos",ObsPos)
    r=kauthamremoveobstacle_client(kauthamremoveobstacle_srv.index)
    if r.response:
        print("Obstacle removed")
    else:
        print("Obstacle NOT removed")
    return r.response

#Function that wraps the call to the kautham service that gets the Inverse Kinematics
def kFindIK (pos, robIndex, armType, maintSameWrist,conf):
    rospy.wait_for_service("/kautham_node/FindIK")
    kauthamfindik_srv= FindIK()
    kauthamfindik_srv.pos= pos
    kauthamfindik_srv.robIndex = robIndex
    kauthamfindik_srv.armType = armType
    kauthamfindik_srv.maintSameWrist= maintSameWrist
    kauthamfindik_srv.conf=conf
    kauthamfindik_client = rospy.ServiceProxy("/kautham_node/FindIK",FindIK)
    r=kauthamfindik_client(kauthamfindik_srv.pos,kauthamfindik_srv.robIndex,kauthamfindik_srv.armType ,kauthamfindik_srv.maintSameWrist,kauthamfindik_srv.conf)
    if r.response:
        print(r.conf)
        print("Obstacle removed")
    else:
        print("Obstacle NOT removed")
    return r.response
