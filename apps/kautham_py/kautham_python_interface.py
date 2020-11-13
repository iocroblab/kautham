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
                    print(round(path[i,j],3), end=" ")
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


# Function that wraps the call to the kautham service that gets the number of edges
def kGetNumEdges():
    #define server client
    rospy.wait_for_service("/kautham_node/GetNumEdges")
    getnumedges_srv=GetNumEdges()
    getnumedges_client= rospy.ServiceProxy("/kautham_node/GetNumEdges",GetNumEdges)
    getnumedges_srv= getnumedges_client()
    print("Num edges= ",getnumedges_srv.num)
    return getnumedges_srv.num

# Function that wraps the call to the kautham service that gets the number of vertices
def kGetNumVertices():
    #define server client
    rospy.wait_for_service("/kautham_node/GetNumVertices")
    getnumvertices_srv=GetNumEdges()
    getnumvertices_client= rospy.ServiceProxy("/kautham_node/GetNumVertices",GetNumVertices)
    getnumvertices_srv= getnumvertices_client()
    print("Num vertices= ",getnumvertices_srv.num)
    return getnumvertices_srv.num

# Function that wraps the call to the kautham service that checks for collision
def kIsCollisionFree (controls):
    rospy.wait_for_service("/kautham_node/CheckCollision")
    checkcollision_srv= CheckCollision()
    checkcollision_client=rospy.ServiceProxy("/kautham_node/CheckCollision",CheckCollision)
    checkcollision_srv.config= controls
    r= checkcollision_client(checkcollision_srv.config,checkcollision_srv.index)
    return r.collisionFree

# Function that wraps the call to the kautham service that test for a rectilinear-free connection between samples
def kConnect(sample1, sample2):
    rospy.wait_for_service("/kautham_node/Connect")
    connect_srv= Connect()
    connect_client=rospy.ServiceProxy("/kautham_node/Connect",Connect)
    
    connect_srv.sample1= sample1
    connect_srv.sample2= sample2
    r= connect_client(connect_srv.sample1, connect_srv.sample2)
    return r.response

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


# Function that wraps the call to the kautham service that sets a the init of a new query to be solved
def kSetInit(init):
    rospy.wait_for_service("/kautham_node/SetInit")
    setinit_srv=SetInit()
    setinit_client=rospy.ServiceProxy("/kautham_node/SetInit",SetInit)
    #load the query request
    setinit_srv.init = init
    #call the query service 
    r= setinit_client(setinit_srv.init)
    if r.response is False:
        print("Init has not been set")
    return r.response


# Function that wraps the call to the kautham service that sets a the init of a new query to be solved
def kSetGoal(goal):
    rospy.wait_for_service("/kautham_node/SetGoal")
    setgoal_srv=SetGoal()
    setgoal_client=rospy.ServiceProxy("/kautham_node/SetGoal",SetGoal)
    #load the query request
    setinit_srv.goal = goal
    #call the query service 
    r= setgoal_client(setgoal_srv.goal)
    if r.response is False:
        print("Goal has not been set")
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
def kGetObstaclePos(indexobs):
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

#Function that wraps the call to the kautham service that gets the pose of a robot
def kGetRobotPos(indexrob):
    rospy.wait_for_service("/kautham_node/GetRobotPos")
    kauthamrobotpos_srv = ObsPos()
    kauthamrobotpos_srv.index= indexrob
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
        print("Robot Pos Set")
    else:
        print("Set robot pos failed")
    return (r.response)

#Function that wraps the call to the kautham service that sets the control file for the robot and sets the query to kautham
def kSetRobControls(controls, init, goal):
    rospy.wait_for_service("/kautham_node/SetRobControls")
    kauthamsetrobcontrols_srv= SetRobControls()
    kauthamsetrobcontrols_srv.controls = controls
    kauthamsetrobcontrols_srv.init = init
    kauthamsetrobcontrols_srv.goal = goal
    kauthamsetrobcontrols_client = rospy.ServiceProxy("/kautham_node/SetRobControls", SetRobControls())
    r=kauthamsetrobcontrols_client(kauthamsetrobcontrols_srv.controls,kauthamsetrobcontrols_srv.init,kauthamsetrobcontrols_srv.goal)
    if r.response:
        print("Controls Set")
    else:
        print("Set Controls failed")
    return (r.response)

#Function that wraps the call to the kautham service that sets the control file for the robot
def kSetRobControlsNoQuery(controls):
    rospy.wait_for_service("/kautham_node/SetRobControlsNoQuery")
    kauthamsetrobcontrolsnoquery_srv= SetRobControlsNoQuery()
    kauthamsetrobcontrolsnoquery_srv.controls=controls
    kauthamsetrobcontrolsnoquery_client = rospy.ServiceProxy("/kautham_node/SetRobControlsNoQuery", SetRobControlsNoQuery())
    r=kauthamsetrobcontrolsnoquery_client(kauthamsetrobcontrolsnoquery_srv.controls)
    if r.response:
        print("Controls and query Set")
    else:
        print("Set Controls and query failed")
    return (r.response)


#Function that wraps the call to the kautham service that sets the default controls for the robot and sets the query to kautham
def kSetDefaultRobControls(init, goal):
    rospy.wait_for_service("/kautham_node/SetDefaultRobControls")
    kauthamsetdefaultrobcontrols_srv= SetDefaultRobControls()
    kauthamsetdefaultrobcontrols_srv.init = init
    kauthamsetdefaultrobcontrols_srv.goal = goal
    kauthamsetdefaultrobcontrols_client = rospy.ServiceProxy("/kautham_node/SetDefaultRobControls", SetDefaultRobControls())
    r=kauthamsetdefaultrobcontrols_client(kauthamsetdefaultrobcontrols_srv.init,kauthamsetdefaultrobcontrols_srv.goal)
    if r.response:
        print("Default Controls Set")
    else:
        print("Set Default Robot control failed")
    return (r.response)

#Function that wraps the call to the kautham service that removes an obstacle
def kRemoveObstacle(indexobs):
    rospy.wait_for_service("/kautham_node/RemoveObstacle")
    kauthamremoveobstacle_srv = RemoveObstacle()
    kauthamremoveobstacle_srv.index = indexobs
    kauthamremoveobstacle_client = rospy.ServiceProxy("/kautham_node/RemoveObstacle",RemoveObstacle)
    r=kauthamremoveobstacle_client(kauthamremoveobstacle_srv.index)
    if r.response:
        print("Obstacle removed")
    else:
        print("Obstacle NOT removed")
    return r.response


#Function that wraps the call to the kautham service that adds of an obstacle
def kAddObstacle(indexobs, scale, home):
    rospy.wait_for_service("/kautham_node/AddObstacle")
    kauthamaddobstacle_srv= AddObstacle()
    kauthamaddobstacle_srv.index = indexobs
    kauthamaddobstacle_srv.scale = scale
    kauthamaddobstacle_srv.home = home
    kauthamaddobstacle_client = rospy.ServiceProxy("/kautham_node/AddObstacle",AddObstacle)
    r=kauthamaddbstacle_client(kauthamaddobstacle_srv.index)
    if r.response:
        print("Obstacle added")
    else:
        print("Obstacle NOT added")
    return r.response

#Function that wraps the call to the kautham service that gets the Inverse Kinematics
def kFindIK (pos, robIndex, armType, maintSameWrist,conf):
    rospy.wait_for_service("/kautham_node/FindIK")
    kauthamfindik_srv= FindIK()
    kauthamfindik_srv.pos = pos
    kauthamfindik_srv.robIndex = robIndex
    kauthamfindik_srv.armType = armType
    kauthamfindik_srv.maintSameWrist = maintSameWrist
    kauthamfindik_srv.conf = conf
    kauthamfindik_client = rospy.ServiceProxy("/kautham_node/FindIK",FindIK)
    r=kauthamfindik_client(kauthamfindik_srv.pos,kauthamfindik_srv.robIndex,kauthamfindik_srv.armType ,kauthamfindik_srv.maintSameWrist,kauthamfindik_srv.conf)
    if r.response:
        print(r.conf)
        print("Obstacle removed")
    else:
        print("Obstacle NOT removed")
    return r.response


# Function that wraps the call to the kautham service that visualizes the scene
def kVisualizeScene (guisliders):
    print("Waiting for /kautham_node/VisualizeScene")
    rospy.wait_for_service("/kautham_node/VisualizeScene")
    visualizationscene_srv= VisualizeScene()
    visualizationscene_srv.guisliders = guisliders
    visualizationscene_client=rospy.ServiceProxy("/kautham_node/VisualizeScene",VisualizeScene)
    print("Calling /kautham_node/VisualizeScene")
    r = visualizationscene_client(visualizationscene_srv.guisliders)
    return True
