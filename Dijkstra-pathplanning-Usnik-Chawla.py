"""
    This scipt returns the optimum path for the point robot to navigate through the obstacle space
    Here we take the initial and goal state for the robot from the text file states.txt
    In states.txt the first line contains the intial state and the second line contains the goal state
    In this scipt we use Dijkstras Algorithm for finding the optimal path
"""

import cv2
import numpy as np
from typing import List
from heapq import heappop,heappush
from turtle import color
import sys

from pkg_resources import yield_lines


#q is the list which we use for implementing Priority queue for Dijkstra algorithm
q=list()

#contains all the node of the graph that have been completely explored
#this list is used for visualization purposes
graph=list()

#Class for handling the states of point robot
class Node:
    
    #Constructor for the state requires coordinates in the map
    def __init__(self,coordinate):
        self.coordinate=coordinate
        self.parentIndex=None
        self.Index=None
        self.COC=-1
    
    #setter for updating the coc for the node    
    def updateCOC(self,cost: float):
        self.COC=cost
    
    #equality operator is overriden to amke sure that the only the coordinate information of the nodes is checked for equality
    def __eq__(self,object):
        return self.coordinate==object.coordinate
    
    #less operator is overriden as a requirement for heap 
    def __lt__(self,object):
        if(self.coordinate[0]<object.coordinate[0]):
            return True
        else:
            return False
        
        
    
def checkObstacle(x: int, y: int) -> bool:
    """
    This function checks wether a given node lies in the obstacle space

    Args:
        x (int): x coordinate of the node
        y (int): y coordinate of the node

    Returns:
        bool: retruns true if the node is in obstacle space
    """
    
    
    #Boundary equations for the hexagon.
    #These equations compensate for the 5mm clearence space
    a=y+0.58*x - 260.84
    b=x-240
    c=y-0.57*x + 60.84
    d=y+0.57*x - 170.02
    e=x-160
    f=y-0.58*x - 29.98
    
    
    
    #Boundary equations for the circle.
    #This equation compensate for the 5mm clearence space
    k=(x-300)**2 + (y-185)**2 -2025
    
    #Boundary equations for the 1st obstacel
    #These equations compensate for the 5mm clearence space
    #line p divides the obstacle into two convex polygons
    l=y+1.23*x-221.40
    m=y-0.31*x-178.85
    n=y-0.85*x-104.83
    o=y+3.20*x - 452.77
    p=y+0.15*x-191.89
    
    
    #boundary eqautions for the border 
    #these compensate for the 5mm clearence at the boundaries
    x1=x-5
    x2=x-395
    y1=y-5
    y2=y-245
    
    
    #conditions for checking if the node lies in the obstacle space
    if((a<=0 and b<=0 and c>=0 and d >=0 and e>=0 and f<=0) or k<=0 or (m<=0 and n>=0 and p>=0) or(l>=0 and o<=0 and p<=0) or (x1<=0 or x2>=0 or y1<=0 or y2>=0)):
        return True
    else:
        return False
       
    
def UP(node: Node):
    """
    This functions takes in a node and applies the action (0,1) to it.
    The coordinates of the new node are [Xnode,Ynode+1]
    This functions returns the refrence for the node if it is already in priority queue
    If the node is not in priority queue it creates a new node and returns it.
    This function does not return the node if it lies in obstacle space

    Args:
        node (Node): Node for which (0,1) action is to be applied

    Returns:
        bool,Node: returns True, node if it does not lie in the obstacle space
                   returns False,None if the new coordinate lies in the obstacle space
    """
    x=node.coordinate[0]
    y=node.coordinate[1]+1
    if(checkObstacle(x,y)):
        return False, None
    
    
    else:
        newNode = Node ([x,y])
        for i,element in  enumerate(q):
            oldcost,popnode=element 
            if popnode==newNode:
                return True,popnode
            
        return True, newNode
   
   
def DOWN(node: Node):
    """
    This functions takes in a node and applies the action (0,-1) to it.
    The coordinates of the new node are [Xnode,Ynode-1]
    This functions returns the refrence for the node if it is already in priority queue
    If the node is not in priority queue it creates a new node and returns it.
    This function does not return the node if it lies in obstacle space

    Args:
        node (Node): Node for which (0,-1) action is to be applied

    Returns:
        bool,Node: returns True, node if it does not lie in the obstacle space
                   returns False,None if the new coordinate lies in the obstacle space
    """
    x=node.coordinate[0]
    y=node.coordinate[1]-1
    if(checkObstacle(x,y)):
        return False, None
    
    else:
        newNode = Node ([x,y])
        for i,element in  enumerate(q):
            oldcost,popnode=element 
            if popnode==newNode:
                return True,popnode
            
        return True, newNode
        
        
    
def LEFT(node: Node):
    """
    This functions takes in a node and applies the action (-1,0) to it.
    The coordinates of the new node are [Xnode-1,Ynode]
    This functions returns the refrence for the node if it is already in priority queue
    If the node is not in priority queue it creates a new node and returns it.
    This function does not return the node if it lies in obstacle space

    Args:
        node (Node): Node for which (-1,0) action is to be applied

    Returns:
        bool,Node: returns True, node if it does not lie in the obstacle space
                   returns False,None if the new coordinate lies in the obstacle space
    """
    x=node.coordinate[0]-1
    y=node.coordinate[1]
    if(checkObstacle(x,y)):
        return False, node
    
    else:
        newNode = Node ([x,y])
        for i,element in  enumerate(q):
            oldcost,popnode=element 
            if popnode==newNode:
                return True,popnode
            
        return True, newNode
    
    
    
def RIGHT(node: Node):
    """
    This functions takes in a node and applies the action (1,0) to it.
    The coordinates of the new node are [Xnode-1,Ynode]
    This functions returns the refrence for the node if it is already in priority queue
    If the node is not in priority queue it creates a new node and returns it.
    This function does not return the node if it lies in obstacle space

    Args:
        node (Node): Node for which (1,0) action is to be applied

    Returns:
        bool,Node: returns True, node if it does not lie in the obstacle space
                   returns False,None if the new coordinate lies in the obstacle space
    """
    x=node.coordinate[0]+1
    y=node.coordinate[1]
    if(checkObstacle(x,y)):
        return False, None
    
    else:
        newNode = Node ([x,y])
        for i,element in  enumerate(q):
            oldcost,popnode=element 
            if popnode==newNode:
                return True,popnode
            
        return True, newNode
    
       
    
def UPRIGHT(node: Node):
    """
    This functions takes in a node and applies the action (1,1) to it.
    The coordinates of the new node are [Xnode+1,Ynode+1]
    This functions returns the refrence for the node if it is already in priority queue
    If the node is not in priority queue it creates a new node and returns it.
    This function does not return the node if it lies in obstacle space

    Args:
        node (Node): Node for which (1,1) action is to be applied

    Returns:
        bool,Node: returns True, node if it does not lie in the obstacle space
                   returns False,None if the new coordinate lies in the obstacle space
    """
    x=node.coordinate[0]+1
    y=node.coordinate[1]+1
    if(checkObstacle(x,y)):
        return False, None
    
    else:
        newNode = Node ([x,y])
        for i,element in  enumerate(q):
            oldcost,popnode=element 
            if popnode==newNode:
                return True,popnode
            
        return True, newNode
  
    

def DOWNRIGHT(node: Node):
    """
    This functions takes in a node and applies the action (1,-1) to it.
    The coordinates of the new node are [Xnode+1,Ynode-1]
    This functions returns the refrence for the node if it is already in priority queue
    If the node is not in priority queue it creates a new node and returns it.
    This function does not return the node if it lies in obstacle space

    Args:
        node (Node): Node for which (1,-1) action is to be applied

    Returns:
        bool,Node: returns True, node if it does not lie in the obstacle space
                   returns False,None if the new coordinate lies in the obstacle space
    """
    x=node.coordinate[0]+1
    y=node.coordinate[1]-1
    if(checkObstacle(x,y)):
        return False, None
    
    else:
        newNode = Node ([x,y])
        for i,element in  enumerate(q):
            oldcost,popnode=element 
            if popnode==newNode:
                return True,popnode
            
        return True, newNode
    
    
def DOWNLEFT(node: Node):
    """
    This functions takes in a node and applies the action (-1,-1) to it.
    The coordinates of the new node are [Xnode-1,Ynode-1]
    This functions returns the refrence for the node if it is already in priority queue
    If the node is not in priority queue it creates a new node and returns it.
    This function does not return the node if it lies in obstacle space

    Args:
        node (Node): Node for which (-1,-1) action is to be applied

    Returns:
        bool,Node: returns True, node if it does not lie in the obstacle space
                   returns False,None if the new coordinate lies in the obstacle space
    """
    x=node.coordinate[0]-1
    y=node.coordinate[1]-1
    if(checkObstacle(x,y)):
        return False, None
    
    else:
        newNode = Node ([x,y])
        for i,element in  enumerate(q):
            oldcost,popnode=element 
            if popnode==newNode:
                return True,popnode
            
        return True, newNode
    
    
def UPLEFT(node: Node):
    """
    This functions takes in a node and applies the action (-1,1) to it.
    The coordinates of the new node are [Xnode-1,Ynode+1]
    This functions returns the refrence for the node if it is already in priority queue
    If the node is not in priority queue it creates a new node and returns it.
    This function does not return the node if it lies in obstacle space

    Args:
        node (Node): Node for which (-1,1) action is to be applied

    Returns:
        bool,Node: returns True, node if it does not lie in the obstacle space
                   returns False,None if the new coordinate lies in the obstacle space
    """
    x=node.coordinate[0]-1
    y=node.coordinate[1]+1
    if(checkObstacle(x,y)):
        return False, None
    
    else:
        newNode = Node ([x,y])
        for i,element in  enumerate(q):
            oldcost,popnode=element 
            if popnode==newNode:
                return True,popnode
            
        return True, newNode
    
def visualize(path: List[int]):
    """
    This functions is implemented after we have found out the optimal path.
    This function is used to visualize how the Dijkstra's algorithm explores around and finds the
    optimal path betwen goal and inital node

    Args:
        path (List[int]): This list contains the index of the node on the optimal path
    """
    
    #creating an inflated array to visualize the obstacle space
    a=np.full((251,401,3),255)
    a=a.astype(np.uint8)
    
    #drawing the circle with colour set to blue
    a=cv2.circle(a,(300,250-185),45,(255,0,0),-1)
    
    #Constructing the boundary clearence 
    a[0:6,:,:]=[0,0,0]
    a[245:,:,:]=[0,0,0]
    a[:,0:6,:]=[0,0,0]
    a[:,395:,:]=[0,0,0]
    
    #creating the hexagon with color set to green
    pt1=np.array([[200,250-145],[240,250-122],
              [240,250-78],[200,250-55],
              [160,250-78],[160,250-122]])

    a=cv2.fillPoly(a,pts=[pt1],color=(0,255,0))

    #creating the obstacle 1 with color set to red
    pt2=np.array([[27,250-188],[137,250-223],[86,250-178],[118,250-77]])

    a=cv2.fillPoly(a,pts=[pt2],color=(0,0,255))
   
    #these two arrays are used to collect  records for the parent index and COC for each closed node in graph list 
    mat1=np.full((250,400),-3)
    mat2=np.full((250,400),-3)
    
    for node in graph:
        mat1[250-round(node.coordinate[1]),round(node.coordinate[0])]=node.parentIndex
        mat2[250-node.coordinate[1],node.coordinate[0]]=node.COC
      
    width= 401
    height= 251
    
    #writer= cv2.VideoWriter('outputVideo.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 600, (width,height))
       
    #These two for loops are used to extract and display the node in the order they were added to graph list  
    for i in range(-1,len(graph)-1):
        
        b=np.array(np.where(mat1==i)).T
        d=list()
        for x,y in b:
            d.append(mat2[x,y])
        d=np.array(d)
        inds=np.argsort(d)
        b=b[inds]
    
        for x,y in b:
        
            a[x,y,:]=[255,0,255]
        
            cv2.imshow("frame",a)
            #writer.write(a)
            cv2.waitKey(3)
    
    #This conatins the coordinates for the node on the optimal path
    coordList=list()        
    for index in path:
        coordList.append(graph[index].coordinate)
    #Drawing the optimal path    
    coordList=(np.array([0,250])-np.array(coordList))*np.array([-1,1])
    a=cv2.polylines(a,[coordList],isClosed=False,color=[0,0,0],thickness=1)
    cv2.imshow("frame",a)
    # for i in range(10):
    #     writer.write(a)
    # writer.release()
    cv2.waitKey(0)    
    cv2.destroyAllWindows()
        
    
    
def BackTrack(finalNode):
    
    """
    This function backtracks on the nodes from the goal state to inital state to form the path.
    This path is then written into file nodePath.txt.
    
    """
    #List containing the indexes for nodes in the path.
    path=list()
    
    #Adding the final node the list.
    path.append(finalNode.Index)
    
    #Iteratively adding the node along the path to the list.
    parent=finalNode.parentIndex
    while parent!=-1:
        
        #Bactracking along the path until the initial node is reached for which the parentIndex=-1.
        path.append(parent)
        parent=graph[parent].parentIndex
    
    #Arranging the path so that it is from start to goal.   
    path=path[::-1]
    
    #this method visualizes the algorithm
    visualize(path)
    
    
    
    
def mainAlgo(x: int, y: int, xg: int, yg:int):
    """
    This is the Dijkstras Algorithm for finding the optimal path between intial and goal node

    Args:
        xinit (int): x coordinate of the intial point
        yinit (int): y coordinate of the initial point
        xgaol (int): x coordinate of the goal point
        ygoal (int): y coordinate of the goal pint
    """
    
    #get the intial and final coordinate for the robot
    xinit,yinit=x,y
    xgoal,ygoal=xg,yg
    
    
    #check if the intial or final coordinate lies in the obstacle space    
    if checkObstacle(xinit,yinit):
        
        print("Intial point in obstacle space.\nRun again with new intial point")
    
    elif(checkObstacle(xgoal,ygoal)):
        
        print("Goal point in obstacle space.\nRun again with new Goal point")
        
        
    
    else:
        #Intitailzing the intial node
        initNode = Node([xinit,yinit])
        initNode.parentIndex=-1
        initNode.updateCOC(0)
        
        #Initializing the goal node
        goalNode = Node([xgoal,ygoal])
    
        #pushing the intial node onto the heap
        heappush(q, (initNode.COC,initNode))
        
        #Iterating through the heap while it is not empty
        while len(q)!=0:
            
            #removing the node from the heap and adding it to the graph as it is a closed node now
            _,node=heappop(q)
            index=len(graph)
            node.Index=index
            graph.append(node)
            
            #If the poped node is the goal node we have found the optimal path
            #we call the backtravk on this poped node for getting the path and visualizing it
            if(node==goalNode):
                
                goalNode=node
                print("Cost of the optimal Path: ", goalNode.COC)
                BackTrack(goalNode)
                break
            
            else:
                #If the poped node is not the goal node we explore all its children by 
                #applying the action set on the node
                
                
                #Applying the action (0,1) to the node.
                #If the possible new state is in obsatcle space we skip to the next action.
                #If the new state is already in the queue we check its COC and compare it with new COC obtained from the node.
                #If it more than the new COC we updates the states COC and parent parameter.
                #If the node is not queue we add it to the queue.
                notInObstacle,childNode=UP(node)   
                newcoc=node.COC+1
                if (notInObstacle) and (childNode not in graph):
                    if childNode.COC!=-1:
                        if newcoc<childNode.COC:
                            childNode.updateCOC(newcoc)
                            childNode.parentIndex=index
                            
                    else:
                        childNode.updateCOC(newcoc)
                        childNode.parentIndex=index
                        heappush(q,(newcoc,childNode))
                    
                    
                
                #Applying the action (1,1) to the node.
                #If the possible new state is in obsatcle space we skip to the next action.
                #If the new state is already in the queue we check its COC and compare it with new COC obtained from the node.
                #If it more than the new COC we updates the states COC and parent parameter.
                #If the node is not queue we add it to the queue.
                notInObstacle,childNode=UPRIGHT(node)    
                newcoc=node.COC+1.4
                if (notInObstacle) and (childNode not in graph):
                    if childNode.COC!=-1:
                        if newcoc<childNode.COC:
                            childNode.updateCOC(newcoc)
                            childNode.parentIndex=index
                            
                    else:
                        childNode.updateCOC(newcoc)
                        childNode.parentIndex=index
                        heappush(q,(newcoc,childNode))
                        
                    
            
            
                #Applying the action (1,0) to the node.
                #If the possible new state is in obsatcle space we skip to the next action.
                #If the new state is already in the queue we check its COC and compare it with new COC obtained from the node.
                #If it more than the new COC we updates the states COC and parent parameter.
                #If the node is not queue we add it to the queue.
                notInObstacle,childNode=RIGHT(node)
                newcoc=node.COC+1       
                if (notInObstacle) and (childNode not in graph):
                    if childNode.COC!=-1:
                        if newcoc<childNode.COC:
                            childNode.updateCOC(newcoc)
                            childNode.parentIndex=index
                            
                    else:
                        childNode.updateCOC(newcoc)
                        childNode.parentIndex=index
                        heappush(q,(newcoc,childNode))
            
            
            
                #Applying the action (1,-1) to the node.
                #If the possible new state is in obsatcle space we skip to the next action.
                #If the new state is already in the queue we check its COC and compare it with new COC obtained from the node.
                #If it more than the new COC we updates the states COC and parent parameter.
                #If the node is not queue we add it to the queue.
                notInObstacle,childNode=DOWNRIGHT(node)
                newcoc=node.COC+1.4 
                if (notInObstacle) and (childNode not in graph):
                    if childNode.COC!=-1:
                        if newcoc<childNode.COC:
                            childNode.updateCOC(newcoc)
                            childNode.parentIndex=index
                            
                    else:
                        childNode.updateCOC(newcoc)
                        childNode.parentIndex=index
                        heappush(q,(newcoc,childNode))      
            
                        
                        
            
        
                #Applying the action (-1,0) to the node.
                #If the possible new state is in obsatcle space we skip to the next action.
                #If the new state is already in the queue we check its COC and compare it with new COC obtained from the node.
                #If it more than the new COC we updates the states COC and parent parameter.
                #If the node is not queue we add it to the queue.
                notInObstacle,childNode=DOWN(node) 
                newcoc=node.COC+1
                if (notInObstacle) and (childNode not in graph):
                    if childNode.COC!=-1:
                        if newcoc<childNode.COC:
                            childNode.updateCOC(newcoc)
                            childNode.parentIndex=index
                            
                    else:
                        childNode.updateCOC(newcoc)
                        childNode.parentIndex=index
                        heappush(q,(newcoc,childNode))       
                
        
        
        
        
                #Applying the action (-1,-1) to the node.
                #If the possible new state is in obsatcle space we skip to the next action.
                #If the new state is already in the queue we check its COC and compare it with new COC obtained from the node.
                #If it more than the new COC we updates the states COC and parent parameter.
                #If the node is not queue we add it to the queue.
                notInObstacle,childNode=DOWNLEFT(node)   
                newcoc=node.COC+1.4
                if (notInObstacle) and (childNode not in graph):
                    if childNode.COC!=-1:
                        if newcoc<childNode.COC:
                            childNode.updateCOC(newcoc)
                            childNode.parentIndex=index
                            
                    else:
                        childNode.updateCOC(newcoc)
                        childNode.parentIndex=index
                        heappush(q,(newcoc,childNode))      
                
                            
                
                
                
                #Applying the action (0,-1) to the node.
                #If the possible new state is in obsatcle space we skip to the next action.
                #If the new state is already in the queue we check its COC and compare it with new COC obtained from the node.
                #If it more than the new COC we updates the states COC and parent parameter.
                #If the node is not queue we add it to the queue.
                notInObstacle,childNode=LEFT(node)     
                newcoc=node.COC+1       
                if (notInObstacle) and (childNode not in graph):
                    if childNode.COC!=-1:
                        if newcoc<childNode.COC:
                            childNode.updateCOC(newcoc)
                            childNode.parentIndex=index
                            
                    else:
                        childNode.updateCOC(newcoc)
                        childNode.parentIndex=index
                        heappush(q,(newcoc,childNode))
                            
                    
                    
                    
                #Applying the action (-1,1) to the node.
                #If the possible new state is in obsatcle space we skip to the next action.
                #If the new state is already in the queue we check its COC and compare it with new COC obtained from the node.
                #If it more than the new COC we updates the states COC and parent parameter.
                #If the node is not queue we add it to the queue.           
                notInObstacle,childNode=UPLEFT(node)     
                newcoc=node.COC+1.4
                if (notInObstacle) and (childNode not in graph):
                    if childNode.COC!=-1:
                        if newcoc<childNode.COC:
                            childNode.updateCOC(newcoc)
                            childNode.parentIndex=index
                            
                    else:
                        childNode.updateCOC(newcoc)
                        childNode.parentIndex=index
                        heappush(q,(newcoc,childNode))   
                        

#Program starts execution from here                        
if __name__=="__main__":
    xinit=int(sys.argv[1])
    yinit=int(sys.argv[2])
    xgoal=int(sys.argv[3])
    ygoal=int(sys.argv[4])   
    mainAlgo(xinit,yinit,xgoal,ygoal)         