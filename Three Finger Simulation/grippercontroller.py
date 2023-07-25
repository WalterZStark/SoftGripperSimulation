
#Code Structure Derived From: https://softroboticstoolkit.com/sofa/tutorial

#Author: Walter Stark

# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
from motionController import MotionController
from finger import open
import numpy as np

# Define global variables
initFound = False
radius = 40
initPullLoc = [-7.0, 0, 13.0 + radius]
initialPos = None

# getTranslated: Add direction to current position
def getTranslated(points, vec):
    r = []
    for v in points:
        r.append([v[0] + vec[0], v[1] + vec[1], v[2] + vec[2]])
    return r

# Find Initial Position of Assembly
def initialPosCalc (fingers):
    global initPullLoc
    m = fingers[0].getChild("ElasticMaterialObject")
    
    # Get all  cable object
    cable = m.getChild("PullingCable").getObject("CableConstraint")
    # Get the pulling point (starting point) for cable
    p = cable.pullPoint.value
    
    p = -np.add(-p,initPullLoc)
    return p

# Function to raise or lower assembly
def raiseAssembly(raised, fingers, amount):
    quantity = -amount
    global initFound, initialPos
    p = initialPosCalc (fingers)
    print(p)
    # determine whether to raise of lower
    if raised:
        quantity = amount
    #translateAssembly by 100 up and open fingers
    if not initFound:
            
        initialPos = [p[0],p[1]+ quantity,p[2]]
        initFound = True

    return initialPos, p



# Define function for translating full assembly
def translateAssembly(finger, direction):
    m = finger.getChild("ElasticMaterialObject")
    mecaobject = m.getObject("dofs")
    # Set rest position as the comination of current value and direction
    mecaobject.findData('rest_position').value = getTranslated(mecaobject.rest_position.value, direction)
    # Get all three cable objects
    cable = m.getChild("PullingCable").getObject("CableConstraint")
    cable2 = m.getChild("PullingCable_2").getObject("CableConstraint")
    cable3 = m.getChild("PullingCable_3").getObject("CableConstraint")
    # Get the pulling point (starting point) for each cable
    p = cable.pullPoint.value
    p2 = cable2.pullPoint.value
    p3 = cable3.pullPoint.value         
    # Set the pull point for each cable
    cable.findData("pullPoint").value = [p[0] + direction[0], p[1] + direction[1], p[2] + direction[2]]
    cable2.findData("pullPoint").value = [p2[0] + direction[0], p2[1] + direction[1], p2[2] + direction[2]]
    cable3.findData("pullPoint").value = [p3[0] + direction[0], p3[1] + direction[1], p3[2] + direction[2]]
    
    return

# GripperController: Class that translates gripper assembly
class GripperController(Sofa.Core.Controller):

    # Initialization Function to pass arguments
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        # Obtain fingers array
        self.fingers = args[0]
        self.name = "GripperController"
        self.offset = args[1]
        self.multiplier = args[2]
        self.object = args[3]
        self.objBeginLoc = args[4]
        self.objFinalLoc = args[5]
        self.cables = args[6]
        

        # Define current object number, time, initial position, and pauselength
        self.objNumber = 0
        self.time = 0.0
        self.pauseLen = 0.05
        

        # Define Boolean Arguments
        self.pause = False
        self.toLocate = False
        self.toDrop = False
        
        

    def onKeypressedEvent(self, e):
        direction = None

        if e["key"] == Key.uparrow:
            direction = [0.0, 1.0, 0.0]
        elif e["key"] == Key.downarrow:
            direction = [0.0, -1.0, 0.0]
        elif e["key"] == Key.leftarrow:
            direction = [5.0, 0.0, 0.0]
        elif e["key"] == Key.rightarrow:
            direction = [-5.0, 0.0, 0.0]

        # Check is a direction is set or if fingers were passed
        if direction is not None and self.fingers is not None:
            # Loop through each finger passed through
            for finger in self.fingers:
                translateAssembly(finger, direction)
    # Define actions that occur after animation
    def onAnimateBeginEvent (self,dt):
       
       '''
       controller = MotionController(self.fingers,self.offset, self.multiplier, self.object[self.objNumber], self.beginLoc[self.objNumber], self.finalLoc[self.objNumber],self.cables)
       direction = controller.directionDetermination()
       # Translate assembly in direction calulated
       for finger in self.fingers:
            translateAssembly(finger,direction)
            print(direction)
     '''
       global initFound
       
       # Check to see if finished going through all objects
       if(self.objNumber < len(self.object)):
           # Check gripper and lift fingers
            if(not self.toLocate):
               # Keep fingers open
               open(self.cables,True,5)
               # Get Values for raising assembly
               initialPos, beginningPos = raiseAssembly(True, self.fingers, 120)
               
               # Define controller
               controller = MotionController(self.fingers,self.offset, self.multiplier, self.object[self.objNumber],self.objBeginLoc[self.objNumber],self.objFinalLoc[self.objNumber],self.cables, 0,beginningPos, initialPos)
               direction = controller.directionDetermination()
               
               # Translate assembly in direction calulated
               translateAssembly(self.fingers[0],direction)
               
               magnitude = np.linalg.norm(direction)
               
               # Check if moving up is finished
               if magnitude < self.multiplier/2:
                   initFound = False
                   self.toLocate = True
            # Locate Object   
            elif(self.toLocate and not self.pause):
               print("Going to Locate object")
               
               
               # Get position of assembly
               beginningPos = initialPosCalc (self.fingers)
               # Define controller 
               controller = MotionController(self.fingers,self.offset, self.multiplier, self.object[self.objNumber], self.objBeginLoc[self.objNumber],self.objFinalLoc[self.objNumber], self.cables, 1, beginningPos, None)
               direction = controller.directionDetermination()
               
               # Translate assembly in direction calulated
               
               
               magnitude = np.linalg.norm(direction)
               
               # Check if moving to location is finished
               if magnitude < self.multiplier/2:
                   

                   # Get Values for lowering assembly
                   initialPos_lower, beginningPos_lower = raiseAssembly(False, self.fingers, 122)
                   
                   # Define controller
                   controller_lower = MotionController(self.fingers,self.offset, self.multiplier, self.object[self.objNumber],self.objBeginLoc[self.objNumber],self.objFinalLoc[self.objNumber],self.cables, 0,beginningPos_lower, initialPos_lower)
                   direction_lower = controller_lower.directionDetermination()
               
                   # Translate assembly in direction calulated
                   translateAssembly(self.fingers[0],direction_lower)
               
                   magnitude_lower = np.linalg.norm(direction_lower)
               
                   # Check if moving down is finished
                   if magnitude_lower < self.multiplier/2:
                      initFound = False
                      self.pause = True
                      # Close fingers
                      open(self.cables,False,8)

               else:
                   translateAssembly(self.fingers[0],direction)

               
            # Pause to allow hand to grasp
            elif(self.pause and not self.toDrop):

                self.time += dt['dt']
                print(self.time)
                # Check whether pause has expired
                if(self.time >= self.pauseLen):
                    open(self.cables,False,13)
                    # Get Values for raising assembly
                    initialPos, beginningPos = raiseAssembly(True, self.fingers, 122)
               
                    # Define controller
                    controller = MotionController(self.fingers,self.offset, self.multiplier, self.object[self.objNumber],self.objBeginLoc[self.objNumber],self.objFinalLoc[self.objNumber],self.cables, 0,beginningPos, initialPos)
                    direction = controller.directionDetermination()
                    
                        
                    #open(self.cables,False,8)
                    # Translate assembly in direction calulated
                    translateAssembly(self.fingers[0],direction)
               
                    magnitude = np.linalg.norm(direction)
               
                    # Check if moving up is finished
                    if magnitude < self.multiplier/2:
                        self.time = 0
                        
                        initFound = False
                        self.toDrop = True
                        
                        
                    
                    
                    
            # Go to drop point
            else:
                print("Going to drop point")
                # Get position of assembly
                beginningPos = initialPosCalc (self.fingers)

                # Get position to move to
                finalAssemblyPosition = [self.objFinalLoc[self.objNumber][0],self.objFinalLoc[self.objNumber][1]+self.offset,self.objFinalLoc[self.objNumber][2]]

                # Define controller
                controller = MotionController(self.fingers,self.offset, self.multiplier, self.object[self.objNumber],self.objBeginLoc[self.objNumber],self.objFinalLoc[self.objNumber],self.cables, 2,beginningPos,finalAssemblyPosition )
                direction = controller.directionDetermination()
            
                # Translate assembly in direction calulated
                translateAssembly(self.fingers[0],direction)
            
                magnitude = np.linalg.norm(direction)
            
                # Check if moving to positon is finished
                if magnitude < self.multiplier/2:
                    
                    
                    # Get Values for raising assembly
                   initialPos_lower, beginningPos_lower = raiseAssembly(False, self.fingers, 100)
                   
                   # Define controller
                   controller_lower = MotionController(self.fingers,self.offset, self.multiplier, self.object[self.objNumber],self.objBeginLoc[self.objNumber],self.objFinalLoc[self.objNumber],self.cables, 0,beginningPos_lower, initialPos_lower)
                   direction_lower = controller_lower.directionDetermination()
               
                   # Translate assembly in direction calulated
                   translateAssembly(self.fingers[0],direction_lower)
               
                   magnitude_lower = np.linalg.norm(direction_lower)
               
                   # Check if moving down is finished
                   if magnitude_lower < self.multiplier/2:
                        self.time += dt['dt']
                        
                        if(self.time >= self.pauseLen):
                            # Open gripper
                            open(self.cables,True,5)
                        if(self.time >= 2*self.pauseLen):
                            self.time = 0
                            # Reset Booleans
                            initFound = False
                            self.pause = False
                            self.toLocate = False
                            self.toDrop = False
                            # Increment Object Number
                            self.objNumber = self.objNumber + 1
                        
                        
                    
                    
                    
                    


        
# Define  createscene function for creating root node
def createScene(rootNode):
    # Add grippercontroller to rootnode (Call GripperController)
    rootNode.addObject(GripperController(None))

    return
