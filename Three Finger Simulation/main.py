
#Code Structure Derived From: https://softroboticstoolkit.com/sofa/tutorial

#Author: Walter Stark

# -*- coding: utf-8 -*-
import sys
sys.path.append('C:/users/walte/SOFA/v22.12.00/plugins/STLIB/lib/python3/site-packages') # This needs to be changed for whoever's computer is running it
import os
from stlib3.scene import MainHeader, ContactHeader
from stlib3.physics.rigid import Floor, Cube, RigidObject
from gripper import Gripper

# Basic path creation from location of pyscn file
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

# Define  createscene function for creating root node
def createScene(rootNode):
   
    # Initial collision property/node setup
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=7, contactDistance=3, frictionCoef=1.5) # Was 7 for alarm distance
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    
    # Set Background color
    rootNode.addObject('BackgroundSetting', color=[1,1,1,1])

    

    # Define floor properties
    Floor(rootNode,
          color=[1.0, 1.0, 1.0, 1.0],
          uniformScale=10,
          translation=[0.0, -160.0, 0.0],
          isAStaticObject=True)

    # Set Beginning Locations
    objBeginLoc = [[0.0, -120, -210.0],[-400, -160, -210.0], [400, -135, -210.0]]
    
    # Implement Pear Object
    pear = rootNode.addChild("pear")
    pear_rigid = RigidObject(surfaceMeshFileName=path+"pear.obj", isAStaticObject=False, translation=objBeginLoc[0], rotation=[0.0, 0.0, 0.0], color=[0, 1, 0, 1],uniformScale=1.2,totalMass=0.1 ,volume=20,inertiaMatrix=[1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0])
    pear_rigid.addObject('UncoupledConstraintCorrection')
    pear.addChild(pear_rigid)

    

    # Implement Espresso Object
    espresso = rootNode.addChild("espresso")
    espresso_rigid = RigidObject(surfaceMeshFileName=path+"espresso.obj", isAStaticObject=False, translation=objBeginLoc[1], rotation=[-90, 0.0, 0.0], color=[1, 1, 1, 1],uniformScale=1.2,totalMass=0.1 ,volume=20,inertiaMatrix=[1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0])
    espresso_rigid.addObject('UncoupledConstraintCorrection')
    espresso.addChild(espresso_rigid)

    # Implement Apple Object
    apple = rootNode.addChild("apple")
    apple_rigid = RigidObject(surfaceMeshFileName=path+"apple.obj", isAStaticObject=False, translation=objBeginLoc[2], rotation=[0.0, 0.0, 0.0], color=[1, 0, 0, 1],uniformScale=1.2,totalMass=0.1 ,volume=20,inertiaMatrix=[1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1000.0])
    apple_rigid.addObject('UncoupledConstraintCorrection')
    apple.addChild(apple_rigid)

    
    # Initialize values for gripper
    offset = 160
    multiplier = 5
    object_in = [pear_rigid,espresso_rigid,apple_rigid]
    objFinalLoc = [[0.0, -125, 200.0],[-400, -160, 210.0], [400, -140, 210.0]] # Added 100 to beginLoc Z


    # Define gripper
    Gripper(rootNode, offset, multiplier, object_in, objBeginLoc, objFinalLoc)

    return rootNode
