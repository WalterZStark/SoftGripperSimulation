# -*- coding: utf-8 -*-

#Code Structure Derived From: https://softroboticstoolkit.com/sofa/tutorial

#Author: Walter Stark

from grippercontroller import GripperController
from finger import Finger

# Define gripper function that is composed of the finger assembly and gripper controller
def Gripper(parentNode, offset, multiplier, object_in, objBeginLoc, objFinalLoc):
    # Create gripper node
    selfNode = parentNode.addChild("Gripper")

    # Define finger assembly
    f1,cables = Finger(selfNode, "Finger1",  rotation = [0,0,0])

    # Add to gripper node, gripper controller (Call gripper control class)
    selfNode.addObject(GripperController([f1], offset, multiplier, object_in, objBeginLoc, objFinalLoc, cables))

    return selfNode

# Define  createscene function for creating root node
def createScene(rootNode):
    from stlib3.scene import MainHeader, ContactHeader
    # Initial collision property/node setup
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=.3)#was 0.08
    
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"

    Gripper(parentNode=rootNode)
    return rootNode
