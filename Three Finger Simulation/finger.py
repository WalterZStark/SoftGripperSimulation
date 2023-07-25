# -*- coding: utf-8 -*-


#Code Structure Derived From: https://softroboticstoolkit.com/sofa/tutorial

#Author: Walter Stark

import sys
sys.path.append('C:/users/walte/SOFA/v22.12.00/plugins/STLIB/lib/python3/site-packages') # This needs to be changed for whoever's computer is running it
import math
import Sofa.Core
import Sofa.constants.Key as Key
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
import numpy as np
import os


# Radius of palm global variable definition
radius = 80 /2
# Basic path creation from location of pyscn file
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

# Controller that is responsible for opening and closing fingers
class FingerController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cables = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        for cable in self.cables:
            displacement = cable.CableConstraint.value[0]
            if e["key"] == Key.plus:
                displacement += 1.

            elif e["key"] == Key.minus:
                displacement -= 1.
                '''
                if displacement < 0:
                    displacement = 0
                '''
            cable.CableConstraint.value = [displacement]

# Function for creating a the whole finger assembly
def Finger(parentNode=None, name="Finger",
           rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[-100, -40, -100, 100, 40, 100], pullPointLocation=[-7.0, 0, 13.0 + radius]):
    finger = parentNode.addChild(name)
    
    # Create elastic object and define material,visual, and mesh properities
    eobject = ElasticMaterialObject(finger,
                volumeMeshFileName=path+'Three Finger Assembly/Three Finger Assembly - VTK - 80D.vtk',
                poissonRatio=0.3,
                youngModulus=1800, 
                totalMass=.4,
                surfaceColor=[0.0, 0.8, 0.7, 1.0],
                surfaceMeshFileName=path+"Three Finger Assembly/Three Finger Assembly - STL - 80D.stl",
                rotation=rotation,
                translation=translation)
    finger.addChild(eobject)
    
    # Create box to define what part of the gripper won't be influenced by the deformation of the fingers
    FixedBox(eobject, atPositions=fixingBox, doVisualization=True)
    
    # Matricies to setup the 2nd and 3rd cable matricies
    roatationMatrix2 = [[ math.cos(math.radians(120)),   0,   math.sin(math.radians(120)) ],
                      [     0,       1,       0     ],
                      [ -math.sin(math.radians(120)),  0,   math.cos(math.radians(120)) ]]
    roatationMatrix3 = [[ math.cos(math.radians(-120)),   0,   math.sin(math.radians(-120)) ],
                      [     0,       1,       0     ],
                      [ -math.sin(math.radians(-120)),  0,   math.cos(math.radians(-120)) ]]
    
    # Initial matrix for one of the fingers defining locations of each of the cables
    cable1Matrix = [
                                [-7.0, -56.5, 13.0 + radius],
                                [-7.0, -80, 13.0 + radius],
                                [-7.0, -106.25, 13.0 + radius],
                                [-7.0, -128.75,  13.0 + radius],

                                [-7.0, -148.5, 13.0 + radius],
                                [-4.0, -152.5, 13.0 + radius],
                                [-2.5, -153.5, 13.0 + radius],
                                [2.5, -153.5, 13.0 + radius],
                                [4.0, -152.5, 13.0 + radius],
                                [7.0, -148.5, 13.0 + radius],

                                [7.0, -128.75, 13.0 + radius],
                                [7.0, -106.25, 13.0 + radius],
                                [7.0, -80, 13.0 + radius],
                                [7.0, -56.5, 13.0 + radius]
                                ]
    
    # Define each of the cable objects and use the first cable matrix to help define the other two matricies
    cable = PullingCable(eobject,
                         "PullingCable",
                         pullPointLocation=pullPointLocation,
                         rotation=rotation,
                         translation=translation,
                         cableGeometry=cable1Matrix);
    cable_2 = PullingCable(eobject,
                       "PullingCable_2",
                       pullPointLocation=np.matmul(pullPointLocation,roatationMatrix2),
                       rotation=rotation,
                       translation=translation,
                       cableGeometry=np.matmul(cable1Matrix,roatationMatrix2)
                      )
    cable_3 = PullingCable(eobject,
                       "PullingCable_3",
                       pullPointLocation=np.matmul(pullPointLocation,roatationMatrix3),
                       rotation=rotation,
                       translation=translation,
                       cableGeometry= np.matmul(cable1Matrix,roatationMatrix3)
                           )
    eobject.addObject(FingerController([cable,cable_2,cable_3]))

    # Set collision mesh for the the three finger assembly
    CollisionMesh(eobject, name="CollisionMesh",
                  surfaceMeshFileName=path+"Three Finger Assembly/Three Finger Assembly - STL - 80D.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1,2])
    cables = [cable,cable_2,cable_3]
    return finger,cables

# Function to open and close fingers
def open(cables,opened, amount):
    # Loops through every cable in set of cables
    for cable in cables:
            if opened:
                displacement = amount
            else:
                displacement = -amount
            cable.CableConstraint.value = [displacement]
# Define  createscene function for creating root node
def createScene(rootNode):
    from stlib3.scene import MainHeader, ContactHeader
    # Initial collision property/node setup
    MainHeader(rootNode, gravity=[0.0, -981.0, 0.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=.3)
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    # Test call figner
    Finger(rootNode, translation=[0.0, 0.0, 0.0],rotation=[0.0,0.0,0.0])
    return rootNode


