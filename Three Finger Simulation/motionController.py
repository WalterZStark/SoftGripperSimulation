#Author: Walter Stark
#Script for moving gripper to desired positions
import Sofa.Core
import numpy as np


# Function for determining current position of object relative to starting
def relativeDetermination(object):
    objectIntermediate = object.getObject('mstate').findData('position').value
    # Obtain XYZ Coordinates
    objectX = objectIntermediate[0][0]
    objectY = objectIntermediate[0][1] 
    objectZ = objectIntermediate[0][2] 
    
    return [objectX,objectY,objectZ]

# Function for determining absolute current position of object
def absoluteDetermination(object,beginLoc):
    relative = relativeDetermination(object) 
    absolute = np.add(beginLoc,relative)
    return absolute

# Contoller for determining position to move to
class MotionController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        # Define input variables
        self.fingers = args[0]
        self.offset = args[1]
        self.multiplier = args[2]
        self.object = args[3]
        
        # Beginning and ending location of object
        self.objBeginLoc = args[4]
        self.objFinalLoc = args[5]

        self.cables = args[6]
        self.method = args[7]
        
        # Beginning and Final Loc of Current Path
        self.beginLoc = args[8]
        self.finalLoc = args[9]
    # Function for finding direction of movement
    def directionDetermination (self):
        
        difference = [0,0,0]
        # Method for lifing/lowering assembly and going to final position
        if(self.method == 0):
            # begin loc - current assembly position, final loc - initial current positon +/- 100 in y
            difference = np.add(self.finalLoc,[ -x for x in self.beginLoc])
        # Method for locating object
        elif(self.method == 1):
            #absoluteLoc = absoluteDetermination(self.object,self.objBeginLoc)
            difference = -np.add([ -x for x in relativeDetermination(self.object)], self.beginLoc)
            difference [1] = 0 # Remove y Component
            
        
        elif(self.method == 2): 
            difference = np.add(self.finalLoc,[ -x for x in self.beginLoc])
            difference [1] = 0
            #print(relativeDetermination(self.object))  
        #print(difference)
         
        # Get absolute location of object
        #absoluteLoc = absoluteDetermination(self.object,self.beginLoc)
        #difference = np.add(self.finalLoc,-absoluteLoc)
        #difference [1] = 0 # Remove y Component
        # Check if on final step
        #print(difference)
        if (all(abs(i) <= self.multiplier  for i in difference)):
            return difference
        else:
            # Get the magnitude of the vector
            norm = np.linalg.norm(difference)
            # Get final direction
            direction = self.multiplier * difference / norm
            return direction

