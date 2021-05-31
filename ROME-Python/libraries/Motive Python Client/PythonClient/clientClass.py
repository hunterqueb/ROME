from NatNetClient import NatNetClient
import math
import time

# we need to get a only a few things,
#   being able to determine which rigid body id is which
#   
#   position and orientation of every rigid body
#   ability to filter the data?

class NatNetClientClass():
    def __init__(self):
        self.streamingClient = NatNetClient()
        self.streamingClient.newFrameListener = self.receiveNewFrame
        self.streamingClient.rigidBodyListener = self.initReceiveRigidBodyFrame
        

        #dictionary to get the id of whatever robot, set to none for now
        self.RigidBodyID = {'GV' : None, 'AR2' : None}

        #dictionary to get the pos and ori of whatever robot, set to none for now
        self.RigidBodyPosition = {'GV': None, 'AR2': None}
        self.RigidBodyOrientation = {'GV': None, 'AR2': None}
        
        #create temp files that will help us get the which robot is which
        self.tempID = 2 * [None]
        self.tempPos = 2 * [None]
        self.tempOri = 2 * [None]
        self.initCounter = 0
        
        self.streamingClient.run()

    def __del__(self):
        pass

    # This is a callback function that gets connected to the NatNet client. It is called once frame
    def receiveNewFrame(self, frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                        labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
        # print( "Received frame", frameNumber )
        # print( "Received rigidBodyCount", rigidBodyCount )
        pass

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receiveRigidBodyFrame(self,id, position, rotation):
        # rotation is in the format of quaternion
        # print( "Received frame for rigid body", id )
        
        if id == self.RigidBodyID['GV']:
            self.RigidBodyPosition['GV'] = position
            self.RigidBodyOrientation['GV'] = rotation
        elif id == self.RigidBodyID['AR2']:
            self.RigidBodyPosition['AR2'] = position
            self.RigidBodyOrientation['AR2'] = rotation
        else:
            print("rigid body id does not match! throwing the frame data away...")
        
        # we should format the rotation to the euler and give the option for quaternion
    

    def initReceiveRigidBodyFrame(self, id, position, rotation):
        # this will will run twice, to get the id of both rigid bodies

        # store the data of the rigid bodies in temp variables
        self.tempID[self.initCounter] = id
        self.tempPos[self.initCounter] = position
        self.tempOri[self.initCounter] = rotation

        # counter to set make sure we do 2 loops of this function
        if self.initCounter == 0:
            self.initCounter += 1
        else:
            # one the second run of the function we compare the vertical axis values of the rigid bodies. from here, the arm will always have markers higher than the GV, 
            # so the depending on the value is what is stored in the dictonaries
            if self.tempPos[0].z > self.tempPos[1].z:
                self.RigidBodyID['AR2'] = self.tempID[0]
                self.RigidBodyID['GV'] = self.tempID[1]
                self.RigidBodyPosition['AR2'] = self.tempPos[0]
                self.RigidBodyPosition['GV'] = self.tempPos[1]
                self.RigidBodyOrientation['AR2'] = self.tempOri[0]
                self.RigidBodyOrientation['GV'] = self.tempOri[1]
            else:
                self.RigidBodyID['AR2'] = self.tempID[1]
                self.RigidBodyID['GV'] = self.tempID[0]
                self.RigidBodyPosition['AR2'] = self.tempPos[1]
                self.RigidBodyPosition['GV'] = self.tempPos[0]
                self.RigidBodyOrientation['AR2'] = self.tempOri[1]
                self.RigidBodyOrientation['GV'] = self.tempOri[0]
            
            # now we delete the temp variables
            del self.tempID
            del self.tempPos
            del self.tempOri
            del self.initCounter
                
            # set the listener to the regular rigidbodyframe listener
            self.streamingClient.rigidBodyListener = self.receiveRigidBodyFrame

    def quat2eul(self,robot):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        
        - inputs is a string of either the gv or robotic arm

        'GV' or 'AR2'
        
        - outputs the eular rotation

        """
        orientationQuat = self.RigidBodyOrientation[robot] 


        w = 0
        x = 0
        y = 0
        z = 0

        # here we need to take the self.RigidBodyOrientation['GV'] and convert it to the w x y and z variables
        # w is scalar part,

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians


if __name__ == "__main__":
    client = NatNetClientClass()
    
    time.sleep(5)

    del client
