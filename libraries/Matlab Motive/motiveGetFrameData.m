function outData = motiveGetFrameData(natnetclient)
    data = natnetclient.getFrame;
    outData = double(data.RigidBody(1).y);