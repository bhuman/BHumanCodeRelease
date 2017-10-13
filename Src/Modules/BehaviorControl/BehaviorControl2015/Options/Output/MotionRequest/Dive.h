//
//  Dive.h
//  Naova
//
//  Created by Marc-Antoine on 17-10-11.
//
option(Dive){
    
    initial_state(setRequest)
    {
        transition
        {
            if(theMotionInfo.motion == MotionRequest::dive)
                goto requestIsExecuted;
        }
        action
        {
            theMotionInfo.motion = MotionRequest::dive;
        }
    }
    
    target_state(requestIsExecuted)
    {
        transition
        {
            if(theMotionInfo.motion != MotionRequest::dive)
                goto setRequest;
        }
        action
        {
            theMotionInfo.motion = MotionRequest::dive;
        }
    }
}
