#include "EmergencyButtonProvider.h"
#include <cstring>

MAKE_MODULE(EmergencyButtonProvider);

EmergencyButtonProvider::EmergencyButtonProvider()
{
  #ifdef TARGET_ROBOT
    VERIFY(socket.setBlocking(false));
    VERIFY(socket.bind("0.0.0.0", udpPort));
  #endif
}


void EmergencyButtonProvider::update(EmergencyButtonState& buttonstate)
{
  #ifdef TARGET_ROBOT
    while((socket.read(socketDataBuffer, sizeof(socketDataBuffer))) > 0)
    {
      //Set button state to pressed on emergency message
      if(!strcmp(socketDataBuffer, "emergency"))
        buttonstate.buttonPressed = true;

      //Reset countdown if connected and no emergency
      else if(!strcmp(socketDataBuffer, "noemergency"))
      {
        buttonstate.buttonPressed = false;
        lastPingTime = theFrameInfo.time;
      }
    }

    if(theFrameInfo.getTimeSince(lastPingTime) > static_cast<int>(maxWaitTimeMillis))
      buttonstate.buttonPressed = true;

  #endif

  //Always listen to debug requests
  DEBUG_RESPONSE("module:EmergencyButtonProvider:press")
    buttonstate.buttonPressed = true;

  DEBUG_RESPONSE_ONCE("module:EmergencyButtonProvider:release")
    buttonstate.buttonPressed = false;
}
