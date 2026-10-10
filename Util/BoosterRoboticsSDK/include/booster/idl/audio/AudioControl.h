#pragma once

#include "AudioCommon.h"
#include "RegisterClientRequest.h"
#include "RegisterClientResponse.h"
#include "GetDoaAngleRequest.h"
#include "GetDoaAngleResponse.h"
#include "GetSystemMuteRequest.h"
#include "GetSystemMuteResponse.h"
#include "GetSystemVolumeRequest.h"
#include "GetSystemVolumeResponse.h"
#include "InitPlayerRequest.h"
#include "InitPlayerResponse.h"
#include "InitRecorderRequest.h"
#include "InitRecorderResponse.h"
#include "PlayerInfoResponse.h"
#include "RecorderInfoResponse.h"
#include "SendPcmData.h"
#include "ServiceResult.h"
#include "SessionRequest.h"
#include "SetPlayerVolume.h"
#include "SetSystemMute.h"
#include "SetSystemVolume.h"

namespace booster_msgs {
namespace audio {

using StartPlayerRequest = SessionRequest;
using StartPlayerResponse = ServiceResult;
using PausePlayerRequest = SessionRequest;
using PausePlayerResponse = ServiceResult;
using StopPlayerRequest = SessionRequest;
using StopPlayerResponse = ServiceResult;
using ResetPlayerRequest = SessionRequest;
using ResetPlayerResponse = ServiceResult;
using DestroyPlayerRequest = SessionRequest;
using DestroyPlayerResponse = ServiceResult;
using GetPlayerInfoRequest = SessionRequest;
using SendPcmDataResponse = ServiceResult;

using StartRecorderRequest = SessionRequest;
using StartRecorderResponse = ServiceResult;
using PauseRecorderRequest = SessionRequest;
using PauseRecorderResponse = ServiceResult;
using StopRecorderRequest = SessionRequest;
using StopRecorderResponse = ServiceResult;
using DestroyRecorderRequest = SessionRequest;
using DestroyRecorderResponse = ServiceResult;
using GetRecorderInfoRequest = SessionRequest;

using SetSystemVolumeResponse = ServiceResult;
using SetSystemMuteResponse = ServiceResult;
using SetPlayerVolumeResponse = ServiceResult;

}  // namespace audio
}  // namespace booster_msgs
