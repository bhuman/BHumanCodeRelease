/*
 * File:   CognitionLogger.cpp
 * Author: Arne Böckmann
 *
 * Created on May 23, 2013, 2:59 PM
 */

#include "CognitionLogger.h"
#include "Tools/Debugging/Asserts.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Blackboard.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Settings.h"
#include <functional>
#include <sstream>
#include <vector>
#include "snappy-c.h"
#include "Tools/Module/ModuleManager.h"
#include "Tools/Debugging/LogFileFormat.h"
#ifdef TARGET_ROBOT
#include <sys/statvfs.h>
#endif
#include <algorithm>
using namespace std;

#define REGISTER_REPRESENTATION(name) \
  representations[#name] = make_pair(id##name,(Streamable*)&Blackboard::theInstance->the##name)

/**Parameters of the logger*/
struct CognitionLogger::Parameters : public Streamable
{
  std::string logFilePath; /**< Where to write the log file.*/
  unsigned int maxBufferSize; /**< Max size of the buffer in bytes. */
  unsigned int blockSize; /**< Size per frame. (in bytes) */
  std::vector<std::string> representations; /**< Contains the representations that should be logged. */
  bool enabled; /**< Determines whether the logger is enabled or disabled. */
  int writePriority;
  bool debugStatistics;
  unsigned minFreeSpace; /**< minimum free space left on the device */
  /**Initializes parameters from a config file */
  Parameters(const std::string& fileName)
  {
    InMapFile conf(fileName);
    ASSERT(conf.exists());

    if(conf.exists())
    {
      conf >> *this;
    }
    ASSERT(maxBufferSize > 0);
  }

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(logFilePath);
      STREAM(maxBufferSize);
      STREAM(blockSize);
      STREAM(representations);
      STREAM(enabled);
      STREAM(writePriority);
      STREAM(debugStatistics);
      STREAM(minFreeSpace);
    STREAM_REGISTER_FINISH;
  }
};

CognitionLogger::CognitionLogger(const ModuleManager& moduleManager) :
                                         params(*(new Parameters("logger.cfg"))),
                                         state(PRE_INITIAL),
                                         initialized(false),
                                         readIndex(0),
                                         writeIndex(0),
                                         frameCounter(0),
                                         framesToWrite(0),
                                         shouldPlayLogSound(false),
                                         moduleManager(moduleManager)
{
  initStateFunctors();
  writerThread.setPriority(params.writePriority);
}

void CognitionLogger::initStateFunctors()
{
  states[PRE_INITIAL] = std::bind(&CognitionLogger::preInitial, this);
  states[INITAL] = std::bind(&CognitionLogger::initial, this);
  states[IDLE] = std::bind(&CognitionLogger::idle, this);
  states[RUNNING] = std::bind(&CognitionLogger::running, this);
  states[ERROR_STATE] = std::bind(&CognitionLogger::error, this);
}

void CognitionLogger::initRepresentations()
{
  REGISTER_REPRESENTATION(RobotPose);
  REGISTER_REPRESENTATION(BallPercept);
  REGISTER_REPRESENTATION(LinePercept);
  REGISTER_REPRESENTATION(FieldBoundary);
  REGISTER_REPRESENTATION(ImageCoordinateSystem);
  REGISTER_REPRESENTATION(GoalPercept);
  REGISTER_REPRESENTATION(RobotsModel);
  REGISTER_REPRESENTATION(BallModel);
  REGISTER_REPRESENTATION(CombinedWorldModel);
  REGISTER_REPRESENTATION(CameraMatrix);
  REGISTER_REPRESENTATION(ObstacleModel);
  REGISTER_REPRESENTATION(BehaviorControlOutput);
  REGISTER_REPRESENTATION(MotionRequest);
  REGISTER_REPRESENTATION(FilteredJointData);
  REGISTER_REPRESENTATION(FrameInfo);
  REGISTER_REPRESENTATION(SideConfidence);
  REGISTER_REPRESENTATION(FilteredSensorData);
  REGISTER_REPRESENTATION(GameInfo);
  REGISTER_REPRESENTATION(OwnTeamInfo);
  REGISTER_REPRESENTATION(MotionInfo);
  REGISTER_REPRESENTATION(RobotHealth);
  REGISTER_REPRESENTATION(ColorReference);
  REGISTER_REPRESENTATION(Thumbnail);
  REGISTER_REPRESENTATION(ActivationGraph);
  REGISTER_REPRESENTATION(CameraInfo);
  REGISTER_REPRESENTATION(ObstacleSpots);
  REGISTER_REPRESENTATION(ObstacleWheel);
  REGISTER_REPRESENTATION(BodyContour);
}

void CognitionLogger::run()
{
  states[state]();
}

void CognitionLogger::preInitial()
{//wait till logger is enabled and blackboard is available, go to INITIAL if it is

  #ifdef TARGET_SIM
  params.enabled = false; //always disable inside simulator
  #endif

  if(params.enabled && Blackboard::theInstance != nullptr)
  {
    state = INITAL;
  }
}

void CognitionLogger::initial()
{
  if(!initialized)
  {
    initRepresentations();
    //initialize the buffer
    for(unsigned int i = 0; i < params.maxBufferSize; i++)
    {
      buffer.push_back(new MessageQueue());
      buffer.back()->setSize(params.blockSize);
    }
    logFilename = generateFilename();
    writerThread.start(this, &CognitionLogger::writeThread);
    writerIdle = false; //this should be false until the writer wrote something for the first time
    sortRepresentations();
    initialized = true;
  }
  state = IDLE;
}

void CognitionLogger::idle()
{
  const int gameState = Blackboard::theInstance->theGameInfo.state;
  if(gameState == STATE_READY || gameState == STATE_SET || gameState == STATE_PLAYING)
  {
    shouldPlayLogSound = true;
    state = RUNNING;
  }
  else if(shouldPlayLogSound && writerIdle && SystemCall::getTimeSince(writerIdleStart) > 5000)
  {//if the writer is idle and 5 seconds have passed we can safely assume that the log file has been completely written
    //as long as we stay in IDLE nothing will be appended to the write buffer
    SystemCall::playSound("logWritten.wav");
    shouldPlayLogSound = false;
  }
}

void CognitionLogger::running()
{
  const int gameState = Blackboard::theInstance->theGameInfo.state;
  if(gameState == STATE_READY || gameState == STATE_SET || gameState == STATE_PLAYING)
  {
    logFrame();
  }
  else
  {
    state = IDLE;
  }
  const unsigned freeSpace = getFreeSpace();
  if(freeSpace < params.minFreeSpace)
  {//end writer thread if disk space has fallen below the minimum allowed value and go into error state
    writerThread.announceStop();
    state = ERROR_STATE;
  }
}

void CognitionLogger::error()
{
  OUTPUT_WARNING("Logger: Critical error");
}


void CognitionLogger::logFrame()
{
  if(writeIndex == mod((readIndex - 1), params.maxBufferSize)) //use mathematically mod instead of % because (readIndex -1) might be negative
  {//buffer is full, can't do anything this frame
    OUTPUT_WARNING("Logger: Writer thread too slow, discarding frame");
    return;
  }

  //write processBegin for the cognition process
  buffer[writeIndex]->out.bin << 'c';
  buffer[writeIndex]->out.finishMessage(idProcessBegin);

  //stream all representations to the queue
  for(const string& repName : representationNames)
  {
    ASSERT(representations.find(repName) != representations.end());
    const pair<MessageID, Streamable*>& representation = representations[repName];
    const unsigned size = getStreamableSize(*representation.second);
    //some streamables do not stream anything. If no data is streamed the queue crashes.
    if(size > 0)
    {
      buffer[writeIndex]->out.bin << *representation.second;
      buffer[writeIndex]->out.finishMessage(representation.first);
    }
    //the buffer has a limited size. Thus it might be full.
    if(buffer[writeIndex]->writeErrorOccurred())
    {
      OUTPUT_WARNING("Logging of " << ::getName(representation.first) << " failed. The buffer is full.");
    }
  }

  //append timing data if any
  MessageQueue& timingData = Global::getTimingManager().getData();
  if(timingData.getNumberOfMessages() > 0)
  {
    timingData.copyAllMessages(*buffer[writeIndex]);
  }
  else
  {
    OUTPUT_WARNING("Logger: unabled to log timing data. No timing data available");
  }

  //write processFinished for the cognition process
  buffer[writeIndex]->out.bin << 'c';
  buffer[writeIndex]->out.finishMessage(idProcessFinished);

  //cognition runs at 60 fps. Therefore use a new block every 60 frames.
  //Thus one block is used per second.
  if(frameCounter >= 60)
  {
    //the next call to logFrame will use a new block.
    frameCounter = 0;
    writeIndex = (writeIndex + 1) % params.maxBufferSize; //we can use normal % instead of mod() because writeIndex is unsigned
    framesToWrite.post(); //signal to the writer process that another block is ready
    if(params.debugStatistics)
    {
      const int diff = params.maxBufferSize - mod(writeIndex - readIndex, params.maxBufferSize);
      OUTPUT_WARNING("Logger buffer is " << diff / (float) params.maxBufferSize * 100 << "% free");
    }
  }

  frameCounter++;
}

unsigned int CognitionLogger::getStreamableSize(const Streamable& s)
{
  sizeCounter << s;
  const unsigned int size = sizeCounter.getSize();
  sizeCounter.reset();
  return size;
}

inline int CognitionLogger::mod(int a, int b)
{
  return (a + b) % b;
}

string CognitionLogger::generateFilename()
{
  #ifdef TARGET_ROBOT
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%F_%T", &tstruct);
    stringstream ss;
    ss << (params.logFilePath) << Global::getSettings().playerNumber << "_"
       << Global::getSettings().robot.c_str() << "_" << buf << ".log";
    return ss.str();
  #endif
    ASSERT(false); //logger should never run inside the simulator
    return "keks";
}

void CognitionLogger::writeThread()
{
  /**
   * Logfile format:
   * magic byte | size of next compressed block | compressed block | size | compressed block | etc...
   * Each block is compressed using libsnappy
   *
   * Block format (after decompression):
   * | -1 (int) | -1 (int) | Frame | Frame | Frame | ... | Frame |
   * The -1 at the beginning indicates that the number of frames that follows is unknown
   *
   * Each frame looks like this:
   * | ProcessBegin | Log data 1 | Log data 2 | ... | Log data n | ProcessFinished |
   * Log data format:
   * | ID ( 1 byte) | Message size (3 byte) | Message |
   */

  //create and open file
  OutBinaryFile file(logFilename);
  ASSERT(file.exists());
  file << logFileCompressed; //write magic byte that indicates a compressed log file
  
  const int uncompressedSize = params.blockSize + 8; // + 8 because of header
  vector<char> uncompresedBuffer;
  uncompresedBuffer.resize(uncompressedSize);//contains data before compression
  const size_t compressedSize = snappy_max_compressed_length(uncompressedSize);
  vector<char> compressedBuffer;
  compressedBuffer.resize(compressedSize); //data after compression


  while(writerThread.isRunning()) //check if we are expecting more data
  {
    if(framesToWrite.wait(100)) //wait 200 ms for new data then check again if we should quit
    {
      writerIdle = false;
      int rIndex = readIndex; //readIndex is shared between both threads. therefore avoid accessing it too often
      const unsigned int numOfMsgs = buffer[rIndex]->getNumberOfMessages();
      if(numOfMsgs > 0)
      {
        OutBinaryMemory mem(uncompresedBuffer.data());
        
        mem << *buffer[rIndex];
        buffer[rIndex]->clear();
        size_t size = compressedSize;
        VERIFY(snappy_compress(uncompresedBuffer.data(), (size_t)mem.getLength(),
                        compressedBuffer.data(), &size) == SNAPPY_OK);
        file << (unsigned)size;
        file.write(compressedBuffer.data(), size);
      }
      readIndex = (rIndex + 1) % params.maxBufferSize;
    }
    else if(!writerIdle)
    {
      writerIdle = true;
      writerIdleStart = SystemCall::getCurrentSystemTime();
    }
  }
}

unsigned CognitionLogger::getFreeSpace() const
{
#ifdef TARGET_ROBOT
  struct statvfs data;
  if(0 == statvfs(logFilename.c_str(), &data))
  {
    return unsigned((data.f_bfree * data.f_bsize)/1024);
  }
#endif
  return 0; // in case of error assume that there is no disk space left
}

void CognitionLogger::sortRepresentations()
{
  vector<string> representationsInOrder = moduleManager.getCurrentRepresentatioNames();
  for(const string& rep : representationsInOrder)
  {
    if(std::find(params.representations.begin(), params.representations.end(), rep) != params.representations.end())
    {
      representationNames.push_back(rep);
    }
  }
}

CognitionLogger::~CognitionLogger()
{
  writerThread.stop();
}