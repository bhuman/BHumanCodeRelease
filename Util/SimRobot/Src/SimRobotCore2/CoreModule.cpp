/**
* @file Simulation/CoreModule.h
* Implementation of class CoreModule
* @author Colin Graf
*/

#include <QDir>
#include <QLabel>

#include "CoreModule.h"
#include "Simulation/PhysicalObject.h"
#include "Simulation/Scene.h"

extern "C" DLL_EXPORT SimRobot::Module* createModule(SimRobot::Application& simRobot)
{
  return new CoreModule(simRobot);
}

SimRobot::Application* CoreModule::application;
CoreModule* CoreModule::module;

CoreModule::CoreModule(SimRobot::Application& application) :
  sceneIcon(":/Icons/bricks.png"), objectIcon(":/Icons/brick.png"), sensorIcon(":/Icons/transmit_go.png"), actuatorIcon(":/Icons/arrow_rotate_clockwise.png"),
  hingeIcon(":/Icons/link.png"), sliderIcon(":/Icons/slider.png"), appearanceIcon(":/Icons/note.png")
{
  CoreModule::application = &application;
  CoreModule::module = this;
}

bool CoreModule::compile()
{
  Q_ASSERT(scene == 0);

  // change working directory
  QString filePath = application->getFilePath();
  QDir::setCurrent(QFileInfo(filePath).dir().path());

  // load simulation
  std::list<std::string> errors;
  if(!loadFile(filePath.toUtf8().constData(), errors))
  {
    QString errorMessage;
    for(std::list<std::string>::const_iterator it = errors.begin(), end = errors.end(); it != end; ++it)
    {
      if(!errorMessage.isEmpty())
        errorMessage += "\n";
      errorMessage += (*it).c_str();
    }
    application->showWarning(QObject::tr("SimRobotCore"), errorMessage);
    return false;
  }

  // register scene graph objects
  registerObjects();
  application->registerObject(*this, actuatorsObject, 0, SimRobot::Flag::hidden);

  // register status bar labels
  class StepsLabel : public QLabel, public SimRobot::StatusLabel
  {
  public:
    StepsLabel() : lastStep(-1) {}
  private:
    unsigned int lastStep;
    QWidget* getWidget() override {return this;}
    void update() override
    {
      unsigned int step = Simulation::simulation->simulationStep;
      if(step != lastStep)
      {
        lastStep = step;
        char buf[33];
        sprintf(buf, "%u steps", step);
        setText(buf);
      }
    }
  };

  class StepsPerSecondLabel : public QLabel, public SimRobot::StatusLabel
  {
  public:
    StepsPerSecondLabel() : lastFps(-1) {}
  private:
    int lastFps;
    QWidget* getWidget() override {return this;}
    void update() override
    {
      int fps = Simulation::simulation->currentFrameRate;
      if(fps != lastFps)
      {
        lastFps = fps;
        char buf[33];
        sprintf(buf, "%u steps/s", fps);
        setText(buf);
      }
    }
  };

  class CollisionsLabel : public QLabel, public SimRobot::StatusLabel
  {
  public:
    CollisionsLabel() : lastCols(-1) {}
  private:
    int lastCols;
    QWidget* getWidget() override {return this;}
    void update() override
    {
      int cols = Simulation::simulation->collisions;
      if(cols != lastCols)
      {
        lastCols = cols;
        char buf[33];
        sprintf(buf, "%u collisions", cols);
        setText(buf);
      }
    }
  };

  class RendererLabel : public QLabel, public SimRobot::StatusLabel
  {
  public:
    RendererLabel() : lastRenderingMethod(-1) {}
  private:
    int lastRenderingMethod;
    QWidget* getWidget() override {return this;}
    void update() override
    {
      int renderingMethod = Simulation::simulation->renderer.getRenderingMethod();
      if(renderingMethod != lastRenderingMethod)
      {
        lastRenderingMethod = renderingMethod;
        static const char* renderingMethods[] = {
          "unkn renderer", "pbuf renderer", "fbuf renderer", "hwin renderer"
        };
        setText(renderingMethods[(renderingMethod < 0 || renderingMethod >= int(sizeof(renderingMethods)/sizeof(*renderingMethods))) ? 0 : renderingMethod]);
      }
    }
  };

  application->addStatusLabel(*this, new StepsLabel());
  application->addStatusLabel(*this, new StepsPerSecondLabel());
  application->addStatusLabel(*this, new CollisionsLabel());
  application->addStatusLabel(*this, new RendererLabel());

  // suggest further modules
  application->registerModule(*this, "File Editor", "SimRobotEditor", SimRobot::Flag::ignoreReset);

  // load controller
  if(simulation->scene->controller != "")
    application->loadModule(simulation->scene->controller.c_str());
  return true;
}

void CoreModule::update()
{
  if(ActuatorsWidget::actuatorsWidget)
    ActuatorsWidget::actuatorsWidget->adoptActuators();
  doSimulationStep();
}
