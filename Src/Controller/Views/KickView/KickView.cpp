/**
 * @file Controller/Views/KickView/KickView.cpp
 *
 * Implementation of class KickView
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include "Controller/RobotConsole.h"
#include "KickView.h"
#include "KickViewWidget.h"
#include "Platform/File.h"

KickViewHeaderedWidget::KickViewHeaderedWidget(KickView& kickView)
{
  kickViewWidget = new KickViewWidget(kickView, parameters, this);
  fileName = QString("noName.kmc");
  setWidget(kickViewWidget);
  QHeaderView* headerView = getHeaderView();
  headerView->setMinimumSectionSize(30);
  headerView->resizeSection(0, 100);
  headerView->resizeSection(1, 100);
}

void KickViewHeaderedWidget::update()
{
  kickViewWidget->update();
}

QMenu* KickViewHeaderedWidget::createFileMenu() const
{
  QMenu* menu = new QMenu(tr("&File"));

  QAction* newAct, * saveAct, * saveAsAct, * loadAct;

  newAct = new QAction(QIcon(":/Icons/kick_new.png"), tr("&New .kmc"), menu);
  newAct->setShortcut(tr("Shift+N"));
  newAct->setStatusTip(tr("Create a new Kick motion file"));

  saveAct = new QAction(QIcon(":/Icons/kick_save.png"), tr("&Save .kmc"), menu);
  saveAct->setShortcut(tr("Shift+S"));
  saveAct->setEnabled(!undo.empty());
  saveAct->setStatusTip(tr("Save Kick motion file under its current name"));
  connect(this, SIGNAL(saveAvailable(bool)), saveAct, SLOT(setEnabled(bool)));

  saveAsAct = new QAction(QIcon(":/Icons/kick_save_as.png"), tr("Save .kmc &As"), menu);
  saveAsAct->setShortcut(tr("Shift+Alt+S"));
  saveAsAct->setStatusTip(tr("Save Kick motion file using a new name"));

  loadAct = new QAction(QIcon(":/Icons/kick_open.png"), tr("&Open .kmc"), menu);
  loadAct->setShortcut(tr("Shift+O"));
  loadAct->setStatusTip(tr("Open a Kick motion file"));

  connect(newAct, SIGNAL(triggered()), this, SLOT(newButtonClicked()));
  connect(saveAct, SIGNAL(triggered()), this, SLOT(saveButtonClicked()));
  connect(saveAsAct, SIGNAL(triggered()), this, SLOT(saveAsButtonClicked()));
  connect(loadAct, SIGNAL(triggered()), this, SLOT(loadButtonClicked()));

  menu->addAction(newAct);
  menu->addAction(loadAct);
  menu->addAction(saveAct);
  menu->addAction(saveAsAct);

  return menu;
}

QMenu* KickViewHeaderedWidget::createEditMenu() const
{
  QMenu* menu = new QMenu(tr("&KickEdit"));

  QAction* undoAct = new QAction(QIcon(":/Icons/arrow_undo.png"), tr("Undo"), menu);
  undoAct->setShortcut(QKeySequence::Undo);
  undoAct->setStatusTip(tr("Undo last change"));
  undoAct->setEnabled(!undo.empty());
  connect(this, SIGNAL(undoAvailable(bool)), undoAct, SLOT(setEnabled(bool)));

  QAction* redoAct = new QAction(QIcon(":/Icons/arrow_redo.png"), tr("Redo"), menu);
  redoAct->setShortcut(QKeySequence::Redo);
  redoAct->setStatusTip(tr("Redo last undone change"));
  redoAct->setEnabled(!redo.empty());
  connect(this, SIGNAL(redoAvailable(bool)), redoAct, SLOT(setEnabled(bool)));

  QAction* show3D = new QAction(tr("Display Phase Drawings"), menu);
  show3D->setStatusTip(tr("Draws curves for every limb either for the current phase or all phases"));
  show3D->setCheckable(true);
  show3D->setChecked(kickViewWidget->getDrawings());

  QAction* singleDraw = new QAction(tr("Display Only Current Phase"), menu);
  singleDraw->setStatusTip(tr("Draws only curves for the current Phase"));
  singleDraw->setCheckable(true);
  singleDraw->setChecked(kickViewWidget->getSingleDrawing());

  QAction* reachedDraw = new QAction(tr("Display Reached Positions"), menu);
  reachedDraw->setStatusTip(tr("Draws the reached positions into the 3D view"));
  reachedDraw->setCheckable(true);
  reachedDraw->setChecked(kickViewWidget->getReachedDrawing());

  QAction* showEditor = new QAction(tr("Display Editor View"), menu);
  showEditor->setStatusTip(tr("Shows the editor view"));
  showEditor->setCheckable(true);
  showEditor->setChecked(kickViewWidget->getEditor());

  QAction* show1D = new QAction(tr("Display 1D Views"), menu);
  show1D->setStatusTip(tr("Shows 1D views of one curve for each axis"));
  show1D->setCheckable(true);
  show1D->setChecked(kickViewWidget->getTra1d());

  QAction* show2D = new QAction(tr("Display 2D Views"), menu);
  show2D->setStatusTip(tr("Shows 2D views of one curve for plane"));
  show2D->setCheckable(true);
  show2D->setChecked(kickViewWidget->getTra2d());

  QAction* showVelo = new QAction(tr("Display Velocity Views"), menu);
  showVelo->setStatusTip(tr("Shows the velocity of one curve"));
  showVelo->setCheckable(true);
  showVelo->setChecked(kickViewWidget->getVelocity());

  QAction* showAccel = new QAction(tr("Display Acceleration Views"), menu);
  showAccel->setStatusTip(tr("Shows the acceleration of one curve"));
  showAccel->setCheckable(true);
  showAccel->setChecked(kickViewWidget->getAccel());

  QAction* noExtraView = new QAction(tr("Display No Extra View"), menu);
  noExtraView->setStatusTip(tr("Hides all extra views"));
  noExtraView->setCheckable(true);
  noExtraView->setChecked(!kickViewWidget->getTra1d()
                          && !kickViewWidget->getTra2d()
                          && !kickViewWidget->getVelocity()
                          && !kickViewWidget->getAccel());

  QAction* followMode = new QAction(tr("Enable Follow Mode"), menu);
  followMode->setStatusTip(tr("The robot will react to changes directly"));
  followMode->setCheckable(true);
  followMode->setChecked(kickViewWidget->getFollowMode());

  connect(undoAct, SIGNAL(triggered()), this, SLOT(undoChanges()));
  connect(redoAct, SIGNAL(triggered()), this, SLOT(redoChanges()));

  connect(singleDraw, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setSingleDrawing(bool)));
  connect(reachedDraw, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setReachedDrawing(bool)));
  connect(show3D, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setDrawings(bool)));
  connect(showEditor, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setEditor(bool)));
  connect(show1D, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setTra1d(bool)));
  connect(show2D, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setTra2d(bool)));
  connect(showVelo, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setVelocity(bool)));
  connect(showAccel, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setAccel(bool)));
  connect(followMode, SIGNAL(toggled(bool)), kickViewWidget, SLOT(setFollowMode(bool)));

  QActionGroup* showing = new QActionGroup(menu);
  showing->addAction(show1D);
  showing->addAction(show2D);
  showing->addAction(showVelo);
  showing->addAction(showAccel);
  showing->addAction(noExtraView);

  menu->addAction(undoAct);
  menu->addAction(redoAct);
  menu->addSeparator();
  menu->addAction(show3D);
  menu->addAction(singleDraw);
  menu->addAction(reachedDraw);
  menu->addAction(showEditor);
  menu->addSeparator();
  menu->addAction(noExtraView);
  menu->addAction(show1D);
  menu->addAction(show2D);
  menu->addAction(showVelo);
  menu->addAction(showAccel);
  menu->addSeparator();
  menu->addAction(followMode);

  return menu;
}

bool KickViewHeaderedWidget::canClose()
{
  if(undo.empty())
    return true;
  switch(QMessageBox::warning(this, tr("KickView"), tr("Do you want to save changes to %1?").arg(fileName), QMessageBox::Save  | QMessageBox::Discard | QMessageBox::Cancel))
  {
    case QMessageBox::Save:
      saveButtonClicked();
      break;
    case QMessageBox::Discard:
      break;
    default:
      return false;
  }
  return true;
}

void KickViewHeaderedWidget::newButtonClicked()
{
  parameters.phaseParameters.clear();
  strcpy(parameters.name, "newKick");
  fileName = QString("newKick.kmc");
  parameters.numberOfPhases = 0;
  parameters.footOrigin = Vector3f(0.f, 60.f, -210.f);
  parameters.armOrigin = Vector3f(0.f, 100.f, 30.f);
  parameters.kdx = 0;
  parameters.kix = 0;
  parameters.kpx = 0;
  parameters.kdy = 0;
  parameters.kiy = 0;
  parameters.kpy = 0;

  parameters.loop = false;
  parameters.standLeft = true;
  parameters.comOrigin = Vector2f(10.f, 0.f);

  undo.clear();
  redo.clear();
  emit undoAvailable(false);
  emit redoAvailable(false);
  kickViewWidget->updateEditorView();
}

void KickViewHeaderedWidget::loadButtonClicked()
{
  char dirname[260];
  sprintf(dirname, "%s/Config/KickEngine/", File::getBHDir());
  fileName = QFileDialog::getOpenFileName(this, tr("Open Kick Motion"), dirname, tr("Kick Motion Config Files (*.kmc)"));
  QString name;
  name = fileName.remove(0, fileName.lastIndexOf("/", fileName.lastIndexOf("/") - 1) + 1);

  InMapFile stream(name.toUtf8().constData());
  if(stream.exists())
  {
    stream >> parameters;
    name = name.remove(0, name.lastIndexOf("/") + 1);
    strcpy(parameters.name, name.remove(name.lastIndexOf("."), name.length()).toUtf8().constData());

    parameters.initFirstPhase();
    kickViewWidget->updateEditorView();
    undo.clear();
    redo.clear();
    emit undoAvailable(false);
    emit redoAvailable(false);
    emit saveAvailable(false);
  }
}

void KickViewHeaderedWidget::saveAsButtonClicked()
{
  char dirname[260];
  sprintf(dirname, "%s/Config/KickEngine/", File::getBHDir());
  fileName = QFileDialog::getSaveFileName(this, tr("Save Kick Motion as..."), dirname, tr("Kick Motion Config Files (*.kmc)"));

  if(fileName.begin() != fileName.end())
  {
    QString temp  = fileName.remove(0, fileName.lastIndexOf("/", fileName.lastIndexOf("/") - 1) + 1);
    temp = temp.remove(0, temp.lastIndexOf("/") + 1);
    strcpy(parameters.name, temp.remove(temp.lastIndexOf("."), temp.length()).toUtf8().constData());
    OutMapFile file(fileName.toUtf8().constData(), true);
    file << parameters;
    undo.clear();
    redo.clear();
    emit undoAvailable(false);
    emit redoAvailable(false);
  }
}
void KickViewHeaderedWidget::saveButtonClicked()
{
  if(fileName.begin() != fileName.end() && fileName != QString("newKick.kmc"))
  {
    QString name;
    name = fileName.remove(0, fileName.lastIndexOf("/", fileName.lastIndexOf("/") - 1) + 1);
    OutMapFile file(fileName.toUtf8().constData(), true);
    file << parameters;
    undo.clear();
    redo.clear();
    emit undoAvailable(false);
    emit redoAvailable(false);
    emit saveAvailable(false);
  }
  else
    saveAsButtonClicked();
}

void KickViewHeaderedWidget::addStateToUndoList()
{
  undo.push_back(parameters);

  emit undoAvailable(true);
  if(fileName.size() > 0 && fileName != QString("newKick.kmc"))
    emit saveAvailable(true);

  if(undo.size() >= 20)
    undo.erase(undo.begin());
}

void KickViewHeaderedWidget::undoChanges()
{
  if(!undo.empty())
  {
    KickEngineParameters last;
    last = undo.back();
    undo.pop_back();
    redo.push_back(parameters);
    emit redoAvailable(true);

    if(redo.size() >= 20) redo.erase(redo.begin());
    if(parameters.numberOfPhases == last.numberOfPhases)
    {
      parameters = last;
      kickViewWidget->updateCommon();
      for(int i = 0; i < parameters.numberOfPhases; i++)
        kickViewWidget->fillModelWithPhaseData(i);
    }
    else
    {
      parameters = last;
      kickViewWidget->updateEditorView();
    }
  }

  if(undo.empty())
  {
    emit undoAvailable(false);
    emit saveAvailable(false);
  }
}

void KickViewHeaderedWidget::redoChanges()
{
  if(!redo.empty())
  {
    KickEngineParameters last;
    last = redo.back();
    redo.pop_back();

    undo.push_back(parameters);
    emit undoAvailable(true);
    if(fileName.begin() != fileName.end() && fileName != QString("newKick.kmc"))
      emit saveAvailable(true);

    if(undo.size() >= 20)
      undo.erase(undo.begin());

    if(parameters.numberOfPhases == last.numberOfPhases)
    {
      parameters = last;
      kickViewWidget->updateCommon();
      for(int i = 0; i < parameters.numberOfPhases; i++)
        kickViewWidget->fillModelWithPhaseData(i);
    }
    else
    {
      parameters = last;
      kickViewWidget->updateEditorView();
    }
    kickViewWidget->updateGL();
  }

  if(redo.empty())
    emit redoAvailable(false);
}

KickView::KickView(const QString& fullName, RobotConsole& console, const MotionRequest& motionRequest, const JointAngles& jointAngles,
                   const JointLimits& jointLimits, const RobotDimensions& robotDimensions, const std::string& mr, SimRobotCore2::Body* robot) :
  fullName(fullName), console(console), motionRequest(motionRequest), jointAngles(jointAngles),
  jointLimits(jointLimits), robotDimensions(robotDimensions), motionRequestCommand(mr), robot(robot)
{}

SimRobot::Widget* KickView::createWidget()
{
  return new KickViewHeaderedWidget(*this);
}
