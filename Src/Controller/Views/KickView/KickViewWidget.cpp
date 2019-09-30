/**
 * @file Controller/Views/KickView/KickViewWidget.cpp
 *
 * Implementation of class KickViewWidget.
 *
 * @author <a href="mailto:judy@tzi.de">Judith Müller</a>
 */

#include "Controller/RobotConsole.h"

#include "KickViewWidget.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Eigen.h"

#include "KickViewMath.h"

#include <vector>
#include <sstream>

#include <QHeaderView>
#include <QFileDialog>
#include <QGridLayout>
#include <QPalette>
#include <QColor>
#include <QLineEdit>
#include <QLabel>
#include <QToolButton>

KickViewWidget::KickViewWidget(KickView& kickView, KickEngineParameters& parameters, KickViewHeaderedWidget* parent) :
  QWidget(parent), parent(parent), glWidget(new KickViewGLWidget(kickView, parameters, this)),
  kickView(kickView), parameters(parameters), dragPlane(0, 1, 0)
{
  //Vertical Layouts
  QVBoxLayout* vbox = new QVBoxLayout(this);
  QVBoxLayout* vbox2 = new QVBoxLayout();
  QVBoxLayout* vbox3 = new QVBoxLayout();

  //horizontal Layouts
  QHBoxLayout* hbox = new QHBoxLayout();
  QHBoxLayout* hbox2 = new QHBoxLayout();

  //make the Button
  QPushButton* play = new QPushButton("Play Motion", this);
  QPushButton* play2 = new QPushButton("Play Motion until Active", this);
  QPushButton* reset = new QPushButton("Reset Robot", this);
  QPushButton* stop = new QPushButton("Stand", this);
  QPushButton* clay = new QPushButton("Record Pose", this);

  addPhase = new QPushButton("Add Phase", this);
  deletePhase = new QPushButton("Delete Phase", this);

  addPhase->setShortcut(tr("Shift+A"));
  deletePhase->setShortcut(tr("Shift+D"));
  play->setShortcut(tr("Shift+P"));
  play2->setShortcut(tr("Shift+Alt+P"));
  reset->setShortcut(tr("Shift+R"));
  stop->setShortcut(tr("Shift+T"));
  clay->setShortcut(tr("Shift+C"));

  addPhase->setToolTip("Shift+A");
  deletePhase->setToolTip("Shift+D");
  play->setToolTip("Shift+P");
  play2->setToolTip("Shift+Alt+P");
  reset->setToolTip("Shift+R");
  stop->setToolTip("Shift+T");
  clay->setToolTip("Shift+C");

  QPalette pal = play->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  play->setPalette(pal);
  play->setBackgroundRole(QPalette::NoRole);
  play->setAutoFillBackground(true);
  play2->setPalette(pal);
  play2->setBackgroundRole(QPalette::NoRole);
  play2->setAutoFillBackground(true);
  reset->setPalette(pal);
  reset->setBackgroundRole(QPalette::NoRole);
  reset->setAutoFillBackground(true);
  stop->setPalette(pal);
  stop->setBackgroundRole(QPalette::NoRole);
  stop->setAutoFillBackground(true);
  clay->setPalette(pal);
  clay->setBackgroundRole(QPalette::NoRole);
  clay->setAutoFillBackground(true);
  addPhase->setPalette(pal);
  addPhase->setBackgroundRole(QPalette::NoRole);
  addPhase->setAutoFillBackground(true);
  deletePhase->setPalette(pal);
  deletePhase->setBackgroundRole(QPalette::NoRole);
  deletePhase->setAutoFillBackground(true);

  QSlider* slider1 = new QSlider(Qt::Horizontal, this);
  QLabel* label1 = new QLabel(tr("Robot Opacity"), this);

  //Set Sliderrange
  pal = slider1->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  slider1->setPalette(pal);
  slider1->setAutoFillBackground(true);
  slider1->setRange(0, 16);

  pal = label1->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  pal.setColor(QPalette::Foreground, Qt::white);
  label1->setPalette(pal);

  pal = palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  setPalette(pal);

  QCheckBox* mirrorCB = new QCheckBox(tr("Mirror"), this);
  mirrorCB->setCheckState(Qt::Unchecked);
  mirrorCB->setBackgroundRole(QPalette::NoRole);
  mirrorCB->setAutoFillBackground(true);
  pal = mirrorCB->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  pal.setColor(QPalette::Foreground, Qt::white);
  mirrorCB->setPalette(pal);

  QCheckBox* armsBackFixCB = new QCheckBox(tr("ArmsBackFix"), this);
  armsBackFixCB->setCheckState(Qt::Unchecked);
  armsBackFixCB->setBackgroundRole(QPalette::NoRole);
  armsBackFixCB->setAutoFillBackground(true);
  pal = armsBackFixCB->palette();
  pal.setColor(QPalette::Background, Qt::darkGray);
  pal.setColor(QPalette::Foreground, Qt::white);
  armsBackFixCB->setPalette(pal);

  QToolButton* softLimb = new QToolButton(this);
  softLimb->setText("Soften Limb...");

  QMenu* limbmenu = new QMenu(this);

  lftra = new QAction(tr("Left Foot Translation off"), this);
  rftra = new QAction(tr("Right Foot Translation off"), this);
  lfrot = new QAction(tr("Left Foot Rotation off"), this);
  rfrot = new QAction(tr("Right Foot Rotation off"), this);
  lhtra = new QAction(tr("Left Hand Translation off"), this);
  rhtra = new QAction(tr("Right Hand Translation off"), this);
  lhrot = new QAction(tr("Left Hand Rotation off"), this);
  rhrot = new QAction(tr("Right Hand Rotation off"), this);

  lftra->setCheckable(true);
  rftra->setCheckable(true);
  lfrot->setCheckable(true);
  rfrot->setCheckable(true);
  lhtra->setCheckable(true);
  rhtra->setCheckable(true);
  lhrot->setCheckable(true);
  rhrot->setCheckable(true);

  lftra->setShortcut(tr("Shift+1"));
  rftra->setShortcut(tr("Shift+2"));
  lfrot->setShortcut(tr("Shift+3"));
  rfrot->setShortcut(tr("Shift+4"));
  lhtra->setShortcut(tr("Shift+5"));
  rhtra->setShortcut(tr("Shift+6"));
  lhrot->setShortcut(tr("Shift+7"));
  rhrot->setShortcut(tr("Shift+8"));

  limbmenu->addAction(lftra);
  limbmenu->addAction(rftra);
  limbmenu->addAction(lfrot);
  limbmenu->addAction(rfrot);
  limbmenu->addAction(lhtra);
  limbmenu->addAction(rhtra);
  limbmenu->addAction(lhrot);
  limbmenu->addAction(rhrot);

  softLimb->setMenu(limbmenu);
  softLimb->setPopupMode(QToolButton::InstantPopup);

  label1->setAutoFillBackground(true);
  mirrorCB->setAutoFillBackground(true);
  armsBackFixCB->setAutoFillBackground(true);
  clay->setAutoFillBackground(true);
  play->setAutoFillBackground(true);
  play2->setAutoFillBackground(true);
  reset->setAutoFillBackground(true);
  stop->setAutoFillBackground(true);

  addPhase->setAutoFillBackground(true);
  deletePhase->setAutoFillBackground(true);

  vbox3->addWidget(mirrorCB, 0, Qt::AlignTop);
  vbox3->addWidget(armsBackFixCB, 0, Qt::AlignTop);
  vbox3->addWidget(play, 0, Qt::AlignTop);
  vbox3->addWidget(play2, 0, Qt::AlignTop);
  vbox3->addWidget(reset, 0, Qt::AlignTop);
  vbox3->addWidget(stop, 0, Qt::AlignTop);

  vbox3->addWidget(label1, 1, Qt::AlignBottom);
  vbox3->addWidget(slider1, 0, Qt::AlignBottom);
  vbox3->addWidget(addPhase, 0, Qt::AlignBottom);
  vbox3->addWidget(deletePhase, 0, Qt::AlignBottom);
  vbox3->addWidget(clay, 0, Qt::AlignBottom);
  vbox3->addWidget(softLimb, 1, Qt::AlignBottom);

  //Make the tabs
  tabber = new TabWidget(this);
  tabber->setAutoFillBackground(true);

  QPalette palette = tabber->palette();
  palette.setColor(QPalette::Background, Qt::darkGray);
  tabber->setPalette(palette);

  treeViewCommon = new QTreeView(this);
  treeViewCommon->setDragDropMode(QTreeView::DragOnly);
  treeViewCommon->setSelectionMode(QTreeView::ExtendedSelection);
  treeViewCommon->header()->setVisible(true);
  treeViewCommon->setRootIsDecorated(true);
  treeViewCommon->setDragEnabled(false);
  treeViewCommon->setAcceptDrops(false);

  //initialize Tabs with some working stuff
  modelCommon = new QStandardItemModel();

  QStandardItem* parentItem = modelCommon->invisibleRootItem();

  QList <QStandardItem*> nameLabels;
  nameLabels.append(makeLabel(QString("Name")));
  nameLabels.append(makeLabel(QString("newKick")));
  parentItem->appendRow(nameLabels);

  QList <QStandardItem*> phaseNumber;
  phaseNumber.append(makeLabel(QString("Number of Phases")));
  phaseNumber.append(makeLabel(QString::number(0)));
  parentItem->appendRow(phaseNumber);

  QStandardItem* footOrigin = makeLabel(QString("Foot Origin"));
  parentItem->appendRow(addXYZLabel(footOrigin));

  QList <QStandardItem*> list2;
  list2.append(makeLabel(QString("")));
  list2.append(makeLabel(QString("")));
  list2.append(makeValue(0.f));
  list2.append(makeValue(50.f));
  list2.append(makeValue(-210.f));
  footOrigin->appendRow(list2);

  QStandardItem* footRotOrigin = makeLabel(QString("Foot Rot Origin"));
  parentItem->appendRow(addXYZLabel(footRotOrigin));

  QList <QStandardItem*> list21;
  list21.append(makeLabel(QString("")));
  list21.append(makeLabel(QString("")));
  list21.append(makeValue(0.f));
  list21.append(makeValue(0.f));
  list21.append(makeValue(0.f));
  footRotOrigin->appendRow(list21);

  QStandardItem* armOrigin = makeLabel(QString("Hand Origin"));
  parentItem->appendRow(addXYZLabel(armOrigin));

  QList <QStandardItem*> list3;
  list3.append(makeLabel(QString("")));
  list3.append(makeLabel(QString("")));
  list3.append(makeValue(0.f));
  list3.append(makeValue(100.f));
  list3.append(makeValue(30.f));
  armOrigin->appendRow(list3);

  QStandardItem* armRotOrigin = makeLabel(QString("Hand Rot Origin"));
  parentItem->appendRow(addXYZLabel(armRotOrigin));

  QList <QStandardItem*> list31;
  list31.append(makeLabel(QString("")));
  list31.append(makeLabel(QString("")));
  list31.append(makeValue(0.f));
  list31.append(makeValue(0.f));
  list31.append(makeValue(0.f));
  armRotOrigin->appendRow(list31);

  QStandardItem* comOrigin = makeLabel(QString("COM Origin (only for no auto COM)"));
  QList <QStandardItem*> comOlist;
  comOlist.append(comOrigin);
  comOlist.append(makeLabel(QString("")));
  comOlist.append(makeLabel(QString("x")));
  comOlist.append(makeLabel(QString("y")));
  parentItem->appendRow(comOlist);
  QList <QStandardItem*> list33;
  list33.append(makeLabel(QString("")));
  list33.append(makeLabel(QString("")));
  list33.append(makeValue(10.f));
  list33.append(makeValue(0.f));
  comOrigin->appendRow(list33);

  QStandardItem* headOrigin = makeLabel(QString("Head Origin"));
  QList <QStandardItem*> headOlist;
  headOlist.append(headOrigin);
  headOlist.append(makeLabel(QString("")));
  headOlist.append(makeLabel(QString("x")));
  headOlist.append(makeLabel(QString("y")));
  parentItem->appendRow(headOlist);
  QList <QStandardItem*> list34;
  list34.append(makeLabel(QString("")));
  list34.append(makeLabel(QString("")));
  list34.append(makeValue(0.f));
  list34.append(makeValue(0.f));
  headOrigin->appendRow(list34);

  QStandardItem* pidx = makeLabel(QString("COM Balance X"));
  QList <QStandardItem*> pidxlist;
  pidxlist.append(pidx);
  pidxlist.append(makeLabel(QString("")));
  pidxlist.append(makeLabel(QString("p")));
  pidxlist.append(makeLabel(QString("i")));
  pidxlist.append(makeLabel(QString("d")));
  parentItem->appendRow(pidxlist);
  QList <QStandardItem*> list4;
  list4.append(makeLabel(QString("")));
  list4.append(makeLabel(QString("")));
  list4.append(makeValue(0.f));
  list4.append(makeValue(0.f));
  list4.append(makeValue(0.f));
  pidx->appendRow(list4);

  QStandardItem* pidy = makeLabel(QString("COM Balance Y"));
  QList <QStandardItem*> pidylist;
  pidylist.append(pidy);
  pidylist.append(makeLabel(QString("")));
  pidylist.append(makeLabel(QString("p")));
  pidylist.append(makeLabel(QString("i")));
  pidylist.append(makeLabel(QString("d")));
  parentItem->appendRow(pidylist);
  QList <QStandardItem*> list5;
  list5.append(makeLabel(QString("")));
  list5.append(makeLabel(QString("")));
  list5.append(makeValue(0.f));
  list5.append(makeValue(0.f));
  list5.append(makeValue(0.f));
  pidy->appendRow(list5);

  parentItem->appendRow(makeStringAndValueRow(QString("Loop"), false));

  parentItem->appendRow(makeStringAndValueRow(QString("Stand Left"), false));

  parentItem->appendRow(makeStringAndValueRow(QString("Ignore Head"), true));

  treeViewCommon->setModel(modelCommon);
  modelCommon->setColumnCount(5);
  modelCommon->setHeaderData(0, Qt::Horizontal, QVariant(""));
  modelCommon->setHeaderData(1, Qt::Horizontal, QVariant(""));
  modelCommon->setHeaderData(2, Qt::Horizontal, QVariant(""));
  modelCommon->setHeaderData(3, Qt::Horizontal, QVariant(""));
  modelCommon->setHeaderData(4, Qt::Horizontal, QVariant(""));

  treeViewCommon->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  treeViewCommon->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  treeViewCommon->header()->setSectionResizeMode(2, QHeaderView::Stretch);
  treeViewCommon->header()->setSectionResizeMode(3, QHeaderView::Stretch);
  treeViewCommon->header()->setSectionResizeMode(4, QHeaderView::Stretch);

  tabber->addTab(treeViewCommon, "Common");

  hbox2->addWidget(tabber, 1);
  hbox2->addLayout(vbox3);
  hbox2->addWidget(glWidget, 3);

  hbox2->addLayout(vbox2);
  vbox->addLayout(hbox2);
  vbox->addLayout(hbox);

  treeViewCommon->expandAll();
  treeViewCommon->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);

  kickMenuBar = new KickMenuBar;
  kickMenuBar->hide();

  QMenu shortcutMenu;
  shortcutMenu.addMenu(kickMenuBar->dragPlaneMenu);
  shortcutMenu.hide();

  //Connect Buttons and Stuff with
  connect(slider1, SIGNAL(valueChanged(int)), this, SLOT(transparencyChanged(int)));
  connect(deletePhase, SIGNAL(clicked()), this, SLOT(removePhase()));
  connect(addPhase, SIGNAL(clicked()), this, SLOT(addPhaseAfterActual()));
  connect(mirrorCB, SIGNAL(stateChanged(int)), this, SLOT(setMirrored(int)));
  connect(armsBackFixCB, SIGNAL(stateChanged(int)), this, SLOT(setArmsBackFix(int)));
  connect(play, SIGNAL(clicked()), this, SLOT(playWholeMotion()));
  connect(reset, SIGNAL(clicked()), this, SLOT(resetRobot()));
  connect(stop, SIGNAL(clicked()), this, SLOT(standRobot()));
  connect(play2, SIGNAL(clicked()), this, SLOT(playMotionTilActive()));
  connect(clay, SIGNAL(clicked()), this, SLOT(recordPose()));

  connect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
  connect(tabber->getTabBar(), SIGNAL(tabMoveRequested(int, int)), this, SLOT(movePhase(int, int)));
  connect(tabber, SIGNAL(currentChanged(int)), this, SLOT(setSelectedFromEditor(int)));

  dragPlaneMapper.setMapping(kickMenuBar->xy_plane, SimRobotCore2::Renderer::xyPlane);
  dragPlaneMapper.setMapping(kickMenuBar->xz_plane, SimRobotCore2::Renderer::xzPlane);
  dragPlaneMapper.setMapping(kickMenuBar->yz_plane, SimRobotCore2::Renderer::yzPlane);
  connect(&dragPlaneMapper, SIGNAL(mapped(int)), SLOT(setDragPlane(int)));

  connect(kickMenuBar->xy_plane, SIGNAL(triggered()), &dragPlaneMapper, SLOT(map()));
  connect(kickMenuBar->xz_plane, SIGNAL(triggered()), &dragPlaneMapper, SLOT(map()));
  connect(kickMenuBar->yz_plane, SIGNAL(triggered()), &dragPlaneMapper, SLOT(map()));

  qobject_cast<QAction*>(dragPlaneMapper.mapping(SimRobotCore2::Renderer::xzPlane))->setChecked(true);

  softenMapper.setMapping(lftra, 0);
  softenMapper.setMapping(rftra, 1);
  softenMapper.setMapping(lfrot, 2);
  softenMapper.setMapping(rfrot, 3);
  softenMapper.setMapping(lhtra, 4);
  softenMapper.setMapping(rhtra, 5);
  softenMapper.setMapping(lhrot, 6);
  softenMapper.setMapping(rhrot, 7);
  connect(&softenMapper, SIGNAL(mapped(int)), SLOT(setStiffness(int)));
  connect(lftra, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(rftra, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(lfrot, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(rfrot, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(lhtra, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(rhtra, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(lhrot, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));
  connect(rhrot, SIGNAL(toggled(bool)), &softenMapper, SLOT(map()));

  //initialize actual Parameters
  parameters.footOrigin = Vector3f(0.f, 60.f, -210.f);
  parameters.armOrigin = Vector3f(0.f, 100.f, 30.f);
  parameters.numberOfPhases = 0;
  parameters.loop = false;
  parameters.kpx = 0.f;
  parameters.kdx = 0.f;
  parameters.kix = 0.f;
  parameters.kpy = 0.f;
  parameters.kdy = 0.f;
  parameters.kiy = 0.f;
  parameters.comOrigin = Vector2f(10.f, 0.f);
  parameters.headOrigin = Vector2f::Zero();
  parameters.standLeft = true;
  parameters.ignoreHead = true;
  strcpy(parameters.name, "newKick");

  selectedPoint.limb = -1;
  selectedPoint.phaseNumber = -1;
  selectedPoint.xzRot = false;

  if(!commands.empty())
    commands.clear();
}

KickViewWidget::~KickViewWidget()
{
  delete kickMenuBar;
  delete glWidget;
}

void KickViewWidget::setDragPlane(int plane)
{
  switch(plane)
  {
    case SimRobotCore2::Renderer::xyPlane:
      dragPlane = Vector3f(0, 0, 1);
      break;
    case SimRobotCore2::Renderer::xzPlane:
      dragPlane = Vector3f(0, 1, 0);
      break;
    case SimRobotCore2::Renderer::yzPlane:
      dragPlane = Vector3f(1, 0, 0);
      break;
  }
  qobject_cast<QAction*>(dragPlaneMapper.mapping(plane))->setChecked(true);
}

void KickViewWidget::setStiffness(int limb)
{
  switch(limb)
  {
    case 0: //lftra
      if(lftra->isChecked())
        commands.emplace_back("set module:KickEngine:lFootTraOff true");
      else
        commands.emplace_back("set module:KickEngine:lFootTraOff false");
      break;
    case 1: //rftra
      if(rftra->isChecked())
        commands.emplace_back("set module:KickEngine:rFootTraOff true");
      else
        commands.emplace_back("set module:KickEngine:rFootTraOff false");
      break;
    case 2: //lfrot
      if(lfrot->isChecked())
        commands.emplace_back("set module:KickEngine:lFootRotOff true");
      else
        commands.emplace_back("set module:KickEngine:lFootRotOff false");
      break;
    case 3: //rfrot
      if(rfrot->isChecked())
        commands.emplace_back("set module:KickEngine:rFootRotOff true");
      else
        commands.emplace_back("set module:KickEngine:rFootRotOff false");
      break;
    case 4: //lhtra
      if(lhtra->isChecked())
        commands.emplace_back("set module:KickEngine:lHandTraOff true");
      else
        commands.emplace_back("set module:KickEngine:lHandTraOff false");
      break;
    case 5: //rhtra
      if(rhtra->isChecked())
        commands.emplace_back("set module:KickEngine:rHandTraOff true");
      else
        commands.emplace_back("set module:KickEngine:rHandTraOff false");
      break;
    case 6: //lhrot
      if(lhrot->isChecked())
        commands.emplace_back("set module:KickEngine:lHandRotOff true");
      else
        commands.emplace_back("set module:KickEngine:lHandRotOff false");
      break;
    case 7: //rhrot
      if(rhrot->isChecked())
        commands.emplace_back("set module:KickEngine:rHandRotOff true");
      else
        commands.emplace_back("set module:KickEngine:rHandRotOff false");
      break;
  }
}

void KickViewWidget::removePhase()
{
  int phaseNumber = tabber->currentIndex() - 1;

  if(phaseNumber > -1)
  {
    parent->addStateToUndoList();
    std::vector<Phase>::iterator it = parameters.phaseParameters.begin() + phaseNumber;
    if(it != parameters.phaseParameters.end())
    {
      disconnect(*(phaseTab.begin() + phaseNumber), SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updatePhaseParameters(QStandardItem*)));
      tabber->setCurrentIndex(phaseNumber);
      tabber->removeTab(phaseNumber + 1);
      for(int i = phaseNumber + 1; i < tabber->count(); i++)
        tabber->setTabText(i, QString("Phase %1").arg(i - 1));

      deleteKids(phaseTab[phaseNumber]->invisibleRootItem());
      delete *(treeView.begin() + phaseNumber);
      delete *(phaseTab.begin() + phaseNumber);
      treeView.erase(treeView.begin() + phaseNumber);
      phaseTab.erase(phaseTab.begin() + phaseNumber);
      parameters.phaseParameters.erase(it);
      parameters.numberOfPhases -= 1;

      for(int limb = 0; limb < Phase::numOfLimbs; limb++)
      {
        if(phaseNumber < parameters.numberOfPhases - 1)
        {
          float factor = static_cast<float>(parameters.phaseParameters[phaseNumber].duration) /
                         static_cast<float>(parameters.phaseParameters[phaseNumber + 1].duration);

          parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
            parameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
            parameters.phaseParameters[phaseNumber].controlPoints[limb][1];

          parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

          parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
            parameters.phaseParameters[phaseNumber].controlPoints[limb][2];
        }
      }
      parameters.initFirstPhase();
      disconnect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
      modelCommon->invisibleRootItem()->child(1, 1)->setData(QString::number(parameters.numberOfPhases), Qt::DisplayRole);
      updateCommon();
    }
  }
}

void KickViewWidget::movePhase(const int fromIndex, int toIndex)
{
  if(parameters.phaseParameters.begin() + fromIndex - 1 != parameters.phaseParameters.end())
  {
    toIndex = (fromIndex < toIndex) ? toIndex - 1 : toIndex;

    QStandardItemModel* temp = phaseTab[fromIndex - 1];
    phaseTab.erase(phaseTab.begin() + fromIndex - 1);
    phaseTab.insert(phaseTab.begin() + toIndex - 1, temp);

    Phase object = parameters.phaseParameters[fromIndex - 1];
    parameters.phaseParameters.erase(parameters.phaseParameters.begin() + fromIndex - 1);
    parameters.phaseParameters.insert(parameters.phaseParameters.begin() + toIndex - 1, object);
    parameters.initFirstPhase();
    updateEditorView();
    tabber->setCurrentIndex(toIndex);
  }
}

void KickViewWidget::makeNewPhaseWithModelAndTree(int phaseNumber)
{
  Phase newPhase;

  parameters.numberOfPhases++;
  disconnect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
  modelCommon->invisibleRootItem()->child(1, 1)->setData(QString::number(parameters.numberOfPhases), Qt::DisplayRole);
  updateCommon();

  newPhase.duration = 1000;
  if(phaseNumber > 0)
  {
    for(int limb = 0; limb < Phase::numOfLimbs; ++limb)
      for(int point = 0; point < Phase::numOfPoints; ++point)
        newPhase.controlPoints[limb][point] = Vector3f::Zero();
  }
  else
  {
    for(int point = 0; point < Phase::numOfPoints; ++point)
    {
      newPhase.controlPoints[Phase::rightFootTra][point] = Vector3f(parameters.footOrigin.x(), -parameters.footOrigin.y(), parameters.footOrigin.z());
      newPhase.controlPoints[Phase::leftFootTra][point] = parameters.footOrigin;
      newPhase.controlPoints[Phase::rightFootRot][point] = Vector3f(-parameters.footRotOrigin.x(), parameters.footRotOrigin.y(), -parameters.footRotOrigin.z());
      newPhase.controlPoints[Phase::leftFootRot][point] = parameters.footRotOrigin;
      newPhase.controlPoints[Phase::rightArmTra][point] = Vector3f(parameters.armOrigin.x(), -parameters.armOrigin.y(), parameters.armOrigin.z());
      newPhase.controlPoints[Phase::leftArmTra][point] = parameters.armOrigin;
      newPhase.controlPoints[Phase::rightHandRot][point] = Vector3f(-parameters.handRotOrigin.x(), parameters.handRotOrigin.y(), -parameters.handRotOrigin.z());
      newPhase.controlPoints[Phase::leftHandRot][point] = parameters.handRotOrigin;
      newPhase.comTra[point] = parameters.comOrigin;
      newPhase.headTra[point] = parameters.headOrigin;
    }
  }

  parameters.phaseParameters.insert(parameters.phaseParameters.begin() + phaseNumber, newPhase);

  QStandardItemModel* model = makeNewModelWithLabels();
  phaseTab.insert(phaseTab.begin() + phaseNumber, model);
  fillModelWithPhaseData(phaseNumber);

  QTreeView* newTree = new QTreeView();
  newTree->setModel(phaseTab[phaseNumber]);
  newTree->setDragDropMode(QTreeView::DragOnly);
  newTree->setSelectionMode(QTreeView::ExtendedSelection);
  newTree->header()->setVisible(true);
  newTree->setRootIsDecorated(true);
  newTree->setDragEnabled(false);
  newTree->setAcceptDrops(false);
  newTree->expandToDepth(3);
  newTree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  newTree->header()->setSectionResizeMode(1, QHeaderView::Stretch);
  newTree->header()->setSectionResizeMode(2, QHeaderView::Stretch);
  newTree->header()->setSectionResizeMode(3, QHeaderView::Stretch);
  newTree->header()->setSectionResizeMode(4, QHeaderView::Stretch);
  newTree->header()->setSectionResizeMode(5, QHeaderView::Stretch);
  treeView.insert(treeView.begin() + phaseNumber, newTree);
}

void KickViewWidget::addPhaseAfterActual()
{
  int phaseNumber = tabber->currentIndex(); // we want to insert after actual Phase

  parent->addStateToUndoList();

  makeNewPhaseWithModelAndTree(phaseNumber);

  tabber->insertTab(phaseNumber + 1, treeView[phaseNumber], QString("new"));

  if(phaseNumber > 0)  //if there is a phase before copy last parameters
  {
    for(int limb = 0; limb < Phase::numOfLimbs; ++limb)
    {
      //Set all Points from new Phase to the End of last Phase
      for(int point = 0; point < Phase::numOfPoints; ++point)
      {
        parameters.phaseParameters[phaseNumber].controlPoints[limb][point] = parameters.phaseParameters[phaseNumber - 1].controlPoints[limb][Phase::numOfPoints - 1];
        parameters.phaseParameters[phaseNumber].comTra[point] = parameters.phaseParameters[phaseNumber - 1].comTra[Phase::numOfPoints - 1];
      }

      //correct controlPoints
      // äquidistant und kolinear p3-p2 = q1-q0
      //p3 = q0 => p3 - p2 = q1 - p3 => 2*p3 - p2 = q1;

      //q1
      if(phaseNumber < parameters.numberOfPhases - 1)
      {
        float factor = static_cast<float>(parameters.phaseParameters[phaseNumber].duration) /
                       static_cast<float>(parameters.phaseParameters[phaseNumber + 1].duration);

        parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
          parameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
          parameters.phaseParameters[phaseNumber].controlPoints[limb][1];

        parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

        parameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
          parameters.phaseParameters[phaseNumber].controlPoints[limb][2];

        parameters.phaseParameters[phaseNumber + 1].comTra[0] =
          parameters.phaseParameters[phaseNumber].comTra[2] -
          parameters.phaseParameters[phaseNumber].comTra[1];

        parameters.phaseParameters[phaseNumber + 1].comTra[0] *= factor;

        parameters.phaseParameters[phaseNumber + 1].comTra[0] +=
          parameters.phaseParameters[phaseNumber].comTra[2];

        //head
        parameters.phaseParameters[phaseNumber + 1].headTra[0] =
          parameters.phaseParameters[phaseNumber].headTra[2] -
          parameters.phaseParameters[phaseNumber].headTra[1];

        parameters.phaseParameters[phaseNumber + 1].headTra[0] *= factor;

        parameters.phaseParameters[phaseNumber + 1].headTra[0] +=
          parameters.phaseParameters[phaseNumber].headTra[2];
      }
      //p1
      parameters.phaseParameters[phaseNumber].controlPoints[limb][0] =
        parameters.phaseParameters[phaseNumber - 1].controlPoints[limb][2] -
        parameters.phaseParameters[phaseNumber - 1].controlPoints[limb][1];

      float factor = static_cast<float>(parameters.phaseParameters[phaseNumber].duration) /
                     static_cast<float>(parameters.phaseParameters[phaseNumber - 1].duration);

      parameters.phaseParameters[phaseNumber].controlPoints[limb][0] *= factor;

      parameters.phaseParameters[phaseNumber].controlPoints[limb][0] +=
        parameters.phaseParameters[phaseNumber - 1].controlPoints[limb][2];

      parameters.phaseParameters[phaseNumber].comTra[0] =
        parameters.phaseParameters[phaseNumber - 1].comTra[2] -
        parameters.phaseParameters[phaseNumber - 1].comTra[1];

      parameters.phaseParameters[phaseNumber].comTra[0] *= factor;

      parameters.phaseParameters[phaseNumber].comTra[0] +=
        parameters.phaseParameters[phaseNumber - 1].comTra[2];

      parameters.phaseParameters[phaseNumber].headTra[0] =
        parameters.phaseParameters[phaseNumber - 1].headTra[2] -
        parameters.phaseParameters[phaseNumber - 1].headTra[1];

      parameters.phaseParameters[phaseNumber].headTra[0] *= factor;

      parameters.phaseParameters[phaseNumber].headTra[0] +=
        parameters.phaseParameters[phaseNumber - 1].headTra[2];
    }
  }
  parameters.initFirstPhase();
  updateEditorView();
  tabber->setCurrentIndex(phaseNumber + 1);
}

std::string KickViewWidget::floatToStr(float f)
{
  char temp[100];
  sprintf(temp, "%.3f", f);
  return temp;
}

std::string KickViewWidget::boolToStr(bool b)
{
  return b ? "true" : "false";
}

void KickViewWidget::setMirrored(int state)
{
  mirror = (state == Qt::Checked);
}

void KickViewWidget::setArmsBackFix(int state)
{
  armsBackFix = (state == Qt::Checked);
}

void KickViewWidget::playWholeMotion()
{
  if(parameters.numberOfPhases > 0)
  {
    playMotion(parameters.numberOfPhases);
    std::stringstream todo(std::stringstream::in | std::stringstream::out);
    std::string::size_type idxMotion = kickView.motionRequestCommand.find("motion");
    std::string::size_type idxSpecialAction = kickView.motionRequestCommand.find("specialActionRequest", idxMotion);
    std::string::size_type idxKickMotion = kickView.motionRequestCommand.find("kickMotionType", idxSpecialAction);
    std::string::size_type idxDynamical = kickView.motionRequestCommand.find("dynPoints", idxKickMotion);
    todo << kickView.motionRequestCommand.substr(0, idxMotion + 9) << "kick; ";
    todo << kickView.motionRequestCommand.substr(idxSpecialAction, idxKickMotion + 17 - idxSpecialAction) << "newKick; mirror = ";
    todo << (mirror ? "true" : "false") << "; " << "armsBackFix = ";
    todo << (armsBackFix ? "true" : "false") << "; " << "autoProceed = ";
    todo << "false" << "; " << "boost = false" << "; " << kickView.motionRequestCommand.substr(idxDynamical);
    commands.push_back(todo.str());
  }
}

void KickViewWidget::playMotionTilActive()
{
  int numberOfPhases = tabber->currentIndex();
  if(numberOfPhases > 0)
  {
    playMotion(numberOfPhases);

    std::stringstream todo(std::stringstream::in | std::stringstream::out);
    std::string::size_type idxMotion = kickView.motionRequestCommand.find("motion");
    std::string::size_type idxSpecialAction = kickView.motionRequestCommand.find("specialActionRequest", idxMotion);
    std::string::size_type idxKickMotion = kickView.motionRequestCommand.find("kickMotionType", idxSpecialAction);
    std::string::size_type idxDynamical = kickView.motionRequestCommand.find("dynPoints", idxKickMotion);
    todo << kickView.motionRequestCommand.substr(0, idxMotion + 9) << "kick; ";
    todo << kickView.motionRequestCommand.substr(idxSpecialAction, idxKickMotion + 17 - idxSpecialAction) << "newKick; mirror = ";
    todo << (mirror ? "true" : "false") << "; " << "armsBackFix = ";
    todo << (armsBackFix ? "true" : "false") << "; " << "autoProceed = ";
    todo << "false" << "; " << "boost = false" << "; " << kickView.motionRequestCommand.substr(idxDynamical);
    commands.push_back(todo.str());
  }
}

void KickViewWidget::playMotion(int phase)
{
  std::string setNewKickMotion = "set module:KickEngine:newKickMotion footOrigin = { x = "
                                 + floatToStr(parameters.footOrigin.x()) + "; y = "
                                 + floatToStr(parameters.footOrigin.y()) + "; z = "
                                 + floatToStr(parameters.footOrigin.z()) + "; }; footRotOrigin = { x = "
                                 + floatToStr(parameters.footRotOrigin.x()) + "; y = "
                                 + floatToStr(parameters.footRotOrigin.y()) + "; z = "
                                 + floatToStr(parameters.footRotOrigin.z()) + "; }; armOrigin = { x = "
                                 + floatToStr(parameters.armOrigin.x()) + "; y = "
                                 + floatToStr(parameters.armOrigin.y()) + "; z = "
                                 + floatToStr(parameters.armOrigin.z()) + "; }; handRotOrigin = { x = "
                                 + floatToStr(parameters.handRotOrigin.x()) + "; y = "
                                 + floatToStr(parameters.handRotOrigin.y()) + "; z = "
                                 + floatToStr(parameters.handRotOrigin.z()) + "; }; comOrigin = { x = "
                                 + floatToStr(parameters.comOrigin.x()) + "; y = "
                                 + floatToStr(parameters.comOrigin.y()) + "; }; headOrigin = { x = "
                                 + floatToStr(parameters.headOrigin.x()) + "; y = "
                                 + floatToStr(parameters.headOrigin.y()) + "; }; boostAngles = [ ";
  std::string separator = "";
  for(const KickEngineParameters::BoostAngle& boostAngle : parameters.boostAngles)
  {
    setNewKickMotion += separator + "{ joint = " + TypeRegistry::getEnumName(boostAngle.joint) + "; angle = " + floatToStr(boostAngle.angle) + "; }";
    separator = ", ";
  }
  setNewKickMotion += "]; kpx = " + floatToStr(parameters.kpx) + "; kix = "
                                  + floatToStr(parameters.kix) + "; kdx = "
                                  + floatToStr(parameters.kdx) + "; kpy = "
                                  + floatToStr(parameters.kpy) + "; kiy = "
                                  + floatToStr(parameters.kiy) + "; kdy = "
                                  + floatToStr(parameters.kdy) + "; loop = "
                                  + boolToStr(parameters.loop) + "; standLeft = "
                                  + boolToStr(parameters.standLeft) + "; ignoreHead = "
                                  + boolToStr(parameters.ignoreHead) + "; adjustKickFootPosition = "
                                  + std::to_string(parameters.adjustKickFootPosition) + "; phaseParameters = [";

  for(int i = 0; i < phase; i++)
  {
    if(i)
      setNewKickMotion += ", ";
    setNewKickMotion += " { duration = "
                        + std::to_string(parameters.phaseParameters[i].duration) + "; "
                        + "leftFootTra1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][1].z()) + "; }; "
                        + "leftFootTra2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootTra][2].z()) + "; }; "
                        + "leftFootRot1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][1].z()) + "; }; "
                        + "leftFootRot2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftFootRot][2].z()) + "; }; "
                        + "rightFootTra1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][1].z()) + "; }; "
                        + "rightFootTra2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootTra][2].z()) + "; }; "
                        + "rightFootRot1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][1].z()) + "; }; "
                        + "rightFootRot2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightFootRot][2].z()) + "; }; "
                        + "leftArmTra1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][1].z()) + "; }; "
                        + "leftArmTra2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftArmTra][2].z()) + "; }; "
                        + "leftHandRot1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][1].z()) + "; }; "
                        + "leftHandRot2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::leftHandRot][2].z()) + "; }; "
                        + "rightArmTra1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][1].z()) + "; }; "
                        + "rightArmTra2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightArmTra][2].z()) + "; }; "
                        + "rightHandRot1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][1].z()) + "; }; "
                        + "rightHandRot2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].controlPoints[Phase::rightHandRot][2].z()) + "; }; "
                        + "comTra1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].comTra[1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].comTra[1].y()) + "; }; "
                        + "comTra2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].comTra[2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].comTra[2].y()) + "; }; "
                        + "headTra1 = { x = "
                        + floatToStr(parameters.phaseParameters[i].headTra[1].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].headTra[1].y()) + "; }; "
                        + "headTra2 = { x = "
                        + floatToStr(parameters.phaseParameters[i].headTra[2].x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].headTra[2].y()) + "; }; "
                        + "odometryOffset = { x = "
                        + floatToStr(parameters.phaseParameters[i].odometryOffset.x()) + "; y = "
                        + floatToStr(parameters.phaseParameters[i].odometryOffset.y()) + "; z = "
                        + floatToStr(parameters.phaseParameters[i].odometryOffset.z()) + "; }; }";
  }

  setNewKickMotion += " ];";
  commands.push_back(setNewKickMotion);
}

void KickViewWidget::resetRobot()
{
  std::string moveRobot = "mv 0 0 300 0 0 180";
  commands.push_back(moveRobot);
  commands.push_back(" ");
}

void KickViewWidget::standRobot()
{
  std::string command = kickView.motionRequestCommand;
  std::string firstOfcom = command.substr(0, command.find("motion") + 9);
  std::string temp = command.substr(command.find("motion") + 10, command.size() - command.find("motion") + 10);
  std::string endOfcom = temp.substr(temp.find_first_of(";"), temp.size());
  std::string stand = firstOfcom + "stand" + endOfcom;
  commands.push_back(stand);
  commands.push_back(" ");
}

void KickViewWidget::setSelectedFromEditor(int index)
{
  selectedPoint.phaseNumber = index - 1;
}

void KickViewWidget::setDrawings(bool value)
{
  phaseDrawings = value;
}

void KickViewWidget::setSingleDrawing(bool value)
{
  singleDraw = value;
}

void KickViewWidget::setReachedDrawing(bool value)
{
  reachedDraw = value;
}

void KickViewWidget::setTra2d(bool value)
{
  tra2dWindows = value;
}

void KickViewWidget::setTra1d(bool value)
{
  tra1dWindows = value;
}

void KickViewWidget::setVelocity(bool value)
{
  velocityWindows = value;
}

void KickViewWidget::setAccel(bool value)
{
  accelWindows = value;
}

void KickViewWidget::setEditor(bool value)
{
  if(value)
    tabber->setHidden(false);
  else
    tabber->hide();
}

void KickViewWidget::setFollowMode(bool value)
{
  followMode = value;
}

void KickViewWidget::transparencyChanged(int i)
{
  ghost = i;
}

void KickViewWidget::updateCommon()
{
  disconnect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
  QStandardItem* rootItem = modelCommon->invisibleRootItem();
  rootItem->child(0, 1)->setData(QString(parameters.name), Qt::DisplayRole);
  rootItem->child(1, 1)->setData(QString::number(parameters.numberOfPhases), Qt::DisplayRole);

  rootItem->child(2, 0)->child(0, 2)->setData(QVariant(parameters.footOrigin.x()), Qt::DisplayRole);
  rootItem->child(2, 0)->child(0, 3)->setData(QVariant(parameters.footOrigin.y()), Qt::DisplayRole);
  rootItem->child(2, 0)->child(0, 4)->setData(QVariant(parameters.footOrigin.z()), Qt::DisplayRole);

  rootItem->child(3, 0)->child(0, 2)->setData(QVariant(parameters.footRotOrigin.x()), Qt::DisplayRole);
  rootItem->child(3, 0)->child(0, 3)->setData(QVariant(parameters.footRotOrigin.y()), Qt::DisplayRole);
  rootItem->child(3, 0)->child(0, 4)->setData(QVariant(parameters.footRotOrigin.z()), Qt::DisplayRole);

  rootItem->child(4, 0)->child(0, 2)->setData(QVariant(parameters.armOrigin.x()), Qt::DisplayRole);
  rootItem->child(4, 0)->child(0, 3)->setData(QVariant(parameters.armOrigin.y()), Qt::DisplayRole);
  rootItem->child(4, 0)->child(0, 4)->setData(QVariant(parameters.armOrigin.z()), Qt::DisplayRole);

  rootItem->child(5, 0)->child(0, 2)->setData(QVariant(parameters.handRotOrigin.x()), Qt::DisplayRole);
  rootItem->child(5, 0)->child(0, 3)->setData(QVariant(parameters.handRotOrigin.y()), Qt::DisplayRole);
  rootItem->child(5, 0)->child(0, 4)->setData(QVariant(parameters.handRotOrigin.z()), Qt::DisplayRole);

  rootItem->child(6, 0)->child(0, 2)->setData(QVariant(parameters.comOrigin.x()), Qt::DisplayRole);
  rootItem->child(6, 0)->child(0, 3)->setData(QVariant(parameters.comOrigin.y()), Qt::DisplayRole);

  rootItem->child(7, 0)->child(0, 2)->setData(QVariant(parameters.headOrigin.x()), Qt::DisplayRole);
  rootItem->child(7, 0)->child(0, 3)->setData(QVariant(parameters.headOrigin.y()), Qt::DisplayRole);

  rootItem->child(8, 0)->child(0, 2)->setData(QVariant(parameters.kpx), Qt::DisplayRole);
  rootItem->child(8, 0)->child(0, 3)->setData(QVariant(parameters.kix), Qt::DisplayRole);
  rootItem->child(8, 0)->child(0, 4)->setData(QVariant(parameters.kdx), Qt::DisplayRole);

  rootItem->child(9, 0)->child(0, 2)->setData(QVariant(parameters.kpy), Qt::DisplayRole);
  rootItem->child(9, 0)->child(0, 3)->setData(QVariant(parameters.kiy), Qt::DisplayRole);
  rootItem->child(9, 0)->child(0, 4)->setData(QVariant(parameters.kdy), Qt::DisplayRole);

  rootItem->child(10, 1)->setData(QVariant(parameters.loop), Qt::DisplayRole);
  rootItem->child(11, 1)->setData(QVariant(parameters.standLeft), Qt::DisplayRole);
  rootItem->child(12, 1)->setData(QVariant(parameters.ignoreHead), Qt::DisplayRole);

  connect(modelCommon, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updateCommonParameters(QStandardItem*)));
}

void KickViewWidget::updateEditorView()
{
  //Change Common Tab

  updateCommon();

  //Clear and Destroy and Resize
  for(int i = 0; i < tabber->count(); i++)
    tabber->removeTab(i);

  //this->repaint();
  tabber->addTab(treeViewCommon, "Common");
  treeViewCommon->expandAll();

  if(!treeView.empty())
  {
    for(QTreeView* itTree : treeView)
      delete itTree;

    treeView.clear();
  }

  if(!phaseTab.empty())
  {
    for(QStandardItemModel* itModel : phaseTab)
    {
      deleteKids(static_cast<QStandardItemModel*>(itModel)->invisibleRootItem());
      disconnect(static_cast<QStandardItemModel*>(itModel), SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updatePhaseParameters(QStandardItem*)));
      delete itModel;
    }
    phaseTab.clear();
  }

  treeView.resize(parameters.numberOfPhases);
  phaseTab.resize(parameters.numberOfPhases);

  //readIn possible Phases
  for(int i = 0; i < parameters.numberOfPhases; i++)
  {
    treeView[i] = new QTreeView();
    treeView[i]->setDragDropMode(QTreeView::DragOnly);
    treeView[i]->setSelectionMode(QTreeView::ExtendedSelection);
    treeView[i]->header()->setVisible(true);
    treeView[i]->setRootIsDecorated(true);
    phaseTab[i] = makeNewModelWithLabels();
    fillModelWithPhaseData(i);
    treeView[i]->setDragEnabled(false);
    treeView[i]->setAcceptDrops(false);
    treeView[i]->setModel(phaseTab[i]);
    treeView[i]->expandToDepth(3);
    treeView[i]->header()->setSectionResizeMode(0, QHeaderView::Interactive);
    treeView[i]->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    treeView[i]->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    treeView[i]->header()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    treeView[i]->header()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
    tabber->addTab(treeView[i], QString("Phase %1").arg(i));
  }

  updateGL();
}

QList <QStandardItem*> KickViewWidget::makeStringAndValueRow(QString string, QVariant value)
{
  QList <QStandardItem*> list;
  QStandardItem* item1, *item2;
  item1 = makeLabel(string);
  item2 = makeValue(value);
  list.append(item1);
  list.append(item2);
  return list;
}

void KickViewWidget::addStringAndValueToList(QList <QStandardItem*>& list, QString string, QVariant value)
{
  QStandardItem* item1, *item2;
  item1 = makeLabel(string);
  item2 = makeValue(value);
  list.append(item1);
  list.append(item2);
}

QStandardItem* KickViewWidget::makeLabel(QString string)
{
  QStandardItem* item = new QStandardItem(string);
  item->setEditable(false);
  item->setDragEnabled(false);
  item->setDropEnabled(false);
  return item;
}

QStandardItem* KickViewWidget::makeValue(QVariant var)
{
  QStandardItem* item = new QStandardItem();
  item->setData(var, Qt::DisplayRole);
  item->setDragEnabled(false);
  item->setDropEnabled(false);
  return item;
}

void KickViewWidget::addControlPoints(QStandardItem* item)
{
  QList <QStandardItem*> list;
  list.append(makeLabel(QString("Point0")));
  list.append(makeLabel(QString("")));

  for(int j = 0; j < 3; j++)
  {
    QStandardItem* it = new QStandardItem();
    it->setData(0.0, Qt::DisplayRole);
    it->setDragEnabled(false);
    it->setDropEnabled(false);
    it->setEditable(false);
    list.append(it);
  }

  item->appendRow(list);

  for(unsigned int k = 1; k < Phase::numOfPoints; k++)
  {
    QList <QStandardItem*> list;
    list.append(makeLabel(QString("Point%1").arg(k)));
    list.append(makeLabel(QString("")));

    for(int i = 0; i < 3; i++)
      list.append(makeValue(0.0));

    item->appendRow(list);
  }
}

QList <QStandardItem*> KickViewWidget::addXYZLabel(QStandardItem* item)
{
  QList <QStandardItem*> list;
  list.append(item);
  list.append(makeLabel(QString("")));
  list.append(makeLabel(QString("x")));
  list.append(makeLabel(QString("y")));
  list.append(makeLabel(QString("z")));

  return list;
}

QStandardItemModel* KickViewWidget::makeNewModelWithLabels()
{
  QStandardItemModel* model = new QStandardItemModel();
  model->setColumnCount(4);
  model->setHeaderData(0, Qt::Horizontal, QVariant(" "));
  model->setHeaderData(1, Qt::Horizontal, QVariant(" "));
  model->setHeaderData(2, Qt::Horizontal, QVariant(" "));
  model->setHeaderData(3, Qt::Horizontal, QVariant(" "));
  model->setHeaderData(4, Qt::Horizontal, QVariant(" "));
  model->setHeaderData(5, Qt::Horizontal, QVariant(" "));

  QStandardItem* parentItem = model->invisibleRootItem();

  parentItem->appendRow(makeStringAndValueRow(QString("Duration in ms"), 0));

  QStandardItem* lFoot = makeLabel(QString("Left Foot"));
  parentItem->appendRow(lFoot);

  QStandardItem* lFootControlPointsTra = makeLabel(QString("Translation"));
  addControlPoints(lFootControlPointsTra);
  lFoot->appendRow(addXYZLabel(lFootControlPointsTra));

  QStandardItem* lFootControlPointsRot = makeLabel(QString("Rotation"));
  addControlPoints(lFootControlPointsRot);
  lFoot->appendRow(addXYZLabel(lFootControlPointsRot));

  QStandardItem* rFoot = makeLabel(QString("Right Foot"));
  parentItem->appendRow(rFoot);

  QStandardItem* rFootControlPointsTra = makeLabel(QString("Translation"));
  addControlPoints(rFootControlPointsTra);
  rFoot->appendRow(addXYZLabel(rFootControlPointsTra));

  QStandardItem* rFootControlPointsRot = makeLabel(QString("Rotation"));
  addControlPoints(rFootControlPointsRot);
  rFoot->appendRow(addXYZLabel(rFootControlPointsRot));

  QStandardItem* lArm = makeLabel(QString("Left Hand"));
  parentItem->appendRow(lArm);

  QStandardItem* lArmTra = makeLabel(QString("Translation"));
  addControlPoints(lArmTra);
  lArm->appendRow(addXYZLabel(lArmTra));

  QStandardItem* lArmRot = makeLabel(QString("Rotation (Spherical Coord. with angle around Vec.)"));
  addControlPoints(lArmRot);
  QList <QStandardItem*> lArmList;
  lArmList.append(lArmRot);
  lArmList.append(makeLabel(QString("")));
  lArmList.append(makeLabel(QString("theta")));
  lArmList.append(makeLabel(QString("phi")));
  lArmList.append(makeLabel(QString("omega")));
  lArm->appendRow(lArmList);

  QStandardItem* rArm = makeLabel(QString("Right Hand"));
  parentItem->appendRow(rArm);

  QStandardItem* rArmTra = makeLabel(QString("Translation"));
  addControlPoints(rArmTra);
  rArm->appendRow(addXYZLabel(rArmTra));

  QStandardItem* rArmRot = makeLabel(QString("Rotation (Spherical Coord. with angle around Vec.)"));
  addControlPoints(rArmRot);
  QList <QStandardItem*> rArmList;
  rArmList.append(rArmRot);
  rArmList.append(makeLabel(QString("")));
  rArmList.append(makeLabel(QString("theta")));
  rArmList.append(makeLabel(QString("phi")));
  rArmList.append(makeLabel(QString("omega")));
  rArm->appendRow(rArmList);

  QStandardItem* comTrajectory = makeLabel(QString("COM Trajectory"));
  parentItem->appendRow(comTrajectory);

  QStandardItem* COMTra = makeLabel(QString("Translation (ground projection)"));
  comTrajectory->appendRow(addXYZLabel(COMTra));

  for(unsigned int k = 0; k < Phase::numOfPoints; k++)
  {
    QList <QStandardItem*> list;
    list.append(makeLabel(QString("Point%1").arg(k)));
    list.append(makeLabel(QString("")));
    for(int i = 0; i < 2; i++)
    {
      list.append(makeValue(0.0));
    }
    COMTra->appendRow(list);
  }

  QStandardItem* headTrajectory = makeLabel(QString("Head Trajectory"));
  parentItem->appendRow(headTrajectory);

  QStandardItem* HEADTra = makeLabel(QString("Translation"));
  headTrajectory->appendRow(addXYZLabel(HEADTra));

  for(unsigned int k = 0; k < Phase::numOfPoints; k++)
  {
    QList <QStandardItem*> list;
    list.append(makeLabel(QString("Point%1").arg(k)));
    list.append(makeLabel(QString("")));
    for(int i = 0; i < 2; i++)
    {
      list.append(makeValue(0.0));
    }
    HEADTra->appendRow(list);
  }

  QList <QStandardItem*> list;
  QStandardItem* odometry = makeLabel(QString("Odometry Change"));
  list.append(odometry);
  list.append(makeLabel(QString("")));
  list.append(makeLabel(QString("x")));
  list.append(makeLabel(QString("y")));
  list.append(makeLabel(QString("rotZ")));
  parentItem->appendRow(list);

  QList <QStandardItem*> list2;
  list2.append(makeLabel(QString("")));
  list2.append(makeLabel(QString("")));
  list2.append(makeValue(0.f));
  list2.append(makeValue(0.f));
  list2.append(makeValue(0.f));
  odometry->appendRow(list2);

  return model;
}

bool KickViewWidget::deleteKids(QStandardItem* rootItem)
{
  if(rootItem->hasChildren())
  {
    for(int i = 0; i < rootItem->rowCount(); i++)
      deleteKids(rootItem->child(i, 0));

    rootItem->removeRows(0, rootItem->rowCount());
  }
  return true;
}

void KickViewWidget::fillModelWithPhaseData(int i)
{
  if(i < parameters.numberOfPhases)
  {
    QStandardItem* rootItem = phaseTab[i]->invisibleRootItem();

    disconnect(phaseTab[i], SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updatePhaseParameters(QStandardItem*)));

    //Duration im ms
    rootItem->child(0, 1)->setData(QVariant(parameters.phaseParameters[i].duration), Qt::DisplayRole);

    for(unsigned int k = 0; k < Phase::numOfPoints; k++)
    {
      rootItem->child(5, 0)->child(0, 0)->child(k, 2)->setData(QVariant(parameters.phaseParameters[i].comTra[k].x()), Qt::DisplayRole);
      rootItem->child(5, 0)->child(0, 0)->child(k, 3)->setData(QVariant(parameters.phaseParameters[i].comTra[k].y()), Qt::DisplayRole);
      rootItem->child(6, 0)->child(0, 0)->child(k, 2)->setData(QVariant(parameters.phaseParameters[i].headTra[k].x()), Qt::DisplayRole);
      rootItem->child(6, 0)->child(0, 0)->child(k, 3)->setData(QVariant(parameters.phaseParameters[i].headTra[k].y()), Qt::DisplayRole);
    }

    rootItem->child(7, 0)->child(0, 2)->setData(QVariant(parameters.phaseParameters[i].odometryOffset.x()), Qt::DisplayRole);
    rootItem->child(7, 0)->child(0, 3)->setData(QVariant(parameters.phaseParameters[i].odometryOffset.y()), Qt::DisplayRole);
    rootItem->child(7, 0)->child(0, 4)->setData(QVariant(parameters.phaseParameters[i].odometryOffset.z()), Qt::DisplayRole);

    int limb = 0, child1 = 1, child2 = 0;
    QStandardItem* item;
    while(limb < Phase::numOfLimbs)
    {
      switch(limb)
      {
        case 0:
          child1 = 1;
          child2 = 0;
          break;
        case 1:
          child1 = 1;
          child2 = 1;
          break;
        case 2:
          child1 = 2;
          child2 = 0;
          break;
        case 3:
          child1 = 2;
          child2 = 1;
          break;
        case 4:
          child1 = 3;
          child2 = 0;
          break;
        case 5:
          child1 = 3;
          child2 = 1;
          break;
        case 6:
          child1 = 4;
          child2 = 0;
          break;
        case 7:
          child1 = 4;
          child2 = 1;
          break;
        default:
          limb = 100;
          break;
      }

      for(unsigned int k = 0; k < Phase::numOfPoints; k++)
      {
        item = rootItem->child(child1, 0)->child(child2, 0);
        item->child(k, 2)->setData(QVariant(parameters.phaseParameters[i].controlPoints[limb][k].x()), Qt::DisplayRole);
        item->child(k, 3)->setData(QVariant(parameters.phaseParameters[i].controlPoints[limb][k].y()), Qt::DisplayRole);
        item->child(k, 4)->setData(QVariant(parameters.phaseParameters[i].controlPoints[limb][k].z()), Qt::DisplayRole);
      }
      limb++;
    }
    connect(phaseTab[i], SIGNAL(itemChanged(QStandardItem*)), this, SLOT(updatePhaseParameters(QStandardItem*)));
  }
}

void KickViewWidget::updateCommonParameters(QStandardItem* item)
{
  parent->addStateToUndoList();

  QStandardItem* mamaItem = item->parent();
  int row = item->row();
  if(!mamaItem)
  {
    if(row == 10) parameters.loop = item->data(Qt::DisplayRole).toInt();
    if(row == 11) parameters.standLeft = item->data(Qt::DisplayRole).toInt();
    if(row == 12) parameters.ignoreHead = item->data(Qt::DisplayRole).toInt();
  }
  else
  {
    int col = item->column();
    if(QString("Foot Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.footOrigin.x();
        parameters.footOrigin.x() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootTra][k].x() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootTra][k].x() += diff;
          }
      }
      if(col == 3)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.footOrigin.y();
        parameters.footOrigin.y() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootTra][k].y() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootTra][k].y() -= diff;
          }
      }
      if(col == 4)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.footOrigin.z();
        parameters.footOrigin.z() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootTra][k].z() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootTra][k].z() += diff;
          }
      }
    }
    if(QString("Foot Rot Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.footRotOrigin.x();
        parameters.footRotOrigin.x() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootRot][k].x() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootRot][k].x() -= diff;
          }
      }
      if(col == 3)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.footRotOrigin.y();
        parameters.footRotOrigin.y() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootRot][k].y() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootRot][k].y() += diff;
          }
      }
      if(col == 4)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.footRotOrigin.z();
        parameters.footRotOrigin.z() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftFootRot][k].z() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightFootRot][k].z() -= diff;
          }
      }
    }
    if(QString("Hand Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.armOrigin.x();
        parameters.armOrigin.x() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftArmTra][k].x() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightArmTra][k].x() += diff;
          }
      }
      if(col == 3)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.armOrigin.y();
        parameters.armOrigin.y() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftArmTra][k].y() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightArmTra][k].y() -= diff;
          }
      }
      if(col == 4)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.armOrigin.z();
        parameters.armOrigin.z() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftArmTra][k].z() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightArmTra][k].z() += diff;
          }
      }
    }
    if(QString("Hand Rot Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.handRotOrigin.x();
        parameters.handRotOrigin.x() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftHandRot][k].x() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightHandRot][k].x() -= diff;
          }
      }
      if(col == 3)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.handRotOrigin.y();
        parameters.handRotOrigin.y() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftHandRot][k].y() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightHandRot][k].y() += diff;
          }
      }
      if(col == 4)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.handRotOrigin.z();
        parameters.handRotOrigin.z() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].controlPoints[Phase::leftHandRot][k].z() += diff;
            parameters.phaseParameters[p].controlPoints[Phase::rightHandRot][k].z() -= diff;
          }
      }
    }
    if(QString("COM Origin (only for no auto COM)") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.comOrigin.x();
        parameters.comOrigin.x() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].comTra[k].x() += diff;
          }
      }
      if(col == 3)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.comOrigin.y();
        parameters.comOrigin.y() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].comTra[k].y() += diff;
          }
      }
    }
    if(QString("Head Origin") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.headOrigin.x();
        parameters.headOrigin.x() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].headTra[k].x() += diff;
          }
      }
      if(col == 3)
      {
        float diff = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
        diff -= parameters.headOrigin.y();
        parameters.headOrigin.y() += diff;
        for(int p = 0; p < parameters.numberOfPhases; ++p)
          for(int k = 0; k < 3; ++k)
          {
            parameters.phaseParameters[p].headTra[k].y() += diff;
          }
      }
    }
    if(QString("COM Balance X") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2) parameters.kpx = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
      if(col == 3) parameters.kix = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
      if(col == 4) parameters.kdx = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
    }
    if(QString("COM Balance Y") == mamaItem->data(Qt::DisplayRole))
    {
      if(col == 2) parameters.kpy = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
      if(col == 3) parameters.kiy = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
      if(col == 4) parameters.kdy = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
    }
  }

  updateEditorView();
  parameters.initFirstPhase();
}

void KickViewWidget::updatePhaseParameters(QStandardItem* item)
{
  QStandardItem* mamaItem = item->parent();
  int row = item->row();
  int phaseNumber = tabber->currentIndex() - 1;
  int col = item->column();
  int limb = -1;

  parent->addStateToUndoList();

  if(!mamaItem)
  {
    if(row == 0) parameters.phaseParameters[phaseNumber].duration = item->data(Qt::DisplayRole).toInt();
  }
  else
  {
    if(QString("Left Foot") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      if(QString("Translation") == mamaItem->data(Qt::DisplayRole))
        limb = Phase::leftFootTra;
      if(QString("Rotation") == mamaItem->data(Qt::DisplayRole))
        limb = Phase::leftFootRot;
    }

    if(QString("Right Foot") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      if(QString("Translation") == mamaItem->data(Qt::DisplayRole))
        limb = Phase::rightFootTra;
      if(QString("Rotation") == mamaItem->data(Qt::DisplayRole))
        limb = Phase::rightFootRot;
    }
    if(QString("Right Hand") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      if(QString("Translation") == mamaItem->data(Qt::DisplayRole))
        limb = Phase::rightArmTra;
      if(QString("Rotation (Spherical Coord. with angle around Vec.)") == mamaItem->data(Qt::DisplayRole))
        limb = Phase::rightHandRot;
    }
    if(QString("Left Hand") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      if(QString("Translation") == mamaItem->data(Qt::DisplayRole))
        limb = Phase::leftArmTra;
      if(QString("Rotation (Spherical Coord. with angle around Vec.)") == mamaItem->data(Qt::DisplayRole))
        limb = Phase::leftHandRot;
    }

    if(QString("COM Trajectory") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      switch(col)
      {
        case 2:
          parameters.phaseParameters[phaseNumber].comTra[row].x() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
        case 3:
          parameters.phaseParameters[phaseNumber].comTra[row].y() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
      }
    }

    if(QString("Head Trajectory") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      switch(col)
      {
        case 2:
          parameters.phaseParameters[phaseNumber].headTra[row].x() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
        case 3:
          parameters.phaseParameters[phaseNumber].headTra[row].y() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
      }
    }

    if(QString("Odometry Change") == mamaItem->parent()->data(Qt::DisplayRole))
    {
      switch(col)
      {
        case 2:
          parameters.phaseParameters[phaseNumber].odometryOffset.x() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
        case 3:
          parameters.phaseParameters[phaseNumber].odometryOffset.y() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
        case 4:
          parameters.phaseParameters[phaseNumber].odometryOffset.z() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
      }
    }
  }

  selectedPoint.limb = limb;
  if(limb > -1)
  {
    if(row < Phase::numOfPoints)
    {
      selectedPoint.pointNumber = row;
      switch(col)
      {
        case 2:
          parameters.phaseParameters[phaseNumber].controlPoints[limb][row].x() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
        case 3:
          parameters.phaseParameters[phaseNumber].controlPoints[limb][row].y() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
        case 4:
          parameters.phaseParameters[phaseNumber].controlPoints[limb][row].z() = static_cast<float>(item->data(Qt::DisplayRole).toDouble());
          break;
      }
    }
  }
  parameters.initFirstPhase();
}

void KickViewWidget::recordPose()
{
  int phaseNumber = tabber->currentIndex() - 1;
  const JointAngles& jointAngles = kickView.jointAngles;

  if(phaseNumber > -1)
  {
    Pose3f p;

    if(lhtra->isChecked())
    {
      p = KickViewMath::calculateHandPos(jointAngles, Joints::lShoulderPitch, kickView.robotDimensions);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftArmTra][2] = p.translation;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftArmTra][1] = p.translation;
    }

    if(rhtra->isChecked())
    {
      p = KickViewMath::calculateHandPos(jointAngles, Joints::rShoulderPitch, kickView.robotDimensions);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightArmTra][2] = p.translation;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightArmTra][1] = p.translation;
    }

    if(lhrot->isChecked())
    {
      p = KickViewMath::calculateHandPos(jointAngles, Joints::lShoulderPitch, kickView.robotDimensions);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftHandRot][2] = p.rotation.getPackedAngleAxisFaulty();
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftHandRot][1] = p.rotation.getPackedAngleAxisFaulty();
    }

    if(rhrot->isChecked())
    {
      p = KickViewMath::calculateHandPos(jointAngles, Joints::rShoulderPitch, kickView.robotDimensions);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightHandRot][2] = p.rotation.getPackedAngleAxisFaulty();
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightHandRot][1] = p.rotation.getPackedAngleAxisFaulty();
    }

    if(lfrot->isChecked())
    {
      float sign = 1.f;
      Joints::Joint joint = Joints::lHipYawPitch;

      RotationMatrix rotateBecauseOfHip = RotationMatrix::aroundZ(jointAngles.angles[joint]).rotateX(-sign * pi_4);
      RotationMatrix footRot = RotationMatrix::aroundX((jointAngles.angles[joint + 1] + pi_4) * -sign).rotateY(jointAngles.angles[joint + 2] + jointAngles.angles[joint + 3]);
      footRot = footRot.inverse() * rotateBecauseOfHip;

      float leg5 = jointAngles.angles[joint + 5] - (std::asin(-footRot.col(2).y()) * sign * -1);
      float leg4 = jointAngles.angles[joint + 4] - (-std::atan2(footRot.col(2).x(), footRot.col(2).z()) * -1);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][2].x() = leg5;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][2].y() = leg4;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][1].x() = leg5;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][1].y() = leg4;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][2].z() = jointAngles.angles[joint] * sign;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][1].z() = jointAngles.angles[joint] * sign;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][2].z() = jointAngles.angles[joint] * -sign;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][1].z() = jointAngles.angles[joint] * -sign;
    }

    if(rfrot->isChecked())
    {
      float sign = -1.f;
      Joints::Joint joint = Joints::rHipYawPitch;

      RotationMatrix rotateBecauseOfHip = RotationMatrix::aroundZ(jointAngles.angles[joint] * sign).rotateX(-sign * pi_4);
      RotationMatrix footRot = RotationMatrix::aroundX((jointAngles.angles[joint + 1] + pi_4) * -sign).rotateY(jointAngles.angles[joint + 2] + jointAngles.angles[joint + 3]);
      footRot = (footRot.inverse() * rotateBecauseOfHip).eval();

      float leg5 = jointAngles.angles[joint + 5] - (std::asin(-footRot.col(2).y()) * sign * -1);
      float leg4 = jointAngles.angles[joint + 4] - (-std::atan2(footRot.col(2).x(), footRot.col(2).z()) * -1);

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][2].x() = leg5;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][2].y() = leg4;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][1].x() = leg5;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][1].y() = leg4;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][2].z() = jointAngles.angles[joint] * -sign;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootRot][1].z() = jointAngles.angles[joint] * -sign;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][2].z() = jointAngles.angles[joint] * sign;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootRot][1].z() = jointAngles.angles[joint] * sign;
    }

    if(lftra->isChecked())
    {
      p = KickViewMath::calculateFootPos(jointAngles, Joints::lHipYawPitch, kickView.robotDimensions).translation;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootTra][2] = p.translation;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::leftFootTra][1] = p.translation;
    }
    if(rftra->isChecked())
    {
      p = KickViewMath::calculateFootPos(jointAngles, Joints::rHipYawPitch, kickView.robotDimensions).translation;

      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootTra][2] = p.translation;
      parameters.phaseParameters[phaseNumber].controlPoints[Phase::rightFootTra][1] = p.translation;
    }

    updateCommon();
    for(int i = 0; i < parameters.numberOfPhases; i++)
    {
      fillModelWithPhaseData(i);
      treeView[i]->expandToDepth(3);
    }
  }
}
