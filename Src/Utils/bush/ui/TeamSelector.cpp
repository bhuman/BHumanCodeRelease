#include <QFrame>
#include <QGridLayout>
#include <QAction>

#include "Tools/Streams/OutStreams.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/ui/TeamSelector.h"
#include "Utils/bush/ui/TeamView.h"
#include "Utils/bush/ui/RobotView.h"

TeamSelector::TeamSelector()
  : teams(),
    teamsMap(),
    teamPages(),
    teamViews()
{
  static const size_t NUM_PLAYERS = 12; //due to F1..F12 keys and two rows with 1-6 players
  selectActions.reserve(NUM_PLAYERS);
  for(int i = 0; i < static_cast<int>(NUM_PLAYERS); ++i)
  {
    QAction* a = new QAction(this);
    a->setShortcut(QKeySequence(Qt::Key_F1 + i));
    addAction(a);
    connect(a, SIGNAL(triggered()), this, SLOT(selectPlayer()));
    selectActions.push_back(a);
  }

  QAction* aNext = new QAction(this);
  aNext->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown));
  addAction(aNext);
  connect(aNext, SIGNAL(triggered()), this, SLOT(selectNext()));

  QAction* aPrev = new QAction(this);
  aPrev->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp));
  addAction(aPrev);
  connect(aPrev, SIGNAL(triggered()), this, SLOT(selectPrev()));
}

void TeamSelector::addTeam(Team* team)
{
  teams.push_back(team);
  teamsMap[team->number] = team;
  std::map<unsigned short, int>::iterator i = teamPages.find(team->number);
  TeamView* teamPage = new TeamView(this, team);
  int index = -1;
  if(i != teamPages.end())
  {
    index = i->second;
    QWidget* oldPage = widget(index);
    removeTab(index);
    oldPage->deleteLater();
    insertTab(index, teamPage, fromString("Team: " + team->name));
  }
  else
  {
    index = addTab(teamPage, fromString("Team: " + team->name));
  }
  teamViews[team->number] = teamPage;
  teamPages[team->number] = index;
}

void TeamSelector::removeTeam(Team* team)
{
  std::map<unsigned short, int>::iterator pageIndexIter = teamPages.find(team->number);
  if(pageIndexIter != teamPages.end())
  {
    removeTab(pageIndexIter->second);
    teamPages.erase(pageIndexIter);
  }
  std::map<unsigned short, TeamView*>::iterator viewIndexIter = teamViews.find(team->number);
  if(viewIndexIter != teamViews.end())
  {
    viewIndexIter->second->deleteLater();
    teamViews.erase(viewIndexIter);
  }
  teamsMap.erase(team->number);
  for(size_t i = 0; i < teams.size(); ++i)
  {
    Team* t = teams[i];
    if(t->name == team->name)
    {
      teams.erase(teams.begin() + i);
      delete t;
    }
  }
  // do not use team from here since it can be invalid
}

Team* TeamSelector::getSelectedTeam() const
{
  if(!teams.size())
    return nullptr;
  int i = currentIndex();
  return teams[i >= 0 ? i : 0];
}

std::vector<Robot*> TeamSelector::getSelectedRobots() const
{
  Team* selectedTeam = getSelectedTeam();
  if(selectedTeam)
    return selectedTeam->getSelectedPlayers();
  else
  {
    Session::getInstance().log(CRITICAL, "TeamSelector: No team selected.");
    return std::vector<Robot*>();
  }
}

void TeamSelector::loadTeams(const QString& filename, bool overwrite)
{
  if(overwrite)
  {
    size_t teamCount = teams.size();
    for(size_t i = 0; i < teamCount; ++i)
      removeTeam(teams[i]);
  }

  std::vector<Team> loadedTeams = Team::getTeams(toString(filename));
  for(size_t i = 0; i < loadedTeams.size(); ++i)
    addTeam(new Team(loadedTeams[i]));
}

void TeamSelector::saveTeams(const QString& filename)
{
  std::vector<Team> _teams;
  _teams.reserve(teams.size());
  for(size_t i = 0; i < teams.size(); ++i)
    _teams.push_back(*teams[i]);
  OutMapFile stream(toString(filename));
  Team::writeTeams(stream, _teams);
}

void TeamSelector::saveTeams()
{
  saveTeams("teams.cfg");
}

void TeamSelector::selectPlayer()
{
  QObject* s = sender();
  if(!s)
    return;
  int number = -1;
  for(int i = 0; i < static_cast<int>(selectActions.size()); ++i)
    if(selectActions[i] == s)
      number = i;

  if(number >= 0)
  {
    Team* t = getSelectedTeam();
    t->setSelectPlayer(number, !t->isPlayerSelected(number));
    teamViews[t->number]->update(number);
  }
}

void TeamSelector::selectNext()
{
  if(currentIndex() + 1 < count())
    setCurrentIndex(currentIndex() + 1);
}

void TeamSelector::selectPrev()
{
  if(currentIndex() > 0)
    setCurrentIndex(currentIndex() - 1);
}
