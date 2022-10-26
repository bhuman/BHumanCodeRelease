#include <QFrame>
#include <QGridLayout>
#include <QAction>
#include <QString>

#include "Platform/BHAssert.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#include "models/Team.h"
#include "models/Robot.h"
#include "ui/TeamSelector.h"
#include "ui/TeamView.h"
#include "ui/RobotView.h"

TeamSelector::TeamSelector()
{
  static const size_t NUM_PLAYERS = 12; //due to F1..F12 keys and two rows with 1-6 players
  selectActions.reserve(NUM_PLAYERS);
  for(int i = 0; i < static_cast<int>(NUM_PLAYERS); ++i)
  {
    QAction* a = new QAction(this);
    a->setShortcut(QKeySequence(Qt::Key_F1 + i));
    addAction(a);
    connect(a, &QAction::triggered, this, &TeamSelector::selectPlayer);
    selectActions.push_back(a);
  }

  QAction* aNext = new QAction(this);
  aNext->setShortcut(QKeySequence(static_cast<int>(Qt::CTRL) + static_cast<int>(Qt::Key_PageDown)));
  addAction(aNext);
  connect(aNext, &QAction::triggered, this, &TeamSelector::selectNext);

  QAction* aPrev = new QAction(this);
  aPrev->setShortcut(QKeySequence(static_cast<int>(Qt::CTRL) + static_cast<int>(Qt::Key_PageUp)));
  addAction(aPrev);
  connect(aPrev, &QAction::triggered, this, &TeamSelector::selectPrev);
}

TeamSelector::~TeamSelector()
{
  for(const Team* team : teams)
    delete team;
}

void TeamSelector::addTeam(Team* team)
{
  ASSERT(std::find(teams.begin(), teams.end(), team) == teams.end());
  teams.push_back(team);
  TeamView* teamPage = new TeamView(this, team);
  teamViews[team] = teamPage;
  teamPages[team] = addTab(teamPage, QString::fromStdString("Team: " + team->name));
  ASSERT(teams.size() == teamViews.size());
  ASSERT(teams.size() == teamPages.size());
}

void TeamSelector::removeTeam(Team* team)
{
  const auto pageIndexIter = teamPages.find(team);
  if(pageIndexIter != teamPages.end())
  {
    removeTab(pageIndexIter->second);
    teamPages.erase(pageIndexIter);
  }
  const auto viewIndexIter = teamViews.find(team);
  if(viewIndexIter != teamViews.end())
  {
    viewIndexIter->second->deleteLater();
    teamViews.erase(viewIndexIter);
  }
  const auto teamsIter = std::remove(teams.begin(), teams.end(), team);
  if(teamsIter != teams.end())
  {
    delete team;
    teams.erase(teamsIter);
  }
  ASSERT(teams.size() == teamViews.size());
  ASSERT(teams.size() == teamPages.size());
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
    FAIL("No team selected.");
    return std::vector<Robot*>();
  }
}

void TeamSelector::loadTeams(const QString& filename, bool overwrite)
{
  if(overwrite)
    while(!teams.empty())
      removeTeam(teams.front());

  std::vector<Team> loadedTeams;
  InMapFile stream(filename.toStdString());
  ASSERT(stream.exists());
  Team::readTeams(stream, loadedTeams);
  for(const Team& team : loadedTeams)
    addTeam(new Team(team));
}

void TeamSelector::saveTeams(const QString& filename)
{
  std::vector<Team> _teams;
  _teams.reserve(teams.size());
  for(const Team* team : teams)
    _teams.push_back(*team);
  OutMapFile stream(filename.toStdString());
  Team::writeTeams(stream, _teams);
}

void TeamSelector::selectPlayer()
{
  QObject* s = sender();
  if(!s)
    return;
  const auto it = std::find(selectActions.begin(), selectActions.end(), s);
  if(it == selectActions.end())
    return;

  const int number = static_cast<int>(it - selectActions.begin());
  Team* t = getSelectedTeam();
  t->setSelectPlayer(number, !t->isPlayerSelected(number));
  teamViews[t]->update(number);
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
