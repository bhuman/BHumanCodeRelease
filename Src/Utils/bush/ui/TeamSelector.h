#pragma once

#include <QTabWidget>
#include <QString>
#include <map>
#include <vector>

class Team;
struct Robot;
class QFrame;
class QAction;
class RobotView;
class TeamView;

class TeamSelector : public QTabWidget
{
  Q_OBJECT

  std::vector<Team*> teams;
  std::map<unsigned short, Team*> teamsMap;
  std::map<unsigned short, int> teamPages;
  std::map<unsigned short, TeamView*> teamViews;
  std::vector<QAction*> selectActions;
  void generateRobotViews(Team& team, QFrame* teamPage);
public:
  TeamSelector();
  void addTeam(Team* team);
  /** Deletes delivered team.
   * Do not use the pointer after a call of removeTeam if it pointed to the same
   * memory as the pointer in teams.
   */
  void removeTeam(Team* team);
  Team* getSelectedTeam() const;
  std::vector<Robot*> getSelectedRobots() const;
  void loadTeams(const QString& filename = "", bool overwrite = true);
  void saveTeams(const QString& filename);
private slots:
  void selectPlayer();
  void selectNext();
  void selectPrev();
  void saveTeams();
};
