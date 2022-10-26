#pragma once

#include <QTabWidget>
#include <QString>
#include <map>
#include <vector>

class Team;
struct Robot;
class QAction;
class TeamView;

class TeamSelector : public QTabWidget
{
  Q_OBJECT

  std::vector<Team*> teams;
  std::map<Team*, int> teamPages;
  std::map<Team*, TeamView*> teamViews;
  std::vector<QAction*> selectActions;

  /**
   * Adds a team to the selector and takes ownership of it.
   * @param team Pointer to the new team.
   */
  void addTeam(Team* team);

  /**
   * Removes a team from the selector if it was registered.
   * @param team Pointer to the team to remove.
   */
  void removeTeam(Team* team);

public:
  TeamSelector();
  ~TeamSelector();
  Team* getSelectedTeam() const;
  std::vector<Robot*> getSelectedRobots() const;
  void loadTeams(const QString& filename = "teams.cfg", bool overwrite = true);
  void saveTeams(const QString& filename = "teams.cfg");
private slots:
  void selectPlayer();
  void selectNext();
  void selectPrev();
};
