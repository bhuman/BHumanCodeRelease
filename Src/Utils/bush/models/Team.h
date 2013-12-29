#pragma once

#include <string>
#include <map>
#include <vector>
#include "Utils/bush/tools/ConfigMap.h"

class Robot;

class Team
{
  std::vector<std::vector<Robot*> > players;
  std::map<Robot*, bool> selectedPlayers;

  void init();
public:
  std::string name;
  unsigned short number;
  unsigned short port;
  std::string color;
  std::string location;
  std::string wlanConfig;
  std::string buildConfig;

  Team();
  Team(const std::string& name, unsigned short number);
  void addPlayer(unsigned int playerNumber, bool  substitutePlayer, Robot& robot);
  std::vector<std::vector<Robot*> > getPlayersPerNumber() const;
  std::vector<Robot*> getPlayers() const;
  std::vector<Robot*> getPlayersWrapped() const;
  unsigned short getPlayerNumber(const Robot& robot) const;
  static std::vector<Team> getTeams(const std::string& filename = "");

  void changePlayer(unsigned short number, unsigned short pos, Robot* robot);
  void setSelectPlayer(Robot* robot, bool select);
  void setSelectPlayer(size_t index, bool select);
  bool isPlayerSelected(Robot* robot);
  bool isPlayerSelected(size_t index);
  std::vector<Robot*> getSelectedPlayers() const;

  static void writeTeams(ConfigMap& cv, const std::vector<Team>& teams);
  static void readTeams(const ConfigMap& cv, std::vector<Team>& teams);
};

CONFIGMAP_STREAM_IN_DELCARE(Team);
CONFIGMAP_STREAM_OUT_DELCARE(Team);
