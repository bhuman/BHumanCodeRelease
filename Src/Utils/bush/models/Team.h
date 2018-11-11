#pragma once

#include <string>
#include <map>
#include <vector>
#include "Tools/Streams/AutoStreamable.h"

struct Robot;

class Team : public Streamable
{
  std::vector<std::vector<Robot*>> players;
  std::map<Robot*, bool> selectedPlayers;

  STREAMABLE(TeamsStreamer,
  {
   TeamsStreamer(std::vector<Team>& teams) : teams(teams) {},
   (std::vector<Team>&) teams,
  });;

  void init();

protected:
  void serialize(In*, Out*);

public:
  std::string name;
  unsigned short number;
  unsigned short port;
  std::string color;
  std::string scenario;
  std::string location;
  std::string wlanConfig;
  bool compile;
  std::string buildConfig;
  unsigned short volume;
  std::string deployDevice;
  int magicNumber;

  Team();
  Team(const std::string& name, unsigned short number);
  void addPlayer(unsigned int playerNumber, bool  substitutePlayer, Robot& robot);
  std::vector<std::vector<Robot*>> getPlayersPerNumber() const;
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

  static void writeTeams(Out& stream, const std::vector<Team>& teams);
  static void readTeams(In& stream, std::vector<Team>& teams);
};
