#pragma once

#include "Streaming/AutoStreamable.h"
#include <array>
#include <map>
#include <string>
#include <vector>

struct Robot;

class Team : public Streamable
{
  static constexpr std::size_t maxPlayers = 7;

  std::array<std::array<Robot*, 2>, maxPlayers> players;
  std::map<Robot*, bool> selectedPlayers;

  STREAMABLE(TeamsStreamer,
  {
    TeamsStreamer(std::vector<Team>& teams) : teams(teams) {},
    (std::vector<Team>&) teams,
  });;

protected:
  /**
   * Read this object from a stream.
   * @param stream The stream from which the object is read.
   */
  void read(In& stream) override;

  /**
   * Write this object to a stream.
   * @param stream The stream to which the object is written.
   */
  void write(Out& stream) const override;

public:
  std::string name;
  unsigned short number = 0;
  std::string color;
  std::string scenario;
  std::string location;
  std::string wlanConfig;
  bool compile = true;
  std::string buildConfig;
  unsigned short volume = 100;
  std::string deployDevice;
  int magicNumber = -1;

  Team(const std::string& name = std::string(), unsigned short number = 0);
  void addPlayer(unsigned int playerNumber, bool  substitutePlayer, Robot* robot);
  const std::array<std::array<Robot*, 2>, maxPlayers>& getPlayersPerNumber() const;
  std::vector<Robot*> getPlayers() const;
  std::vector<Robot*> getPlayersWrapped() const;
  unsigned short getPlayerNumber(const Robot& robot) const;

  void changePlayer(unsigned short number, unsigned short pos, Robot* robot);
  void setSelectPlayer(Robot* robot, bool select);
  void setSelectPlayer(size_t index, bool select);
  bool isPlayerSelected(Robot* robot);
  bool isPlayerSelected(size_t index);
  std::vector<Robot*> getSelectedPlayers() const;

  static void writeTeams(Out& stream, const std::vector<Team>& teams);
  static void readTeams(In& stream, std::vector<Team>& teams);
};
