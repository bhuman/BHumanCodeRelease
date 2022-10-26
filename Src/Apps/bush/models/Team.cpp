#include "Team.h"
#include "models/Robot.h"
#include "Session.h"
#include "tools/Platform.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"

Team::Team(const std::string& name, unsigned short number) :
  name(name),
  number(number)
{
  players.fill({nullptr, nullptr});
}

/**
 * Adds a robot to the players of this Team. If there already is a robot with playerNumber, this robot will be omitted.
 *
 * @param playerNumber The number of the robot in the team.
 *                Substitute robots have the same number as the robots they substitute.
 * @param substitutePlayer Defines weather the robot is a substiturte for another robot, or not.
 * @param robot The robot to add.
 */
void Team::addPlayer(unsigned int playerNumber, bool substitutePlayer, Robot* robot)
{
  const std::size_t index = substitutePlayer ? 1 : 0;
  if(players[playerNumber][index])
    return;
  else
    players[playerNumber][index] = robot;
  selectedPlayers[robot] = false;
}

const std::array<std::array<Robot*, 2>, Team::maxPlayers>& Team::getPlayersPerNumber() const
{
  return players;
}

std::vector<Robot*> Team::getPlayers() const
{
  std::vector<Robot*> robots;
  robots.reserve(players.size());
  for(size_t i = 0; i < players.size(); ++i)
    for(size_t j = 0; j < players[i].size(); ++j)
      robots.push_back(players[i][j]);
  return robots;
}

std::vector<Robot*> Team::getPlayersWrapped() const
{
  std::vector<Robot*> robots;
  size_t max = 0;
  for(size_t i = 0; i < players.size(); ++i)
    if(players[i].size() > max)
      max = players[i].size();

  robots.reserve(players.size() * max);
  for(size_t i = 0; i < max; ++i)
    for(size_t j = 0; j < players.size(); ++j)
      robots.push_back(players[j][i]);
  return robots;
}

void Team::changePlayer(unsigned short number, unsigned short pos, Robot* robot)
{
  players[number - 1][pos] = robot;
}

void Team::read(In& stream)
{
  STREAM(name);
  STREAM(number);
  STREAM(color);
  STREAM(scenario);
  STREAM(location);
  STREAM(compile);
  STREAM(buildConfig);
  STREAM(wlanConfig);
  STREAM(volume);
  STREAM(deployDevice);
  STREAM(magicNumber);
  std::vector<std::string> players;
  STREAM(players);
  std::map<std::string, Robot*> robots = Session::getInstance().robotsByName;
  for(size_t i = 0; i < players.size(); ++i)
    if(players[i] != "_")
      addPlayer(i % maxPlayers, i / maxPlayers, robots[players[i]]);
}

void Team::write(Out& stream) const
{
  STREAM(name);
  STREAM(number);
  STREAM(color);
  STREAM(scenario);
  STREAM(location);
  STREAM(compile);
  STREAM(buildConfig);
  STREAM(wlanConfig);
  STREAM(volume);
  STREAM(deployDevice);
  STREAM(magicNumber);
  std::vector<std::string> players;
  std::vector<Robot*> robots = getPlayersWrapped();
  for(Robot* r : robots)
    players.push_back(r ? r->name : "_");
  STREAM(players);
}

void Team::setSelectPlayer(Robot* robot, bool select)
{
  if(robot)
    selectedPlayers[robot] = select;
}

void Team::setSelectPlayer(size_t index, bool select)
{
  Robot* r = players[(index % maxPlayers)][index / maxPlayers];
  setSelectPlayer(r, select);
}

bool Team::isPlayerSelected(Robot* robot)
{
  return robot ? selectedPlayers[robot] : false;
}

bool Team::isPlayerSelected(size_t index)
{
  Robot* r = players[(index % maxPlayers)][index / maxPlayers];
  return isPlayerSelected(r);
}

std::vector<Robot*> Team::getSelectedPlayers() const
{
  std::vector<Robot*> robots;
  robots.reserve(selectedPlayers.size());
  for(std::map<Robot*, bool>::const_iterator i = selectedPlayers.begin();
      i != selectedPlayers.end();
      ++i)
    if(i->second)
      robots.push_back(i->first);
  return robots;
}

unsigned short Team::getPlayerNumber(const Robot& robot) const
{
  //TODO: maybe it is better to have a map which saves the playernumber for
  //every robot of the team
  std::vector<Robot*> robots = getPlayersWrapped();
  for(size_t i = 0; i < robots.size(); ++i)
    if(robots[i] != 0 && robots[i]->name == robot.name)
      return i % maxPlayers + 1;
  return 0;
}

void Team::writeTeams(Out& stream, const std::vector<Team>& teams)
{
  stream << TeamsStreamer(const_cast<std::vector<Team>&>(teams));
}

void Team::readTeams(In& stream, std::vector<Team>& teams)
{
  TeamsStreamer s(teams);
  stream >> s;
}
