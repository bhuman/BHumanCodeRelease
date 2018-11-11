#include "Platform/File.h"
#include "Tools/Streams/InStreams.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/tools/Platform.h"

static const size_t MAX_PLAYERS = 6;

void Team::init()
{
  players.reserve(MAX_PLAYERS);
  for(size_t i = 0; i < MAX_PLAYERS; ++i)
  {
    std::vector<Robot*> number;
    number.reserve(2);
    number.push_back(0);
    number.push_back(0);
    players.push_back(number);
  }
}

Team::Team()
  : players(),
    selectedPlayers(),
    name(""),
    number(0),
    port(0),
    color(""),
    scenario(""),
    location(""),
    wlanConfig(""),
    compile(true),
    buildConfig(""),
    volume(100),
    deployDevice(""),
    magicNumber(-1)
{
  init();
}

Team::Team(const std::string& name, unsigned short number)
  : players(),
    selectedPlayers(),
    name(name),
    number(number),
    port(0),
    color(""),
    scenario(""),
    location(""),
    wlanConfig(""),
    compile(true),
    buildConfig(""),
    volume(100),
    deployDevice(""),
    magicNumber(-1)
{
  init();
  this->port = number + 10000;
}

/**
 * Adds a robot to the players of this Team. If there already is a robot with playerNumber, this robot will be omitted.
 *
 * @param playerNumber The number of the robot in the team.
 *                Substitute robots have the same number as the robots they substitute.
 * @param substitutePlayer Defines weather the robot is a substiturte for another robot, or not.
 * @param robot The robot to add.
 */
void Team::addPlayer(unsigned int playerNumber, bool  substitutePlayer, Robot& robot)
{
  if(substitutePlayer)
  {
    if(players[playerNumber][1])
      return;
    else
      players[playerNumber][1] = &robot;
  }
  else
  {
    if(players[playerNumber][0])
      return;
    else
      players[playerNumber][0] = &robot;
  }
  selectedPlayers[&robot] = false;
}

std::vector<std::vector<Robot*>> Team::getPlayersPerNumber() const
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

std::vector<Team> Team::getTeams(const std::string& filename)
{
  std::string _filename = filename;
  if(filename == "")
    _filename = "teams.cfg";

  std::vector<Team> teams;
  InMapFile stream(linuxToPlatformPath(_filename));
  if(!stream.exists())
  {
    Session::getInstance().log(CRITICAL, "Team: Cannot read teams from " + _filename);
    return teams;
  }
  readTeams(stream, teams);
  return teams;
}

void Team::changePlayer(unsigned short number, unsigned short pos, Robot* robot)
{
  players[number - 1][pos] = robot;
}

void Team::serialize(In* in, Out* out)
{
  STREAM(name);
  STREAM(number);
  STREAM(port);
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
  if(out)
  {
    std::vector<Robot*> robots = getPlayersWrapped();
    for(Robot* r : robots)
      players.push_back(r ? r->name : "_");
  }
  STREAM(players);
  if(in)
  {
    std::map<std::string, Robot*> robots = Session::getInstance().robotsByName;
    for(size_t i = 0; i < players.size(); ++i)
      if(players[i] != "_")
        addPlayer(i % MAX_PLAYERS, i / MAX_PLAYERS, *robots[players[i]]);
  }
}

void Team::setSelectPlayer(Robot* robot, bool select)
{
  if(robot)
    selectedPlayers[robot] = select;
}

void Team::setSelectPlayer(size_t index, bool select)
{
  Robot* r = players[(index % MAX_PLAYERS)][index / MAX_PLAYERS];
  setSelectPlayer(r, select);
}

bool Team::isPlayerSelected(Robot* robot)
{
  return robot ? selectedPlayers[robot] : false;
}

bool Team::isPlayerSelected(size_t index)
{
  Robot* r = players[(index % MAX_PLAYERS)][index / MAX_PLAYERS];
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
      return i % MAX_PLAYERS + 1;
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
  for(Team& t : teams)
    Session::getInstance().log(TRACE, "Team: Loaded team \"" + t.name + "\".");
}
