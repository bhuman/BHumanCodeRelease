#include "Platform/File.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/tools/Platform.h"

static const size_t MAX_PLAYERS = 5;

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
    location(""),
    wlanConfig(""),
    buildConfig("")
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
    location(""),
    wlanConfig(""),
    buildConfig("")
{
  init();
  this->port = number * 100 + 10001;
}

/**
 * Adds a robot to the players of this Team. If there already is a robot with playerNumber, this robot will be omitted.
 *
 * @param playerNumber The number of the robot in the team.
 *				Substitute robots have the same number as the robots they substitute.
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

std::vector<std::vector<Robot*> > Team::getPlayersPerNumber() const
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
  ConfigMap cm;
  int status = cm.read(linuxToPlatformPath(_filename));
  if(status < 1)
  {
    Session::getInstance().log(CRITICAL, "Team: Cannot read teams from " + _filename);
    return teams;
  }
  readTeams(cm, teams);
  return teams;
}

void Team::changePlayer(unsigned short number, unsigned short pos, Robot* robot)
{
  size_t i = number - 1;
  players[i][pos] = robot;
}

ConfigMap& operator<< (ConfigMap& cv, const Team& team)
{
  cv["number"]      << team.number;
  cv["port"]        << team.port;
  cv["color"]       << team.color;
  cv["location"]    << team.location;
  cv["buildConfig"] << team.buildConfig;
  cv["wlanConfig"]  << team.wlanConfig;

  std::vector<Robot*> robots = team.getPlayersWrapped();
  for(size_t i = 0; i < robots.size(); ++i)
  {
    Robot* r = robots[i];
    if(r)
      cv["players"][i] << r->name;
    else
      cv["players"][i] << "_";
  }
  return cv;
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

const ConfigMap& operator>> (const ConfigMap& cv, Team& team)
{
  std::map<std::string, Robot*> robots = Session::getInstance().robotsByName;

  cv["number"] >> team.number;
  if(cv.hasKey("port", false))
    cv["port"] >> team.port;
  else
    team.port = team.number * 100 + 10001;
  if(cv.hasKey("color", false))
    cv["color"] >> team.color;
  else
    team.color = "red";
  if(cv.hasKey("location", false))
    cv["location"] >> team.location;
  else
    team.location = "Default";
  if(cv.hasKey("wlanConfig", false))
    cv["wlanConfig"] >> team.wlanConfig;
  if(cv.hasKey("buildConfig", false))
    cv["buildConfig"] >> team.buildConfig;
  std::vector<std::string> playerNames;
  cv["players"] >> playerNames;
  for(size_t i = 0; i < playerNames.size(); ++i)
    if(playerNames[i] != "_")
    {
      team.addPlayer(i % MAX_PLAYERS, i / MAX_PLAYERS, *robots[playerNames[i]]);
    }
  return cv;
}

void Team::writeTeams(ConfigMap& cv, const std::vector<Team>& teams)
{
  for(size_t i = 0; i < teams.size(); ++i)
  {
    const Team& t = teams[i];
    cv[t.name] << t;
  }
}

void Team::readTeams(const ConfigMap& cv, std::vector<Team>& teams)
{
  teams.clear();
  std::vector<std::string> keys = cv.getKeys(false);
  for(size_t i = 0; i < keys.size(); ++i)
  {
    std::string& key = keys[i];
    Team t;
    cv[key] >> t;
    t.name = key;
    teams.push_back(t);
    Session::getInstance().log(TRACE, "Team: Loaded team \"" + t.name + "\".");
  }
}

CONFIGMAP_STREAM_IN(ConfigMap, Team);
CONFIGMAP_STREAM_OUT(ConfigMap, Team);

