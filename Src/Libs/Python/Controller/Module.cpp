/**
 * @file Module.cpp
 *
 * This file implements Python bindings for simulating B-Human instances.
 *
 * @author Arne Hasselbring
 */

#include "Controller.h"
#include "Framework/Settings.h"
#include "Platform/File.h"
#include "Streaming/Enum.h"
#include "Streaming/FunctionList.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

// Required to extract version info.
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

PYBIND11_MODULE(controller, m)
{
#ifdef VERSION_INFO
  m.attr("__version__") = TOSTRING(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  m.attr("__name__") = "pybh.controller";
  m.doc() = R"bhdoc(
B-Human Controller

Python bindings for simulating B-Human instances.
)bhdoc";

  FunctionList::execute();

  m.def("get_bh_dir", &File::getBHDir, R"bhdoc(Returns the path to the B-Human root directory.

Returns:
    The path to the B-Human root directory.
)bhdoc");

  py::class_<Controller>(m, "Controller", "An interface to control a set of robots.")
    .def(pybind11::init<>())
    .def("add_player", &Controller::addPlayer, R"bhdoc(Adds a player to the controller.

Args:
    name: The name of the player (only influences thread names but not the config search path).
    team_number: The team number of the player.
    field_player_color: The jersey color of the field players in this player's team.
    goalkeeper_color: The jersey color of the goalkeeper in this player's team.
    player_number: The (jersey) number of the player.
    location: The location in the configuration file search path.
    scenario: The scenario in the configuration file search path.
)bhdoc", py::arg("name"), py::arg("team_number"), py::arg("field_player_color"), py::arg("goalkeeper_color"), py::arg("player_number"), py::arg("location"), py::arg("scenario"))
    .def("start", &Controller::start, "Starts all robot threads.")
    .def("stop", &Controller::stop, "Stops all robot threads.")
    .def("update", &Controller::update, "Triggers a frame in all robots.");

  auto teamColorType = py::enum_<Settings::TeamColor>(m, "TeamColor");
  FOREACH_ENUM(Settings::TeamColor, teamColor)
    teamColorType.value(TypeRegistry::getEnumName(teamColor), teamColor);
}
