/**
 * @file Module.cpp
 *
 * This file implements some Python bindings to go through logs.
 *
 * @author Arne Hasselbring
 * @author Jan Fiedler
 */

#include "Log.h"
#include "Frame.h"
#include "Types.h"
#include "Framework/LoggingTools.h"
#include "Platform/SystemCall.h"
#include "Streaming/FunctionList.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <vector>

namespace py = pybind11;

/** Wrap LoggingTools::parseName. */
static py::tuple parseLogName(const std::string& path)
{
  std::string headName;
  std::string bodyName;
  std::string scenario;
  std::string location;
  std::string identifier;
  int playerNumber = -1;
  std::string suffix;

  LoggingTools::parseName(path.substr(path.find_last_of("/\\") + 1), &headName, &bodyName, &scenario, &location, &identifier, &playerNumber, &suffix);
  return py::make_tuple(headName, bodyName, scenario, location, identifier, playerNumber, suffix);
}

SystemCall::Mode SystemCall::getMode()
{
  return logFileReplay;
}


// Required to extract version info.
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

struct ArrayAccessor
{
  PyTypeVariant operator()(const std::vector<Value*>::iterator& it)
  {
    return (*it)->toVariant();
  }
};

PYBIND11_MODULE(logs, m)
{
#ifdef VERSION_INFO
  m.attr("__version__") = TOSTRING(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  m.attr("__name__") = "pybh.logs";
  m.doc() = R"bhdoc(
B-Human Logs Wrapper

Python bindings to go through B-Human log files.
)bhdoc";

  FunctionList::execute();

  m.def("parse_log_name", &parseLogName, R"bhdoc(Parses a log file name into its components.

Args:
    path: The file name to parse.

Returns:
    headName, bodyName, scenario, location, identifier, playerNumber, suffix of the log.
)bhdoc", py::arg("path"));

  py::class_<Log>(m, "Log")
    .def(py::init<const std::string&, bool>(), R"bhdoc(This class represents an log File.

The entire log is always loaded into the RAM.

Args:
    path: The file path.
    keep_going: Whether corrupt frames should be ignored as much as possible.
)bhdoc", py::arg("path"), py::arg("keep_going") = false)
    .def_readonly("headName", &Log::headName, "The head name of the log.")
    .def_readonly("bodyName", &Log::bodyName, "The body name of the log.")
    .def_readonly("scenario", &Log::scenario, "The scenario of the log.")
    .def_readonly("location", &Log::location, "The location of the log.")
    .def_readonly("identifier", &Log::identifier, "The identifier of the log.")
    .def_readonly("playerNumber", &Log::playerNumber, "The player number of the log.")
    .def_readonly("suffix", &Log::suffix, "The suffix of the log.")
    .def("__len__", [](const Log& log) { return log.numberOfFrames; })
    // The log is alive as long as a reference to a frame exists.
    .def("__iter__", &Log::iter, py::keep_alive<0, 1>()); // loop

  py::class_<Frame>(m, "Frame", "Represents the data of a single frame in the log.")
    .def_readonly("thread", &Frame::thread, "The thread of this frame.")
    .def_property_readonly("representations", &Frame::getRepresentations, "A list of names of the representations this frame has.")
    .def_property_readonly("annotations", &Frame::getAnnotations, "A list of annotations this frame has.")
    .def("__next__", &Frame::next) // loop
    .def("__iter__", &Frame::iter) // loop
    .def("__contains__", &Frame::contains, py::arg("x")) // 'x' in frame
    .def("__getitem__", &Frame::getitem, py::arg("index")); // frame['x']

  py::class_<Value>(m, "Value", "A base class for all value types for usage in the same STL-containers.");

  py::class_<Literal, Value>(m, "Literal", R"bhdoc(Represents a literal.

Is mostly extracted automatically by the lib.
)bhdoc")
    .def_property_readonly("literal", &Literal::operator PyTypeVariant, "Get the real literal.");

  //py::bind_vector<Array>(m, "Array");  // Cannot be used because of __getitem__.
  py::class_<Array, Value>(m, "Array", "A :class:`list` of type :class:`.Value` .")
    .def("__len__", [](const Array& array) {
      return array.size();
    })
    .def("__getitem__", [](Array& array, std::size_t index) -> PyTypeVariant {
      return array.at(index)->toVariant();
    }, py::return_value_policy::reference_internal, py::arg("index"))
    .def("__iter__", [](Array& array) {
      return py::detail::make_iterator_impl<ArrayAccessor, py::return_value_policy::reference_internal, decltype(array.begin()), decltype(array.end()), PyTypeVariant>(array.begin(), array.end());
    }, py::keep_alive<0, 1>());

  py::class_<Record, Value>(m, "Record", "A collection of named attributes (i.e. a streamable class).")
    .def("__getattr__", &Record::getattr, py::return_value_policy::reference_internal, py::arg("attr")) // record.x
    .def("__iter__", [](Record& record) {
      const auto& keys = record.getKeys();
      return py::make_iterator(keys.begin(), keys.end());
    }, py::keep_alive<0, 1>());

  py::class_<Annotation>(m, "Annotation", "An event annotation of a frame.")
    .def_readonly("name", &Annotation::name, "A short name of the annotation creator.")
    .def_readonly("description", &Annotation::description, "A description of the event.");
}
