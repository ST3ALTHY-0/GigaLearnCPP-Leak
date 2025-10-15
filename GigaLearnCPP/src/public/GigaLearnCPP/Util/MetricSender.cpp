#include "MetricSender.h"

#include "Timer.h"
#include <filesystem>

namespace py = pybind11;
using namespace GGL;

GGL::MetricSender::MetricSender(std::string _projectName, std::string _groupName, std::string _runName, std::string runID) :
	projectName(_projectName), groupName(_groupName), runName(_runName) {

	RG_LOG("Initializing MetricSender...");

	try {
		// Import Python sys module
		py::module sys = py::module::import("sys");
		py::list path = sys.attr("path");

		// Debug: Show what PY_EXEC_PATH contains
		RG_LOG(" > PY_EXEC_PATH: " << PY_EXEC_PATH);
		
		// Get the directory containing the executable (build folder)
		std::filesystem::path buildDir = std::filesystem::path(PY_EXEC_PATH).parent_path();
		RG_LOG(" > Build dir from PY_EXEC_PATH: " << buildDir.string());

		// Add a python_scripts subfolder in the build directory to sys.path
		std::filesystem::path pythonScriptsPath = buildDir / "python_scripts";
		RG_LOG(" > Python scripts path: " << pythonScriptsPath.string());
		
		// Also try using the current working directory
		std::filesystem::path currentDir = std::filesystem::current_path();
		std::filesystem::path currentPythonScripts = currentDir / "python_scripts";
		RG_LOG(" > Current working dir: " << currentDir.string());
		RG_LOG(" > Current python_scripts path: " << currentPythonScripts.string());
		
		// Add both paths to be safe
		path.append(pythonScriptsPath.string());
		path.append(currentPythonScripts.string());

		RG_LOG(" > Added to Python path: " << pythonScriptsPath.string());
		RG_LOG(" > Added to Python path: " << currentPythonScripts.string());
		
		// Check if the files actually exist
		std::filesystem::path metricFile1 = pythonScriptsPath / "metric_receiver.py";
		std::filesystem::path metricFile2 = currentPythonScripts / "metric_receiver.py";
		RG_LOG(" > metric_receiver.py exists at path 1: " << (std::filesystem::exists(metricFile1) ? "YES" : "NO"));
		RG_LOG(" > metric_receiver.py exists at path 2: " << (std::filesystem::exists(metricFile2) ? "YES" : "NO"));

		// Import the metric_receiver module
		pyMod = py::module::import("metric_receiver");
	} catch (std::exception& e) {
		RG_ERR_CLOSE("MetricSender: Failed to import metrics receiver, exception: " << e.what());
	}

	try {
		auto returedRunID = pyMod.attr("init")(PY_EXEC_PATH, projectName, groupName, runName, runID);
		curRunID = returedRunID.cast<std::string>();
		RG_LOG(" > " << (runID.empty() ? "Starting" : "Continuing") << " run with ID : \"" << curRunID << "\"...");
	} catch (std::exception& e) {
		RG_ERR_CLOSE("MetricSender: Failed to initialize in Python, exception: " << e.what());
	}

	RG_LOG(" > MetricSender initialized.");
}

void GGL::MetricSender::Send(const Report& report) {
	py::dict reportDict = {};

	for (auto& pair : report.data)
		reportDict[pair.first.c_str()] = pair.second;

	try {
		pyMod.attr("add_metrics")(reportDict);
	} catch (std::exception& e) {
		RG_ERR_CLOSE("MetricSender: Failed to add metrics, exception: " << e.what());
	}
}

GGL::MetricSender::~MetricSender() {
	
}