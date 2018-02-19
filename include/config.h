#pragma once

#include <string>

namespace sat {
namespace config {

inline int getVersionMajor() { return 1; }
inline int getVersionMinor() { return 1; }
inline int getVersionPatch() { return 0; }
inline std::string getInstallDirectory()   { return "C:/Users/Alberto/CMakeBuilds/6c27f071-9f08-a63f-a64c-2ff8c421b476/install/x64-Debug";   }
inline std::string getVersion() { return std::string("1") + "." + "1" + "." + "0"; }

} // namespace config
} // namespace sat
