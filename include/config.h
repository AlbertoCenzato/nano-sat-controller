#pragma once

#include <string>

namespace sat {
namespace config {

inline int getVersionMajor() { return 1; }
inline int getVersionMinor() { return 0; }
inline int getVersionPatch() { return 0; }
inline std::string getInstallDirectory()   { return "C:/Users/alber/CMakeBuilds/cf4f4653-9c7a-8939-bc51-1cdbd8984c2a/install/x86-Debug";   }
inline std::string getVersion() { return std::string("1") + "." + "0" + "." + "0"; }

} // namespace config
} // namespace sat
