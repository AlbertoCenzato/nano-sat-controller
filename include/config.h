#pragma once

#include <string>

namespace sat {
namespace config {

inline int getVersionMajor() { return 0; }
inline int getVersionMinor() { return 5; }
inline std::string getInstallDirectory()   { return "C:/Users/alber/CMakeBuilds/cf4f4653-9c7a-8939-bc51-1cdbd8984c2a/install/x86-Debug";   }
inline std::string getVersion() { return std::to_string(getVersionMajor()) + "." + std::to_string(getVersionMinor()); }

} // namespace config
} // namespace sat
