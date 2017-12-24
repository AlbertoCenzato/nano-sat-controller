#pragma once

#include <string>

namespace sat {
namespace config {

inline int getVersionMajor() { return 0; }
inline int getVersionMinor() { return 5; }
inline std::string getInstallDirectory()   { return "/home/pi/NanoSatController";   }
inline std::string getVersion() { return std::to_string(getVersionMajor()) + "." + std::to_string(getVersionMinor()); }

} // namespace config
} // namespace sat
