#pragma once

#include <string>

namespace sat {
namespace config {

inline int getVersionMajor() { return 1; }
inline int getVersionMinor() { return 0; }
inline std::string getInstallDirectory()   { return "/usr/local";   }
inline std::string getVersion() { return std::to_string(getVersionMajor()) + "." + std::to_string(getVersionMinor()); }

} // namespace config
} // namespace sat
