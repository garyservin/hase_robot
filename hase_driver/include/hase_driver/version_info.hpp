/**
 * @file include/hase_driver/version_info.hpp
 *
 * @brief Version info for the hase driver.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki_core/hydro-devel/kobuki_driver/LICENSE
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef HASE_VERSION_HPP_
#define HASE_VERSION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <sstream>
#include <stdint.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hase {

/*****************************************************************************
** Interfaces
*****************************************************************************/
/**
 * Class holding version info for the hase driver.
 */
class VersionInfo {
public:
  static std::string getSoftwareVersion();
};

} // namespace hase
#endif /* HASE_VERSION_HPP_ */
