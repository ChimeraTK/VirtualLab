/*
 * VirtualTime.h
 *
 *  Created on: Nov 11, 2015
 *      Author: Martin Hierholzer
 */

#ifndef VIRTUALTIME_H_
#define VIRTUALTIME_H_

#include <stdint.h>

namespace ChimeraTK { namespace VirtualLab {

  /**
   * Type and units for virtual time.
   *
   * Important note: since 64 bit integer is used as a type, attention has to be
   * paid when converting from double, since the precision might not be
   * sufficient. Example: "100*days + 1*picoseconds" will work as expected, while
   * "100*days + 1.*picoseconds" (with additional decimal point) will be equal
   * 100*days, since double precision is not sufficient to resolve the difference.
   */
  typedef int64_t VirtualTime;
  const static VirtualTime picoseconds = 1;
  const static VirtualTime nanoseconds = 1000 * picoseconds;
  const static VirtualTime microseconds = 1000 * nanoseconds;
  const static VirtualTime milliseconds = 1000 * microseconds;
  const static VirtualTime seconds = 1000 * milliseconds;
  const static VirtualTime minutes = 60 * seconds;
  const static VirtualTime hours = 60 * minutes;
  const static VirtualTime days = 24 * hours;

}} // namespace ChimeraTK::VirtualLab

// Compatibility
namespace mtca4u { namespace VirtualLab {
  typedef int64_t VirtualTime;
  const static VirtualTime picoseconds = 1;
  const static VirtualTime nanoseconds = 1000 * picoseconds;
  const static VirtualTime microseconds = 1000 * nanoseconds;
  const static VirtualTime milliseconds = 1000 * microseconds;
  const static VirtualTime seconds = 1000 * milliseconds;
  const static VirtualTime minutes = 60 * seconds;
  const static VirtualTime hours = 60 * minutes;
  const static VirtualTime days = 24 * hours;
}} // namespace mtca4u::VirtualLab

#endif /* VIRTUALTIME_H_ */
