/*
 * SignalSource.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: martin.hierholzer@desy.de
 */

#include "SignalSource.h"

namespace mtca4u { namespace VirtualLab {

  /*******************************************************************************************************************/
  SignalSource::SignalSource()
  : valueNeededCallback(NULL),
    timeTolerance(1),
    historyLength(1),
    currentTime(0)
  {}

  /*******************************************************************************************************************/
  void SignalSource::setCallback(const boost::function<double(VirtualTime)> &callback) {
    valueNeededCallback = callback;
  }

  /*******************************************************************************************************************/
  void SignalSource::setTimeTolerance(VirtualTime time) {
    timeTolerance = time;
  }

  /*******************************************************************************************************************/
  void SignalSource::setMaxHistoryLength(VirtualTime timeDifference) {
    historyLength = timeDifference;
  }

  /*******************************************************************************************************************/
  ConstantSignalSource::ConstantSignalSource(double theValue)
  : value(theValue)
  {
    valueNeededCallback = boost::bind(&ConstantSignalSource::constantCallback, this, _1);
    timeTolerance = std::numeric_limits<VirtualTime>::max();
  }

}} // namespace mtca4u::VirtuaLab


