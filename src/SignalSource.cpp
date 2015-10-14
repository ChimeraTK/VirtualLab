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
    timeTolerance(std::numeric_limits<double>::epsilon()),
    historyLength(std::numeric_limits<double>::epsilon()),
    currentTime(0)
  {}

  /*******************************************************************************************************************/
  void SignalSource::setCallback(const boost::function<double(double)> &callback) {
    valueNeededCallback = callback;
  }

  /*******************************************************************************************************************/
  void SignalSource::setTimeTolerance(double time) {
    timeTolerance = time;
  }

  /*******************************************************************************************************************/
  void SignalSource::setMaxHistoryLength(double timeDifference) {
    historyLength = timeDifference;
  }

  /*******************************************************************************************************************/
  ConstantSignalSource::ConstantSignalSource(double theValue)
  : value(theValue)
  {
    valueNeededCallback = boost::bind(&ConstantSignalSource::constantCallback, this, _1);
    timeTolerance = std::numeric_limits<double>::max();
  }

}} // namespace mtca4u::VirtuaLab


