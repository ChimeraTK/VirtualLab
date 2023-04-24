/*
 * SignalSource.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: martin.hierholzer@desy.de
 */

#include "SignalSource.h"

namespace ChimeraTK { namespace VirtualLab {

  /*******************************************************************************************************************/
  SignalSource::SignalSource() {}

  /*******************************************************************************************************************/
  void SignalSource::setCallback(const boost::function<double(VirtualTime)>& callback) {
    buffer.setComputeFunction(callback);
  }

  /*******************************************************************************************************************/
  void SignalSource::setValidityPeriod(VirtualTime period) { buffer.setValidityPeriod(period); }

  /*******************************************************************************************************************/
  void SignalSource::setMaximumGap(VirtualTime maxGap) { buffer.setMaximumGap(maxGap); }

  /*******************************************************************************************************************/
  void SignalSource::setHugeGap(VirtualTime hugeGap) { buffer.setHugeGap(hugeGap); }

  /*******************************************************************************************************************/
  void SignalSource::setMaxHistoryLength(VirtualTime timeDifference) {
    buffer.setMaxHistoryLength(timeDifference);
    if(!onHistoryLengthChanged.empty()) {
      onHistoryLengthChanged(timeDifference);
    }
  }

  /*******************************************************************************************************************/
  void SignalSource::setOnHistoryLengthChanged(const boost::function<void(VirtualTime)>& callback) {
    onHistoryLengthChanged = callback;
  }

  /*******************************************************************************************************************/
  ConstantSignalSource::ConstantSignalSource(double theValue) : SignalSource(), value(theValue) {
    setCallback(boost::bind(&ConstantSignalSource::constantCallback, this, boost::placeholders::_1));
    setValidityPeriod(std::numeric_limits<VirtualTime>::max());
  }

}} // namespace ChimeraTK::VirtualLab
