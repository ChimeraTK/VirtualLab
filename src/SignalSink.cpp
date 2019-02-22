/*
 * SignalSink.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: martin.hierholzer@desy.de
 */

#include "SignalSink.h"

namespace ChimeraTK { namespace VirtualLab {

  /*******************************************************************************************************************/
  SignalSink::SignalSink(double defaultValue) : historyLength(0) {
    signalSource = boost::static_pointer_cast<SignalSource>(boost::make_shared<ConstantSignalSource>(defaultValue));
  }

  /*******************************************************************************************************************/
  void SignalSink::connect(const boost::shared_ptr<SignalSource>& source) {
    signalSource = source;
    signalSource->setMaxHistoryLength(historyLength);
  }

  /*******************************************************************************************************************/
  void SignalSink::setMaxHistoryLength(VirtualTime timeDifference) {
    historyLength = timeDifference;
    signalSource->setMaxHistoryLength(historyLength);
  }

}} // namespace ChimeraTK::VirtualLab
