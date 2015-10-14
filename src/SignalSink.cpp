/*
 * SignalSink.cpp
 *
 *  Created on: Oct 14, 2015
 *      Author: martin.hierholzer@desy.de
 */

#include "SignalSink.h"

namespace mtca4u { namespace VirtualLab {

  /*******************************************************************************************************************/
  SignalSink::SignalSink(boost::shared_ptr<SignalSource> &source)
  : signalSource(source)
  {}

  /*******************************************************************************************************************/
  SignalSink::SignalSink(double defaultValue)
  {
    signalSource = boost::static_pointer_cast<SignalSource>(
        boost::make_shared<ConstantSignalSource>(defaultValue) );
  }

  /*******************************************************************************************************************/
  void SignalSink::connect(const boost::shared_ptr<SignalSource> &source) {
    signalSource = source;
  }

}} // namespace mtca4u::VirtuaLab
