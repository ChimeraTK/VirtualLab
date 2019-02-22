/*
 * SignalSink.h
 *
 *  Created on: Oct 13, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef SIGNALSINK_H
#define SIGNALSINK_H

#include "SignalSource.h"
#include <boost/make_shared.hpp>

namespace ChimeraTK { namespace VirtualLab {

  /// A SignalSink requensts time-dependent signal values from a SignalSource.
  /// Objects of this class will normally be members of a VirtualLabBackend or a
  /// model component.
  class SignalSink {
   public:
    /// Constructor: create sink with default value returned when not connected
    SignalSink(double defaultValue);

    /// [call from VirtualLab setup code] (re-)connect this SignalSink with a
    /// SignalSource. Any previous connection to another source will be severed.
    void connect(const boost::shared_ptr<SignalSource>& source);

    /// [call from backend/model] obtain value for the given time
    inline double getValue(VirtualTime time) { return signalSource->getValue(time); }

    /// [call from backend/model] set maximum time difference a getValue() request
    /// may go into the past
    void setMaxHistoryLength(VirtualTime timeDifference);

   protected:
    /// the source providing our signal
    boost::shared_ptr<SignalSource> signalSource;

    /// history length to be requested from the source
    VirtualTime historyLength;
  };

}} // namespace ChimeraTK::VirtualLab

// Compatibility
namespace mtca4u { namespace VirtualLab {
  class SignalSink : public ChimeraTK::VirtualLab::SignalSink {
    using ChimeraTK::VirtualLab::SignalSink::SignalSink;
  };
}} // namespace mtca4u::VirtualLab

#endif /* SIGNALSINK_H */
