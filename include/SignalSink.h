/*
 * SignalSink.h
 *
 *  Created on: Oct 13, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef SIGNALSINK_H
#define SIGNALSINK_H

#include <boost/make_shared.hpp>
#include "SignalSource.h"

namespace mtca4u { namespace VirtualLab {

  /// A SignalSink requensts time-dependent signal values from a SignalSource. Objects of this class will normally be
  /// members of a VirtualLabBackend or a model component.
  class SignalSink {

    public:

      /// Constructor: create sink with default value returned when not connected
      SignalSink(double defaultValue);

      /// Constructor: create sink already connected with a source.
      SignalSink(boost::shared_ptr<SignalSource> &source);

      /// [call from VirtualLab setup code] (re-)connect this SignalSink with a SignalSource. Any previous connection
      /// to another source will be severed.
      void connect(const boost::shared_ptr<SignalSource> &source);

      /// [call from backend] obtain value for the given time
      inline double getValue(double time) {
        return signalSource->getValue(time);
      }

    protected:

      /// the source providing our signal
      boost::shared_ptr<SignalSource> signalSource;

  };

}}  // namespace mtca4u::VirtualLab

#endif /* SIGNALSINK_H */
