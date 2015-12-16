/*
 * SignalSource.h
 *
 *  Created on: Oct 13, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef SIGNALSOURCE_H
#define SIGNALSOURCE_H

#include <map>
#include <limits>
#include <sstream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <mtca4u/Exception.h>

#include "VirtualTime.h"
#include "StateVariableSet.h"


namespace mtca4u { namespace VirtualLab {

  /**
   *  Exception class
   */
  class SignalSourceException : public Exception {
    public:
      enum {REQUEST_FAR_PAST};
      SignalSourceException(const std::string &message, unsigned int exceptionID)
      : Exception( message, exceptionID ){}
  };

  /**
   *  A SignalSource provides time-dependent signal values to a SignalSink. Objects of this class will normally be
   *  members of a VirtualLabBackend or a model component.
   */
  class SignalSource {

    public:

      SignalSource();

      /** [call from backend/model] set callback function to be called when a new value needs to be computed. The
       *  callback function must return the new value. The new value will be automatically placed into the buffer.
       *
       *  It is mandatory to set this callback function!
       */
      void setCallback(const boost::function<double(VirtualTime)> &callback);

      /** [call from backend/model] set validity period. A value for the time T will be used when a value for the time
       *  t < T+period is requested. Backends may set this e.g. to the sampling time, since the output value will not
       *  change during this time. Models may set this e.g. to a fraction of the model's time constant to save computing
       *  time.
       *
       *  The validity period defaults to 1, which effectively disables the functionality. The provided value must
       *  be larger than 0.
       */
      void setValidityPeriod(VirtualTime period);

      /** DEPRECATED, alias for setValidityPeriod()
       *  @todo remove after release of version 00.02
       */
      void setTimeTolerance(VirtualTime time) {
        std::cout << "SignalSource::setTimeTolerance(): This function is DEPCRECATED, use setValidityPeriod() instead!" << std::endl;
        setValidityPeriod(time);
      }

      /** [call from backend/model] provide new value for the given time
       */
      inline void feedValue(VirtualTime time, double value) {
        buffer.feedState(time,value);
      }

      /** [called from sink] obtain value for the given time. time must be >= 0.
       */
      inline double getValue(VirtualTime time) {
        // not yet initialised: compute the value manually
        if(buffer.getLatestTime() < 0) {
          double val = (buffer.getComputeFunction())(time);
          buffer.feedState(time,val);
        }

        // get value from buffer
        try {
          return buffer.getState(time);
        }
        catch(StateVariableSetException &e) {
          throw SignalSourceException(e.what(), SignalSourceException::REQUEST_FAR_PAST);
        }
      }

      /** [called from sink] set maximum time difference a getValue() request may go into the past
       */
      void setMaxHistoryLength(VirtualTime timeDifference);

      /** [call from model/backend] set callback function to be called when setMaxHistoryLength() is called
       *  The callback's argument is the new history length.
       */
      void setOnHistoryLengthChanged(const boost::function<void(VirtualTime)> &callback);

    protected:

      /// function called when connect() is called
      boost::function<void(VirtualTime)> onHistoryLengthChanged;

      /// store value in a StateVariableSet with just a double as type
      StateVariableSet<double> buffer;

  };

  /**
   * A simple SignalSource providing just constant values
   */
  class ConstantSignalSource : public SignalSource {

    public:

      ConstantSignalSource(double theValue);

    protected:

      /// callback just returing the constant value
      inline double constantCallback(double) {
        return value;
      }

      /// the value to be passed to the sink
      double value;

  };

}}  // namespace mtca4u::VirtualLab




#endif /* SIGNALSOURCE_H */
