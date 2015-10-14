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
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <mtca4u/Exception.h>

namespace mtca4u { namespace VirtualLab {

  /// Exception class
  class SignalSourceException : public Exception {
    public:
      enum {NO_VALUE,REQUEST_FAR_PAST};
      SignalSourceException(const std::string &message, unsigned int exceptionID)
      : Exception( message, exceptionID ){}
  };

  /// A SignalSource provides time-dependent signal values to a SignalSink. Objects of this class will normally be
  /// members of a VirtualLabBackend or a model component.
  class SignalSource {

    public:

      SignalSource();

      /// [call from backend/model] set callback function to be called when a new value needs to be computed. The
      /// callback function must return the new value. The new value will be automatically placed into the buffer.
      void setCallback(const boost::function<double(double)> &callback);

      /// [call from backend/model] set time tolerance. A value for the time T will be used when a value for the time
      /// T+tolerance is requested. Backends may set this e.g. to the sampling time, since the output value will not
      /// change during this time. Models may set this e.g. to a fraction of the model's time constant to save computing
      /// time.
      void setTimeTolerance(double time);

      /// [call from backend/model] provide new value for the given time
      inline void feedValue(double time, double value) {
        // save value into buffer
        buffer[time] = value;
        // update current time
        if(time > currentTime) {
          currentTime = time;
          // clear old values from history
          if(buffer.begin()->first < time - historyLength) {
            auto firstToKeep = buffer.upper_bound(time - historyLength);
            buffer.erase(buffer.begin(), firstToKeep);
          }
        }
      }

      /// [called from sink] obtain value for the given time
      inline double getValue(double time) {
        // if buffer is empty, request sample
        if(buffer.empty()) return getValueFromCallback(time);
        // check if request goes too far into the past
        if(currentTime - historyLength > time) {
          throw SignalSourceException("Value request is too far into the past.",SignalSourceException::REQUEST_FAR_PAST);
        }
        // search in buffer: find the first element after the requested time
        auto it = buffer.upper_bound(time + std::numeric_limits<double>::epsilon());
        // if this is the first element in the buffer, no sample for the requested time exists
        if(it == buffer.begin()) return getValueFromCallback(time);
        // decrement to get the most recent sample before the requested time
        --it;
        // if no sample found or sample is too old, request one via callback
        if(it->first < time - timeTolerance) return getValueFromCallback(time);
        // return value from buffer
        return it->second;
      }

      /// [called from sink] set maximum time difference a getValue() request may go into the past
      void setMaxHistoryLength(double timeDifference);

    protected:

      /// obtain a new value via the callback function and place it into the buffer. Helper for getValue()
      inline double getValueFromCallback(double time) {
        if(valueNeededCallback == NULL) {
          throw SignalSourceException("No value matching the given time found.",SignalSourceException::NO_VALUE);
        }
        double val = valueNeededCallback(time);
        feedValue(time, val);
        return val;
      }

      /// callback called when new value is needed from backend or model
      boost::function<double(double)> valueNeededCallback;

      /// buffer of values (in dependence of time)
      std::map<double,double> buffer;

      /// time tolerance
      double timeTolerance;

      /// history length
      double historyLength;

      /// time of last fed sample
      double currentTime;

  };

  /// A simple SignalSource providing just constant values
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
