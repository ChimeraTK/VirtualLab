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
#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <mtca4u/Exception.h>

#include "VirtualTime.h"
#include "StateVariableSet.h"


namespace ChimeraTK { namespace VirtualLab {

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

      /** [call from backend/model] Set maximum time gap. If a value further into the future of the latest computed
       *  value than the maximum gap time is requested, intermediate values will be computed so that no values are
       *  further apart than the gap.
       *
       *  If the gap is not set, intermediate states are never computed. The maximum gap must be larger than the
       *  validity period to have an effect.
       */
      void setMaximumGap(VirtualTime maxGap);

      /** [call from backend/model] Set - in addition to the maximum gap - an even bigger gap, which will be used to
       *  fill larger steps. If a state is requested into the far future (more than 2*hugeGap), intermediate steps
       *  with a distance of hugeGap (instead of maxGap) will be inserted to fill the gap. Only the last hugeGap
       *  interval before the requested state will be filled with intermediate steps in a distance of maxGap.
       *
       *  The hugeGap must be larger than maxGap. If not set, the feature is disabled and only maxGap is used to fill
       *  large steps.
       */
      void setHugeGap(VirtualTime hugeGap);

      /** Enable interpolation. No states will be fully computed by the model closer together than the maximum gap
       *  time. Instead requests for these times will be interpolated. Note that this usually will lead to requests
       *  to other model components being generated into the "future" (from the current request), since values are
       *  needed on both sides for an interpolation (in contrast to an extrapolation, which might be invalid). */
      void setEnableInterpolation(bool enable) {
#ifndef ENABLE_EXPERIMENTAL_FEATURES
        class ExperimentalFeatureNotEnabled{};
        throw ExperimentalFeatureNotEnabled();
#endif
        buffer.setEnableInterpolation(enable);
      }

      /** Set the function which interpolates between two states. The passed function has must accept the following
       *  arguments (in order):
       *  - the first support state to base the interpolation on
       *  - the second support state
       *  - the time of the first support state
       *  - the time of the second support state
       *  - the requested time to return the interpolated state for */
      void setInterpolateFunction(
          const boost::function<double(const double&, const double&, VirtualTime, VirtualTime, VirtualTime)> &callback) {
        buffer.setInterpolateFunction(callback);
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
        return buffer.getState(time);
      }

      /** Obtain the latest computed value.
       *  This function will usually be called inside the compute function (see setCallback()) to have a basis
       *  for the computations. Use getLatestTime() to obtain the time of the latest state.
       *
       *  Guarantee: When this function is called inside the compute function, the returned value will be in the past
       *  and not older than the gap configured with setMaximumGap().
       */
      inline double getLatestValue() {
        return buffer.getLatestState();
      }

      /** Obtain the time of latest computed value. See getLatestValue() for further comments.
       */
      inline VirtualTime getLatestTime() {
        return buffer.getLatestTime();
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

}}  // namespace ChimeraTK::VirtualLab


// Compatibility
namespace mtca4u { namespace VirtualLab {
  class SignalSource : public ChimeraTK::VirtualLab::SignalSource {
    using ChimeraTK::VirtualLab::SignalSource::SignalSource;
  };
}}

#endif /* SIGNALSOURCE_H */
