/*
 * StateVariableSet.h
 *
 *  Created on: Dec 10, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef STATEVARIABLESET_H
#define STATEVARIABLESET_H

#include <map>
#include <limits>
#include <sstream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <mtca4u/Exception.h>

#include "VirtualTime.h"

namespace mtca4u { namespace VirtualLab {

  /// Exception class
  class StateVariableSetException : public Exception {
    public:
      enum {REQUEST_FAR_PAST};
      StateVariableSetException(const std::string &message, unsigned int exceptionID)
      : Exception( message, exceptionID ){}
  };

  /** A set of state variables contains a struct of time-dependent variables. In this class a mechanism is provided
    * which helps keeping track of these variables properly and computing values as needed.
    *
    * An example use-case for this class is to perform an integration (or sum) as it is needed for an integral
    * controller. Since the VirtualLab framework might request values from signal sources and sinks in an arbitrary
    * order, it is not possible to store them as normal member variables of the VirtualLabBackend or the model class.
    */
  template<class STATE>
  class StateVariableSet {

    public:

      StateVariableSet()
      : maxGap(0),
        timeTolerance(0),
        historyLength(0),
        currentTime(0)
      {
      }

      /** Set the initial state for time 0.
       *
       *  It is mandatory to set the initial state before the first call to getState().
       */
      void setInitialState(STATE &state) {
        buffer.clear();
        buffer[0] = state;
      }

      /** Set callback function which will compute a new state for a given time.
       *
       *  Guarantee: When this function is called for a time T, the latest state requested by getLatestState() will be
       *  in the past w.r.t. T and not older than the gap configured with setMaximumGap().
       *
       *  It is mandatory to set this function.
       */
      void setComputeFunction(const boost::function<const STATE(VirtualTime)> &callback) {
        compute = callback;
      }

      /** Set maximum time gap. If a state further into the future of the latest computed state than the maximum gap
       *  time is requested, intermediate states will be computed so that no states are further apart than the gap.
       *
       *  It is mandatory to configure this gap.
       */
      void setMaximumGap(VirtualTime time) {
        maxGap = time;
      }

      /** Set maximum time difference a getValue() request may go into the past.
       *
       *  It is mandatory to configure the history length.
       */
      void setMaxHistoryLength(VirtualTime timeDifference) {
        historyLength = timeDifference;
      }

      /** Set time tolerance. A value for the time T will be used when a value for the time T+tolerance is requested.
       *  It is not possible to have two states closer together in time than this tolerance.
       *
       *  Setting the tolerance is optional, it will default to 0.
       */
      void setTimeTolerance(VirtualTime time) {
        timeTolerance = time;
      }

      /// Obtain the state for the given time.
      inline const STATE& getState(VirtualTime time) {
        // search in buffer: find the first element after the requested time
        auto it = buffer.upper_bound(time);
        // if this is the first element in the buffer, request goes too far into the past
        if(it == buffer.begin())  {
          std::stringstream s;
          s << "Value request is too far into the past: ";
          s << "requested time = " << time << ", oldest history = " << buffer.begin()->first;
          throw StateVariableSetException(s.str(),StateVariableSetException::REQUEST_FAR_PAST);
        }
        // if this is end(), a new value needs to be computed
        if(it == buffer.end()) return getValueFromCallback(time);
        // decrement to get the most recent sample before the requested time
        --it;
        // if sample is too old, request one via callback
        if(it->first < time - timeTolerance) return getValueFromCallback(time);
        // return value from buffer
        return it->second;
      }

      /** Obtain the latest computed state.
       *  This function will usually be called inside the compute function (see setComputeFunction()) to have a basis
       *  for the computations. Use getLatestTime() to obtain the time of the latest state.
       *
       *  Guarantee: When this function is called inside the compute function, the returned state will be in the past
       *  and not older than the gap configured with setMaximumGap().
       */
      inline const STATE& getLatestState() {
        return buffer.rbegin()->second;
      }

      /** Obtain the time of latest computed state. See getLatestState() for further comments.
       */
      inline VirtualTime getLatestTime() {
        return currentTime;
      }

      /** Obtain the current map of states.
       */
      inline const std::map<VirtualTime,STATE>& getAllStates() {
        return buffer;
      }

    protected:

      /// obtain a new state via the callback function and place it into the buffer. Helper for getValue()
      inline const STATE& getValueFromCallback(VirtualTime time) {
        /*
         * If request goes into the past (w.r.t. currentTime), erase newer states to force recomputing them. This is
         */
        if(time < currentTime) {
          auto firstToDelete = buffer.upper_bound(time);
          if(firstToDelete == buffer.begin()) {
            std::stringstream s;
            s << "Value request is too far into the past: ";
            s << "requested time = " << time << ", oldest history = " << (buffer.begin()->first);
            throw StateVariableSetException(s.str(),StateVariableSetException::REQUEST_FAR_PAST);
          }
          buffer.erase(firstToDelete, buffer.end());
          currentTime = buffer.rbegin()->first;
        }

        /*
         * Propagate the current time until the requested time in steps of maxGap.
         */
        // First compute the number of steps
        // Note: this is rounding-up the integer division (time-CurrentTime)/maxGap
        unsigned int nSteps = (time-currentTime-1)/maxGap+1;

        // Loop over the time in nSteps steps. The first step is potentially smaller, all consequtive steps are
        // maxGap big.
        for(VirtualTime t = time-(nSteps-1)*maxGap; t <= time; t += maxGap) {

          // obtain the new state
          STATE state = compute(t);

          // save value into buffer and update the current time
          buffer[t] = state;
          currentTime = t;

        }

        // clear old values from history
        if( buffer.size() > 1 && buffer.begin()->first < (time - historyLength) ) {
          auto firstToKeep = buffer.upper_bound(time - historyLength - 1);
          buffer.erase(buffer.begin(), firstToKeep);
        }

        // return the new state (do not use buffer[], as it will silently insert new objects if something went wrong)
        return buffer.find(time)->second;
      }

      /// callback called when a state needs to be computed
      boost::function<const STATE(VirtualTime)> compute;

      /// buffer of values (in dependence of time)
      std::map<VirtualTime,STATE> buffer;

      /// maximum gap
      VirtualTime maxGap;

      /// time tolerance
      VirtualTime timeTolerance;

      /// history length
      VirtualTime historyLength;

      /// time of last fed sample
      VirtualTime currentTime;

  };

}}  // namespace mtca4u::VirtualLab


#endif /* STATEVARIABLESET_H */
