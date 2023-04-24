/*
 * StateVariableSet.h
 *
 *  Created on: Dec 10, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef STATEVARIABLESET_H
#define STATEVARIABLESET_H

#include <ChimeraTK/Exception.h>
#include <boost/bind/bind.hpp>
#include <boost/function.hpp>
#include <limits>
#include <map>
#include <sstream>

#include "VirtualTime.h"

namespace ChimeraTK { namespace VirtualLab {

  /** A set of state variables contains a struct of time-dependent variables. In
   * this class a mechanism is provided which helps keeping track of these
   * variables properly and computing values as needed.
   *
   * An example use-case for this class is to perform an integration (or sum) as
   * it is needed for an integral controller. Since the VirtualLab framework might
   * request values from signal sources and sinks in an arbitrary order, it is not
   * possible to store them as normal member variables of the VirtualLabBackend or
   * the model class.
   */
  template<class STATE>
  class StateVariableSet {
   public:
    StateVariableSet()
    : maxGap(std::numeric_limits<VirtualTime>::max() / 2), hugeGap(std::numeric_limits<VirtualTime>::max() / 2),
      validityPeriod(1), historyLength(0), currentTime(std::numeric_limits<VirtualTime>::min()),
      enableInterpolation(false) {
#ifdef ENABLE_EXPERIMENTAL_FEATURES
      using boost::placeholders;
      interpolate = boost::bind(&StateVariableSet::defaultInterpolate, this, _1, _2, _3, _4, _5);
#endif
    }

    /** Set the initial state for time 0.
     *
     *  It is mandatory to set the initial state before the first call to
     * getState().
     */
    void setInitialState(const STATE& state) { feedState(0, state); }

    /** Set callback function which will compute a new state for a given time.
     *
     *  Guarantee: When this function is called for a time T, the latest state
     * requested by getLatestState() will be in the past w.r.t. T and not older
     * than the gap configured with setMaximumGap().
     *
     *  It is mandatory to set this function.
     */
    void setComputeFunction(const boost::function<const STATE(VirtualTime)>& callback) { compute = callback; }

    /** Obtain the callback function previously set with setComputeFunction().
     */
    const boost::function<const STATE(VirtualTime)>& getComputeFunction() { return compute; }

    /** Set maximum time gap. If a state further into the future of the latest
     * computed state than the maximum gap time is requested, intermediate states
     * will be computed so that no states are further apart than the gap.
     *
     *  If the gap is not set, intermediate states are never computed. The maximum
     * gap must be larger than the validity period to have an effect.
     */
    void setMaximumGap(VirtualTime time) { maxGap = time; }

    /** Set - in addition to the maximum gap - an even bigger gap, which will be
     * used to fill larger steps. If a state is requested into the far future
     * (more than 2*hugeGap), intermediate steps with a distance of hugeGap
     *  (instead of maxGap) will be inserted to fill the gap. Only the last
     * hugeGap interval before the requested state will be filled with
     * intermediate steps in a distance of maxGap.
     *
     *  The hugeGap must be larger than maxGap. If not set, the feature is
     * disabled and only maxGap is used to fill large steps.
     */
    void setHugeGap(VirtualTime time) { hugeGap = time; }

    /** Experimental feature: enable by defining ENABLE_EXPERIMENTAL_FEATURES
     *  Enable interpolation. No states will be fully computed by the model closer
     * together than the maximum gap time. Instead requests for these times will
     * be interpolated. Note that this usually will lead to requests to other
     * model components being generated into the "future" (from the current
     * request), since values are needed on both sides for an interpolation (in
     * contrast to an extrapolation, which might be invalid). */
    void setEnableInterpolation(bool enable) {
#ifndef ENABLE_EXPERIMENTAL_FEATURES
      class ExperimentalFeatureNotEnabled {};
      throw ExperimentalFeatureNotEnabled();
#endif
      enableInterpolation = enable;
    }

    /** Set the function which interpolates between two states. The passed
     * function has must accept the following arguments (in order):
     *  - the first support state to base the interpolation on
     *  - the second support state
     *  - the time of the first support state
     *  - the time of the second support state
     *  - the requested time to return the interpolated state for */
    void setInterpolateFunction(
        const boost::function<STATE(const STATE&, const STATE&, VirtualTime, VirtualTime, VirtualTime)>& callback) {
#ifndef ENABLE_EXPERIMENTAL_FEATURES
      class ExperimentalFeatureNotEnabled {};
      throw ExperimentalFeatureNotEnabled();
#endif
      interpolate = callback;
    }

    /** Set maximum time difference a getValue() request may go into the past.
     *
     *  If the history length is not set, no history is kept and only the latest
     * state is retained. The history length must be larger then the validity
     * period to have an effect and should normally be larger then the maximum
     * gap.
     */
    void setMaxHistoryLength(VirtualTime timeDifference) { historyLength = timeDifference; }

    /** Set validity period. A state for the time T will be used when a state for
     * a time < T+validityPeriod (and > T) is requested. It is not possible to
     * have two states closer together in time than this period.
     *
     *  Setting the validity period is optional, it will default to 1 (i.e. no
     * effect). The period must be > 0.
     */
    void setValidityPeriod(VirtualTime period) { validityPeriod = period; }

    /** Obtain the state for the given time. time must be >= 0.
     *  The optional second argument forceNoInterpolation allows to disable a
     * potentially enabled interpolation and thus force a full recomputation of
     * the state. This is mainly used inside the interpolation code to make sure
     *  there are no recursion problems, but it could be helpful in other contexts
     * as well. */
    inline const STATE& getState(VirtualTime time, bool forceNoInterpolation = false) {
#ifndef ENABLE_EXPERIMENTAL_FEATURES
      (void)forceNoInterpolation; // avoid warning
#endif
      // check if time is the current time and return the latest element
      if(time >= currentTime && time < currentTime + validityPeriod) {
        return getLatestState();
      }
      // search in buffer: find the first element after the requested time
      auto it = buffer.upper_bound(time);
      if(it == buffer.begin()) {
        std::stringstream s;
        s << "Value request is too far into the past: ";
        s << "requested time = " << time << ", oldest history = " << buffer.begin()->first;
        s << ", current time = " << currentTime;
        throw ChimeraTK::logic_error(s.str());
      }
      // if this is end(), a new value needs to be computed
      if(it == buffer.end()) {
#ifdef ENABLE_EXPERIMENTAL_FEATURES
        if(!forceNoInterpolation) {
          return getValueInterpolated(time, buffer.rbegin()->first);
        }
        else {
#endif
          return getValueFromCallback(time);
#ifdef ENABLE_EXPERIMENTAL_FEATURES
        }
#endif
      }
      // decrement to get the most recent sample before the requested time
      --it;
      // if sample is too old, request one via callback
      if(time >= it->first + validityPeriod) {
#ifdef ENABLE_EXPERIMENTAL_FEATURES
        if(!forceNoInterpolation) {
          return getValueInterpolated(time, it->first);
        }
        else {
#endif
          return getValueFromCallback(time);
#ifdef ENABLE_EXPERIMENTAL_FEATURES
        }
#endif
      }
      // return value from buffer
      return it->second;
    }

    /** Obtain the latest computed state.
     *  This function will usually be called inside the compute function (see
     * setComputeFunction()) to have a basis for the computations. Use
     * getLatestTime() to obtain the time of the latest state.
     *
     *  Guarantee: When this function is called inside the compute function, the
     * returned state will be in the past and not older than the gap configured
     * with setMaximumGap().
     */
    inline const STATE& getLatestState() {
      assert(buffer.size() > 0);
      return buffer.rbegin()->second;
    }

    /** Obtain the time of latest computed state. See getLatestState() for further
     * comments.
     */
    inline VirtualTime getLatestTime() { return currentTime; }

    /** Obtain the current map of states.
     */
    inline const std::map<VirtualTime, STATE>& getAllStates() { return buffer; }

    /** Feed a state to the buffer. This can be used to provide values/states
     * outside the compute callback function. e.g. if the values are computed in a
     * different context and should be stored for potential later use.
     *
     *  If time is < currentTime, newer entries will be removed. After inserting
     * the new entry, the old history will be removed as well.
     *
     *  Note: No check is performed if the gap to the previous state is larger
     * than the maximum gap!
     */
    inline void feedState(VirtualTime time, STATE state) {
      truncateFuture(time);
      buffer[time] = state;
      currentTime = time;
      truncatePast();
    }

   protected:
    /** Truncate a future part of the buffer: remove all entries newer then the
     * given time. Has no effect, if time >= currentTime/
     */
    inline void truncateFuture(VirtualTime time) {
      if(time < currentTime) {
        auto firstToDelete = buffer.upper_bound(time);
        // Note: getState() already ensures that the request does not go before
        // the first map element, so we are never deleting the entire map here.
        buffer.erase(firstToDelete, buffer.end());
        currentTime = buffer.rbegin()->first;
      }
    }

    /** Truncate old history of the buffer: remove all entries older than
     * currentTime - historyLength.
     */
    inline void truncatePast() {
      // clear old values from history
      if(buffer.size() > 1 && buffer.begin()->first < (currentTime - historyLength)) {
        auto firstToKeep = buffer.upper_bound(currentTime - historyLength - 1);
        buffer.erase(buffer.begin(), firstToKeep);
      }
    }

    /// obtain a new state via interpolation, if enabled, or via callback
    /// otherwise
    inline const STATE& getValueInterpolated(VirtualTime timeRequested, VirtualTime previousStep) {
#ifndef ENABLE_EXPERIMENTAL_FEATURES
      class ExperimentalFeatureNotEnabled {};
      throw ExperimentalFeatureNotEnabled();
#endif
      // no interpolation or requested time is later than the interpolation period
      // (maxGap)
      if(!enableInterpolation || timeRequested >= previousStep + maxGap) {
        return getValueFromCallback(timeRequested);
      }

      // save interpolated state into the buffer and return it
      auto state1 = getState(previousStep, true);
      auto state2 = getState(previousStep + maxGap, true);
      buffer[timeRequested] = interpolate(state1, state2, previousStep, previousStep + maxGap, timeRequested);
      return buffer[timeRequested];
    }

    /// obtain a new state via the callback function and place it into the buffer.
    /// Helper for getValue()
    inline const STATE& getValueFromCallback(VirtualTime time) {
      // If request goes into the past (w.r.t. currentTime), erase newer states to
      // force recomputing them.
      truncateFuture(time);

      // Compute the number of steps necessary to reach the requested time,
      // obaying hugeGap. The last step at the requested time itself is not
      // computed here, since maxGap shall be obeyed in the last gap (see below).
      // Note: this is effectively rounding-up the integer division
      // (time-CurrentTime)/hugeGap
      unsigned int nHugeSteps = (time - currentTime - 1) / hugeGap + 1;

      // Loop over the time in nHugeSteps steps. The first step is potentially
      // smaller, all consequtive steps are maxGap big. Omit the last step, so the
      // requested time is not yet reached.
      for(unsigned int i = 0; i < nHugeSteps - 1; i++) {
        VirtualTime t = time - (nHugeSteps - 1) * hugeGap + i * hugeGap;

        // obtain the new state
        STATE state = compute(t);

        // save value into buffer and update the current time
        buffer[t] = state;
        currentTime = t;
      }

      // Compute the number of steps necessary to reach the requested time,
      // obaying maxGap. Note: this is effectively rounding-up the integer
      // division (time-CurrentTime)/maxGap
      unsigned int nSteps = (time - currentTime - 1) / maxGap + 1;

      // Loop over the time in nSteps steps. The first step is potentially
      // smaller, all consequtive steps are maxGap big.
      for(unsigned int i = 0; i < nSteps; i++) {
        VirtualTime t = time - (nSteps - 1) * maxGap + i * maxGap;

        // obtain the new state
        STATE state = compute(t);

        // save value into buffer and update the current time
        buffer[t] = state;
        currentTime = t;
      }

      // remove old history
      truncatePast();

      // return the new state (do not use buffer[], as it will silently insert new
      // objects if something went wrong)
      return buffer.find(time)->second;
    }

    /// callback called when a state needs to be computed
    boost::function<const STATE(VirtualTime)> compute;

    /// buffer of values (in dependence of time)
    std::map<VirtualTime, STATE> buffer;

    /// maximum gap and huge gap
    VirtualTime maxGap, hugeGap;

    /// validity period
    VirtualTime validityPeriod;

    /// history length
    VirtualTime historyLength;

    /// time of last fed sample
    VirtualTime currentTime;

    /// enable or disable interpolation
    bool enableInterpolation;

    /// function called to interpolate between two states
    boost::function<STATE(const STATE&, const STATE&, VirtualTime, VirtualTime, VirtualTime)> interpolate;

    /// default interpolate function to throw an exception
    STATE defaultInterpolate(const STATE&, const STATE&, VirtualTime, VirtualTime, VirtualTime) {
#ifndef ENABLE_EXPERIMENTAL_FEATURES
      class ExperimentalFeatureNotEnabled {};
      throw ExperimentalFeatureNotEnabled();
#endif
      std::cout << "Interpolation enabled but no interpolate function set." << std::endl;
      throw ChimeraTK::logic_error("Interpolation enabled but no interpolate function set.");
    }
  };

}} // namespace ChimeraTK::VirtualLab

// Compatibility
namespace mtca4u { namespace VirtualLab {
  template<class STATE>
  class StateVariableSet : public ChimeraTK::VirtualLab::StateVariableSet<STATE> {
    using ChimeraTK::VirtualLab::StateVariableSet<STATE>::StateVariableSet;
  };
}} // namespace mtca4u::VirtualLab

#endif /* STATEVARIABLESET_H */
