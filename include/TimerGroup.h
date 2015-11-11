/*
 * TimerGroup.h
 *
 *  Created on: Sep 2, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef TIMERGROUP_H
#define TIMERGROUP_H

#include <boost/fusion/container.hpp>
#include <boost/fusion/sequence.hpp>
#include <boost/fusion/algorithm.hpp>

#include "VirtualTime.h"

namespace mtca4u { namespace VirtualLab {

///
/// Helper macro: DO NOT DIRECTLY USE
///
/// Count number of macro arguments in variadic macros.
/// Works up to 20 arguments (extend following the obvious pattern if more is needed)
///
#define COUNT_ARGS_HELPER(x00,x01,x02,x03,x04,x05,x06,x07,x08,x09,x10,x11,x12,x13,x14,x15,x6,x17,x18,x19,x20,...) x20
#define COUNT_ARGS(...) COUNT_ARGS_HELPER( __VA_ARGS__ , 20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1)

///
/// Helper macros: DO NOT DIRECTLY USE
///
/// DECLARE_TIMER_GROUP_MAKEMAP* macros are used to initialise the name vector.
/// It essentially surrounds all given arguments by:
///   names.push_back("__ARGUMENT__");
/// Thus it builds a list of pairs suitable to initialise the vector.
/// These macros work up to 10 arguments (extend following the obvious pattern if more is needed).
///
#define DECLARE_TIMER_GROUP_MAKEMAP(...)  DECLARE_TIMER_GROUP_MAKEMAP_ (COUNT_ARGS ( __VA_ARGS__ ), __VA_ARGS__ )
#define DECLARE_TIMER_GROUP_MAKEMAP_(n,...)  DECLARE_TIMER_GROUP_MAKEMAP__ (n, __VA_ARGS__ )
#define DECLARE_TIMER_GROUP_MAKEMAP__(n,...)  DECLARE_TIMER_GROUP_MAKEMAP_ ## n (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_1(x,...)  names.push_back(#x);
#define DECLARE_TIMER_GROUP_MAKEMAP_2(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_1 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_3(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_2 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_4(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_3 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_5(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_4 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_6(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_5 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_7(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_6 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_8(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_7 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_9(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_8 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_MAKEMAP_10(x,...)  names.push_back(#x); DECLARE_TIMER_GROUP_MAKEMAP_9 (__VA_ARGS__)

///
/// Helper macros: DO NOT DIRECTLY USE
///
/// DECLARE_TIMER_GROUP_MAKETUPLEA* macros are used to declare the fusion::vector holding the timers.
/// It essentially surrounds all given arguments by:
///   decltype(__ARGUMENT__)&
/// Thus it builds a list of types of all timers suitable to declare the vector.
/// These macros work up to 10 arguments (extend following the obvious pattern if more is needed).
///
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR(...)  DECLARE_TIMER_GROUP_DECLARE_VECTOR_ (COUNT_ARGS ( __VA_ARGS__ ), __VA_ARGS__ )
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_(n,...)  DECLARE_TIMER_GROUP_DECLARE_VECTOR__ (n, __VA_ARGS__ )
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR__(n,...)  DECLARE_TIMER_GROUP_DECLARE_VECTOR_ ## n (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_1(x,...)  decltype(x)&
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_2(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_1 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_3(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_2 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_4(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_3 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_5(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_4 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_6(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_5 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_7(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_6 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_8(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_7 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_9(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_8 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_DECLARE_VECTOR_10(x,...)  decltype(x)&, DECLARE_TIMER_GROUP_DECLARE_VECTOR_9 (__VA_ARGS__)

///
/// Helper macros: DO NOT DIRECTLY USE
///
/// DECLARE_TIMER_GROUP_INIT_VECTOR* macros are used to initialise the fusion::vector with the timer instances
/// It essentially surrounds all given arguments by:
///   dev->__ARGUMENT__
/// Thus it builds a list timer instances in the device suitable to initialise the vector.
/// These macros work up to 10 arguments (extend following the obvious pattern if more is needed).
///
#define DECLARE_TIMER_GROUP_INIT_VECTOR(...)  DECLARE_TIMER_GROUP_INIT_VECTOR_ (COUNT_ARGS ( __VA_ARGS__ ), __VA_ARGS__ )
#define DECLARE_TIMER_GROUP_INIT_VECTOR_(n,...)  DECLARE_TIMER_GROUP_INIT_VECTOR__ (n, __VA_ARGS__ )
#define DECLARE_TIMER_GROUP_INIT_VECTOR__(n,...)  DECLARE_TIMER_GROUP_INIT_VECTOR_ ## n (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_1(x,...)  dev->x
#define DECLARE_TIMER_GROUP_INIT_VECTOR_2(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_1 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_3(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_2 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_4(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_3 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_5(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_4 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_6(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_5 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_7(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_6 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_8(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_7 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_9(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_8 (__VA_ARGS__)
#define DECLARE_TIMER_GROUP_INIT_VECTOR_10(x,...) dev->x, DECLARE_TIMER_GROUP_INIT_VECTOR_9 (__VA_ARGS__)

/**
 * Timer group base class.
 * The template agument must be a boost::fusion::vector of the timer types.
 * The implementation must initialise the timerTypes vector "timers" with all timers and the map names with the
 * timer names and the corresponding index in the timerTypes vector.
 *
 * This implementation is done for VirtualDevices using the DECLARE_TIMER_GROUP macro.
 */
template<class timerTypes>
class TimerGroup {
  public:
    TimerGroup() : findRemainingByName_index(0),timers(NULL),current(0) {}
    ~TimerGroup() {}

    /// Get remaining time until the next timer fires, or -1 if no timer has been set.
    VirtualTime getRemaining() {
      VirtualTime remaining = std::numeric_limits<VirtualTime>::max();
      boost::fusion::for_each(*timers, findRemaining(remaining));
      if(remaining >= std::numeric_limits<VirtualTime>::max()) return -1;
      return remaining;
    }

    /// Get remaining time until the specified timer fires, or -1 if the timer has not been set.
    /// Note: this call will be relatively expensive due to the string argument. If possible (e.g. inside a
    /// VirtualDevice implementation), access the subtimer directly.
    VirtualTime getRemaining(std::string name) {
      VirtualTime remaining = -2;
      boost::fusion::for_each(*timers, findRemainingByName(*this,name,remaining));
      return remaining;
    }

    /// Advance the timer's current time by tval. Returns true if any timer was fired.
    /// If tval < 0, this function does nothing. If tval > getRemaining(), the timer is advanced in multiple steps to
    /// make sure repetitive events are fired.
    bool advance(VirtualTime tval) {
      if(tval < 0) return false;
      current += tval;
      bool hasFired = false;
      do {
        VirtualTime tstep = fmin(tval, getRemaining());
        boost::fusion::for_each(*timers, advanceTimer(tstep,hasFired));
        tval -= tstep;
      } while(tval > 0);
      return hasFired;
    }

    /// Advance the timer to the next requested time. Returns true if any timer was fired. Otherwise none of the
    /// timers was set and thus the current time remains unchanged
    bool advance() {
      return advance(getRemaining());
    }

    /// Advance the group to the next requested time of the given sub-timer. Returns true if any timer was fired.
    /// Note: this call will be relatively expensive due to the string argument. If possible (e.g. inside a
    /// VirtualDevice implementation), access the subtimer directly, obtain its remaining time and call advance(VirtualTime).
    bool advance(std::string name) {
      return advance(getRemaining(name));
    }

    /// Get current time
    VirtualTime getCurrent()
    {
      return current;
    }

  protected:

    /// functor to find the smallest remaining time
    struct findRemaining {
        findRemaining(VirtualTime &_minRemaining) : minRemaining(_minRemaining) {}
        template<class T>
        void operator()(T& t) const {
            VirtualTime tval = t.getRemaining();
          if(tval >= 0 && tval < minRemaining) minRemaining = tval;
        }
      private:
        VirtualTime &minRemaining;
    };

    /// functor to find the remaining time of a given timer
    int findRemainingByName_index;
    struct findRemainingByName {
        findRemainingByName(TimerGroup<timerTypes> &_group, std::string &_name, VirtualTime &_remaining)
        : group(_group),
          name(_name),
          remaining(_remaining)
        {
          group.findRemainingByName_index = 0;
        }
        template<class T>
        void operator()(T& t) const {
          if(group.names[group.findRemainingByName_index++] != name) return;
          remaining = t.getRemaining();
        }
      private:
        TimerGroup<timerTypes> &group;
        std::string &name;
        VirtualTime &remaining;
    };

    /// functor to advance timers by the given time
    struct advanceTimer {
        advanceTimer(VirtualTime _tval, bool &_hasFired) : tval(_tval), hasFired(_hasFired) {}
        template<class T>
        void operator()(T& t) const {
          bool r;
          r = t.advance(tval);
          if(r) hasFired = true;
        }
      private:
        VirtualTime tval;
        bool &hasFired;
    };

    /// fusion::vector of timer instances
    timerTypes *timers;

    /// std::vector of timer names
    std::vector<std::string> names;

    /// current time
    VirtualTime current;

};


}} // namespace mtca4u::VirtualLab


#endif /* TIMERGROUP_H */
