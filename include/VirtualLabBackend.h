/*
 * VirtualDevice.h - base class for MTCA.4u dummy devices that fit into the physics simulation framework
 *
 *  Created on: Aug 27, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef VIRTUALDEVICE_H
#define VIRTUALDEVICE_H

#include <iostream>
#include <limits>
#include <string>
#include <map>
#include <mutex>

#include <boost/shared_ptr.hpp>

#include <boost/fusion/container/set.hpp>
#include <boost/fusion/algorithm.hpp>
#include <boost/fusion/include/at_key.hpp>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/euml/euml.hpp>

#include <mtca4u/DummyBackend.h>
#include <mtca4u/DummyRegisterAccessor.h>
#include <mtca4u/BackendFactory.h>
#include <mtca4u/DeviceAccessVersion.h>

#include "TimerGroup.h"

using namespace boost::msm::front::euml;        // this is required when using boost::msm
namespace msm = boost::msm;
namespace mpl = boost::mpl;

/** \def DECLARE_EVENT(name)
 * Declare an event. Use instead of BOOST_MSM_EUML_EVENT, as we must not create instances for the events as well.
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_EVENT(name)                                                                                     \
  class name : public msm::front::euml::euml_event< name > {}

/** \def DECLARE_STATE(name)
 * Declare a plain state, without any entry or exit functions etc.
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_STATE(name)                                                                                     \
  class name : public msm::front::state<> , public msm::front::euml::euml_state<name> {}

/** \def DECLARE_LOGGING_STATE(name)
 * Declare a logging state. Entry and exit of this state will be looged to std::cout.
 *
 * (test coverage hint: this macro is used in the test, but output is not actually tested) */
#define DECLARE_LOGGING_STATE(name)                                                                             \
  class name : public msm::front::state<> , public msm::front::euml::euml_state<name> {                         \
    public:                                                                                                     \
      template <class Event,class FSM>                                                                          \
      void on_entry(Event const&,FSM&) { std::cout << "Entering state: " << #name << std::endl; }               \
      template <class Event,class FSM>                                                                          \
      void on_exit(Event const&,FSM&) { std::cout << "Leaving state: " << #name << std::endl; }                 \
  }

/** \def DECLARE_REGISTER(UserType, name)
 * Declare a dummy register accessor for single-word or 1D-array registers.
 * UserType is the data type the data should be accessed by. The conversion is handled internally using the
 * FixedPointConverter.
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_REGISTER(UserType, name) mtca4u::DummyRegisterAccessor<UserType> name

/** \def DECLARE_MUXED_REGISTER(UserType, name)
 * Declare a dummy register accessor for multiplexed 2D-array registers.
 * UserType is the data type the data should be accessed by. The conversion is handled internally using the
 * FixedPointConverter.
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_MUXED_REGISTER(UserType, name) mtca4u::DummyMultiplexedRegisterAccessor<UserType> name

/** \def WRITEEVENT_TABLE
 * Provide a "table" of events and register names using the CONNECT_REGISTER_EVENT macro for write events.
 * The table must be terminated with END_WRITEEVENT_TABLE.
 *
 * (test coverage hint: these two macros are used in the test) */
#define WRITEEVENT_TABLE                                                                                        \
  void regWriteEvents(uint8_t bar, uint32_t address, size_t sizeInBytes) {                                      \
    (void)bar; (void)address; (void)sizeInBytes;

#define END_WRITEEVENT_TABLE }

/** \def READEVENT_TABLE
 * Provide a "table" of events and register names using the CONNECT_REGISTER_EVENT macro for read events
 * The table must be terminated with END_READEVENT_TABLE.
 *
 * (test coverage hint: these two macros are used in the test) */
#define READEVENT_TABLE                                                                                         \
  void regReadEvents(uint8_t bar, uint32_t address, size_t sizeInBytes) {                                       \
    (void)bar; (void)address; (void)sizeInBytes;

#define END_READEVENT_TABLE }

/** \def CONNECT_REGISTER_EVENT(eventName, regsterAccessor)
 * Connect events with registers. The first argument eventName is the name of an event previously declared using
 * BOOST_MSM_EUML_EVENT. The second argument is a DummyRegisterAccessor for the register to connect with. Use this
 * macro in the of the WRITE_EVENT_TABLE and READ_EVENT_TABLE macros.
 * Do not separate multiple calls to CONNECT_REGISTER_EVENT inside the same WRITE_EVENT_TABLE/READ_EVENT_TABLE macro
 * by any separator (i.e. no comma or semicolon in between - starting a new line is allowed, though!).
 *
 * (test coverage hint: this macro is used in the test) */
#define CONNECT_REGISTER_EVENT(eventName, regsterAccessor)                                                      \
  {                                                                                                             \
    if(regsterAccessor.isAddressInRange(bar,address,sizeInBytes)) {                                             \
      try {                                                                                                     \
        theStateMachine.process_event(eventName ());                                                            \
      }                                                                                                         \
      catch(...) {                                                                                              \
        std::cerr << "ERROR in VirtualLabBackend: Exception thrown while processing register event. The state " \
                  << "machine is now in an unusable condition. Make sure to catch all exceptions in your "      \
                  << "actions!" << std::endl;                                                                   \
      }                                                                                                         \
    }                                                                                                           \
  }

/** \def DECLARE_REGISTER_GUARD(guardName, condition)
 * Declare a guard condition on the value of a register. The resulting guard condition can be used on an event
 * defined with the CONNECT_REGISTER_EVENT macro. The first argument guardName is the name of the resulting guard
 * class and the second argument will be parsed as the guard condition (standard C++ syntax). Inside the condition,
 * the variable "value" can be used which contains the value of the register written in the event (type: int32_t,
 * always the first word written).
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_REGISTER_GUARD(guardName, condition)                                                            \
    BOOST_MSM_EUML_ACTION(guardName ## _) {                                                                     \
        template <class Fsm,class Evt,class SourceState,class TargetState>                                      \
        bool operator()(Evt const& ,Fsm& fsm,SourceState&,TargetState& )                                        \
        {                                                                                                       \
            return fsm.dev->guardName ## _funct();                                                              \
        }                                                                                                       \
    };                                                                                                          \
    typedef BOOST_TYPEOF( guardName ## _ ) guardName;                                                           \
    bool guardName ## _funct() {                                                                                \
      int32_t value = *lastWrittenData;                                                                         \
      (void)value;                                                                                              \
      return ( condition );                                                                                     \
    }

/** \def DECLARE_GUARD(guardName)
 * Declare a guard condition. The argument guardName is the name of the resulting guard class.
 * The code must follow this macro, contain a return statement returning a boolean and has to be terminated
 * with END_DECLARE_GUARD.
 *
 * (test coverage hint: these two macros are used in the test) */
#define DECLARE_GUARD(guardName)                                                                                \
    BOOST_MSM_EUML_ACTION(guardName ## _) {                                                                     \
        template <class Fsm,class Evt,class SourceState,class TargetState>                                      \
        bool operator()(Evt const& ,Fsm& fsm,SourceState&,TargetState& )                                        \
        {                                                                                                       \
            return fsm.dev->guardName ## _funct();                                                              \
        }                                                                                                       \
    };                                                                                                          \
    typedef BOOST_TYPEOF( guardName ## _ ) guardName;                                                           \
    bool guardName ## _funct() {

#define END_DECLARE_GUARD }

/** \def DECLARE_ACTION(actionName)
 * Declare an action with arbitrary code. The argument actionName is the name of the resulting action class.
 * The code must follow this macro and has to be terminated with END_DECLARE_ACTION.
 *
 * (test coverage hint: these two macros are used in the test) */
#define DECLARE_ACTION(actionName)                                                                              \
    BOOST_MSM_EUML_ACTION(actionName ## _) {                                                                    \
        template <class Fsm,class Evt,class SourceState,class TargetState>                                      \
        void operator()(Evt const& ,Fsm& fsm,SourceState&,TargetState& )                                        \
        {                                                                                                       \
            fsm.dev->actionName ## _funct();                                                                    \
        }                                                                                                       \
    };                                                                                                          \
    typedef BOOST_TYPEOF( actionName ## _ ) actionName;                                                         \
    void actionName ## _funct() {

#define END_DECLARE_ACTION }

/** \def DECLARE_TIMER(name,event)
 * Declare a timer with a given name. Will fire the given event.
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_TIMER(name,event) Timer<event> name

/** \def DECLARE_TIMER_GROUP(name, ...)
 * Declare a timer group with a given name. The additional arguments must be the timers part of
 * this group, previously declared using DECLARE_TIMER.
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_TIMER_GROUP(name, ...)                                                                          \
    typedef boost::fusion::vector< DECLARE_TIMER_GROUP_DECLARE_VECTOR(__VA_ARGS__) > name ## __;                \
    class name ## _: public TimerGroup<name ## __> {                                                            \
      public:                                                                                                   \
        name ## _(dummyDeviceType *_dev)                                                                        \
        : TimerGroup(),                                                                                         \
          dev(_dev)                                                                                             \
        {                                                                                                       \
          DECLARE_TIMER_GROUP_MAKEMAP(__VA_ARGS__)                                                              \
          timers = new name ## __( DECLARE_TIMER_GROUP_INIT_VECTOR(__VA_ARGS__) );                              \
        }                                                                                                       \
      protected:                                                                                                \
        dummyDeviceType *dev;                                                                                   \
    };                                                                                                          \
    name ## _ name

/** \def DECLARE_STATE_MACHINE(stateMachineName, initialState, transitionTable)
 * Declare a state machine (usually a sub-state machine, also see DECLARE_MAIN_STATE_MACHINE)
 * dummyDeviceType is the class name of the physDummyDevice implementation the state machine will be used in
 * stateMachineName is the name of the state machine itself. Several types will be defined based on this names with
 * one or more trailing underscores.
 * initialState is the name of the initial state. Multiple states can be added by separating them with "<<".
 * transitionTable is the transition table in eUML syntax.
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_STATE_MACHINE(stateMachineName, initialState, transitionTable)                                  \
    BOOST_MSM_EUML_TRANSITION_TABLE((                                                                           \
        transitionTable                                                                                         \
    ),stateMachineName ## _table)                                                                               \
    BOOST_MSM_EUML_DECLARE_STATE_MACHINE(( stateMachineName ## _table,                                          \
                                           init_ << initialState ),                                             \
                                           stateMachineName ## __)                                              \
    class stateMachineName ## _ : public stateMachineName ## __  {                                              \
      public:                                                                                                   \
        stateMachineName ## _()                                                                                 \
          : stateMachineName ## __()                                                                            \
        {}                                                                                                      \
        void setDummyDevice(dummyDeviceType *_dev) {dev = _dev;}                                                \
        dummyDeviceType *dev;                                                                                   \
    };                                                                                                          \
    typedef msm::back::state_machine<stateMachineName ## _> stateMachineName

/** \def DECLARE_MAIN_STATE_MACHINE(initialState, transitionTable)
 * Declare the main state machine. This is just like DECLARE_STATE_MACHINE but with a fixed name "mainStateMachine".
 * Also the state machine will be instantiated under the name "theStateMachine", as expected by other parts of this
 * framework.
 *
 * (test coverage hint: this macro is used in the test) */
#define DECLARE_MAIN_STATE_MACHINE(initialState, transitionTable)                                               \
    BOOST_MSM_EUML_TRANSITION_TABLE((                                                                           \
        transitionTable                                                                                         \
    ),mainStateMachine_table)                                                                                   \
    BOOST_MSM_EUML_DECLARE_STATE_MACHINE(( mainStateMachine_table,                                              \
                                           init_ << initialState ),                                             \
                                           mainStateMachine__)                                                  \
    class mainStateMachine_ : public mainStateMachine__  {                                                      \
      public:                                                                                                   \
        mainStateMachine_(dummyDeviceType *_dev)                                                                \
          : mainStateMachine__(),                                                                               \
            dev(_dev)                                                                                           \
        {}                                                                                                      \
        dummyDeviceType *dev;                                                                                   \
    };                                                                                                          \
    typedef msm::back::state_machine<mainStateMachine_> mainStateMachine;                                       \
    mainStateMachine theStateMachine

/** \def CONSTRUCTOR(name,...)
 * Declare the constructor of the VirtualLabBackend. The first argument must be the class name. The other arguments
 * must be the list of member initialisers, the code of the cunstructor follows this macro and must be terminated
 * with END_CONSTRUCTOR, even if no code is put into the constructor.
 * This macro will also insert the appropriate createInstance function used in the BackendFactory. It will maintain
 * a map of all instances, so the same instance can be obtained multiple times when using the same instance name
 * in the SDM URI (like "instanceID" in "sdm://./myVirtualLabBackend:instanceID=mapfile.map").
 * To register the backend type with the BackendFactory, the macro REGISTER_BACKEND_TYPE must be used.
 *
 * Sometimes you need to force the linkter to link the object code that registers the backend in the factory (e.g.
 * if you do not explicitly use any other part of the library). In this case, just refer to
 * \verbatim<yourBackend>::backendRegisterer.dummy\endverbatim somewhere in your code (e.g. by setting it to 1).
 *
 * @attention The backends are not thread-safe. When using a VirtualLabBackend from a frontend Device concurrently in
 * multiple threads, a transparent locking decorator must be used. When using it concurrently from the backend-side
 * (e.g. through the backend register accessors, timers, sinks and sources etc.) you must ensure proper locking
 * yourself.
 *
 * (test coverage hint: these two macros are used in the test) */
#define CONSTRUCTOR(name,...)                                                                                   \
    /* createInstance() function used by the BackendFactory. Creates only one instance per instance name! */    \
    static boost::shared_ptr<DeviceBackend> createInstance(std::string, std::string instance,                   \
        std::list<std::string> parameters, std::string mapFileName="") {                                        \
      if(mapFileName == "") mapFileName = parameters.front(); /* compatibility, remove after deviceaccess 0.6 is out */   \
      if(mapFileName == "" || instance == "") {                                                                 \
        throw ChimeraTK::logic_error("No map file name or instance ID given in the map file.");                  \
      }                                                                                                         \
      /* search instance map and create new instance, if bot found under the name */                            \
      if(getInstanceMap().find(instance) == getInstanceMap().end()) {                                           \
        boost::shared_ptr<mtca4u::DeviceBackend> ptr( new name(mapFileName) );                                  \
        getInstanceMap().insert( std::make_pair(instance,ptr) );                                                \
        return ptr;                                                                                             \
      }                                                                                                         \
      /* return existing instance from the map */                                                               \
      return boost::shared_ptr<mtca4u::DeviceBackend>(getInstanceMap()[instance]);                              \
    }                                                                                                           \
    /* Static and global instance map (plain static members don't work header-only!) */                         \
    static std::map< std::string, boost::shared_ptr<mtca4u::DeviceBackend> >& getInstanceMap() {                \
      static std::map< std::string, boost::shared_ptr<mtca4u::DeviceBackend> > instanceMap;                     \
      return instanceMap;                                                                                       \
    }                                                                                                           \
    /* Class to register the backend type with the factory. */                                                  \
    class BackendRegisterer {                                                                                   \
      public:                                                                                                   \
        BackendRegisterer() : dummy(0) {                                                                        \
          std::cout << "VirtualLabBackend::BackendRegisterer: registering backend type " << #name << std::endl;	\
          mtca4u::BackendFactory::getInstance().registerBackendType(#name,"",&name::createInstance, CHIMERATK_DEVICEACCESS_VERSION); \
        }                                                                                                       \
        /* dummy variable we can reference to force linking the object code when just using the header */       \
        int dummy;                                                                                              \
    };                                                                                                          \
    static BackendRegisterer backendRegisterer;                                                                 \
    /* Actual constructor of the VirtualLabBackend class. */                                                    \
    name(std::string mapFileName) :                                                                             \
      VirtualLabBackend(mapFileName),                                                                           \
      ## __VA_ARGS__ ,                                                                                          \
      theStateMachine(this)                                                                                     \
      {

#define END_CONSTRUCTOR }

/** \def INIT_SUB_STATE_MACHINE(name)
 * Initialise a sub-state machine. This should be called inside of the constructor.
 *
 * (test coverage hint: this macro is used in the test) */
#define INIT_SUB_STATE_MACHINE(name)                                                                            \
    theStateMachine.get_state< name * >()->setDummyDevice(this)

/** \def REGISTER_BACKEND_TYPE(name)
 * Register backend type with the BackendFactory. Must be placed into the C++ source file for any VirtualLabBackend
 * even when not intending to use the backend factory (not into the header file)!
 *
 * (test coverage hint: this macro is used in the test) */
#define REGISTER_BACKEND_TYPE(name)                                                                             \
    name::BackendRegisterer name::backendRegisterer


namespace mtca4u { namespace VirtualLab {

/*********************************************************************************************************************/
/** Base class for VirtualLab dummy devices.
 *
 *  The VirtualLabBackend is an extension of the DummyBackend. Like its base class, it implements a set of registers
 *  defined in a map file in memory. VirtualLabBackend helps implementing the functionality, which will be provided
 *  by the firmware and hardware in case of a real device. The implementation of firmware functionality is realised
 *  using a state machine. Inputs and outputs of the simulated device can be added using the SignalSink and SignalSource
 *  classes, which can then be connected with other VirtualLab components in the virtual lab setup code.
 *
 *  Use the following macros to build your VirtualLabBackend:
 *  - \ref CONSTRUCTOR to create the constructor including parts needed for registration with the BackendFactory
 *  - \ref REGISTER_BACKEND_TYPE needs to be placed in the C++ file to register with the BackendFactory
 *  - \ref DECLARE_REGISTER and DECLARE_MUXED_REGISTER to obtain register accessors for registers
 *  - \ref DECLARE_MAIN_STATE_MACHINE to declare your main state machine with transition table
 *  - \ref DECLARE_STATE_MACHINE to declare any sub state machine
 *  - \ref DECLARE_STATE, \ref DECLARE_EVENT, \ref DECLARE_GUARD and \ref DECLARE_ACTION to build state machine
 *         components
 *  - \ref DECLARE_LOGGING_STATE to create states with debug output to the console
 *  - \ref DECLARE_REGISTER_GUARD to declare a guard condition using a register
 *  - \ref WRITEEVENT_TABLE, \ref READEVENT_TABLE and \ref CONNECT_REGISTER_EVENT to create events firing on reading
 *         or writing registers
 *  - \ref DECLARE_TIMER and \ref DECLARE_TIMER_GROUP to create timers and put them into a group
 *  - \ref INIT_SUB_STATE_MACHINE inside the \ref CONSTRUCTOR to initialise a sub state machine created with
 *         \ref DECLARE_STATE_MACHINE
 *
 *  Note: The bare minimum usually is one \ref DECLARE_STATE_MACHINE with one \ref DECLARE_STATE, plus one
 *  \ref DECLARE_TIMER_GROUP containing one timer.
 */
template<class derived>
class VirtualLabBackend : public DummyBackend
{
  public:

    // constructor via standard device model decription (as used by the DeviceFactory)
    VirtualLabBackend(std::string mapFileName) :
      DummyBackend(mapFileName),
      lastWrittenData(NULL),
      lastWrittenSize(0)
    {
      // start the main state machine
      static_cast<derived&>(*this).theStateMachine.start();
    }

    virtual ~VirtualLabBackend() {}

    /// override writeArea to fire the events
    void write(uint8_t bar, uint32_t address, int32_t const *data, size_t sizeInBytes) override {
      std::lock_guard<std::mutex> guard(deviceLock);

      // save as last written data, for use inside guards of the events we may trigger now
      lastWrittenData = data;
      lastWrittenSize = sizeInBytes;

      // perform the actual write
      DummyBackend::write(bar, address, data, sizeInBytes);

      // trigger events
      regWriteEvents(bar, address, sizeInBytes);
    }

    /// override readArea to fire the events
    void read(uint8_t bar, uint32_t address, int32_t *data, size_t sizeInBytes) override {
      std::lock_guard<std::mutex> guard(deviceLock);

      // trigger events
      regReadEvents(bar, address, sizeInBytes);

      // perform the actual write
      DummyBackend::read(bar, address, data, sizeInBytes);
    }

  protected:

    /// define the dummyDeviceType used in the macros
    typedef derived dummyDeviceType;

    /// trigger register-write events. Will be implemented using WRITE_EVENT_TABLE in the device implementation
    virtual void regWriteEvents(uint8_t /* bar */, uint32_t /* address */, size_t /* sizeInBytes */) {} //LCOV_EXCL_LINE

    /// trigger register-read events. Will be implemented using READ_EVENT_TABLE in the device implementation
    virtual void regReadEvents(uint8_t /* bar */, uint32_t /* address */, size_t /* sizeInBytes */) {}  //LCOV_EXCL_LINE

    /// VirtualDevice::Timer class
    template<class timerEvent>
    class Timer {
      public:
        Timer(derived *_dev) : dev(_dev),request(-1),current(0) {}

        /// Set the timer to fire in tval
        void set(VirtualTime tval) {
          request = tval+current;
        }

        /// Clear the timer (so it will not fire any more unless it is set again)
        void clear() {
          request = -1;
        }

        /// Advance the timer's current time by tval. Returns true if the timer was fired.
        /// If tval < 0 this function does nothing.
        /// Note: calling this function with tval > getRemaining() will fire the event only a single time!
        bool advance(VirtualTime tval) {
          if(tval < 0) return false;
          current += tval;
          if(request > 0 && current >= request) {
            request = -1;
            try {
              dev->theStateMachine.process_event( timerEvent() );
            }
            catch(...) {
              std::cerr << "ERROR in VirtualLabBackend: Exception thrown while processing timer event. The state "
                        << "machine is now in an unusable condition. Make sure to catch all exceptions in your "
                        << "actions!" << std::endl;
            }
            return true;
          }
          return false;
        }

        /// Obtain remaining time until the timer fires. Will be negative if the timer is not set.
        VirtualTime getRemaining() {
          if(request > 0) {
            return request-current;
          }
          else {
            return -1;
          }
        }

        /// Get current timer
        VirtualTime getCurrent() {
          return current;
        }

      protected:

        /// the dummy device with the state machine
        derived *dev;

        /// requested time
        VirtualTime request;

        /// current time
        VirtualTime current;
    };

    /// last written data (into any register) and its size. Will be used in guard conditions.
    int32_t const *lastWrittenData;
    size_t lastWrittenSize;

    /// mutex to prevent concurrent access to the device from different threads
    std::mutex deviceLock;

};


}}// namespace mtca4u::VirtualLab

#endif /* VIRTUALDEVICE_H */
