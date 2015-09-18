/*
 * VirtualDevice.h - base class for MTCA.4u dummy devices that fit into the physics simulation framework
 *
 *  Created on: Aug 27, 2015
 *      Author: martin.hierholzer@desy.de
 */

#ifndef VIRTUALDEVICE_H
#define VIRTUALDEVICE_H

#include <limits>
#include <string>

#include <boost/shared_ptr.hpp>

#include <boost/fusion/container/set.hpp>
#include <boost/fusion/algorithm.hpp>
#include <boost/fusion/include/at_key.hpp>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/euml/euml.hpp>

#include <MtcaMappedDevice/DummyBackend.h>
#include <MtcaMappedDevice/DummyRegisterAccessor.h>

#include "timer.h"

using namespace boost::msm::front::euml;        // this is required when using boost::msm
namespace msm = boost::msm;
namespace mpl = boost::mpl;

///
/// Declare an event. Use instead of BOOST_MSM_EUML_EVENT, as we must not create instances for the events as well.
///
#define DECLARE_EVENT(name)                                                                                     \
  class name : public msm::front::euml::euml_event< name > {};

///
/// Declare a plain state, without any entry or exit functions etc.
///
#define DECLARE_STATE(name)                                                                                     \
  class name : public msm::front::state<> , public msm::front::euml::euml_state<name> {};

/*
///
/// Declare a state with actions. The second arg decl can be used to define onEntry and onExit actions with the macros
/// STATE_ON_ENTRY and STATE_ON_EXIT.
///
#define DECLARE_ACTION_STATE(name,decl)                                                                         \
  class name : public msm::front::state<> , public msm::front::euml::euml_state<name> {                         \
    public:                                                                                                     \
      decl                                                                                                      \
  };

///
/// for use inside DECLARE_STATE(), see documentation there. Put the code to be executed on entry in braces after the
/// macro invocation.
///
#define STATE_ON_ENTRY                                                                                          \
   template <class Event,class FSM>                                                                             \
   void on_entry(Event const&,FSM&)

///
/// for use inside DECLARE_STATE(), see documentation there Put the code to be executed on exit in braces after the
/// macro invocation.
///
#define STATE_ON_EXIT                                                                                           \
   template <class Event,class FSM>                                                                             \
   void on_exit(Event const&,FSM&)
*/

///
/// Declare a logging state. Entry and exit of this state will be looged to std::cout.
///
#define DECLARE_LOGGING_STATE(name)                                                                             \
  class name : public msm::front::state<> , public msm::front::euml::euml_state<name> {                         \
    public:                                                                                                     \
      template <class Event,class FSM>                                                                          \
      void on_entry(Event const&,FSM&) { std::cout << "Entering state: " << #name << std::endl; }               \
      template <class Event,class FSM>                                                                          \
      void on_exit(Event const&,FSM&) { std::cout << "Leaving state: " << #name << std::endl; }                 \
  };

///
/// Declare a dummy register accessor for single-word or 1D-array registers.
/// UserType is the data type the data should be accessed by. The conversion is handled internally using the
/// FixedPointConverter.
///
#define DECLARE_REGISTER(UserType, name) mtca4u::DummyRegisterAccessor<UserType> name;

///
/// Declare a dummy register accessor for multiplexed 2D-array registers.
/// UserType is the data type the data should be accessed by. The conversion is handled internally using the
/// FixedPointConverter.
///
#define DECLARE_MUXED_REGISTER(UserType, name) mtca4u::DummyMultiplexedRegisterAccessor<UserType> name;

///
/// Provide a "table" of events and register names using the CONNECT_REGISTER_EVENT macro for write events.
/// The table must be terminated with END_WRITEEVENT_TABLE.
///
#define WRITEEVENT_TABLE                                                                                        \
  void regWriteEvents(uint8_t bar, uint32_t address, int32_t const *data, size_t sizeInBytes) {                 \
    (void)bar; (void)address; (void)data; (void)sizeInBytes;

#define END_WRITEEVENT_TABLE }

///
/// Provide a "table" of events and register names using the CONNECT_REGISTER_EVENT macro for read events
/// The table must be terminated with END_READEVENT_TABLE.
///
#define READEVENT_TABLE(table)                                                                                  \
  void regReadEvents(uint8_t bar, uint32_t address, int32_t const *data, size_t sizeInBytes) {                  \
    (void)bar; (void)address; (void)data; (void)sizeInBytes;

#define END_READEVENT_TABLE }

///
/// Connect events with register names. The first argument eventName is the name of an event previously
/// declared using BOOST_MSM_EUML_EVENT. Use this macro in the of the WRITE_EVENT_TABLE and READ_EVENT_TABLE macros.
/// Do not separate multiple calls to CONNECT_REGISTER_EVENT inside the same WRITE_EVENT_TABLE/READ_EVENT_TABLE macro by
/// any separator (i.e. no comma in between - starting a new line is allowed, though!).
///
#define CONNECT_REGISTER_EVENT(eventName, registerModule, registerName)                                         \
  {                                                                                                             \
  RegisterInfoMap::RegisterInfo elem;                                                                           \
    _registerMapping->getRegisterInfo(registerName, elem, registerModule);                                      \
    if(bar == elem.reg_bar && address >= elem.reg_address && address < elem.reg_address+elem.reg_size) {        \
      theStateMachine.process_event(eventName ());                                                              \
    }                                                                                                           \
  }

///
/// Declare a guard condition on the value of a register. The resulting guard condition can be used on an event
/// defined with the CONNECT_REGISTER_EVENT macro. The first argument guardName is the name of the resulting guard
/// class and the second argument will be parsed as the guard condition (standard C++ syntax). Inside the condition,
/// the variable "value" can be used which contains the value of the register written in the event (type: int32_t,
/// always the first word written).
///
#define DECLARE_REGISTER_GUARD(guardName, condition)                                                            \
    class guardName :  msm::front::euml::euml_action<guardName>                                                 \
    {                                                                                                           \
      public:                                                                                                   \
        guardName() {}                                                                                          \
        typedef guardName action_name;                                                                          \
        template <class Fsm,class Evt,class SourceState,class TargetState>                                      \
        bool operator()(Evt const& ,Fsm& fsm,SourceState&,TargetState& )                                        \
        {                                                                                                       \
            dummyDeviceType *dev = fsm.dev;                                                                     \
            assert( dev->lastWrittenData != NULL );                                                             \
            int32_t value = *(dev->lastWrittenData);                                                            \
            (void)value;                                                                                        \
            return ( condition );                                                                               \
        }                                                                                                       \
    };

///
/// Declare a guard condition. The argument guardName is the name of the resulting guard class.
/// The code must follow this macro, contain a return statement returning a boolean and has to be terminated
/// with END_DECLARE_GUARD.
///
#define DECLARE_GUARD(guardName)                                                                                \
    class guardName :  msm::front::euml::euml_action<guardName>                                                 \
    {                                                                                                           \
      public:                                                                                                   \
        guardName() {}                                                                                          \
        typedef guardName action_name;                                                                          \
        template <class Fsm,class Evt,class SourceState,class TargetState>                                      \
        bool operator()(Evt const& ,Fsm& fsm,SourceState&,TargetState& )                                        \
        {                                                                                                       \
            dummyDeviceType *dev = fsm.dev;                                                                     \
            (void)dev;

#define END_DECLARE_GUARD }};

///
/// Declare an action with arbitrary code. The argument actionName is the name of the resulting action class.
/// The code must follow this macro and has to be terminated with END_DECLARE_ACTION.
///
#define DECLARE_ACTION(actionName)                                                                              \
    class actionName :  msm::front::euml::euml_action<actionName>                                               \
    {                                                                                                           \
      public:                                                                                                   \
      actionName() {}                                                                                           \
        typedef actionName action_name;                                                                         \
        template <class Fsm,class Evt,class SourceState,class TargetState>                                      \
        void operator()(Evt const& ,Fsm& fsm,SourceState&,TargetState& )                                        \
        {                                                                                                       \
            dummyDeviceType *dev = fsm.dev;                                                                     \
            (void)dev;

#define END_DECLARE_ACTION }};

///
/// Declare a timer with a given name. Will fire the given event.
///
#define DECLARE_TIMER(name,event) timer<event> name;

///
/// Declare a timer group with a given name. The additional arguments must be the timers part of
/// this group, previously declared using DECLARE_TIMER.
///
#define DECLARE_TIMER_GROUP(name, ...)                                                                          \
    typedef boost::fusion::vector< DECLARE_TIMER_GROUP_DECLARE_VECTOR(__VA_ARGS__) > name ## __;                \
    class name ## _: public timerGroup<name ## __> {                                                            \
      public:                                                                                                   \
        name ## _(dummyDeviceType *_dev)                                                                        \
        : timerGroup(),                                                                                         \
          dev(_dev)                                                                                             \
        {                                                                                                       \
          DECLARE_TIMER_GROUP_MAKEMAP(__VA_ARGS__)                                                              \
          timers = new name ## __( DECLARE_TIMER_GROUP_INIT_VECTOR(__VA_ARGS__) );                              \
        }                                                                                                       \
      protected:                                                                                                \
        dummyDeviceType *dev;                                                                                   \
    };                                                                                                          \
    name ## _ name;

///
/// Declare a state machine.
/// dummyDeviceType is the class name of the physDummyDevice implementation the state machine will be used in
/// stateMachineName is the name of the state machine itself. Several types will be defined based on this names with
/// one or more trailing underscores.
/// initialState is the name of the initial state. Multiple states can be added by separating them with "<<".
/// transitionTable is the transition table in eUML syntax.
///
#define DECLARE_STATE_MACHINE(stateMachineName, initialState, transitionTable)                                  \
    BOOST_MSM_EUML_TRANSITION_TABLE((                                                                           \
        transitionTable                                                                                         \
    ),stateMachineName ## _table)                                                                               \
    BOOST_MSM_EUML_DECLARE_STATE_MACHINE(( stateMachineName ## _table,                                          \
                                           init_ << initialState ),                                             \
                                           stateMachineName ## __)                                              \
    class stateMachineName ## _ : public stateMachineName ## __  {                                              \
      public:                                                                                                   \
        stateMachineName ## _(dummyDeviceType *_dev)                                                            \
        : stateMachineName ## __(),                                                                             \
          dev(_dev)                                                                                             \
        {}                                                                                                      \
        stateMachineName ## _()                                                                                 \
          : stateMachineName ## __(),                                                                           \
          dev(NULL)                                                                                             \
        {}                                                                                                      \
        void setDummyDevice(dummyDeviceType *_dev) {dev = _dev;}                                                \
        dummyDeviceType *dev;                                                                                   \
    };                                                                                                          \
    typedef msm::back::state_machine<stateMachineName ## _> stateMachineName;

///
/// Declare the main state machine. This is just like DECLARE_STATE_MACHINE but with a fixed name "mainStateMachine".
/// Also the state machine will be instantiated under the name "theStateMachine", as expected by other parts of this
/// framework.
///
#define DECLARE_MAIN_STATE_MACHINE(initialState, transitionTable)                                               \
    DECLARE_STATE_MACHINE(mainStateMachine, initialState, transitionTable)                                      \
    mainStateMachine theStateMachine;

///
/// Declare the constructor of the VirtualLabBackend. The first argument must be the class name. The other arguments
/// must be the list of member initialisers, the code of the cunstructor follows this macro and must be terminated
/// with END_CONSTRUCTOR, even if no code is put into the constructor.
///
#define CONSTRUCTOR(name,...)                                                                                   \
    name(std::string host, std::string instance, std::list< std::string > parameters) :                         \
    VirtualLabBackend(host,instance,parameters),                                                                \
      ## __VA_ARGS__ ,                                                                                          \
      theStateMachine(this)                                                                                     \
    {

#define END_CONSTRUCTOR }



namespace mtca4u { namespace VirtualLab {

/*********************************************************************************************************************/
/** The dummy device opens a mapping file instead of a device, and
 *  implements all registers defined in the mapping file in memory.
 *  Like this it mimics the real PCIe device.
 *
 *  This class helps implementing dummy devices using a state machine and allows an easy connection with the
 *  virtual lab simulation framework.
 *
 *  The physDummyDevice class is a template of the derived implementation, which means it follows a CRTP
 */
template<class derived>
class VirtualLabBackend : public DummyBackend
{
  public:

    // constructor via standard device model decription (as used by the DeviceFactory)
    VirtualLabBackend(std::string host, std::string instance, std::list< std::string > parameters) :
      DummyBackend(host,instance,parameters),
      lastWrittenData(NULL),
      lastWrittenSize(0)
    {
      // start the main state machine
      static_cast<derived&>(*this).theStateMachine.start();
    }

    virtual ~VirtualLabBackend() {}

    /// override writeArea to fire the events
    virtual void write(uint8_t bar, uint32_t address, int32_t const *data, size_t sizeInBytes) {
      // save as last written data, for use inside guards of the events we may trigger now
      lastWrittenData = data;
      lastWrittenSize = sizeInBytes;

      // perform the actual write
      DummyBackend::write(bar, address, data, sizeInBytes);

      // trigger events
      regWriteEvents(bar,address,data,sizeInBytes);
    }

    /// override readArea to fire the events
    virtual void read(uint8_t bar, uint32_t address, int32_t *data, size_t sizeInBytes) {
      // trigger events
      regReadEvents(bar, address, data, sizeInBytes);

      // perform the actual write
      DummyBackend::read(bar, address, data, sizeInBytes);
    }

  protected:

    /// define the dummyDeviceType used in the macros
    typedef derived dummyDeviceType;

    /// trigger register-write events. Will be implemented using WRITE_EVENT_TABLE in the device implementation
    virtual void regWriteEvents(uint8_t bar, uint32_t address, int32_t const *data, size_t sizeInBytes) {
      (void) bar; (void) address; (void) data; (void) sizeInBytes;
    };

    /// trigger register-read events. Will be implemented using READ_EVENT_TABLE in the device implementation
    virtual void regReadEvents(uint8_t bar, uint32_t address, int32_t const *data, size_t sizeInBytes) {
      (void) bar; (void) address; (void) data; (void) sizeInBytes;
    };

    /// VirtualDevice::timer class
    template<class timerEvent>
    class timer {
      public:
        timer(derived *_dev) : dev(_dev),request(-1),current(0) {}

        /// Set the timer to fire in tval milliseconds
        void set(double tval) {
          request = tval+current;
        }

        /// Advance the timer's current time by tval milliseconds. Returns true if the timer was fired.
        /// If tval < 0 this function does nothing.
        /// Note: calling this function with tval > getRemaining() will fire the event only a single time!
        bool advance(double tval) {
          if(tval < 0) return false;
          current += tval;
          if(request > 0 && current >= request) {
            request = -1;
            dev->theStateMachine.process_event( timerEvent() );
            return true;
          }
          return false;
        }

        /// Obtain remaining time until the timer fires. Will be negative if the timer is not set.
        double getRemaining() {
          if(request > 0) {
            return request-current;
          }
          else {
            return -1;
          }
        }

        /// Get current timer
        double getCurrent() {
          return current;
        }

      protected:

        /// the dummy device with the state machine
        derived *dev;

        /// requested time
        double request;

        /// current time
        double current;
    };

    /// last written data (into any register) and its size. Will be used in guard conditions.
    int32_t const *lastWrittenData;
    size_t lastWrittenSize;

};


}}// namespace mtca4u::VirtualLab

#endif /* VIRTUALDEVICE_H */
