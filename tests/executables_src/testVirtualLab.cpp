#include <tuple>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <boost/test/included/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <mtca4u/Device.h>
#include "VirtualLabBackend.h"
#include "SignalSink.h"
#include "StateVariableSet.h"

using namespace boost::unit_test_framework;

using namespace mtca4u;
using namespace mtca4u::VirtualLab;

#define TEST_MAPPING_FILE "test.mapp"
#define TEST_DMAP_FILE "dummies.dmap"

/**********************************************************************************************************************/
// forward declaration so we can declare it friend of VirtualTestDevice
class VirtualLabTest;

/**********************************************************************************************************************/
/** The VirtualTestDevice is a VirtualDevice implementation to test the framework
 */
class VirtualTestDevice : public VirtualLabBackend<VirtualTestDevice>
{
  public:

    friend class VirtualLabTest;

    CONSTRUCTOR(VirtualTestDevice,
      someCounter(0),
      readCount(0),
      writeCount(0),
      writeMuxedCount(0),
      write42Count(0),
      readWithFlagCount(0),
      someRegister(this,"APP0","SOME_REGISTER"),
      someMuxedRegister(this,"APP0","DAQ0_ADCA"),
      myTimer(this),
      mySecondTimer(this),
      timers__(this),
      timers(this),
      someFlag(false),
      subCounter(0)
    )
      INIT_SUB_STATE_MACHINE(subMachine)
    END_CONSTRUCTOR

    // Overload open and close to send events on device open and close. For a real VirtualLabBackend, this should
    // usually not be done, as the state of the device driver should not be part of the state machine.
    virtual void open() {
      someCounter = 0;
      readCount = 0;
      writeCount = 0;
      writeMuxedCount = 0;
      write42Count = 0;
      readWithFlagCount = 0;
      someFlag = false;
      VirtualLabBackend::open();
      theStateMachine.process_event(onDeviceOpen());
    }
    virtual void close() {
      VirtualLabBackend::close();
      theStateMachine.process_event(onDeviceClose());
    }

    /// states
    DECLARE_STATE(DevClosed)
    DECLARE_STATE(DevOpen)
    DECLARE_LOGGING_STATE(SomeIntermediateState)
    DECLARE_LOGGING_STATE(CountingState)
    DECLARE_LOGGING_STATE(TimerTest)

    /// events
    DECLARE_EVENT(onDeviceOpen)
    DECLARE_EVENT(onDeviceClose)
    DECLARE_EVENT(onTimer)
    DECLARE_EVENT(onSecondTimer)
    DECLARE_EVENT(goCounting)
    DECLARE_EVENT(onRead)
    DECLARE_EVENT(onWrite)
    DECLARE_EVENT(onWriteMuxed)
    DECLARE_EVENT(startSubMachine)
    DECLARE_EVENT(stopSubMachine)
    DECLARE_EVENT(runDoubleAction)
    DECLARE_EVENT(setBothTimers)
    DECLARE_EVENT(goTimerTest)

    /// counting action: increase counter and set timer again
    DECLARE_ACTION(countingAction)
        someCounter++;
        myTimer.set(1*seconds);
    END_DECLARE_ACTION

    /// our counter for the counting action
    int someCounter;

    /// read and write actions
    DECLARE_ACTION(readAction)
        readCount++;
    END_DECLARE_ACTION

    /// read and write actions
    DECLARE_ACTION(readWithFlagAction)
        readWithFlagCount++;
    END_DECLARE_ACTION

    /// read and write actions
    DECLARE_ACTION(writeAction)
        writeCount++;
    END_DECLARE_ACTION

    /// read and write actions
    DECLARE_ACTION(write42Action)
        write42Count++;
    END_DECLARE_ACTION

    /// read and write actions
    DECLARE_ACTION(writeMuxedAction)
        writeMuxedCount++;
    END_DECLARE_ACTION

    /// counters for read and write events
    int readCount;
    int writeCount;
    int writeMuxedCount;
    int write42Count;
    int readWithFlagCount;

    /// connect read and write events
    READEVENT_TABLE
      CONNECT_REGISTER_EVENT(onRead,someRegister)
    END_READEVENT_TABLE
    WRITEEVENT_TABLE
      CONNECT_REGISTER_EVENT(onWrite,someRegister)
      CONNECT_REGISTER_EVENT(onWriteMuxed,someMuxedRegister)
    END_WRITEEVENT_TABLE

    /// register accessors
    DECLARE_REGISTER(int, someRegister);
    DECLARE_MUXED_REGISTER(int, someMuxedRegister);

    /// timer group
    DECLARE_TIMER(myTimer, onTimer)
    DECLARE_TIMER(mySecondTimer, onSecondTimer)
    DECLARE_TIMER_GROUP(timers__, myTimer, mySecondTimer)

    /// derived timer group class to make the test class a friend
    class timers_ : public timers___ {
      public:
        friend class VirtualLabTest;
        timers_(dummyDeviceType *_dev) : timers___(_dev) {};
    };
    timers_ timers;

    /// action to set timer
    DECLARE_ACTION(doSetTimer)
      myTimer.set(1*seconds);
      someCounter++;
    END_DECLARE_ACTION

    /// action to set both timers
    DECLARE_ACTION(doSetBothTimers)
      myTimer.set(1*seconds);
      mySecondTimer.set(100*seconds);
    END_DECLARE_ACTION

    /// register guard: allow transition if 42 is written
    DECLARE_REGISTER_GUARD(is42Written, value == 42 )

    /// guard testing if a flag is set or not
    bool someFlag;
    DECLARE_GUARD(someGuard)
      return someFlag;
    END_DECLARE_GUARD

    /// define a small sub-state machine
    DECLARE_STATE(subInit)
    DECLARE_EVENT(subEvent)
    DECLARE_ACTION(subCount)
      subCounter++;
    END_DECLARE_ACTION
    int subCounter;
    DECLARE_STATE_MACHINE(subMachine, subInit(), (
      subInit() + subEvent() / subCount()
    ))

    /// action throwing an exception
    DECLARE_EVENT(requestException)
    DECLARE_STATE(awaitException)
    DECLARE_ACTION(throwException)
      throw std::string("This is some exception destroying our state machine!");
    END_DECLARE_ACTION

    /// define the state machine structure
    DECLARE_MAIN_STATE_MACHINE(DevClosed(), (
      // =======================================================================================================
      // open and close the device
      // note: an actual virtual device should never distinguish between closed and opened states, since real
      // hardware does not, either!
      DevClosed() + onDeviceOpen() == DevOpen(),
      DevOpen() + onDeviceClose() == DevClosed(),
      SomeIntermediateState() + onDeviceClose() == DevClosed(),
      CountingState() + onDeviceClose() == DevClosed(),
      TimerTest() + onDeviceClose() == DevClosed(),

      // start and stop the DAQ
      DevOpen() + onTimer() == SomeIntermediateState(),
      SomeIntermediateState() + onSecondTimer() == DevOpen(),

      // counting state (for counting how often a timer fired)
      DevOpen() + goCounting() == CountingState(),
      CountingState() + onTimer() / countingAction(),

      // read and write events
      DevOpen() + onRead() [ ! someGuard() ] / readAction(),
      DevOpen() + onRead() [ someGuard() ] / readWithFlagAction(),
      DevOpen() + onWrite() [ ! is42Written() ] / writeAction(),
      DevOpen() + onWrite() [ is42Written() ] / write42Action(),
      DevOpen() + onWriteMuxed() / writeMuxedAction(),

      // test running two actions on a single event (abusing some existing actions...)
      DevOpen() + runDoubleAction() / ( readAction(), writeAction() ),

      // sub machine
      DevOpen() + startSubMachine() == subMachine(),
      subMachine() + stopSubMachine() == DevOpen(),

      // set timers via actions
      DevOpen() + goTimerTest() / doSetTimer() == TimerTest(),
      TimerTest() + onTimer() / doSetTimer(),
      TimerTest() + setBothTimers() / doSetBothTimers(),
      TimerTest() + onSecondTimer() / doSetBothTimers(),

      // exception throwing inside a timer
      DevOpen() + requestException() == awaitException(),
      awaitException() + onTimer() / throwException()
    ))

};

REGISTER_BACKEND_TYPE(VirtualTestDevice)

/**********************************************************************************************************************/
class VirtualLabTest {
  public:
    VirtualLabTest()
    : onHistoryLengthChanged_argument(0),
      onValueNeeded_returnValue(0),
      onValueNeeded_argument(0)
    {
      std::list<std::string> params;
      params.push_back(TEST_MAPPING_FILE);
      device = boost::static_pointer_cast<VirtualTestDevice>( VirtualTestDevice::createInstance("", "0", params) );
    }

    /// test the device open and close events
    void testDevOpenClose();

    /// test actions
    void testActions();

    /// test guards
    void testGuards();

    /// test the timer group system
    void testTimerGroup();

    /// test read and write events
    void testReadWriteEvents();

    /// test sub-state machine
    void testSubMachine();

    /// test sink and source mechanism stand-alone
    void testSinkSource();

    /// test throwing an exception
    void testThrowException();

    /// test the StateVariableSet
    void testStateVariableSet();


  private:
    boost::shared_ptr<VirtualTestDevice> device;
    friend class DummyDeviceTestSuite;

    /// callback function for history length changed
    void onHistoryLengthChanged(VirtualTime time) {
      onHistoryLengthChanged_argument = time;
    }
    VirtualTime onHistoryLengthChanged_argument;

    /// callback function for value needed
    double onValueNeeded(VirtualTime time) {
      onValueNeeded_argument = time;
      return onValueNeeded_returnValue;
    }
    double onValueNeeded_returnValue;
    VirtualTime onValueNeeded_argument;

    /// callback function to compute states (StateVariableSet)
    int onCompute(VirtualTime time) {
      onCompute_argument = time;
      return onCompute_returnValue;
    }
    int onCompute_returnValue;
    VirtualTime onCompute_argument;


};

/**********************************************************************************************************************/
class  VirtualLabTestSuite : public test_suite {
  public:
    VirtualLabTestSuite() : test_suite("VirtualLab test suite") {
      boost::shared_ptr<VirtualLabTest> virtualLabTest( new VirtualLabTest );

      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testDevOpenClose, virtualLabTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testActions, virtualLabTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testGuards, virtualLabTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testTimerGroup, virtualLabTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testReadWriteEvents, virtualLabTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testSubMachine, virtualLabTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testSinkSource, virtualLabTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testThrowException, virtualLabTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualLabTest::testStateVariableSet, virtualLabTest ) );

    }};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] )
{
  mtca4u::BackendFactory::getInstance().setDMapFilePath(TEST_DMAP_FILE);

  framework::master_test_suite().p_name.value = "VirtualLab test suite";
  framework::master_test_suite().add(new VirtualLabTestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void VirtualLabTest::testDevOpenClose() {
  std::cout << "testDevOpenClose" << std::endl;

  // open and close the device and check the states
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 0 );
  device->open();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 1 );
  device->close();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 0 );
  device->open();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 1 );
  device->close();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 0 );

  // open virtual device (= backend) via a Device frontend
  Device frontend;
  frontend.open("DUMMY");
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 1 );
  frontend.close();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 0 );

}

/**********************************************************************************************************************/
void VirtualLabTest::testTimerGroup() {
  std::cout << "testTimerGroup" << std::endl;

  // check if name vector is correctly initialised
  BOOST_CHECK( device->timers.names.size() == 2 );
  BOOST_CHECK( device->timers.names[0] == "myTimer" );
  BOOST_CHECK( device->timers.names[1] == "mySecondTimer" );

  // check if getRemaining works if timers are not set
  BOOST_CHECK( device->myTimer.getRemaining() == -1 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );

  // set a timer
  device->myTimer.set(10*seconds);

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 10*seconds );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == 10*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 10*seconds );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );

  // advance by 5 seconds
  device->timers.advance(5*seconds);

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 5*seconds );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == 5*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 5*seconds );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );

  // set second timer
  device->mySecondTimer.set(20*seconds);

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 5*seconds );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 20*seconds );
  BOOST_CHECK( device->timers.getRemaining() == 5*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 5*seconds );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 20*seconds );

  // advance by 1
  device->timers.advance(1*seconds);

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 4*seconds );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 19*seconds );
  BOOST_CHECK( device->timers.getRemaining() == 4*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 4*seconds );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 19*seconds );

  // advance to myTimer
  device->timers.advance("myTimer");

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == -1 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 15*seconds );
  BOOST_CHECK( device->timers.getRemaining() == 15*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 15*seconds );

  // advance to myTimer which is not set
  device->timers.advance("myTimer");

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == -1 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 15*seconds );
  BOOST_CHECK( device->timers.getRemaining() == 15*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 15*seconds );

  // set first timer again and advance one time
  device->myTimer.set(20*seconds);
  device->timers.advance();

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 5*seconds );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == 5*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 5*seconds );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );

  // advance to the end
  device->timers.advance();

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == -1 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );

  // check current time
  BOOST_CHECK( device->myTimer.getCurrent() == 30*seconds );
  BOOST_CHECK( device->mySecondTimer.getCurrent() == 30*seconds );
  BOOST_CHECK( device->timers.getCurrent() == 30*seconds );

  // open the device
  device->open();

  // set first timer and make it fire, then check if in SomeIntermediateState()
  device->myTimer.set(5*seconds);
  device->timers.advance();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 2 );

  // set first timer again and make it fire, then check if still in SomeIntermediateState()
  device->myTimer.set(5*seconds);
  device->timers.advance();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 2 );

  // set second timer and make it fire, then check if in DeviceOpen()
  device->mySecondTimer.set(5*seconds);
  device->timers.advance();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 1 );

  // go into CountingState() to count how often the timer fires
  device->theStateMachine.process_event(VirtualTestDevice::goCounting());
  device->myTimer.set(1*seconds);
  BOOST_CHECK( device->someCounter == 0 );
  device->timers.advance();
  BOOST_CHECK( device->someCounter == 1 );
  device->timers.advance(10*seconds);
  BOOST_CHECK( device->someCounter == 11 );

  // reset device
  device->close();
  device->open();
  device->someCounter = 0;

  // go into timer test mode and check if myTimer is correctly set
  device->theStateMachine.process_event(VirtualTestDevice::goTimerTest());
  BOOST_CHECK( device->myTimer.getRemaining() == 1*seconds );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == 1*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 1*seconds );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );
  BOOST_CHECK( device->someCounter == 1 );

  // advance to fire myTimer: should set it again
  for(int i=0; i<5; i++) {
    device->timers.advance();
    BOOST_CHECK( device->myTimer.getRemaining() == 1*seconds );
    BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
    BOOST_CHECK( device->timers.getRemaining() == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("myTimer") == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );
    BOOST_CHECK( device->someCounter == 2+i );
  }
  for(int i=0; i<3; i++) {
    device->timers.advance("myTimer");
    BOOST_CHECK( device->myTimer.getRemaining() == 1*seconds );
    BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
    BOOST_CHECK( device->timers.getRemaining() == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("myTimer") == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );
    BOOST_CHECK( device->someCounter == 7+i );
  }
  for(int i=0; i<3; i++) {
    device->timers.advance(1*seconds);
    BOOST_CHECK( device->myTimer.getRemaining() == 1*seconds );
    BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
    BOOST_CHECK( device->timers.getRemaining() == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("myTimer") == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );
    BOOST_CHECK( device->someCounter == 10+i );
  }

  // switch to using both timers
  device->theStateMachine.process_event(VirtualTestDevice::setBothTimers());
  BOOST_CHECK( device->myTimer.getRemaining() == 1*seconds );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 100*seconds );
  BOOST_CHECK( device->timers.getRemaining() == 1*seconds );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 1*seconds );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 100*seconds );
  BOOST_CHECK( device->someCounter == 12 );

  // advance mySecondsTimer, should set and fire myTimer 100 times
  device->someCounter = 0;
  for(int i=0; i<3; i++) {
    device->timers.advance("mySecondTimer");
    BOOST_CHECK( device->myTimer.getRemaining() == 1*seconds );
    BOOST_CHECK( device->mySecondTimer.getRemaining() == 100*seconds );
    BOOST_CHECK( device->timers.getRemaining() == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("myTimer") == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 100*seconds );
    BOOST_CHECK( device->someCounter == 100*(i+1) );
  }

  // advance mySecondsTimer in a different way, should set and fire myTimer 100 times
  device->someCounter = 0;
  for(int i=0; i<5; i++) {
    device->timers.advance(100*seconds);
    BOOST_CHECK( device->myTimer.getRemaining() == 1*seconds );
    BOOST_CHECK( device->mySecondTimer.getRemaining() == 100*seconds );
    BOOST_CHECK( device->timers.getRemaining() == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("myTimer") == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 100*seconds );
    BOOST_CHECK( device->someCounter == 100*(i+1) );
  }

  // just advance 1000 seconds, should set and fire myTimer 1000 times
  device->someCounter = 0;
  for(int i=0; i<3; i++) {
    device->timers.advance(1000*seconds);
    BOOST_CHECK( device->myTimer.getRemaining() == 1*seconds );
    BOOST_CHECK( device->mySecondTimer.getRemaining() == 100*seconds );
    BOOST_CHECK( device->timers.getRemaining() == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("myTimer") == 1*seconds );
    BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 100*seconds );
    BOOST_CHECK( device->someCounter == 1000*(i+1) );
  }

  // close the device
  device->close();


}

/**********************************************************************************************************************/
void VirtualLabTest::testReadWriteEvents() {
  int data;
  RegisterInfoMap::RegisterInfo elem;

  std::cout << "testReadWriteEvents" << std::endl;

  // open device
  device->open();

  // write to register
  device->_registerMapping->getRegisterInfo("SOME_REGISTER", elem, "APP0");
  data = 120;
  BOOST_CHECK( device->writeCount == 0 );
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->writeCount == 1 );
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->writeCount == 2 );

  // read from register
  BOOST_CHECK( device->readCount == 0 );
  device->read(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->readCount == 1 );
  device->read(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->readCount == 2 );

  // write to muxed register
  device->_registerMapping->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCA", elem, "APP0");
  data = 120;
  BOOST_CHECK( device->writeMuxedCount == 0 );
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->writeMuxedCount == 1 );
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->writeMuxedCount == 2 );


  device->close();
}

/**********************************************************************************************************************/
void VirtualLabTest::testGuards() {
  int data;
  RegisterInfoMap::RegisterInfo elem;

  std::cout << "testGuards" << std::endl;

  // open device
  device->open();

  // write to register something passing the register guard
  device->_registerMapping->getRegisterInfo("SOME_REGISTER", elem, "APP0");
  data = 42;
  BOOST_CHECK( device->write42Count == 0 );
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->write42Count == 1 );
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->write42Count == 2 );

  // read from register with the flag set, so the normal guard is passed
  device->someFlag = true;
  device->_registerMapping->getRegisterInfo("SOME_REGISTER", elem, "APP0");
  BOOST_CHECK( device->readWithFlagCount == 0 );
  device->read(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->readWithFlagCount == 1 );
  device->read(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK( device->readWithFlagCount == 2 );

  device->close();
}

/**********************************************************************************************************************/
void VirtualLabTest::testSubMachine() {
  std::cout << "testSubMachine" << std::endl;

  // open device
  device->open();

  // test if events of the submachine while it is not running do any harm (should have no effect)
  BOOST_CHECK( device->subCounter == 0 );
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK( device->subCounter == 0 );

  // start sub machine
  device->theStateMachine.process_event(VirtualTestDevice::startSubMachine());

  // fire submachine event
  BOOST_CHECK( device->subCounter == 0 );
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK( device->subCounter == 1 );
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK( device->subCounter == 2 );

  // fire event of main machine which is currently not effective
  device->theStateMachine.process_event(VirtualTestDevice::goCounting());
  BOOST_CHECK( device->subCounter == 2 );
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK( device->subCounter == 3 );

  // fire event to exit submachine
  device->theStateMachine.process_event(VirtualTestDevice::stopSubMachine());
  BOOST_CHECK( device->subCounter == 3 );
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK( device->subCounter == 3 );

  // enter submachine another time
  device->theStateMachine.process_event(VirtualTestDevice::startSubMachine());
  BOOST_CHECK( device->subCounter == 3 );
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK( device->subCounter == 4 );
  device->theStateMachine.process_event(VirtualTestDevice::stopSubMachine());
  BOOST_CHECK( device->subCounter == 4 );
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK( device->subCounter == 4 );

  // close device
  device->close();
}

/**********************************************************************************************************************/
void VirtualLabTest::testActions() {
  std::cout << "testActions" << std::endl;

  // open device
  device->open();

  // send event to trigger double action
  BOOST_CHECK( device->readCount == 0 );
  BOOST_CHECK( device->writeCount == 0 );
  device->theStateMachine.process_event(VirtualTestDevice::runDoubleAction());
  BOOST_CHECK( device->readCount == 1 );
  BOOST_CHECK( device->writeCount == 1 );
  device->theStateMachine.process_event(VirtualTestDevice::runDoubleAction());
  BOOST_CHECK( device->readCount == 2 );
  BOOST_CHECK( device->writeCount == 2 );

  // close device
  device->close();
}

/**********************************************************************************************************************/
void VirtualLabTest::testSinkSource() {
  std::cout << "testSinkSource" << std::endl;

  // test constant source
  SignalSink sink(42);
  sink.setMaxHistoryLength(20*milliseconds);
  BOOST_CHECK( sink.getValue(-10*milliseconds) == 42 );
  BOOST_CHECK( sink.getValue(0*milliseconds) == 42 );
  BOOST_CHECK( sink.getValue(10*milliseconds) == 42 );

  // re-connect with another constant source
  auto constSource = boost::make_shared<ConstantSignalSource>(120);
  sink.connect( boost::static_pointer_cast<SignalSource>(constSource) );
  BOOST_CHECK( sink.getValue(-10*milliseconds) == 120 );    // the sink has no history
  BOOST_CHECK( sink.getValue(0*milliseconds) == 120 );
  BOOST_CHECK( sink.getValue(10*milliseconds) == 120 );

  // create non-constant source and connect with it
  auto source = boost::make_shared<SignalSource>();
  sink.connect(source);

  // nothing in the buffer yet
  BOOST_CHECK_THROW( sink.getValue(0*milliseconds), SignalSourceException );

  // feed some values to the source and read from sink
  source->feedValue(0*milliseconds,10.);
  source->feedValue(100*microseconds,99.);
  source->feedValue(1*milliseconds,-30.);
  source->feedValue(10*milliseconds,666.);

  BOOST_CHECK( sink.getValue(0*milliseconds) == 10. );
  BOOST_CHECK( sink.getValue(100*microseconds) == 99. );
  BOOST_CHECK( sink.getValue(1*milliseconds) == -30. );
  BOOST_CHECK( sink.getValue(10*milliseconds) == 666. );
  BOOST_CHECK_THROW( sink.getValue(11*milliseconds), SignalSourceException );

  // add tolerance to the source and test it
  source->setTimeTolerance(2*milliseconds);
  BOOST_CHECK_THROW( sink.getValue(-1*picoseconds), SignalSourceException );
  BOOST_CHECK( sink.getValue(0*milliseconds) == 10. );
  BOOST_CHECK( sink.getValue(100*microseconds - 1*picoseconds) == 10. );
  BOOST_CHECK( sink.getValue(100*microseconds) == 99. );
  BOOST_CHECK( sink.getValue(1*milliseconds - 1*picoseconds) == 99. );
  BOOST_CHECK( sink.getValue(1*milliseconds) == -30. );
  BOOST_CHECK( sink.getValue(3*milliseconds) == -30. );
  BOOST_CHECK_THROW( sink.getValue(3*milliseconds + 1*picoseconds), SignalSourceException );
  BOOST_CHECK_THROW( sink.getValue(10*milliseconds - 1*picoseconds), SignalSourceException );
  BOOST_CHECK( sink.getValue(10*milliseconds) == 666. );
  BOOST_CHECK( sink.getValue(11*milliseconds) == 666. );
  BOOST_CHECK( sink.getValue(12*milliseconds) == 666. );
  BOOST_CHECK_THROW( sink.getValue(12*milliseconds + 1*picoseconds), SignalSourceException );

  // check if old values are removed from buffer
  source->feedValue(20*milliseconds,123.);
  BOOST_CHECK( sink.getValue(0*milliseconds) == 10. );
  source->feedValue(20*milliseconds + 1*picoseconds,124.);
  BOOST_CHECK_THROW( sink.getValue(0*milliseconds), SignalSourceException );

  // change history length and check it
  sink.setMaxHistoryLength(5*milliseconds);
  source->feedValue(25*milliseconds,125.);
  BOOST_CHECK( sink.getValue(20*milliseconds) == 123. );
  source->feedValue(25*milliseconds + 1*picoseconds,126.);
  BOOST_CHECK_THROW( sink.getValue(20*milliseconds), SignalSourceException );

  // go to large times and check if it is still working
  source->feedValue(100*days,127.);
  source->feedValue(100*days + 5*milliseconds,128.);
  BOOST_CHECK( sink.getValue(100*days) == 127. );
  BOOST_CHECK( sink.getValue(100*days + 5*milliseconds) == 128. );
  source->feedValue(100*days + 5*milliseconds + 1*picoseconds,129.);
  BOOST_CHECK_THROW( sink.getValue(100*days), SignalSourceException );

  // test SignalSource's callback function "onHistoryLengthChanged"
  source->setOnHistoryLengthChanged( boost::bind( &VirtualLabTest::onHistoryLengthChanged, this, _1 ) );
  onHistoryLengthChanged_argument = 0;
  sink.setMaxHistoryLength(42*seconds);
  BOOST_CHECK( onHistoryLengthChanged_argument == 42*seconds );

  // test SignalSource's callback function when needing a new value
  source->setCallback( boost::bind( &VirtualLabTest::onValueNeeded, this, _1 ) );
  onValueNeeded_returnValue = 234.567;
  BOOST_CHECK( sink.getValue(101*days) == 234.567 );
  BOOST_CHECK( onValueNeeded_argument == 101*days );

}

/**********************************************************************************************************************/
void VirtualLabTest::testThrowException() {
  std::cout << "testThrowException" << std::endl;

  // prepare redirecting cerr
  struct cerr_redirect {
      cerr_redirect( std::streambuf * new_buffer )
          : old( std::cerr.rdbuf( new_buffer ) )
      { }

      ~cerr_redirect( ) {
          std::cerr.rdbuf( old );
      }

  private:
      std::streambuf * old;
  };

  // need to create our own device for this test, as it destroys the state machine
  std::list<std::string> params;
  params.push_back(TEST_MAPPING_FILE);
  auto myDevice = boost::static_pointer_cast<VirtualTestDevice>( VirtualTestDevice::createInstance("", "1", params) );
  myDevice->open();

  // go into awaitException state and set the timer (which will trigger the exception action)
  myDevice->theStateMachine.process_event(VirtualTestDevice::requestException());
  myDevice->myTimer.set(1*seconds);

  // request exception, should print a warning, so redirect cerr for this test
  boost::test_tools::output_test_stream output;
  {
    cerr_redirect guard( output.rdbuf( ) );
    myDevice->timers.advance("myTimer");                // this will trigger the exception being caught inside the timer
  }

  BOOST_CHECK( output.is_equal( "ERROR in VirtualLabBackend: Exception thrown while processing timer event. The state machine is now in an unusable condition. Make sure to catch all exceptions in your actions!\n" ) );

  // close device
  myDevice->close();

}

/**********************************************************************************************************************/
void VirtualLabTest::testStateVariableSet() {
  std::cout << "testStateVariableSet" << std::endl;

  // a simple "set" with a single integer
  StateVariableSet<int> simpleState;

  // check for exception if no initial state was set
  BOOST_CHECK_THROW( simpleState.getState(0), StateVariableSetException );

  // set initial state and get it back
  simpleState.setInitialState(42);
  BOOST_CHECK( simpleState.getState(0) == 42 );
  BOOST_CHECK( simpleState.getLatestState() == 42 );

  // check for exception (thrown by boost) if state is not available and compute function ist not set
  BOOST_CHECK_THROW( simpleState.getState(1), std::runtime_error );

  // set tolerance to 1 and get state for t=1 again (same as for t=0 then), t=2 must still fail
  simpleState.setTimeTolerance(1);
  BOOST_CHECK( simpleState.getState(1) == 42 );
  BOOST_CHECK_THROW( simpleState.getState(2), std::runtime_error );

  // same with bigger tolerance
  simpleState.setTimeTolerance(1*seconds);
  BOOST_CHECK( simpleState.getState(1*seconds) == 42 );
  BOOST_CHECK_THROW( simpleState.getState(1*seconds + 1), std::runtime_error );

  // set compute function
  simpleState.setComputeFunction( boost::bind(&VirtualLabTest::onCompute, this, _1) );

}
