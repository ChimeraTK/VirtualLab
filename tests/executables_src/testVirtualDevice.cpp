#include <tuple>
#include <algorithm>
#include <math.h>
#include <boost/test/included/unit_test.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <mtca4u/Device.h>
#include "VirtualLabBackend.h"

using namespace boost::unit_test_framework;

using namespace mtca4u;
using namespace mtca4u::VirtualLab;

#define TEST_MAPPING_FILE "test.mapp"

/**********************************************************************************************************************/
// forward declaration so we can declare it friend
// of TestableDummydevice->
class VirtualDeviceTest;

/**********************************************************************************************************************/
/** The VirtualTestDevice is a VirtualDevice implementation to test the framework
 */
class VirtualTestDevice : public VirtualLabBackend<VirtualTestDevice>
{
  public:

    friend class VirtualDeviceTest;

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

    /// counting action: increase counter and set timer again
    DECLARE_ACTION(countingAction)
        dev->someCounter++;
        dev->myTimer.set(1);
    END_DECLARE_ACTION

    /// our counter for the counting action
    int someCounter;

    /// read and write actions
    DECLARE_ACTION(readAction)
        dev->readCount++;
    END_DECLARE_ACTION

    /// read and write actions
    DECLARE_ACTION(readWithFlagAction)
        dev->readWithFlagCount++;
    END_DECLARE_ACTION

    /// read and write actions
    DECLARE_ACTION(writeAction)
        dev->writeCount++;
    END_DECLARE_ACTION

    /// read and write actions
    DECLARE_ACTION(write42Action)
        dev->write42Count++;
    END_DECLARE_ACTION

    /// read and write actions
    DECLARE_ACTION(writeMuxedAction)
        dev->writeMuxedCount++;
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
        friend class VirtualDeviceTest;
        timers_(dummyDeviceType *_dev) : timers___(_dev) {};
    };
    timers_ timers;

    /// register guard: allow transition if 42 is written
    DECLARE_REGISTER_GUARD(is42Written, value == 42 )

    /// guard testing if a flag is set or not
    bool someFlag;
    DECLARE_GUARD(someGuard)
      return dev->someFlag;
    END_DECLARE_GUARD

    /// define a small sub-state machine
    DECLARE_STATE(subInit)
    DECLARE_EVENT(subEvent)
    DECLARE_ACTION(subCount)
      dev->subCounter++;
    END_DECLARE_ACTION
    int subCounter;
    DECLARE_STATE_MACHINE(subMachine, subInit(), (
      subInit() + subEvent() / subCount()
    ))

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

      // sub machine
      DevOpen() + startSubMachine() == subMachine(),
      subMachine() + stopSubMachine() == DevOpen()
    ))

};

REGISTER_BACKEND_TYPE(VirtualTestDevice)

/**********************************************************************************************************************/
class VirtualDeviceTest {
  public:
    VirtualDeviceTest()
    {
      std::list<std::string> params;
      params.push_back(TEST_MAPPING_FILE);
      device = boost::static_pointer_cast<VirtualTestDevice>( VirtualTestDevice::createInstance("", "0", params) );
    }

    /// test the device open and close events
    void testDevOpenClose();

    /// test the timer group system
    void testTimerGroup();

    /// test read and write events
    void testReadWriteEvents();

    /// test guards
    void testGuards();

    /// test sub-state machine
    void testSubMachine();


  private:
    boost::shared_ptr<VirtualTestDevice> device;
    friend class DummyDeviceTestSuite;


};

/**********************************************************************************************************************/
class  DummyDeviceTestSuite : public test_suite {
  public:
    DummyDeviceTestSuite() : test_suite("DummyDevice test suite") {
      boost::shared_ptr<VirtualDeviceTest> dummyDeviceTest( new VirtualDeviceTest );

      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testDevOpenClose, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testTimerGroup, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testReadWriteEvents, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testGuards, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testSubMachine, dummyDeviceTest ) );
    }};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] )
{
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new DummyDeviceTestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void VirtualDeviceTest::testDevOpenClose() {
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
void VirtualDeviceTest::testTimerGroup() {
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
  device->myTimer.set(10);

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 10 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == 10 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 10 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );

  // advance by 5
  device->timers.advance(5);

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 5 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == 5 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 5 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == -1 );

  // set second timer
  device->mySecondTimer.set(20);

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 5 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 20 );
  BOOST_CHECK( device->timers.getRemaining() == 5 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 5 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 20 );

  // advance by 1
  device->timers.advance(1);

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 4 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 19 );
  BOOST_CHECK( device->timers.getRemaining() == 4 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 4 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 19 );

  // advance to myTimer
  device->timers.advance("myTimer");

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == -1 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 15 );
  BOOST_CHECK( device->timers.getRemaining() == 15 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 15 );

  // advance to myTimer which is not set
  device->timers.advance("myTimer");

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == -1 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == 15 );
  BOOST_CHECK( device->timers.getRemaining() == 15 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device->timers.getRemaining("mySecondTimer") == 15 );

  // set first timer again and advance one time
  device->myTimer.set(20);
  device->timers.advance();

  // check if getRemaining gives the right time
  BOOST_CHECK( device->myTimer.getRemaining() == 5 );
  BOOST_CHECK( device->mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device->timers.getRemaining() == 5 );
  BOOST_CHECK( device->timers.getRemaining("myTimer") == 5 );
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
  BOOST_CHECK( device->myTimer.getCurrent() == 30 );
  BOOST_CHECK( device->mySecondTimer.getCurrent() == 30 );
  BOOST_CHECK( device->timers.getCurrent() == 30 );

  // open the device
  device->open();

  // set first timer and make it fire, then check if in SomeIntermediateState()
  device->myTimer.set(5);
  device->timers.advance();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 2 );

  // set first timer again and make it fire, then check if still in SomeIntermediateState()
  device->myTimer.set(5);
  device->timers.advance();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 2 );

  // set second timer and make it fire, then check if in DeviceOpen()
  device->mySecondTimer.set(5);
  device->timers.advance();
  BOOST_CHECK( device->theStateMachine.current_state()[0] == 1 );

  // go into CountingState() to count how often the timer fires
  device->theStateMachine.process_event(VirtualTestDevice::goCounting());
  device->myTimer.set(1);
  BOOST_CHECK( device->someCounter == 0 );
  device->timers.advance();
  BOOST_CHECK( device->someCounter == 1 );
  device->timers.advance(10);
  BOOST_CHECK( device->someCounter == 11 );

  // close the device
  device->close();
}

/**********************************************************************************************************************/
void VirtualDeviceTest::testReadWriteEvents() {
  int data;
  RegisterInfoMap::RegisterInfo elem;

  std::cout << "testReadWriteEvents" << std::endl;

  // open device
  device->open();

  // write to register
  device->_registerMapping->getRegisterInfo("SOME_REGISTER", elem, "APP0");
  data = 120;
  BOOST_CHECK( device->writeCount == 0 );
  device->write(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->writeCount == 1 );
  device->write(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->writeCount == 2 );

  // read from register
  BOOST_CHECK( device->readCount == 0 );
  device->read(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->readCount == 1 );
  device->read(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->readCount == 2 );

  // write to muxed register
  device->_registerMapping->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCA", elem, "APP0");
  data = 120;
  BOOST_CHECK( device->writeMuxedCount == 0 );
  device->write(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->writeMuxedCount == 1 );
  device->write(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->writeMuxedCount == 2 );


  device->close();
}

/**********************************************************************************************************************/
void VirtualDeviceTest::testGuards() {
  int data;
  RegisterInfoMap::RegisterInfo elem;

  std::cout << "testGuards" << std::endl;

  // open device
  device->open();

  // write to register something passing the register guard
  device->_registerMapping->getRegisterInfo("SOME_REGISTER", elem, "APP0");
  data = 42;
  BOOST_CHECK( device->write42Count == 0 );
  device->write(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->write42Count == 1 );
  device->write(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->write42Count == 2 );

  // read from register with the flag set, so the normal guard is passed
  device->someFlag = true;
  device->_registerMapping->getRegisterInfo("SOME_REGISTER", elem, "APP0");
  BOOST_CHECK( device->readWithFlagCount == 0 );
  device->read(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->readWithFlagCount == 1 );
  device->read(elem.reg_bar, elem.reg_address, &data, sizeof(int));
  BOOST_CHECK( device->readWithFlagCount == 2 );

  device->close();
}

/**********************************************************************************************************************/
void VirtualDeviceTest::testSubMachine() {
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
