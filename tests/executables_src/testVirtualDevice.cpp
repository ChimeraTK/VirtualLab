#include <tuple>
#include <math.h>
#include <boost/test/included/unit_test.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "VirtualDevice.h"

using namespace boost::unit_test_framework;

using namespace mtca4u;
using namespace mtca4u::VirtualLab;

/**********************************************************************************************************************/
// forward declaration so we can declare it friend
// of TestableDummyDevice.
class VirtualDeviceTest;

/**********************************************************************************************************************/
/** The VirtualTestDevice is a VirtualDevice implementation to test the framework
 */
class VirtualTestDevice : public VirtualDevice<VirtualTestDevice>
{
  public:

    friend class VirtualDeviceTest;

    VirtualTestDevice() :
      someCounter(0),
      myTimer(this),
      mySecondTimer(this),
      timers__(this),
      timers(this),
      theStateMachine(this)
    {
      theStateMachine.start();
    }

    virtual ~VirtualTestDevice() {}

    /// on device open: fire the device-open event
    virtual void openDev(const std::string &mappingFileName, int perm=O_RDWR, devConfigBase *pConfig=NULL) {

      // open the underlying dummy device
      VirtualDevice::openDev(mappingFileName, perm, pConfig);

      // send onDeviceOpen event
      theStateMachine.process_event(onDeviceOpen());

      // setup registers
      someRegister.open(this,"APP0","SOME_REGISTER");

    }

    /// on device close: fire the device-close event
    virtual void closeDev() {
      VirtualDevice::closeDev();
      theStateMachine.process_event(onDeviceClose());
    }

    /// states
    DECLARE_STATE(DevClosed)
    DECLARE_STATE(DevOpen)
    DECLARE_STATE(SomeIntermediateState)
    DECLARE_STATE(CountingState)

    /// events
    DECLARE_EVENT(onDeviceOpen)
    DECLARE_EVENT(onDeviceClose)
    DECLARE_EVENT(onTimer)
    DECLARE_EVENT(onSecondTimer)
    DECLARE_EVENT(goCounting)

    /// counting action: increase counter and set timer again
    DECLARE_ACTION(countingAction,
        dev->someCounter++;
        dev->myTimer.set(1);
    )

    /// our counter for the counting action
    int someCounter;

    /// register accessors
    intRegister someRegister;

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

    /// define the state machine structure
    DECLARE_STATE_MACHINE(mainStateMachine, DevClosed(), (
      // =======================================================================================================
      // open and close the device
      DevClosed() + onDeviceOpen() == DevOpen(),
      DevOpen() + onDeviceClose() == DevClosed(),
      SomeIntermediateState() + onDeviceClose() == DevClosed(),
      CountingState() + onDeviceClose() == DevClosed(),

      // start and stop the DAQ
      DevOpen() + onTimer() == SomeIntermediateState(),
      SomeIntermediateState() + onSecondTimer() == DevOpen(),

      // counting state (for counting how often a timer fired)
      DevOpen() + goCounting() == CountingState(),
      CountingState() + onTimer() / countingAction()
    ))

    mainStateMachine theStateMachine;
};

/**********************************************************************************************************************/
class VirtualDeviceTest {
  public:
    VirtualDeviceTest() {}

    /// test the device open and close events
    void testDevOpenClose();

    /// test the timer group system
    void testTimerGroup();

    /// test the register accessor
    void testRegisterAccessor();

  private:
    VirtualTestDevice device;
    friend class DummyDeviceTestSuite;


};

/**********************************************************************************************************************/
class  DummyDeviceTestSuite : public test_suite {
  public:
    DummyDeviceTestSuite() : test_suite("DummyDevice test suite") {
      boost::shared_ptr<VirtualDeviceTest> dummyDeviceTest( new VirtualDeviceTest );

      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testDevOpenClose, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testTimerGroup, dummyDeviceTest ) );
      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testRegisterAccessor, dummyDeviceTest ) );
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
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 0 );
  device.openDev("test.mapp");
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 1 );
  device.closeDev();
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 0 );
  device.openDev("test.mapp");
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 1 );
  device.closeDev();
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 0 );
}

/**********************************************************************************************************************/
void VirtualDeviceTest::testTimerGroup() {
  std::cout << "testTimerGroup" << std::endl;

  // check if name vector is correctly initialised
  BOOST_CHECK( device.timers.names.size() == 2 );
  BOOST_CHECK( device.timers.names[0] == "myTimer" );
  BOOST_CHECK( device.timers.names[1] == "mySecondTimer" );

  // check if getRemaining works if timers are not set
  BOOST_CHECK( device.myTimer.getRemaining() == -1 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device.timers.getRemaining() == -1 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == -1 );

  // set a timer
  device.myTimer.set(10);

  // check if getRemaining gives the right time
  BOOST_CHECK( device.myTimer.getRemaining() == 10 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device.timers.getRemaining() == 10 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == 10 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == -1 );

  // advance by 5
  device.timers.advance(5);

  // check if getRemaining gives the right time
  BOOST_CHECK( device.myTimer.getRemaining() == 5 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device.timers.getRemaining() == 5 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == 5 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == -1 );

  // set second timer
  device.mySecondTimer.set(20);

  // check if getRemaining gives the right time
  BOOST_CHECK( device.myTimer.getRemaining() == 5 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == 20 );
  BOOST_CHECK( device.timers.getRemaining() == 5 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == 5 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == 20 );

  // advance by 1
  device.timers.advance(1);

  // check if getRemaining gives the right time
  BOOST_CHECK( device.myTimer.getRemaining() == 4 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == 19 );
  BOOST_CHECK( device.timers.getRemaining() == 4 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == 4 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == 19 );

  // advance to myTimer
  device.timers.advance("myTimer");

  // check if getRemaining gives the right time
  BOOST_CHECK( device.myTimer.getRemaining() == -1 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == 15 );
  BOOST_CHECK( device.timers.getRemaining() == 15 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == 15 );

  // advance to myTimer which is not set
  device.timers.advance("myTimer");

  // check if getRemaining gives the right time
  BOOST_CHECK( device.myTimer.getRemaining() == -1 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == 15 );
  BOOST_CHECK( device.timers.getRemaining() == 15 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == 15 );

  // set first timer again and advance one time
  device.myTimer.set(20);
  device.timers.advance();

  // check if getRemaining gives the right time
  BOOST_CHECK( device.myTimer.getRemaining() == 5 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device.timers.getRemaining() == 5 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == 5 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == -1 );

  // advance to the end
  device.timers.advance();

  // check if getRemaining gives the right time
  BOOST_CHECK( device.myTimer.getRemaining() == -1 );
  BOOST_CHECK( device.mySecondTimer.getRemaining() == -1 );
  BOOST_CHECK( device.timers.getRemaining() == -1 );
  BOOST_CHECK( device.timers.getRemaining("myTimer") == -1 );
  BOOST_CHECK( device.timers.getRemaining("mySecondTimer") == -1 );

  // check current time
  BOOST_CHECK( device.myTimer.getCurrent() == 30 );
  BOOST_CHECK( device.mySecondTimer.getCurrent() == 30 );
  BOOST_CHECK( device.timers.getCurrent() == 30 );

  // open the device
  device.openDev("test.mapp");

  // set first timer and make it fire, then check if in SomeIntermediateState()
  device.myTimer.set(5);
  device.timers.advance();
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 2 );

  // set first timer again and make it fire, then check if still in SomeIntermediateState()
  device.myTimer.set(5);
  device.timers.advance();
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 2 );

  // set second timer and make it fire, then check if in DeviceOpen()
  device.mySecondTimer.set(5);
  device.timers.advance();
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 1 );

  // go into CountingState() to count how often the timer fires
  device.theStateMachine.process_event(VirtualTestDevice::goCounting());
  device.myTimer.set(1);
  BOOST_CHECK( device.someCounter == 0 );
  device.timers.advance();
  BOOST_CHECK( device.someCounter == 1 );
  device.timers.advance(10);
  BOOST_CHECK( device.someCounter == 11 );

  // close the device
  device.closeDev();
}

/**********************************************************************************************************************/
void VirtualDeviceTest::testRegisterAccessor() {
  std::cout << "testRegisterAccessor" << std::endl;

  // open the device
  device.openDev("test.mapp");

  // test get()
  device._barContents[1][0] = 0;
  BOOST_CHECK( device.someRegister.get() == 0 );
  device._barContents[1][0] = 42;
  BOOST_CHECK( device.someRegister.get() == 42 );

  // test get() with index
  device._barContents[1][1] = 0;
  BOOST_CHECK( device.someRegister.get(1) == 0 );
  device._barContents[1][1] = 120;
  BOOST_CHECK( device.someRegister.get(1) == 120 );

  // test set()
  device.someRegister.set(0);
  BOOST_CHECK( device._barContents[1][0] == 0 );
  device.someRegister.set(33);
  BOOST_CHECK( device._barContents[1][0] == 33 );

  // test set() with index
  device.someRegister.set(0,5);
  BOOST_CHECK( device._barContents[1][5] == 0 );
  device.someRegister.set(99,5);
  BOOST_CHECK( device._barContents[1][5] == 99 );

  // test operator=
  device.someRegister = 3;
  BOOST_CHECK( device._barContents[1][0] == 3 );

  // test operator[] on r.h.s.
  device._barContents[1][0] = 5;
  device._barContents[1][3] = 77;
  BOOST_CHECK( device.someRegister[0] == 5 );
  BOOST_CHECK( device.someRegister[3] == 77 );

  // test operator[] on l.h.s.
  device.someRegister[0] = 666;
  device.someRegister[9] = 999;
  BOOST_CHECK( device._barContents[1][0]== 666 );
  BOOST_CHECK( device._barContents[1][9]== 999 );

  // close the device
  device.closeDev();
}
