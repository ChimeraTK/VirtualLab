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

#define TEST_MAPPING_FILE "test.mapp"

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

    VirtualTestDevice(std::string host, std::string instance, std::list< std::string > parameters) :
      VirtualDevice(host,instance,parameters),
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

    virtual void open() {
      VirtualDevice::open();

      // send onDeviceOpen event
      theStateMachine.process_event(onDeviceOpen());

      // setup registers
      someRegister.open(this,"APP0","SOME_REGISTER");
      someMuxedRegister.open(this,"APP0","DAQ0_ADCA");
    }

    /// on device close: fire the device-close event
    virtual void close() {
      VirtualDevice::close();
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
    VirtualDeviceTest()
    : device(".",TEST_MAPPING_FILE,std::list<std::string>())
    {}

    /// test the device open and close events
    void testDevOpenClose();

    /// test the timer group system
    void testTimerGroup();

    /// test the register accessor
    void testRegisterAccessor();

    /// test the register accessor
    void testMuxedRegisterAccessor();

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
      add( BOOST_CLASS_TEST_CASE( &VirtualDeviceTest::testMuxedRegisterAccessor, dummyDeviceTest ) );
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
  device.open();
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 1 );
  device.close();
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 0 );
  device.open();
  BOOST_CHECK( device.theStateMachine.current_state()[0] == 1 );
  device.close();
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
  device.open();

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
  device.close();
}

/**********************************************************************************************************************/
void VirtualDeviceTest::testRegisterAccessor() {
  std::cout << "testRegisterAccessor" << std::endl;

  // open the device
  device.open();

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
  device.someRegister[1] = 111;
  device.someRegister[2] = 222;
  device.someRegister[3] = 333;
  device.someRegister[4] = 444;
  BOOST_CHECK( device._barContents[1][1]== 111 );
  BOOST_CHECK( device._barContents[1][2]== 222 );
  BOOST_CHECK( device._barContents[1][3]== 333 );
  BOOST_CHECK( device._barContents[1][4]== 444 );

  // test increment and decrement operators
  BOOST_CHECK( device.someRegister[1]++ == 111 );
  BOOST_CHECK( device.someRegister[2]-- == 222 );
  BOOST_CHECK( ++device.someRegister[3] == 334 );
  BOOST_CHECK( --device.someRegister[4] == 443 );
  BOOST_CHECK( device._barContents[1][1]== 112 );
  BOOST_CHECK( device._barContents[1][2]== 221 );
  BOOST_CHECK( device._barContents[1][3]== 334 );
  BOOST_CHECK( device._barContents[1][4]== 443 );

  // close the device
  device.close();
}

/**********************************************************************************************************************/
void VirtualDeviceTest::testMuxedRegisterAccessor() {
  std::cout << "testMuxedRegisterAccessor" << std::endl;

  // open the device
  device.open();

  // since our register does not have a fixed type, we use this union/struct to fill the bar content directly
  // the packed attribute prevents the compiler from adding a padding between the struct fields
  union _mixedReg {
    struct _cooked {
      int32_t r0;
      int16_t r1;
      int16_t r2;
      int8_t r3;
      int8_t r4;
      int32_t r5;
      int16_t r6;
      int32_t r7;
      int32_t r8;
      int32_t r9;
      int32_t r10;
      int32_t r11;
      int32_t r12;
      int32_t r13;
      int32_t r14;
      uint32_t r15;
    } __attribute__((packed)) cooked;
    int32_t raw[13];
  } mixedReg;

  int pitch = sizeof(mixedReg.raw);

  // fill the register
  mixedReg.cooked.r0 = 42;
  mixedReg.cooked.r1 = 120;
  mixedReg.cooked.r2 = 222;
  mixedReg.cooked.r3 = -110;
  mixedReg.cooked.r4 = 1;
  mixedReg.cooked.r5 = 33;
  mixedReg.cooked.r6 = 6;
  mixedReg.cooked.r7 = 7;
  mixedReg.cooked.r8 = 8;
  mixedReg.cooked.r9 = 9;
  mixedReg.cooked.r10 = 10;
  mixedReg.cooked.r11 = 11;
  mixedReg.cooked.r12 = 12;
  mixedReg.cooked.r13 = 13;
  mixedReg.cooked.r14 = 14;
  mixedReg.cooked.r15 = 15;
  for(int i=0; i<13; i++) device._barContents[0xD][i] = mixedReg.raw[i];

  // test the test, to be sure the union is not going wrong
  BOOST_CHECK( (int)((char*)&(mixedReg.cooked.r5) - (char*)&(mixedReg.cooked.r0)) == 10);
  BOOST_CHECK( (int)((char*)&(mixedReg.cooked.r10) - (char*)&(mixedReg.cooked.r0)) == 28);
  BOOST_CHECK( (int)((char*)&(mixedReg.cooked.r15) - (char*)&(mixedReg.cooked.r0)) == pitch-4);
  BOOST_CHECK( device._barContents[0xD][0] == 42 );
  BOOST_CHECK( device._barContents[0xD][1] == 120 + 0x10000 * 222 );
  BOOST_CHECK( device._barContents[0xD][9] == 12 );
  BOOST_CHECK( device._barContents[0xD][12] == 15 );

  mixedReg.cooked.r0 = 1;
  mixedReg.cooked.r1 = 11;
  mixedReg.cooked.r2 = 22;
  mixedReg.cooked.r3 = 33;
  mixedReg.cooked.r4 = 0;
  mixedReg.cooked.r5 = 55;
  mixedReg.cooked.r6 = 66;
  mixedReg.cooked.r7 = 77;
  mixedReg.cooked.r8 = 88;
  mixedReg.cooked.r9 = 99;
  mixedReg.cooked.r10 = 100;
  mixedReg.cooked.r11 = 111;
  mixedReg.cooked.r12 = 222;
  mixedReg.cooked.r13 = 333;
  mixedReg.cooked.r14 = 444;
  mixedReg.cooked.r15 = 555;
  for(int i=0; i<13; i++) device._barContents[0xD][pitch/4+i] = mixedReg.raw[i];

  // test the test, to be sure the union is not going wrong
  BOOST_CHECK( device._barContents[0xD][pitch/4+0] == 1 );
  BOOST_CHECK( device._barContents[0xD][pitch/4+1] == 11 + 0x10000 * 22 );
  BOOST_CHECK( device._barContents[0xD][pitch/4+9] == 222 );
  BOOST_CHECK( device._barContents[0xD][pitch/4+12] == 555 );

  // fill the rest of the register (has 1048576 elements in total in the mapp file)
  for(int i = 2; i < 1048576/16; i++) {
    mixedReg.cooked.r0 = i + 0;
    mixedReg.cooked.r1 = i + 1;
    mixedReg.cooked.r2 = i + 2;
    mixedReg.cooked.r3 = i + 3;
    mixedReg.cooked.r4 = i + 4;
    mixedReg.cooked.r5 = i + 5;
    mixedReg.cooked.r6 = i + 6;
    mixedReg.cooked.r7 = i + 7;
    mixedReg.cooked.r8 = i + 8;
    mixedReg.cooked.r9 = i + 9;
    mixedReg.cooked.r10 = i + 10;
    mixedReg.cooked.r11 = i + 11;
    mixedReg.cooked.r12 = i + 12;
    mixedReg.cooked.r13 = i + 13;
    mixedReg.cooked.r14 = i + 14;
    mixedReg.cooked.r15 = i + 15;
    for(int k=0; k<13; k++) device._barContents[0xD][i*(pitch/4)+k] = mixedReg.raw[k];
  }

  // test reading by [][] operator
  BOOST_CHECK( device.someMuxedRegister[0][0] == 42 );
  BOOST_CHECK( device.someMuxedRegister[1][0] == 120 );
  BOOST_CHECK( device.someMuxedRegister[2][0] == 222 );
  BOOST_CHECK( device.someMuxedRegister[3][0] == -110 );
  BOOST_CHECK( device.someMuxedRegister[4][0] == 1 );
  BOOST_CHECK( device.someMuxedRegister[5][0] == 33 );
  BOOST_CHECK( device.someMuxedRegister[6][0] == 6 );
  BOOST_CHECK( device.someMuxedRegister[7][0] == 7 );
  BOOST_CHECK( device.someMuxedRegister[8][0] == 8 );
  BOOST_CHECK( device.someMuxedRegister[9][0] == 9 );
  BOOST_CHECK( device.someMuxedRegister[10][0] == 10 );
  BOOST_CHECK( device.someMuxedRegister[11][0] == 11 );
  BOOST_CHECK( device.someMuxedRegister[12][0] == 12 );
  BOOST_CHECK( device.someMuxedRegister[13][0] == 13 );
  BOOST_CHECK( device.someMuxedRegister[14][0] == 14 );
  BOOST_CHECK( device.someMuxedRegister[15][0] == 15 );

  BOOST_CHECK( device.someMuxedRegister[0][1] == 1 );
  BOOST_CHECK( device.someMuxedRegister[1][1] == 11 );
  BOOST_CHECK( device.someMuxedRegister[2][1] == 22 );
  BOOST_CHECK( device.someMuxedRegister[3][1] == 33 );
  BOOST_CHECK( device.someMuxedRegister[4][1] == 0 );
  BOOST_CHECK( device.someMuxedRegister[5][1] == 55 );
  BOOST_CHECK( device.someMuxedRegister[6][1] == 66 );
  BOOST_CHECK( device.someMuxedRegister[7][1] == 77 );
  BOOST_CHECK( device.someMuxedRegister[8][1] == 88 );
  BOOST_CHECK( device.someMuxedRegister[9][1] == 99 );
  BOOST_CHECK( device.someMuxedRegister[10][1] == 100 );
  BOOST_CHECK( device.someMuxedRegister[11][1] == 111 );
  BOOST_CHECK( device.someMuxedRegister[12][1] == 222 );
  BOOST_CHECK( device.someMuxedRegister[13][1] == 333 );
  BOOST_CHECK( device.someMuxedRegister[14][1] == 444 );
  BOOST_CHECK( device.someMuxedRegister[15][1] == 555 );

  for(int i = 2; i < 1048576/16; i++) {
    for(int k = 0; k<16; k++) {
      int expectedValue = i + k;
      if(k == 1 || k == 2 || k == 6) {  // 16 bit
        expectedValue = *(reinterpret_cast<int16_t*>(&expectedValue));
      }
      else if(k == 3) {  // 8 bit
        expectedValue = *(reinterpret_cast<int8_t*>(&expectedValue));
      }
      else if(k == 4) {  // 1 bit
        expectedValue = expectedValue & 0x1;
      }
      else if(k == 7) {  // 24 bit
        expectedValue = expectedValue & 0xFFFFFF;
      }
      std::stringstream message;
      message << "someMuxedRegister[" << k << "][" << i << "] == " << device.someMuxedRegister[k][i] << " but " << expectedValue << " expected.";
      BOOST_CHECK_MESSAGE( device.someMuxedRegister[k][i] == expectedValue, message.str() );
    }
  }

  std::cerr << std::flush;
  std::cout << std::flush;

  // close the device
  device.close();
}
