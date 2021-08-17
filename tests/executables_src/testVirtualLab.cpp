#include <algorithm>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/make_shared.hpp>
#include <boost/test/included/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>
#include <iostream>
#include <math.h>
#include <tuple>

#include "SignalSink.h"
#include "StateVariableSet.h"
#include "VirtualLabBackend.h"
#include <ChimeraTK/Device.h>
#include <ChimeraTK/parserUtilities.h>

using namespace boost::unit_test_framework;

using namespace ChimeraTK;
using namespace ChimeraTK::VirtualLab;

#define TEST_MAPPING_FILE "test.mapp"
#define TEST_DMAP_FILE "dummies.dmap"

// instantiate template class to get a correct coverage report
template class ChimeraTK::VirtualLab::StateVariableSet<int>;

/**********************************************************************************************************************/
// forward declaration so we can declare it friend of VirtualTestDevice
class VirtualLabTest;

/**********************************************************************************************************************/
/** The VirtualTestDevice is a VirtualDevice implementation to test the
 * framework
 */
class VirtualTestDevice : public VirtualLabBackend<VirtualTestDevice> {
 public:
  friend class VirtualLabTest;

  CONSTRUCTOR(VirtualTestDevice, someCounter(0), readCount(0), writeCount(0), writeMuxedCount(0), write42Count(0),
      readWithFlagCount(0), someRegister(this, "APP0", "SOME_REGISTER"), someMuxedRegister(this, "APP0", "DAQ0_ADCA"),
      myTimer(this), mySecondTimer(this), timers__(this), timers(this), someFlag(false), subCounter(0))
  INIT_SUB_STATE_MACHINE(subMachine);
  END_CONSTRUCTOR

  // Overload open and close to send events on device open and close. For a real
  // VirtualLabBackend, this should usually not be done, as the state of the
  // device driver should not be part of the state machine.
  void open() override {
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
  void closeImpl() override { theStateMachine.process_event(onDeviceClose()); }

  /// states
  DECLARE_STATE(DevClosed);
  DECLARE_STATE(DevOpen);
  DECLARE_LOGGING_STATE(SomeIntermediateState);
  DECLARE_LOGGING_STATE(CountingState);
  DECLARE_LOGGING_STATE(TimerTest);

  /// events
  DECLARE_EVENT(onDeviceOpen);
  DECLARE_EVENT(onDeviceClose);
  DECLARE_EVENT(onTimer);
  DECLARE_EVENT(onSecondTimer);
  DECLARE_EVENT(goCounting);
  DECLARE_EVENT(onRead);
  DECLARE_EVENT(onWrite);
  DECLARE_EVENT(onWriteMuxed);
  DECLARE_EVENT(startSubMachine);
  DECLARE_EVENT(stopSubMachine);
  DECLARE_EVENT(runDoubleAction);
  DECLARE_EVENT(setBothTimers);
  DECLARE_EVENT(goTimerTest);

  /// counting action: increase counter and set timer again
  DECLARE_ACTION(countingAction)
  someCounter++;
  myTimer.set(1 * seconds);
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
  CONNECT_REGISTER_EVENT(onRead, someRegister);
  END_READEVENT_TABLE
  WRITEEVENT_TABLE
  CONNECT_REGISTER_EVENT(onWrite, someRegister);
  CONNECT_REGISTER_EVENT(onWriteMuxed, someMuxedRegister);
  END_WRITEEVENT_TABLE

  /// register accessors
  DECLARE_REGISTER(int, someRegister);
  DECLARE_MUXED_REGISTER(int, someMuxedRegister);

  /// timer group
  DECLARE_TIMER(myTimer, onTimer);
  DECLARE_TIMER(mySecondTimer, onSecondTimer);
  DECLARE_TIMER_GROUP(timers__, myTimer, mySecondTimer);

  /// derived timer group class to make the test class a friend
  class timers_ : public timers___ {
   public:
    friend class VirtualLabTest;
    timers_(dummyDeviceType* _dev) : timers___(_dev){};
  };
  timers_ timers;

  /// action to set timer
  DECLARE_ACTION(doSetTimer)
  myTimer.set(1 * seconds);
  someCounter++;
  END_DECLARE_ACTION

  /// action to set both timers
  DECLARE_ACTION(doSetBothTimers)
  myTimer.set(1 * seconds);
  mySecondTimer.set(100 * seconds);
  END_DECLARE_ACTION

  /// register guard: allow transition if 42 is written
  DECLARE_REGISTER_GUARD(is42Written, value == 42);

  /// guard testing if a flag is set or not
  bool someFlag;
  DECLARE_GUARD(someGuard)
  return someFlag;
  END_DECLARE_GUARD

  /// define a small sub-state machine
  DECLARE_STATE(subInit);
  DECLARE_EVENT(subEvent);
  DECLARE_ACTION(subCount)
  subCounter++;
  END_DECLARE_ACTION
  int subCounter;
  DECLARE_STATE_MACHINE(subMachine, subInit(), (subInit() + subEvent() / subCount()));

  /// action throwing an exception
  DECLARE_EVENT(requestException);
  DECLARE_STATE(awaitException);
  DECLARE_ACTION(throwException)
  throw std::string("This is some exception destroying our state machine!");
  END_DECLARE_ACTION

  /// define the state machine structure
  DECLARE_MAIN_STATE_MACHINE(DevClosed(),
      (
          // =======================================================================================================
          // open and close the device
          // note: an actual virtual device should never distinguish between
          // closed and opened states, since real hardware does not, either!
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
          DevOpen() + onRead()[!someGuard()] / readAction(),
          DevOpen() + onRead()[someGuard()] / readWithFlagAction(),
          DevOpen() + onWrite()[!is42Written()] / writeAction(),
          DevOpen() + onWrite()[is42Written()] / write42Action(),
          DevOpen() + onWriteMuxed() / writeMuxedAction(),

          // test running two actions on a single event (abusing some existing
          // actions...)
          DevOpen() + runDoubleAction() / (readAction(), writeAction()),

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
          awaitException() + onTimer() / throwException()));
};

REGISTER_BACKEND_TYPE(VirtualTestDevice);
/**********************************************************************************************************************/

std::string convertPathRelativeToDmapToAbs(const std::string& mapfileName) {
  std::string dmapDir = ChimeraTK::parserUtilities::extractDirectory(BackendFactory::getInstance().getDMapFilePath());
  std::string absPathToDmapDir = ChimeraTK::parserUtilities::convertToAbsolutePath(dmapDir);
  // the map file is relative to the dmap file location. Convert the relative
  // mapfilename to an absolute path
  return ChimeraTK::parserUtilities::concatenatePaths(absPathToDmapDir, mapfileName);
}

/**********************************************************************************************************************/
class VirtualLabTest {
 public:
  VirtualLabTest()
  : onHistoryLengthChanged_argument(0), onValueNeeded_returnValue(0), onValueNeeded_returnValue_increment(0),
    onValueNeeded_argument(0), onCompute_returnValue(0), onCompute_returnValue_increment(0), onCompute_argument(0) {
    std::map<std::string, std::string> params;
    params["map"] = convertPathRelativeToDmapToAbs(TEST_MAPPING_FILE);
    device = boost::static_pointer_cast<VirtualTestDevice>(VirtualTestDevice::createInstance("0", params));
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
  void onHistoryLengthChanged(VirtualTime time) { onHistoryLengthChanged_argument = time; }
  VirtualTime onHistoryLengthChanged_argument;

  /// callback function for value needed
  double onValueNeeded(VirtualTime time) {
    onValueNeeded_argument = time;
    onValueNeeded_returnValue += onValueNeeded_returnValue_increment;
    return onValueNeeded_returnValue;
  }
  double onValueNeeded_returnValue;
  double onValueNeeded_returnValue_increment;
  VirtualTime onValueNeeded_argument;

  /// callback function to compute states (StateVariableSet)
  int onCompute(VirtualTime time) {
    onCompute_argument = time;
    onCompute_returnValue += onCompute_returnValue_increment;
    return onCompute_returnValue;
  }
  int onCompute_returnValue;
  int onCompute_returnValue_increment;
  VirtualTime onCompute_argument;
};

/**********************************************************************************************************************/
class VirtualLabTestSuite : public test_suite {
 public:
  VirtualLabTestSuite() : test_suite("VirtualLab test suite") {
    boost::shared_ptr<VirtualLabTest> virtualLabTest(new VirtualLabTest);

    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testDevOpenClose, virtualLabTest));
    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testActions, virtualLabTest));
    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testGuards, virtualLabTest));
    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testTimerGroup, virtualLabTest));
    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testReadWriteEvents, virtualLabTest));
    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testSubMachine, virtualLabTest));
    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testSinkSource, virtualLabTest));
    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testThrowException, virtualLabTest));
    add(BOOST_CLASS_TEST_CASE(&VirtualLabTest::testStateVariableSet, virtualLabTest));
  }
};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite(int /*argc*/, char* /*argv*/ []) {
  ChimeraTK::BackendFactory::getInstance().setDMapFilePath(TEST_DMAP_FILE);

  framework::master_test_suite().p_name.value = "VirtualLab test suite";
  framework::master_test_suite().add(new VirtualLabTestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void VirtualLabTest::testDevOpenClose() {
  std::cout << "testDevOpenClose" << std::endl;

  // open and close the device and check the states
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 0);
  device->open();
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 1);
  device->close();
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 0);
  device->open();
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 1);
  device->close();
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 0);

  // open virtual device (= backend) via a Device frontend
  Device frontend;
  frontend.open("DUMMY");
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 1);
  frontend.close();
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 0);
}

/**********************************************************************************************************************/
void VirtualLabTest::testTimerGroup() {
  std::cout << "testTimerGroup" << std::endl;

  // check if name vector is correctly initialised
  BOOST_CHECK(device->timers.names.size() == 2);
  BOOST_CHECK(device->timers.names[0] == "myTimer");
  BOOST_CHECK(device->timers.names[1] == "mySecondTimer");

  // check if getRemaining works if timers are not set
  BOOST_CHECK(device->myTimer.getRemaining() == -1);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
  BOOST_CHECK(device->timers.getRemaining() == -1);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == -1);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);

  // set a timer
  device->myTimer.set(10 * seconds);

  // check if getRemaining gives the right time
  BOOST_CHECK(device->myTimer.getRemaining() == 10 * seconds);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
  BOOST_CHECK(device->timers.getRemaining() == 10 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == 10 * seconds);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);

  // advance by 5 seconds
  device->timers.advance(5 * seconds);

  // check if getRemaining gives the right time
  BOOST_CHECK(device->myTimer.getRemaining() == 5 * seconds);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
  BOOST_CHECK(device->timers.getRemaining() == 5 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == 5 * seconds);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);

  // set second timer
  device->mySecondTimer.set(20 * seconds);

  // check if getRemaining gives the right time
  BOOST_CHECK(device->myTimer.getRemaining() == 5 * seconds);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == 20 * seconds);
  BOOST_CHECK(device->timers.getRemaining() == 5 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == 5 * seconds);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == 20 * seconds);

  // advance by 1
  device->timers.advance(1 * seconds);

  // check if getRemaining gives the right time
  BOOST_CHECK(device->myTimer.getRemaining() == 4 * seconds);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == 19 * seconds);
  BOOST_CHECK(device->timers.getRemaining() == 4 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == 4 * seconds);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == 19 * seconds);

  // advance to myTimer
  device->timers.advance("myTimer");

  // check if getRemaining gives the right time
  BOOST_CHECK(device->myTimer.getRemaining() == -1);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == 15 * seconds);
  BOOST_CHECK(device->timers.getRemaining() == 15 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == -1);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == 15 * seconds);

  // advance to myTimer which is not set
  device->timers.advance("myTimer");

  // check if getRemaining gives the right time
  BOOST_CHECK(device->myTimer.getRemaining() == -1);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == 15 * seconds);
  BOOST_CHECK(device->timers.getRemaining() == 15 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == -1);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == 15 * seconds);

  // set first timer again and advance one time
  device->myTimer.set(20 * seconds);
  device->timers.advance();

  // check if getRemaining gives the right time
  BOOST_CHECK(device->myTimer.getRemaining() == 5 * seconds);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
  BOOST_CHECK(device->timers.getRemaining() == 5 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == 5 * seconds);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);

  // advance to the end
  device->timers.advance();

  // check if getRemaining gives the right time
  BOOST_CHECK(device->myTimer.getRemaining() == -1);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
  BOOST_CHECK(device->timers.getRemaining() == -1);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == -1);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);

  // check current time
  BOOST_CHECK(device->myTimer.getCurrent() == 30 * seconds);
  BOOST_CHECK(device->mySecondTimer.getCurrent() == 30 * seconds);
  BOOST_CHECK(device->timers.getCurrent() == 30 * seconds);

  // open the device
  device->open();

  // set first timer and make it fire, then check if in SomeIntermediateState()
  device->myTimer.set(5 * seconds);
  device->timers.advance();
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 2);

  // set first timer again and make it fire, then check if still in
  // SomeIntermediateState()
  device->myTimer.set(5 * seconds);
  device->timers.advance();
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 2);

  // set second timer and make it fire, then check if in DeviceOpen()
  device->mySecondTimer.set(5 * seconds);
  device->timers.advance();
  BOOST_CHECK(device->theStateMachine.current_state()[0] == 1);

  // go into CountingState() to count how often the timer fires
  device->theStateMachine.process_event(VirtualTestDevice::goCounting());
  device->myTimer.set(1 * seconds);
  BOOST_CHECK(device->someCounter == 0);
  device->timers.advance();
  BOOST_CHECK(device->someCounter == 1);
  device->timers.advance(10 * seconds);
  BOOST_CHECK(device->someCounter == 11);

  // reset device
  device->close();
  device->open();
  device->someCounter = 0;

  // go into timer test mode and check if myTimer is correctly set
  device->theStateMachine.process_event(VirtualTestDevice::goTimerTest());
  BOOST_CHECK(device->myTimer.getRemaining() == 1 * seconds);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
  BOOST_CHECK(device->timers.getRemaining() == 1 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == 1 * seconds);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);
  BOOST_CHECK(device->someCounter == 1);

  // advance to fire myTimer: should set it again
  for(int i = 0; i < 5; i++) {
    device->timers.advance();
    BOOST_CHECK(device->myTimer.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
    BOOST_CHECK(device->timers.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("myTimer") == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);
    BOOST_CHECK(device->someCounter == 2 + i);
  }
  for(int i = 0; i < 3; i++) {
    device->timers.advance("myTimer");
    BOOST_CHECK(device->myTimer.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
    BOOST_CHECK(device->timers.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("myTimer") == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);
    BOOST_CHECK(device->someCounter == 7 + i);
  }
  for(int i = 0; i < 3; i++) {
    device->timers.advance(1 * seconds);
    BOOST_CHECK(device->myTimer.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->mySecondTimer.getRemaining() == -1);
    BOOST_CHECK(device->timers.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("myTimer") == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == -1);
    BOOST_CHECK(device->someCounter == 10 + i);
  }

  // switch to using both timers
  device->theStateMachine.process_event(VirtualTestDevice::setBothTimers());
  BOOST_CHECK(device->myTimer.getRemaining() == 1 * seconds);
  BOOST_CHECK(device->mySecondTimer.getRemaining() == 100 * seconds);
  BOOST_CHECK(device->timers.getRemaining() == 1 * seconds);
  BOOST_CHECK(device->timers.getRemaining("myTimer") == 1 * seconds);
  BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == 100 * seconds);
  BOOST_CHECK(device->someCounter == 12);

  // advance mySecondsTimer, should set and fire myTimer 100 times
  device->someCounter = 0;
  for(int i = 0; i < 3; i++) {
    device->timers.advance("mySecondTimer");
    BOOST_CHECK(device->myTimer.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->mySecondTimer.getRemaining() == 100 * seconds);
    BOOST_CHECK(device->timers.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("myTimer") == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == 100 * seconds);
    BOOST_CHECK(device->someCounter == 100 * (i + 1));
  }

  // advance mySecondsTimer in a different way, should set and fire myTimer 100
  // times
  device->someCounter = 0;
  for(int i = 0; i < 5; i++) {
    device->timers.advance(100 * seconds);
    BOOST_CHECK(device->myTimer.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->mySecondTimer.getRemaining() == 100 * seconds);
    BOOST_CHECK(device->timers.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("myTimer") == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == 100 * seconds);
    BOOST_CHECK(device->someCounter == 100 * (i + 1));
  }

  // just advance 1000 seconds, should set and fire myTimer 1000 times
  device->someCounter = 0;
  for(int i = 0; i < 3; i++) {
    device->timers.advance(1000 * seconds);
    BOOST_CHECK(device->myTimer.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->mySecondTimer.getRemaining() == 100 * seconds);
    BOOST_CHECK(device->timers.getRemaining() == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("myTimer") == 1 * seconds);
    BOOST_CHECK(device->timers.getRemaining("mySecondTimer") == 100 * seconds);
    BOOST_CHECK(device->someCounter == 1000 * (i + 1));
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
  BOOST_CHECK(device->writeCount == 0);
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->writeCount == 1);
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->writeCount == 2);

  // read from register
  BOOST_CHECK(device->readCount == 0);
  device->read(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->readCount == 1);
  device->read(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->readCount == 2);

  // write to muxed register
  device->_registerMapping->getRegisterInfo("AREA_MULTIPLEXED_SEQUENCE_DAQ0_ADCA", elem, "APP0");
  data = 120;
  BOOST_CHECK(device->writeMuxedCount == 0);
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->writeMuxedCount == 1);
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->writeMuxedCount == 2);

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
  BOOST_CHECK(device->write42Count == 0);
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->write42Count == 1);
  device->write(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->write42Count == 2);

  // read from register with the flag set, so the normal guard is passed
  device->someFlag = true;
  device->_registerMapping->getRegisterInfo("SOME_REGISTER", elem, "APP0");
  BOOST_CHECK(device->readWithFlagCount == 0);
  device->read(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->readWithFlagCount == 1);
  device->read(elem.bar, elem.address, &data, sizeof(int));
  BOOST_CHECK(device->readWithFlagCount == 2);

  device->close();
}

/**********************************************************************************************************************/
void VirtualLabTest::testSubMachine() {
  std::cout << "testSubMachine" << std::endl;

  // open device
  device->open();

  // test if events of the submachine while it is not running do any harm
  // (should have no effect)
  BOOST_CHECK(device->subCounter == 0);
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK(device->subCounter == 0);

  // start sub machine
  device->theStateMachine.process_event(VirtualTestDevice::startSubMachine());

  // fire submachine event
  BOOST_CHECK(device->subCounter == 0);
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK(device->subCounter == 1);
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK(device->subCounter == 2);

  // fire event of main machine which is currently not effective
  device->theStateMachine.process_event(VirtualTestDevice::goCounting());
  BOOST_CHECK(device->subCounter == 2);
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK(device->subCounter == 3);

  // fire event to exit submachine
  device->theStateMachine.process_event(VirtualTestDevice::stopSubMachine());
  BOOST_CHECK(device->subCounter == 3);
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK(device->subCounter == 3);

  // enter submachine another time
  device->theStateMachine.process_event(VirtualTestDevice::startSubMachine());
  BOOST_CHECK(device->subCounter == 3);
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK(device->subCounter == 4);
  device->theStateMachine.process_event(VirtualTestDevice::stopSubMachine());
  BOOST_CHECK(device->subCounter == 4);
  device->theStateMachine.process_event(VirtualTestDevice::subEvent());
  BOOST_CHECK(device->subCounter == 4);

  // close device
  device->close();
}

/**********************************************************************************************************************/
void VirtualLabTest::testActions() {
  std::cout << "testActions" << std::endl;

  // open device
  device->open();

  // send event to trigger double action
  BOOST_CHECK(device->readCount == 0);
  BOOST_CHECK(device->writeCount == 0);
  device->theStateMachine.process_event(VirtualTestDevice::runDoubleAction());
  BOOST_CHECK(device->readCount == 1);
  BOOST_CHECK(device->writeCount == 1);
  device->theStateMachine.process_event(VirtualTestDevice::runDoubleAction());
  BOOST_CHECK(device->readCount == 2);
  BOOST_CHECK(device->writeCount == 2);

  // close device
  device->close();
}

/**********************************************************************************************************************/
void VirtualLabTest::testSinkSource() {
  std::cout << "testSinkSource" << std::endl;

  // test constant source
  SignalSink sink(42);
  sink.setMaxHistoryLength(20 * milliseconds);
  BOOST_CHECK(sink.getValue(0 * milliseconds) == 42);
  BOOST_CHECK(sink.getValue(10 * milliseconds) == 42);
  BOOST_CHECK(sink.getValue(42 * milliseconds) == 42);
  BOOST_CHECK(sink.getValue(std::numeric_limits<VirtualTime>::max()) == 42);

  // re-connect with another constant source
  auto constSource = boost::make_shared<ConstantSignalSource>(120);
  sink.connect(boost::static_pointer_cast<SignalSource>(constSource));
  BOOST_CHECK(sink.getValue(0 * milliseconds) == 120); // the sink has no history
  BOOST_CHECK(sink.getValue(10 * milliseconds) == 120);
  BOOST_CHECK(sink.getValue(42 * milliseconds) == 120);
  BOOST_CHECK(sink.getValue(std::numeric_limits<VirtualTime>::max()) == 120);

  // create non-constant source and connect with it
  auto source = boost::make_shared<SignalSource>();
  sink.connect(source);

  // nothing in the buffer yet
  BOOST_CHECK_THROW(sink.getValue(0 * milliseconds), std::runtime_error);

  // feed some values to the source and read from sink
  source->feedValue(0 * milliseconds, 10.);
  source->feedValue(100 * microseconds, 99.);
  source->feedValue(1 * milliseconds, -30.);
  source->feedValue(10 * milliseconds, 666.);

  BOOST_CHECK(sink.getValue(0 * milliseconds) == 10.);
  BOOST_CHECK(sink.getValue(100 * microseconds) == 99.);
  BOOST_CHECK(sink.getValue(1 * milliseconds) == -30.);
  BOOST_CHECK(sink.getValue(10 * milliseconds) == 666.);
  BOOST_CHECK_THROW(sink.getValue(11 * milliseconds), std::runtime_error);

  // add tolerance to the source and test it
  source->setValidityPeriod(2 * milliseconds);
  BOOST_CHECK(sink.getValue(0 * milliseconds) == 10.);
  BOOST_CHECK(sink.getValue(100 * microseconds - 1) == 10.);
  BOOST_CHECK(sink.getValue(100 * microseconds) == 99.);
  BOOST_CHECK(sink.getValue(1 * milliseconds - 1) == 99.);
  BOOST_CHECK(sink.getValue(1 * milliseconds) == -30.);
  BOOST_CHECK(sink.getValue(3 * milliseconds - 1) == -30.);
  BOOST_CHECK_THROW(sink.getValue(3 * milliseconds), std::runtime_error);
  source->feedValue(10 * milliseconds,
      666.); // this value was removed in the last call...
  BOOST_CHECK_THROW(sink.getValue(10 * milliseconds - 1), std::runtime_error);
  source->feedValue(10 * milliseconds,
      666.); // this value was removed in the last call...
  BOOST_CHECK(sink.getValue(10 * milliseconds) == 666.);
  BOOST_CHECK(sink.getValue(11 * milliseconds) == 666.);
  BOOST_CHECK(sink.getValue(12 * milliseconds - 1) == 666.);
  BOOST_CHECK_THROW(sink.getValue(12 * milliseconds), std::runtime_error);

  // check if old values are removed from buffer
  source->feedValue(20 * milliseconds, 123.);
  BOOST_CHECK(sink.getValue(0 * milliseconds) == 10.);
  source->feedValue(20 * milliseconds + 1, 124.);
  BOOST_CHECK_THROW(sink.getValue(0 * milliseconds), ChimeraTK::logic_error);

  // change history length and check it
  sink.setMaxHistoryLength(5 * milliseconds);
  source->feedValue(25 * milliseconds, 125.);
  BOOST_CHECK(sink.getValue(20 * milliseconds) == 123.);
  source->feedValue(25 * milliseconds + 1, 126.);
  BOOST_CHECK_THROW(sink.getValue(20 * milliseconds), ChimeraTK::logic_error);

  // go to large times and check if it is still working
  source->feedValue(100 * days, 127.);
  source->feedValue(100 * days + 5 * milliseconds, 128.);
  BOOST_CHECK(sink.getValue(100 * days) == 127.);
  BOOST_CHECK(sink.getValue(100 * days + 5 * milliseconds) == 128.);
  source->feedValue(100 * days + 5 * milliseconds + 1, 129.);
  BOOST_CHECK_THROW(sink.getValue(100 * days), ChimeraTK::logic_error);

  // test SignalSource's callback function "onHistoryLengthChanged"
  source->setOnHistoryLengthChanged(boost::bind(&VirtualLabTest::onHistoryLengthChanged, this, _1));
  onHistoryLengthChanged_argument = 0;
  sink.setMaxHistoryLength(42 * seconds);
  BOOST_CHECK(onHistoryLengthChanged_argument == 42 * seconds);

  // test SignalSource's callback function when needing a new value. Do this
  // with a fresh source to test also getting the first value with the callback
  // function
  source = boost::make_shared<SignalSource>();
  sink.connect(source);
  source->setCallback(boost::bind(&VirtualLabTest::onValueNeeded, this, _1));

  onValueNeeded_returnValue = 234.567;
  BOOST_CHECK(sink.getValue(1 * seconds) == 234.567);
  BOOST_CHECK(onValueNeeded_argument == 1 * seconds);

  onValueNeeded_returnValue = 345.678;
  BOOST_CHECK(sink.getValue(2 * seconds) == 345.678);
  BOOST_CHECK(onValueNeeded_argument == 2 * seconds);

  // test maximum and huge gaps (more detailed tests are done on the underlying
  // StateVariableSet directly)
  source->setMaximumGap(1 * seconds);
  source->setHugeGap(100 * seconds);

  onValueNeeded_returnValue = 0;
  onValueNeeded_returnValue_increment = 1;
  BOOST_CHECK(sink.getValue(444 * seconds) == 104); // 4 huge gap and 100 normal gaps
  BOOST_CHECK(onValueNeeded_argument == 444 * seconds);
}

/**********************************************************************************************************************/
void VirtualLabTest::testThrowException() {
  std::cout << "testThrowException" << std::endl;

  // prepare redirecting cerr
  struct cerr_redirect {
    cerr_redirect(std::streambuf* new_buffer) : old(std::cerr.rdbuf(new_buffer)) {}

    ~cerr_redirect() { std::cerr.rdbuf(old); }

   private:
    std::streambuf* old;
  };

  // need to create our own device for this test, as it destroys the state
  // machine
  std::map<std::string, std::string> params;
  params["map"] = TEST_MAPPING_FILE;
  auto myDevice = boost::static_pointer_cast<VirtualTestDevice>(VirtualTestDevice::createInstance("", params));
  myDevice->open();

  // go into awaitException state and set the timer (which will trigger the
  // exception action)
  myDevice->theStateMachine.process_event(VirtualTestDevice::requestException());
  myDevice->myTimer.set(1 * seconds);

  // request exception, should print a warning, so redirect cerr for this test
  boost::test_tools::output_test_stream output;
  {
    cerr_redirect guard(output.rdbuf());
    myDevice->timers.advance("myTimer"); // this will trigger the exception
                                         // being caught inside the timer
  }

  BOOST_CHECK(output.is_equal("ERROR in VirtualLabBackend: Exception thrown while processing timer "
                              "event. The state machine is now in an unusable condition. Make sure to "
                              "catch all exceptions in your actions!\n"));

  // close device
  myDevice->close();
}

/**********************************************************************************************************************/
void VirtualLabTest::testStateVariableSet() {
  std::cout << "testStateVariableSet" << std::endl;

  // a simple "set" with a single integer
  StateVariableSet<int> simpleState;

  // check for exception if no initial state was set
  BOOST_CHECK_THROW(simpleState.getState(0), ChimeraTK::logic_error);
  BOOST_CHECK(simpleState.getAllStates().size() == 0);

  // set initial state and get it back
  simpleState.setInitialState(42);
  BOOST_CHECK(simpleState.getAllStates().size() == 1);
  BOOST_CHECK(simpleState.getState(0) == 42);
  BOOST_CHECK(simpleState.getLatestState() == 42);
  BOOST_CHECK(simpleState.getLatestTime() == 0);

  // check for exception (thrown by boost) if state is not available and compute
  // function ist not set
  BOOST_CHECK_THROW(simpleState.getState(1), std::runtime_error);
  BOOST_CHECK(simpleState.getLatestTime() == 0);

  // set validity period to 2 and get state for t=1 again (same as for t=0
  // then), t=2 must still fail
  simpleState.setValidityPeriod(2);
  BOOST_CHECK(simpleState.getState(1) == 42);
  BOOST_CHECK_THROW(simpleState.getState(2), std::runtime_error);
  BOOST_CHECK(simpleState.getLatestTime() == 0);

  // same with bigger validity period
  simpleState.setValidityPeriod(1 * seconds);
  BOOST_CHECK(simpleState.getState(1 * seconds - 1) == 42);
  BOOST_CHECK_THROW(simpleState.getState(1 * seconds), std::runtime_error);
  BOOST_CHECK(simpleState.getLatestTime() == 0);
  BOOST_CHECK(simpleState.getAllStates().size() == 1);

  // set compute function
  simpleState.setComputeFunction(boost::bind(&VirtualLabTest::onCompute, this, _1));

  // obtain a value using the compute function
  onCompute_returnValue = 12345;
  BOOST_CHECK(simpleState.getState(2 * seconds) == 12345);
  BOOST_CHECK(onCompute_argument == 2 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 2 * seconds);
  BOOST_CHECK(simpleState.getAllStates().size() == 1);

  // obtain a value within the validity period of the previous (no compute
  // function called)
  onCompute_returnValue = 666;
  BOOST_CHECK(simpleState.getState(3 * seconds - 1) == 12345);
  BOOST_CHECK(onCompute_argument == 2 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 2 * seconds);

  // go beyond validity period
  BOOST_CHECK(simpleState.getState(3 * seconds) == 666);
  BOOST_CHECK(onCompute_argument == 3 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 3 * seconds);

  // go into past without history length set
  BOOST_CHECK_THROW(simpleState.getState(0), ChimeraTK::logic_error);
  BOOST_CHECK_THROW(simpleState.getState(3 * seconds - 1), ChimeraTK::logic_error);
  BOOST_CHECK(simpleState.getLatestTime() == 3 * seconds);
  BOOST_CHECK(simpleState.getAllStates().size() == 1);

  // set history length and test it
  simpleState.setMaxHistoryLength(10 * seconds);

  onCompute_returnValue = 777;
  BOOST_CHECK(simpleState.getState(4 * seconds) == 777);
  BOOST_CHECK(onCompute_argument == 4 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 4 * seconds);
  BOOST_CHECK(simpleState.getState(3 * seconds) == 666);
  BOOST_CHECK(simpleState.getAllStates().size() == 2);

  onCompute_returnValue = 888;
  BOOST_CHECK(simpleState.getState(13 * seconds) == 888);
  BOOST_CHECK(onCompute_argument == 13 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 13 * seconds);
  BOOST_CHECK(simpleState.getState(3 * seconds) == 666);
  BOOST_CHECK(simpleState.getAllStates().size() == 3);

  onCompute_returnValue = 999;
  simpleState.setValidityPeriod(1);
  BOOST_CHECK(simpleState.getState(13 * seconds + 1) == 999);
  BOOST_CHECK(onCompute_argument == 13 * seconds + 1);
  BOOST_CHECK(simpleState.getLatestTime() == 13 * seconds + 1);
  BOOST_CHECK_THROW(simpleState.getState(3 * seconds), ChimeraTK::logic_error);
  BOOST_CHECK(simpleState.getAllStates().size() == 3);

  // test history length shorter than validity period (a rather pointless
  // configuration...)
  simpleState.setValidityPeriod(1 * seconds);
  simpleState.setMaxHistoryLength(1 * milliseconds);

  onCompute_returnValue = 100;
  BOOST_CHECK(simpleState.getState(100 * seconds) == 100);
  BOOST_CHECK(onCompute_argument == 100 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 100 * seconds);
  BOOST_CHECK_THROW(simpleState.getState(13 * seconds + 1), ChimeraTK::logic_error);

  onCompute_returnValue = 101;
  BOOST_CHECK(simpleState.getState(100 * seconds + 1 * milliseconds - 1) == 100);
  BOOST_CHECK(onCompute_argument == 100 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 100 * seconds);
  BOOST_CHECK(simpleState.getState(100 * seconds) == 100);

  BOOST_CHECK(simpleState.getState(101 * seconds - 1) == 100);
  BOOST_CHECK(onCompute_argument == 100 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 100 * seconds);
  BOOST_CHECK(simpleState.getState(100 * seconds) == 100);

  BOOST_CHECK(simpleState.getState(101 * seconds) == 101);
  BOOST_CHECK(onCompute_argument == 101 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 101 * seconds);
  BOOST_CHECK_THROW(simpleState.getState(100 * seconds), ChimeraTK::logic_error);

  // set maximum gap and test it
  simpleState.setMaximumGap(100 * seconds);
  simpleState.setValidityPeriod(10 * seconds);
  simpleState.setMaxHistoryLength(1000 * seconds);
  onCompute_returnValue = 10;
  onCompute_returnValue_increment = 1; // increment by 1 in onCompute() *before* returning the returnValue;

  BOOST_CHECK(simpleState.getState(201 * seconds) == 11); // single step
  BOOST_CHECK(onCompute_argument == 201 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 201 * seconds);

  BOOST_CHECK(simpleState.getState(401 * seconds) == 13); // two steps
  BOOST_CHECK(onCompute_argument == 401 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 401 * seconds);

  BOOST_CHECK(simpleState.getState(301 * seconds) == 12); // intermediate step was already generated
  BOOST_CHECK(onCompute_argument == 401 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 401 * seconds);

  BOOST_CHECK(simpleState.getState(311 * seconds - 1) == 12); // within validity period of intermediate step
  BOOST_CHECK(onCompute_argument == 401 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 401 * seconds);

  BOOST_CHECK(simpleState.getState(401 * seconds) == 13); // the latest state should be unaffected
  BOOST_CHECK(onCompute_argument == 401 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 401 * seconds);

  BOOST_CHECK(simpleState.getState(311 * seconds) == 14); // insert new step
  BOOST_CHECK(onCompute_argument == 311 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 311 * seconds);

  BOOST_CHECK(simpleState.getState(401 * seconds) == 15); // originally latest state was deleted and will be recomputed
                                                          // here
  BOOST_CHECK(onCompute_argument == 401 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 401 * seconds);

  BOOST_CHECK(simpleState.getState(550 * seconds) == 17); // non-integer number of steps
  BOOST_CHECK(onCompute_argument == 550 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 550 * seconds);

  BOOST_CHECK(simpleState.getState(450 * seconds) == 16); // the intermediate step was generated closer to the older
                                                          // step
  BOOST_CHECK(onCompute_argument == 550 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 550 * seconds);

  // test history length shorter than gap
  simpleState.setMaximumGap(100 * seconds);
  simpleState.setValidityPeriod(10 * seconds);
  simpleState.setMaxHistoryLength(1 * seconds);

  BOOST_CHECK(simpleState.getState(350 * seconds) == 18);
  BOOST_CHECK(onCompute_argument == 350 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 350 * seconds);
  BOOST_CHECK_THROW(simpleState.getState(311 * seconds), ChimeraTK::logic_error);
  BOOST_CHECK(simpleState.getAllStates().size() == 1);

  BOOST_CHECK(simpleState.getState(800 * seconds) == 23);
  BOOST_CHECK(onCompute_argument == 800 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 800 * seconds);
  BOOST_CHECK_THROW(simpleState.getState(700 * seconds), ChimeraTK::logic_error);
  BOOST_CHECK_THROW(simpleState.getState(800 * seconds - 1), ChimeraTK::logic_error);
  BOOST_CHECK(simpleState.getAllStates().size() == 1);

  // test feedState()
  simpleState.setMaximumGap(10 * seconds);
  simpleState.setValidityPeriod(1 * seconds);
  simpleState.setMaxHistoryLength(100 * seconds);

  simpleState.feedState(900 * seconds - 1,
      1234); // simply feeding a state should keep the previous one
  BOOST_CHECK(simpleState.getState(800 * seconds) == 23);
  BOOST_CHECK(simpleState.getState(900 * seconds - 1) == 1234);
  BOOST_CHECK(simpleState.getAllStates().size() == 2);

  simpleState.feedState(901 * seconds, 2345); // old state should be removed
  BOOST_CHECK_THROW(simpleState.getState(800 * seconds), ChimeraTK::logic_error);
  BOOST_CHECK(simpleState.getState(900 * seconds - 1) == 1234);
  BOOST_CHECK(simpleState.getState(901 * seconds) == 2345);
  BOOST_CHECK(simpleState.getAllStates().size() == 2);

  simpleState.feedState(910 * seconds, 3456); // adding a 3rd state
  BOOST_CHECK(simpleState.getState(900 * seconds - 1) == 1234);
  BOOST_CHECK(simpleState.getState(901 * seconds) == 2345);
  BOOST_CHECK(simpleState.getState(910 * seconds) == 3456);
  BOOST_CHECK(simpleState.getAllStates().size() == 3);

  simpleState.feedState(930 * seconds,
      4567); // 4th state with a too big gap (no check performed here)
  BOOST_CHECK_THROW(simpleState.getState(800 * seconds), ChimeraTK::logic_error);
  BOOST_CHECK(simpleState.getState(900 * seconds - 1) == 1234);
  BOOST_CHECK(simpleState.getState(901 * seconds) == 2345);
  BOOST_CHECK(simpleState.getState(910 * seconds) == 3456);
  BOOST_CHECK(simpleState.getState(930 * seconds) == 4567);
  BOOST_CHECK(simpleState.getAllStates().size() == 4);

  simpleState.feedState(905 * seconds,
      666); // add intermediate step, removing future steps
  BOOST_CHECK_THROW(simpleState.getState(800 * seconds), ChimeraTK::logic_error);
  BOOST_CHECK(simpleState.getState(900 * seconds - 1) == 1234);
  BOOST_CHECK(simpleState.getState(901 * seconds) == 2345);
  BOOST_CHECK(simpleState.getState(905 * seconds) == 666);
  BOOST_CHECK(simpleState.getAllStates().size() == 3);
  onCompute_returnValue = 777;
  BOOST_CHECK(simpleState.getState(910 * seconds) == 778);
  BOOST_CHECK(onCompute_argument == 910 * seconds);
  BOOST_CHECK(simpleState.getAllStates().size() == 4);
  BOOST_CHECK(simpleState.getState(930 * seconds) == 780); // this will fill the gap
  BOOST_CHECK(onCompute_argument == 930 * seconds);
  BOOST_CHECK(simpleState.getAllStates().size() == 6);

  // test huge gap
  simpleState.setHugeGap(1000 * seconds);
  simpleState.setMaximumGap(90 * seconds);
  simpleState.setValidityPeriod(10 * seconds);
  simpleState.setMaxHistoryLength(10000 * seconds);

  simpleState.feedState(100800 * seconds, 0);
  onCompute_returnValue = 100;
  onCompute_returnValue_increment = 1; // increment by 1 in onCompute() *before* returning the returnValue;

  BOOST_CHECK(simpleState.getState(100810 * seconds) == 101);
  BOOST_CHECK(onCompute_argument == 100810 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 100810 * seconds);
  BOOST_CHECK(simpleState.getAllStates().size() == 2);

  BOOST_CHECK(simpleState.getState(105000 * seconds) == 117); // 4 huge gaps and 12 normal gaps
  BOOST_CHECK(onCompute_argument == 105000 * seconds);
  BOOST_CHECK(simpleState.getLatestTime() == 105000 * seconds);
  BOOST_CHECK(simpleState.getAllStates().size() == 18);

  for(int i = 0; i < 12; i++) {
    BOOST_CHECK(simpleState.getState(105000 * seconds - i * 90 * seconds) == 117 - i);
  }

  for(int i = 1; i < 5; i++) {
    BOOST_CHECK(simpleState.getState(105000 * seconds - i * 1000 * seconds) == 106 - i);
  }
}
