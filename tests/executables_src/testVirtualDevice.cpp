#include <math.h>
#include <boost/test/included/unit_test.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <MtcaMappedDevice/devMap.h>

#include "VirtualDevice.h"

using namespace boost::unit_test_framework;
using namespace mtca4u::VirtualLab;

/**********************************************************************************************************************/
// forward declaration so we can declare it friend
// of TestableDummyDevice.
class DummyDeviceTest;

/**********************************************************************************************************************/
/** The TestableDummyDevice is derived from 
 *  DummyDevice to get access to the protected members.
 *  This is done by declaring DummyDeviceTest as a friend.
 */
class TestableDummyDevice : public VirtualDevice<TestableDummyDevice>
{
    friend class DummyDeviceTest;
};

/**********************************************************************************************************************/
class DummyDeviceTest {
  public:
    DummyDeviceTest() {}

    void test();

  private:
    TestableDummyDevice _dummyDevice;
    friend class DummyDeviceTestSuite;


};

/**********************************************************************************************************************/
class  DummyDeviceTestSuite : public test_suite {
  public:
    DummyDeviceTestSuite() : test_suite("DummyDevice test suite") {
      boost::shared_ptr<DummyDeviceTest> dummyDeviceTest( new DummyDeviceTest );

      add( BOOST_CLASS_TEST_CASE( &DummyDeviceTest::test, dummyDeviceTest ) );
    }};

/**********************************************************************************************************************/
test_suite* init_unit_test_suite( int /*argc*/, char* /*argv*/ [] )
{
  framework::master_test_suite().p_name.value = "AD16 DummyDevice test suite";
  framework::master_test_suite().add(new DummyDeviceTestSuite);

  return NULL;
}

/**********************************************************************************************************************/
void DummyDeviceTest::test() {
  std::cout << "test" << std::endl;

}
