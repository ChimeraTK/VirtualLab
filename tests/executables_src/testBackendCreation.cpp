#define BOOST_TEST_MODULE BackendCreationTest

#include <boost/test/included/unit_test.hpp>
using namespace boost::unit_test_framework;

#include "VirtualLabBackend.h"
#include <ChimeraTK/BackendFactory.h>
using namespace ChimeraTK::VirtualLab;

/** This test is checking the creation and the interplay with the backend
 * factory.
 */

/** First we need a virtual lab backend. The simplest possible one just derrives
 *  so we can instantiate the object.
 */
class SimplestVLBackend : public VirtualLabBackend<SimplestVLBackend> {
public:
  CONSTRUCTOR(SimplestVLBackend)
  END_CONSTRUCTOR

  // we need a state machine with just one state and an empty transistion table
  DECLARE_STATE(Done);
  DECLARE_EVENT(Nothing);
  DECLARE_MAIN_STATE_MACHINE(Done(), (Done() + Nothing()));
};
/// Register the backend type with the factory.
REGISTER_BACKEND_TYPE(SimplestVLBackend);

BOOST_AUTO_TEST_SUITE(BackendCreationTestSuite)

BOOST_AUTO_TEST_CASE(testBackendCreation) {
  /// Set the dmap file
  ChimeraTK::BackendFactory &factory = ChimeraTK::BackendFactory::getInstance();
  factory.setDMapFilePath("dummies.dmap");

  // We need it to evaluate if the object actually is the same or not.
  // Never dereference it, it might be invalid if the shared pointer we got it
  // from goes out of scope!
  ChimeraTK::DeviceBackend *backendRawPointer = 0;

  { // open a scope so the shared_ptr can go out of scope
    // these are share_ptrs of DeviceBackend
    boost::shared_ptr<ChimeraTK::DeviceBackend> simple0A =
        factory.createBackend("SIMPLE0A");
    backendRawPointer = simple0A.get();
  }
  // now get the second dmap entry of the same instance
  boost::shared_ptr<ChimeraTK::DeviceBackend> simple0B =
      factory.createBackend("SIMPLE0B");
  BOOST_CHECK(simple0B.get() == backendRawPointer);

  // get 0A again, just to be sure...
  boost::shared_ptr<ChimeraTK::DeviceBackend> simple0A =
      factory.createBackend("SIMPLE0A");
  BOOST_CHECK(simple0B == simple0A);

  // now check that the instantiation really works, simple 1 must be different
  boost::shared_ptr<ChimeraTK::DeviceBackend> simple1 =
      factory.createBackend("SIMPLE1");
  BOOST_CHECK(simple1 != simple0A);
}

BOOST_AUTO_TEST_SUITE_END()
