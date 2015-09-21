/*
 * performance_test.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: Martin Hierholzer
 */

#include "VirtualLabBackend.h"

#include <boost/random/uniform_int.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/timer.hpp>

using namespace mtca4u::VirtualLab;

/**********************************************************************************************************************/
/** The VirtualTestDevice is a VirtualDevice implementation to test the framework
 */
class VirtualTestDevice : public VirtualLabBackend<VirtualTestDevice>
{
  public:

    CONSTRUCTOR(VirtualTestDevice,
      bigPlain(this,"PERFTEST","BIGPLAIN"),
      someMuxedRegister(this,"APP0","DAQ0_ADCA"),
      myTimer(this),
      timers(this),
      currentOffset(0),
      readNext(0) )
    END_CONSTRUCTOR

    virtual ~VirtualTestDevice() {}

    /// states
    DECLARE_STATE(DevIdle)

    /// events
    DECLARE_EVENT(onTimer)

    /// counting action: increase counter and set timer again
    DECLARE_ACTION(fillingAction)
        dev->bigPlain[dev->currentOffset] = dev->readNext;
        dev->currentOffset++;
        dev->myTimer.set(1);
    END_DECLARE_ACTION

    /// register accessors
    DECLARE_REGISTER(int, bigPlain);
    DECLARE_MUXED_REGISTER(int, someMuxedRegister);

    /// timer group
    DECLARE_TIMER(myTimer, onTimer)
    DECLARE_TIMER_GROUP(timers, myTimer)

    /// pointer inside the buffer
    int currentOffset;

    /// buffer to read data from in the next daq event
    int32_t readNext;

    /// define the state machine structure
    DECLARE_MAIN_STATE_MACHINE(DevIdle(), (
      // =======================================================================================================

      // handle filling of register by timer
      DevIdle() + onTimer() / fillingAction()
    ))

    /// allow access to _barContent
    friend int main();
};


int main() {


  std::list<std::string>       para;
  para.push_back(std::string("test.mapp"));
  VirtualTestDevice     dev(".","virtualDevice",para);

  dev.open();

  // prepare random number generator
  boost::mt11213b rng;
  boost::uniform_int<> w16(-32768,32767);

  // prepare a buffer
  const size_t nel = 10000000;  // 10 million
  int32_t *buffer = new int32_t[nel];

  // prepare timers to measure the wall time
  boost::timer theTimer;
  double nullTime = 0;
  double rawTime = 0;
  double accessorTime = 0;
  double statemachineTime = 0;

  // run multiple iterations to increase precision
  const int niter = 10;
  for(int iter=0; iter<niter; iter++) {
    double sum;

    //
    // "null" benchmark: just the surrounding stuff to get the offset (for comparison)
    //
    theTimer.restart();

    // fill the buffer
    for(unsigned int i=0; i<nel; i++) {
      int val = w16(rng);
      buffer[i] = val;
    }

    // compute sum
    sum = 0.;
    for(unsigned int i=0; i<nel; i++) sum += buffer[i];
    std::cout << "iter = " << iter << "n   sum = " << sum << "\r" << std::flush;

    nullTime += theTimer.elapsed();

    //
    // benchmark direct barContents access (for comparison)
    //
    theTimer.restart();

    // fill the buffer
    for(unsigned int i=0; i<nel; i++) {
      int val = w16(rng);
      buffer[i] = val;
    }

    // copy to register using raw write
    for(unsigned int i=0; i<nel; i++) dev._barContents[2][i] = buffer[i];

    // compute sum
    sum = 0.;
    for(unsigned int i=0; i<nel; i++) sum += dev._barContents[2][i];
    std::cout << "iter = " << iter << "r   sum = " << sum << "\r" << std::flush;

    rawTime += theTimer.elapsed();

    //
    // benchmark register accessor
    //
    theTimer.restart();

    // fill the buffer
    for(unsigned int i=0; i<nel; i++) {
      int val = w16(rng);
      buffer[i] = val;
    }

    // copy to register using accessor
    for(unsigned int i=0; i<nel; i++) dev.bigPlain[i] = buffer[i];

    // compute sum
    sum = 0.;
    for(unsigned int i=0; i<nel; i++) sum += dev._barContents[2][i];
    std::cout << "iter = " << iter << "a   sum = " << sum << "\r" << std::flush;

    accessorTime += theTimer.elapsed();

    //
    // benchmark state machine + register accessor
    //
    theTimer.restart();

    // set the virtual timer and the register offset pointer
    dev.myTimer.set(1);
    dev.currentOffset = 0;

    // fill the buffer
    for(unsigned int i=0; i<nel; i++) {
      int val = w16(rng);
      buffer[i] = val;
    }

    // copy to register using timer events and the register accessor
    for(unsigned int i=0; i<nel; i++) {
      dev.readNext = buffer[i];
      dev.timers.advance(1);
    }

    // compute sum
    sum = 0.;
    for(unsigned int i=0; i<nel; i++) sum += dev._barContents[2][i];
    std::cout << "iter = " << iter << "s   sum = " << sum << "\r" << std::flush;

    statemachineTime += theTimer.elapsed();
}

  std::cout << "Performed " << nel*niter << " writes each." << std::endl;
  std::cout << "CPU time used using no write: " << nullTime << " seconds" << std::endl;
  std::cout << "CPU time used using raw write: " << rawTime << " seconds" << std::endl;
  std::cout << "CPU time used using accessor write: " << accessorTime << " seconds" << std::endl;
  std::cout << "CPU time used using state machine virtual timer and accessor write: " << statemachineTime << " seconds" << std::endl;

  return 0;
}
