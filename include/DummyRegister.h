/*
 * DummyRegister.h
 *
 *  Created on: Sep 15, 2015
 *      Author: Martin Hierholzer
 */

#ifndef SOURCE_DIRECTORY__INCLUDE_DUMMYREGISTER_H_
#define SOURCE_DIRECTORY__INCLUDE_DUMMYREGISTER_H_

#include <MtcaMappedDevice/MultiplexedDataAccessor.h>

namespace mtca4u { namespace VirtualLab {

  /*********************************************************************************************************************/
  /// Temporary proxy class.
  /// Will be returned in place of l.h.s. references to the register elements
  template<typename T>
  class VirtualRegisterProxy {

    public:

      VirtualRegisterProxy(FixedPointConverter &_fpc, int _nbytes, int32_t &_buffer)
      : fpc(_fpc), nbytes(_nbytes), buffer(_buffer) {}

      /// Implicit type conversion to user type T.
      /// This covers already a lot of operations like arithmetics and comparison
      inline operator T() {
        return fpc.template toCooked<T>(buffer);
      }

      /// assignment operator
      inline VirtualRegisterProxy<T> operator=(T rhs)
      {
        int32_t raw = fpc.toRaw(rhs);
        memcpy(&buffer, &raw, nbytes);
        return *this;
      }

      /// pre-increment operator
      inline VirtualRegisterProxy<T> operator++() {
        T v = fpc.template toCooked<T>(buffer);
        return operator=( v + 1 );
      }

      /// pre-decrement operator
      inline VirtualRegisterProxy<T> operator--() {
        T v = fpc.template toCooked<T>(buffer);
        return operator=( v - 1 );
      }

      /// post-increment operator
      inline T operator++(int) {
        T v = fpc.template toCooked<T>(buffer);
        operator=( v + 1 );
        return v;
      }

      /// post-decrement operator
      inline T operator--(int) {
        T v = fpc.template toCooked<T>(buffer);
        operator=( v - 1 );
        return v;
      }

    private:

      /// fixed point converter to be used for this element
      FixedPointConverter &fpc;

      /// number of bytes per word
      int nbytes;

      /// reference of the raw buffer of this element
      int32_t &buffer;
  };

  /*********************************************************************************************************************/
  /// Temporary proxy class for sequences.
  /// Will be returned by the first [] operator.
  template<typename T>
  class VirtualRegisterSequenceProxy {

    public:

      VirtualRegisterSequenceProxy(FixedPointConverter &_fpc, int _nbytes, int _pitch, int32_t &_buffer)
      : fpc(_fpc), nbytes(_nbytes), pitch(_pitch), buffer(_buffer) {}

      /// Get or set register content by [] operator.
      inline VirtualRegisterProxy<T> operator[](int sample)
      {
        char *basePtr = reinterpret_cast<char*>(&buffer);
        return VirtualRegisterProxy<T>(fpc, nbytes, *(reinterpret_cast<int32_t*>(basePtr + pitch*sample)) );
      }

    private:

      /// fixed point converter to be used for this sequence
      FixedPointConverter &fpc;

      /// number of bytes per word
      int nbytes;

      /// pitch in bytes (distance between samples of the same sequence)
      int pitch;

      /// reference to the raw buffer (first word of the sequence)
      int32_t &buffer;
  };


/*********************************************************************************************************************/
/// Register accessor for single word or 1D array registers
template<typename T, class VirtualDevice>
class DummyRegister {
  public:

    /// "Open" the register: obtain the register information from the mapping file.
    /// Call this function in the overloaded openDev() function.
    void open(VirtualDevice *_dev, std::string module, std::string name)
    {
      dev = _dev;
      dev->_registerMapping->getRegisterInfo(name, registerInfo, module);
      fpc =  FixedPointConverter(registerInfo.reg_width, registerInfo.reg_frac_bits, registerInfo.reg_signed);
    }

    /// Get register content by index.
    inline T get(int index=0)
    {
      return fpc.template toCooked<T>(getElement(index));
    }

    /// Set register content by index.
    inline void set(T value, int index=0)
    {
      getElement(index) = fpc.toRaw(value);
    }

    /// Get or set register content by [] operator.
    inline VirtualRegisterProxy<T> operator[](int index)
    {
      return getProxy(index);
    }

    /// Set register content by = operator.
    inline VirtualRegisterProxy<T> operator=(T rhs)
    {
      set(rhs);
      return getProxy(0);
    }

  protected:

    /// register map information
    RegisterInfoMap::RegisterInfo registerInfo;

    /// pointer to VirtualDevice
    VirtualDevice *dev;

    /// fixed point converter
    FixedPointConverter fpc;

    /// return element
    /// todo: implement range check?
    inline int32_t& getElement(int index) {
      return dev->_barContents[registerInfo.reg_bar][registerInfo.reg_address/sizeof(int32_t) + index];
    }

    /// return a proxy object
    inline VirtualRegisterProxy<T> getProxy(int index) {
      return VirtualRegisterProxy<T>(fpc, sizeof(int32_t), getElement(index));
    }

};

/*********************************************************************************************************************/
/// Register accessor for multiplexed 2D array registers
template<typename T, class VirtualDevice>
class DummyMultiplexedRegister {
  public:

    /// "Open" the register: obtain the register information from the mapping file.
    /// Call this function in the overloaded openDev() function.
    void open(VirtualDevice *_dev, std::string module, std::string name)
    {
      dev = _dev;
      dev->_registerMapping->getRegisterInfo(MULTIPLEXED_SEQUENCE_PREFIX+name, registerInfo, module);

      int i = 0;
      pitch = 0;
      while(true) {
        // obtain register information for sequence
        RegisterInfoMap::RegisterInfo elem;
        std::stringstream sequenceNameStream;
        sequenceNameStream << SEQUENCE_PREFIX << name << "_" << i++;
        try{
          dev->_registerMapping->getRegisterInfo( sequenceNameStream.str(), elem, module );
        }
        catch(MapFileException &) {
          break;
        }
        // create fixed point converter for sequence
        fpc.push_back( FixedPointConverter(elem.reg_width, elem.reg_frac_bits, elem.reg_signed) );
        // store offsets and number of bytes per word
        offsets.push_back(elem.reg_address);
        nbytes.push_back(elem.reg_size);
        // determine pitch
        pitch += elem.reg_size;
      }

      if(fpc.empty()) {
        throw MultiplexedDataAccessorException( "No sequenes found for name \""+name+"\".",
            MultiplexedDataAccessorException::EMPTY_AREA );
      }
    }

    /// Get or set register content by [] operators.
    /// The first [] denotes the sequence (aka. channel number), the second [] indicates the sample inside the sequence.
    /// This means that in a sense the first index is the faster counting index.
    /// Example: myMuxRegister[3][987] will give you the 988th sample of the 4th channel.
    /// todo: implement range check?
    inline VirtualRegisterSequenceProxy<T> operator[](int sequence)
    {
      int8_t *basePtr = reinterpret_cast<int8_t*>(dev->_barContents[registerInfo.reg_bar].data());
      int32_t *seq = reinterpret_cast<int32_t*>(basePtr + (registerInfo.reg_address + offsets[sequence]));
      return VirtualRegisterSequenceProxy<T>(fpc[sequence], nbytes[sequence], pitch, *seq);
    }

  protected:

    /// register map information
    RegisterInfoMap::RegisterInfo registerInfo;

    /// pointer to VirtualDevice
    VirtualDevice *dev;

    /// pointer to fixed point converter
    std::vector<FixedPointConverter> fpc;

    /// offsets in bytes for sequences
    std::vector<int> offsets;

    /// number of bytes per word for sequences
    std::vector<int> nbytes;

    /// pitch in bytes (distance between samples of the same sequence)
    int pitch;

};

}}// namespace mtca4u::VirtualLab

#endif /* SOURCE_DIRECTORY__INCLUDE_DUMMYREGISTER_H_ */
