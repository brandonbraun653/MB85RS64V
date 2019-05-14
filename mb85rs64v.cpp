/********************************************************************************
 *  File Name:
 *    mb8rs64v.cpp
 *
 *  Description:
 *    Implements the device driver for the Fujitsu FRAM Memory 64kb
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#include "mb85rs64v.hpp"

namespace FRAM::Fujitsu
{
  Chimera::Status_t MB85RS64V::write( const size_t address, const uint8_t *const data, const size_t length )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::read( const size_t address, uint8_t *const data, const size_t length )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::erase( const size_t address, const size_t length )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::writeCompleteCallback( const Chimera::Function::void_func_uint32_t func )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::readCompleteCallback( const Chimera::Function::void_func_uint32_t func )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  bool MB85RS64V::isInitialized()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::attachSPI( const Chimera::SPI::SPIClass_sPtr &spi )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::attachSPI( Chimera::SPI::SPIClass_uPtr spi )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

}  // namespace FRAM::Fujitsu
