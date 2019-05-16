/********************************************************************************
 *  File Name:
 *    mb8rs64v.cpp
 *
 *  Description:
 *    Implements the device driver for the Fujitsu FRAM Memory 64kb
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <cstring>

/* Chimera Includes */
#include <Chimera/utilities.hpp>

#include "mb85rs64v.hpp"

namespace FRAM::Fujitsu
{
  MB85RS64V::MB85RS64V() : spi( nullptr ), initialized(false)
  {
    txBuffer.fill( 0 );
    rxBuffer.fill( 0 );
  }

  MB85RS64V::~MB85RS64V()
  {
  }

  Chimera::Status_t MB85RS64V::initialize( Chimera::SPI::Setup *const setup, const Chimera::Hardware::SubPeripheralMode mode )
  {
    using namespace Chimera::SPI;
    using namespace Chimera::Hardware;
    
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !spi )
    {
      result = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      /*------------------------------------------------
      Go ahead and initialize the SPI driver if the user passed in a setup
      ------------------------------------------------*/
      if ( setup )
      {
        Chimera::SPI::Setup tempSetup;
        memcpy( &tempSetup, setup, sizeof( Chimera::SPI::Setup ) );
        result = spi->init( tempSetup );
      }

      /*------------------------------------------------
      Swap the control mode into what's needed for the chip
      ------------------------------------------------*/
      if ( result == Chimera::CommonStatusCodes::OK )
      {
        spi->setChipSelectControlMode( ChipSelectMode::MANUAL );
        spi->setPeripheralMode( SubPeripheral::TXRX, mode );
      }

      initialized = true;
    }

    return result;
  }

  uint32_t MB85RS64V::readID()
  {
    static constexpr uint8_t CMD_LEN = 5;
    uint32_t id = 0u;
    
    if ( initialized )
    {
      /*------------------------------------------------
      Push the buffers to a known state an initialize the transfer
      ------------------------------------------------*/
      txBuffer.fill( 0 );
      rxBuffer.fill( 0 );
      
      txBuffer[ 0 ] = MB85RS64V_OP_RDID;
      
      /*------------------------------------------------
      Perform the transfer
      ------------------------------------------------*/
      spi->setChipSelect( Chimera::GPIO::State::LOW );
      spi->readWriteBytes( txBuffer.data(), rxBuffer.data(), CMD_LEN, 500 );
      
      
      // TODO: Will need to wait for transfer to complete if this is DMA...
      spi->setChipSelect( Chimera::GPIO::State::HIGH );
    
      /*------------------------------------------------
      Pull out the data, offset by a byte due to the OP code. If 
      necessary, swap the byte ording as the FRAM is big endian.
      ------------------------------------------------*/
      memcpy( &id, rxBuffer.data() + 1u, sizeof( id ) );
      
#if defined( CHIMERA_LITTLE_ENDIAN )
      id = Chimera::Utilities::swap( id );
#endif 
    }

    return id;
  }

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
    return initialized;
  }

  Chimera::Status_t MB85RS64V::attachSPI( const Chimera::SPI::SPIClass_sPtr &spi )
  {
    this->spi = spi;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t MB85RS64V::attachSPI( Chimera::SPI::SPIClass_uPtr spi )
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }



}  // namespace FRAM::Fujitsu
