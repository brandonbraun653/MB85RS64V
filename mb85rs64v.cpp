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
  static constexpr size_t addressMask = 0x1FFF;

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

  uint8_t MB85RS64V::readStatus()
  {
    static constexpr uint8_t CMD_LEN = 2;
    uint8_t status = 0xFF;

    if ( initialized )
    {
      /*------------------------------------------------
      Push the buffers to a known state an initialize the transfer
      ------------------------------------------------*/
      txBuffer.fill( 0 );
      rxBuffer.fill( 0 );
      
      txBuffer[ 0 ] = MB85RS64V_OP_RDSR;
      
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
      memcpy( &status, rxBuffer.data() + 1u, sizeof( status ) ); 
    }

    return status;
  }

  Chimera::Status_t MB85RS64V::write( const size_t address, const uint8_t *const data, const size_t length )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !initialized )
    {
      result = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      static constexpr size_t CMD_LEN = 3;
      
      /*------------------------------------------------
      Upon completion of the previous write cycle, the write enable flag 
      is cleared and must be set before another write can be executed.
      ------------------------------------------------*/
      writeEnable();

      txBuffer[ 0 ] = MB85RS64V_OP_WRITE;
      txBuffer[ 1 ] = ( ( address & addressMask ) >> 8 ) & 0xFF;
      txBuffer[ 2 ] = address & 0xFF;
      
      spi->setChipSelect( Chimera::GPIO::State::LOW );
      spi->writeBytes( txBuffer.data(), CMD_LEN, 500 );
      
      // TODO: Will need to wait for transfer to complete if this is DMA...
      
      spi->writeBytes( data, length, 500 );
      
      // TODO: Will need to wait for transfer to complete if this is DMA...
      
      spi->setChipSelect( Chimera::GPIO::State::HIGH );
    }

    return result;
  }

  Chimera::Status_t MB85RS64V::read( const size_t address, uint8_t *const data, const size_t length )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !initialized )
    {
      result = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      static constexpr size_t CMD_LEN = 3;

      txBuffer[ 0 ] = MB85RS64V_OP_READ;
      txBuffer[ 1 ] = ( ( address & addressMask ) >> 8 ) & 0xFF;
      txBuffer[ 2 ] = address & 0xFF;
      
      spi->setChipSelect( Chimera::GPIO::State::LOW );
      spi->writeBytes( txBuffer.data(), CMD_LEN, 500 );
      
      // TODO: Will need to wait for transfer to complete if this is DMA...
      
      spi->readBytes( data, length, 500 );
      
      // TODO: Will need to wait for transfer to complete if this is DMA...
      
      spi->setChipSelect( Chimera::GPIO::State::HIGH );
    }

    return result;
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

  Chimera::Status_t MB85RS64V::writeEnable()
  {
    static constexpr uint8_t CMD_LEN = 1;
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_INITIALIZED;

    if ( initialized )
    {
      spi->setChipSelect( Chimera::GPIO::State::LOW );
      result = spi->writeBytes( &MB85RS64V_OP_WREN, sizeof( MB85RS64V_OP_WREN ), 500 );
      spi->setChipSelect( Chimera::GPIO::State::HIGH );
    }

    return result;
  }
  
  Chimera::Status_t MB85RS64V::writeDisable()
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_INITIALIZED;

    if ( initialized )
    {
      spi->setChipSelect( Chimera::GPIO::State::LOW );
      spi->writeBytes( &MB85RS64V_OP_WRDI, sizeof( MB85RS64V_OP_WRDI ), 500 );
      spi->setChipSelect( Chimera::GPIO::State::HIGH );
      result = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  Chimera::Status_t MB85RS64V::writeStatusRegister( const uint8_t val )
  {
    static constexpr uint8_t CMD_LEN = 2;
    Chimera::Status_t result = Chimera::CommonStatusCodes::NOT_INITIALIZED;

    if ( initialized )
    {
      txBuffer[ 0 ] = MB85RS64V_OP_WRSR;
      txBuffer[ 1 ] = val;

      spi->setChipSelect( Chimera::GPIO::State::LOW );
      spi->writeBytes( txBuffer.data(), CMD_LEN, 500 );

      // TODO: Will need to wait for transfer to complete if this is DMA...
      spi->setChipSelect( Chimera::GPIO::State::HIGH );

      result = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }


}  // namespace FRAM::Fujitsu
