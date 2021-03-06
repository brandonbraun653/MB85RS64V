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
#include <utility>

/* Chimera Includes */
#include <Chimera/types/event_types.hpp>
#include <Chimera/utilities.hpp>

/* Driver Includes */
#include "mb85rs64v.hpp"

namespace FRAM::Fujitsu
{
  static constexpr size_t addressMask = 0x1FFF;

  /*------------------------------------------------
  Device Driver Functions
  ------------------------------------------------*/
  MB85RS64V::MB85RS64V() : spi( nullptr ), initialized( false )
  {
    txBuffer.fill( 0 );
    rxBuffer.fill( 0 );
  }

  MB85RS64V::~MB85RS64V()
  {
  }

  Chimera::Status_t MB85RS64V::initialize( Chimera::SPI::DriverConfig *const setup )
  {
    using namespace Chimera::SPI;

    Chimera::Status_t result = Chimera::Status::OK;

    if ( !spi || !CSPin )
    {
      result = Chimera::Status::NOT_INITIALIZED;
    }
    else
    {
      /*------------------------------------------------
      Go ahead and initialize the SPI driver if the user passed in a setup
      ------------------------------------------------*/
      if ( setup )
      {
        Chimera::SPI::DriverConfig tempSetup;
        memcpy( &tempSetup, setup, sizeof( tempSetup ) );

        /* Force chip select mode, data transfer width, etc */
        tempSetup.HWInit.csMode   = Chimera::SPI::CSMode::MANUAL;
        tempSetup.HWInit.dataSize = Chimera::SPI::DataSize::SZ_8BIT;
        tempSetup.HWInit.bitOrder = Chimera::SPI::BitOrder::MSB_FIRST;

        /* Protect Clock Mode Operation */
        if ( ( tempSetup.HWInit.clockMode != Chimera::SPI::ClockMode::MODE0 )
             || ( tempSetup.HWInit.clockMode != Chimera::SPI::ClockMode::MODE3 ) )
        {
          tempSetup.HWInit.clockMode = MB85RS64V_SPI_MODE;
        }

        /* Protect Clock Frequency */
        if ( tempSetup.HWInit.clockFreq > MB85RS64V_SPI_MAX_CLK )
        {
          tempSetup.HWInit.clockFreq = MB85RS64V_SPI_MAX_CLK;
        }

        result = spi->init( tempSetup );
      }

      initialized = ( result == Chimera::Status::OK );
    }

    return result;
  }

  uint32_t MB85RS64V::readID()
  {
    static constexpr uint8_t CMD_LEN = 5;
    uint32_t id                      = 0u;

    if ( initialized && Chimera::Threading::LockGuard( *this ).lock() )
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
      setChipSelect( Chimera::GPIO::State::LOW );

      spi->readWriteBytes( txBuffer.data(), rxBuffer.data(), CMD_LEN, 500 );
      spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );

      setChipSelect( Chimera::GPIO::State::HIGH );

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
    uint8_t status                   = 0xFF;

    if ( initialized && Chimera::Threading::LockGuard( *this ).lock() )
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
      setChipSelect( Chimera::GPIO::State::LOW );

      spi->readWriteBytes( txBuffer.data(), rxBuffer.data(), CMD_LEN, 500 );
      spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );

      setChipSelect( Chimera::GPIO::State::HIGH );

      /*------------------------------------------------
      Pull out the data, offset by a byte due to the OP code. If
      necessary, swap the byte ording as the FRAM is big endian.
      ------------------------------------------------*/
      memcpy( &status, rxBuffer.data() + 1u, sizeof( status ) );
    }

    return status;
  }

  Chimera::Status_t MB85RS64V::writeEnable()
  {
    static constexpr uint8_t CMD_LEN = 1;
    Chimera::Status_t result         = Chimera::Status::NOT_AVAILABLE;

    if ( initialized && Chimera::Threading::LockGuard( *this ).lock() )
    {
      setChipSelect( Chimera::GPIO::State::LOW );

      result = spi->writeBytes( &MB85RS64V_OP_WREN, sizeof( MB85RS64V_OP_WREN ), 500 );
      spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );

      setChipSelect( Chimera::GPIO::State::HIGH );
    }

    return result;
  }

  Chimera::Status_t MB85RS64V::writeDisable()
  {
    Chimera::Status_t result = Chimera::Status::NOT_AVAILABLE;

    if ( initialized && Chimera::Threading::LockGuard( *this ).lock() )
    {
      setChipSelect( Chimera::GPIO::State::LOW );

      spi->writeBytes( &MB85RS64V_OP_WRDI, sizeof( MB85RS64V_OP_WRDI ), 500 );
      spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );

      setChipSelect( Chimera::GPIO::State::HIGH );
      result = Chimera::Status::OK;
    }

    return result;
  }

  Chimera::Status_t MB85RS64V::writeStatusRegister( const uint8_t val )
  {
    static constexpr uint8_t CMD_LEN = 2;
    Chimera::Status_t result         = Chimera::Status::NOT_AVAILABLE;

    if ( initialized )
    {
      txBuffer[ 0 ] = MB85RS64V_OP_WRSR;
      txBuffer[ 1 ] = val;

      setChipSelect( Chimera::GPIO::State::LOW );

      spi->writeBytes( txBuffer.data(), CMD_LEN, 500 );
      spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );

      setChipSelect( Chimera::GPIO::State::HIGH );

      result = Chimera::Status::OK;
    }

    return result;
  }

  /*------------------------------------------------
  Access Protected Interface
  ------------------------------------------------*/
  Chimera::Status_t MB85RS64V::writeProtect( const bool state )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::readProtect( const bool state )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  /*------------------------------------------------
  Memory Device Interface
  ------------------------------------------------*/
  Chimera::Status_t MB85RS64V::write( const size_t address, const uint8_t *const data, const size_t length )
  {
    Chimera::Status_t result = Chimera::Status::NOT_AVAILABLE;

    if ( initialized && Chimera::Threading::LockGuard( *this ).lock() )
    {
      static constexpr size_t CMD_LEN = 3;
      result                          = Chimera::Status::OK;

      /*------------------------------------------------
      Upon completion of the previous write cycle, the write enable flag
      is cleared and must be set before another write can be executed.
      ------------------------------------------------*/
      result |= writeEnable();

      txBuffer[ 0 ] = MB85RS64V_OP_WRITE;
      txBuffer[ 1 ] = ( ( address & addressMask ) >> 8 ) & 0xFF;
      txBuffer[ 2 ] = address & 0xFF;

      result |= setChipSelect( Chimera::GPIO::State::LOW );

      result |= spi->writeBytes( txBuffer.data(), CMD_LEN, 500 );
      result |= spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );
      result |= spi->writeBytes( data, length, 500 );
      result |= spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );

      result |= setChipSelect( Chimera::GPIO::State::HIGH );
    }

    return result;
  }

  Chimera::Status_t MB85RS64V::read( const size_t address, uint8_t *const data, const size_t length )
  {
    Chimera::Status_t result = Chimera::Status::OK;

    if ( initialized && Chimera::Threading::LockGuard( *this ).lock() )
    {
      static constexpr size_t CMD_LEN = 3;
      result                          = Chimera::Status::OK;

      txBuffer[ 0 ] = MB85RS64V_OP_READ;
      txBuffer[ 1 ] = ( ( address & addressMask ) >> 8 ) & 0xFF;
      txBuffer[ 2 ] = address & 0xFF;

      result |= setChipSelect( Chimera::GPIO::State::LOW );

      result |= spi->writeBytes( txBuffer.data(), CMD_LEN, 500 );
      result |= spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );
      result |= spi->readBytes( data, length, 500 );
      result |= spi->await( Chimera::Event::Trigger::TRANSFER_COMPLETE, 100 );

      result |= setChipSelect( Chimera::GPIO::State::HIGH );
    }

    return result;
  }

  Chimera::Status_t MB85RS64V::erase( const size_t address, const size_t length )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::writeCompleteCallback( const Chimera::Function::void_func_uint32_t func )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::readCompleteCallback( const Chimera::Function::void_func_uint32_t func )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  bool MB85RS64V::isInitialized()
  {
    return initialized;
  }

  /*------------------------------------------------
  SPI Acceptor Interface
  ------------------------------------------------*/
  Chimera::Status_t MB85RS64V::attachSPI( Chimera::SPI::SPIClass_sPtr &spi )
  {
    this->spi = spi;
    return Chimera::Status::OK;
  }

  Chimera::Status_t MB85RS64V::attachSPI( Chimera::SPI::SPIClass_sPtr &spi, Chimera::SPI::DriverConfig &setup )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::attachSPI( Chimera::SPI::SPIClass_uPtr spi )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::attachCS( Chimera::GPIO::PinInit &CSConfig )
  {
    if ( !CSPin )
    {
      CSPin = std::make_unique<Chimera::GPIO::GPIOClass>();
    }

    return CSPin->init( CSConfig );
  }

  Chimera::Status_t MB85RS64V::attachCS( Chimera::GPIO::GPIOClass_sPtr &CSPin )
  {
    return Chimera::Status::NOT_SUPPORTED;
  }

  Chimera::Status_t MB85RS64V::attachCS( Chimera::GPIO::GPIOClass_uPtr CSPin )
  {
    this->CSPin = std::move( CSPin );
    return Chimera::Status::OK;
  }

  /*------------------------------------------------
  Private class functions
  ------------------------------------------------*/
  Chimera::Status_t MB85RS64V::setChipSelect( const Chimera::GPIO::State value )
  {
    if ( CSPin )
    {
      return CSPin->setState( value );
    }
    else
    {
      return setChipSelect( value );
    }
  }
}  // namespace FRAM::Fujitsu
