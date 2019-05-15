/********************************************************************************
 *  File Name:
 *    mb85rs64v_tests_stm32f4.cpp
 *
 *  Description:
 *    On device embedded tests for the MB85RS64V FRAM chip. Targeted for the STM32F4
 *    Nucleo development board.
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>

/* Boost Includes */
#include <boost/circular_buffer.hpp>

/* Test Driver Includes */
#include "TinyEmbeddedTest.h"

/* Chimera Includes */
#include "chimeraPort.hpp"
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>

/* Module Under Test */
#include "mb85rs64v.hpp"

static Chimera::SPI::SPIClass_sPtr spi;
static FRAM::Fujitsu::MB85RS64V fram;
static Chimera::SPI::Setup setup;

static void reset_test()
{
  using namespace Chimera::Hardware;
  using namespace Chimera::SPI;
  using namespace Chimera::GPIO;
  
  static bool driverAllocated = false;

  if ( !driverAllocated )
  {
    spi = std::make_shared<Chimera::SPI::SPIClass>();
  }

  memset( &setup, 0, sizeof( Chimera::SPI::Setup ) );

  setup.MOSI.port      = Port::PORTC;
  setup.MOSI.pin       = 12;
  setup.MOSI.alternate = GPIO_AF6_SPI3;

  setup.MISO.port      = Port::PORTC;
  setup.MISO.pin       = 11;
  setup.MISO.alternate = GPIO_AF6_SPI3;

  setup.SCK.port      = Port::PORTC;
  setup.SCK.pin       = 10;
  setup.SCK.alternate = GPIO_AF6_SPI3;

  setup.CS.port = Port::PORTD;
  setup.CS.pin  = 2;

  setup.channel = 3;
}

TEST_GROUP( FRAM_OperationalTests ){};

TEST( FRAM_OperationalTests, initialization )
{
}
