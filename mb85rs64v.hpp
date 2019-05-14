/********************************************************************************
 *  File Name:
 *    mb8rs64v.hpp
 *
 *  Description:
 *    Implements the device driver for the Fujitsu FRAM Memory 64kb. Was originally
 *    developed on the Adafruit breakout board.
 *
 *  Datasheet:
 *    https://cdn-shop.adafruit.com/datasheets/MB85RS64V-DS501-00015-4v0-E.pdf
 *
 *  2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef FUJITSU_FRAM_MEMORY_MB85RS64V_DRIVER_HPP
#define FUJITSU_FRAM_MEMORY_MB85RS64V_DRIVER_HPP

/* C++ Includes */
#include <cstdint>
#include <memory>
#include <limits>

/* Chimera Includes */
#include <Chimera/interface/spi_intf.hpp>
#include <Chimera/modules/memory/device.hpp>
#include <Chimera/types/spi_types.hpp>

namespace FRAM::Fujitsu
{
  /*------------------------------------------------
  Maximum clock frequency for the chip: 20MHz
  ------------------------------------------------*/
  static constexpr size_t MB85RS64V_SPI_MAX_CLK = 20000000;

  /*------------------------------------------------
  Default operating SPI mode. Also supports MODE3
  ------------------------------------------------*/
  static constexpr Chimera::SPI::ClockMode MB85RS64V_SPI_MODE = Chimera::SPI::ClockMode::MODE0;

  /*------------------------------------------------
  Data order expected by the chip
  ------------------------------------------------*/
  static constexpr Chimera::SPI::BitOrder MB85RS64V_BIT_ORDER = Chimera::SPI::BitOrder::MSB_FIRST;

  /*------------------------------------------------
  Device descriptor information. No page/block/sector size
  as the device is byte by byte access like normal SRAM. Very cool.
  ------------------------------------------------*/
  static constexpr Chimera::Modules::Memory::Descriptor MB85RS64V_Descriptor{
    std::numeric_limits<size_t>::max(), /* Page Size */
    std::numeric_limits<size_t>::max(), /* Block Size */
    std::numeric_limits<size_t>::max(), /* Sector Size */
    0u,                                 /* Start Address */
    8192u                               /* End Address: 8kB */
  };

  /*------------------------------------------------
  Power Supply Voltage Operating Range
  ------------------------------------------------*/
  static constexpr float MB85RS64V_MIN_PWR_V = 3.0f;
  static constexpr float MB85RS64V_MAX_PWR_V = 5.5f;

  /*------------------------------------------------
  Typical Current Draw
  ------------------------------------------------*/
  static constexpr float MB85RS64V_TYP_CURRENT_mA  = 1.5f;
  static constexpr float MB85RS64V_STBY_CURRENT_uA = 10.0f;

  /*------------------------------------------------
  Supported OpCodes: Pg.6 of datasheet
  ------------------------------------------------*/
  static constexpr uint8_t MB85RS64V_OP_WREN  = 0x06; /**< Set Write Enable Latch */
  static constexpr uint8_t MB85RS64V_OP_WRDI  = 0x04; /**< Reset Write Enable Latch */
  static constexpr uint8_t MB85RS64V_OP_RDSR  = 0x05; /**< Read Status Register */
  static constexpr uint8_t MB85RS64V_OP_WRSR  = 0x01; /**< Write Status Register */
  static constexpr uint8_t MB85RS64V_OP_READ  = 0x03; /**< Read from Memory */
  static constexpr uint8_t MB85RS64V_OP_WRITE = 0x02; /**< Write to Memory */
  static constexpr uint8_t MB85RS64V_OP_RDID  = 0x9F; /**< Read Device ID */

  class MB85RS64V : public Chimera::Modules::Memory::Device, public Chimera::SPI::SPIAcceptor
  {
  public:
    Chimera::Status_t initialize();

    Chimera::Status_t write( const size_t address, const uint8_t *const data, const size_t length ) final override;

    Chimera::Status_t read( const size_t address, uint8_t *const data, const size_t length ) final override;

    Chimera::Status_t erase( const size_t address, const size_t length ) final override;

    Chimera::Status_t writeCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    Chimera::Status_t readCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    Chimera::Status_t eraseCompleteCallback( const Chimera::Function::void_func_uint32_t func ) final override;

    bool isInitialized() final override;

    Chimera::Status_t attachSPI( const Chimera::SPI::SPIClass_sPtr &spi ) final override;

    Chimera::Status_t attachSPI( Chimera::SPI::SPIClass_uPtr spi ) final override;

  protected:
  private:
  };

  using MB85RS64V_sPtr = std::shared_ptr<MB85RS64V>;
  using MB85RS64V_uPtr = std::unique_ptr<MB85RS64V>;

}  // namespace FRAM::Fujitsu

#endif /* !FUJITSU_FRAM_MEMORY_MB85RS64V_DRIVER_HPP */