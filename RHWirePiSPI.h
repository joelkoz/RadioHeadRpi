// RHWirePiSPI.h
// Author: Joel Kozikowski

#ifndef RHWirePiSPI_h
#define RHWirePiSPI_h

#include <RHGenericDriver.h>

// This is the bit in the SPI address that marks it as a write
#define RH_SPI_WRITE_MASK 0x80

class RHGenericSPI;

/////////////////////////////////////////////////////////////////////
/// \class RHWirePiSPI RHWirePiSPI.h <RHWirePiSPI.h>
/// \brief Base class for a RadioHead driver that uses WiringPi and
/// its SPI library to access the SPI bus to communicate with its transport hardware.
///
class RHWirePiSPI : public RHGenericDriver
{
public:
    /// Constructor
    /// \param[in] slaveSelectPin The controler pin to use to select the desired SPI device. This pin will be driven LOW
    /// during SPI communications with the SPI device that uis iused by this Driver. Pin numbers
    /// are in "wiringPi" format (e.g. default slave select (aka CE0) is 10, which is pin 24 on
    /// the 40 pin header)
    RHWirePiSPI(uint8_t slaveSelectPin = 10, uint8_t spiChannel = 0);


    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    bool init();

    /// Reads a single register from the SPI device
    /// \param[in] reg Register number
    /// \return The value of the register
    uint8_t  spiRead(uint8_t reg);

    /// Writes a single byte to the SPI device
    /// \param[in] reg Register number
    /// \param[in] val The value to write
    /// \return Some devices return a status byte during the first data transfer. This byte is returned.
    ///  it may or may not be meaningfule depending on the the type of device being accessed.
    uint8_t spiWrite(uint8_t reg, uint8_t val);

    /// Reads a number of consecutive registers from the SPI device using burst read mode
    /// \param[in] reg Register number of the first register
    /// \param[in] dest Array to write the register values to. Must be at least len bytes
    /// \param[in] len Number of bytes to read
    /// \return Some devices return a status byte during the first data transfer. This byte is returned.
    ///  it may or may not be meaningfule depending on the the type of device being accessed.
    uint8_t spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len);

    /// Write a number of consecutive registers using burst write mode
    /// \param[in] reg Register number of the first register
    /// \param[in] src Array of new register values to write. Must be at least len bytes
    /// \param[in] len Number of bytes to write
    /// \return Some devices return a status byte during the first data transfer. This byte is returned.
    ///  it may or may not be meaningfule depending on the the type of device being accessed.
    uint8_t spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len);


    void attachInterrupt(uint8_t gpio, void (*function)(void), int mode);
    void detachInterrupt(uint8_t gpio, void (*function)(void));
    
private:
   void selectSlave();
   void unselectSlave();
protected:
    /// The pin number of the Slave Selct pin that is used to select the desired device.
    uint8_t             _slaveSelectPin;
    uint8_t             _spiChannel;

};

#endif
