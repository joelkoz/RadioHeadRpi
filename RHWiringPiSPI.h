// RHWiringPiSPI.h
// Author: Joel Kozikowski
#ifndef RHWiringPiSPI_h
#define RHWiringPiSPI_h

#include <RHGenericSPI.h>


/////////////////////////////////////////////////////////////////////
/// \class RHWiringPiSPI RHWiringPiSPI.h <RHWiringPiSPI.h>
/// \brief An SPI driver for RadioHead that uses WiringPi and
/// its SPI library to access the SPI bus to communicate with its 
/// transport hardware. This is a low level driver that does not
/// manage any slave selection pins. That is left to higher 
/// level drivers (most notebly RHSPIDriver)
///
class RHWiringPiSPI : public RHGenericSPI
{
public:
    /// Constructor
    /// \param[in] slaveSelectPin The controler pin to use to select the desired SPI device. This pin will be driven LOW
    /// during SPI communications with the SPI device that uis iused by this Driver. Pin numbers
    /// are in "wiringPi" format (e.g. default slave select (aka CE0) is 10, which is pin 24 on
    /// the 40 pin header)
    RHWiringPiSPI(uint8_t spiChannel = 0);


    /// Initialise the Driver transport hardware and software.
    virtual void begin();
    virtual void begin(char*) { begin(); }

    virtual void end() {}

    virtual uint8_t transfer(uint8_t data);

protected:
    /// The pin number of the Slave Selct pin that is used to select the desired device.
    uint8_t _spiChannel;

};

extern RHWiringPiSPI hardware_spi;

#endif