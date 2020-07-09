#include <RHWiringPiSPI.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

RHWiringPiSPI hardware_spi;

RHWiringPiSPI::RHWiringPiSPI(uint8_t spiChannel)
    : 
    RHGenericSPI(),
    _spiChannel(spiChannel)
{
}

void RHWiringPiSPI::begin()
{
    wiringPiSPISetup(_spiChannel, 500000);
    delay(100);
}

uint8_t RHWiringPiSPI::transfer(uint8_t data) {
    wiringPiSPIDataRW(_spiChannel, data, 1);
    return data;
}
