#include <RHWirePiSPI.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

typedef unsigned char byte;

void RHWirePiSPI::selectSlave()
{
    digitalWrite(_slaveSelectPin, LOW);
}

void RHWirePiSPI::unselectSlave()
{
    digitalWrite(_slaveSelectPin, HIGH);
}

uint8_t RHWirePiSPI::spiRead(uint8_t reg)
{
    byte spibuf[2];

    selectSlave();
    spibuf[0] = reg & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(_spiChannel, spibuf, 2);
    unselectSlave();

    return spibuf[1];
}

uint8_t RHWirePiSPI::spiWrite(uint8_t reg, uint8_t value)
{
    byte spibuf[2];

    spibuf[0] = reg | 0x80;
    spibuf[1] = value;
    selectSlave();
    wiringPiSPIDataRW(_spiChannel, spibuf, 2);

    unselectSlave();

    return spibuf[0];
}


RHWirePiSPI::RHWirePiSPI(uint8_t slaveSelectPin, uint8_t spiChannel)
    : 
    _slaveSelectPin(slaveSelectPin),
    _spiChannel(spiChannel)
{
}


bool RHWirePiSPI::init()
{
    // Initialise the slave select pin
    pinMode(_slaveSelectPin, OUTPUT);
    digitalWrite(_slaveSelectPin, HIGH);

    wiringPiSPISetup(_spiChannel, 500000);
    delay(100);
    return true;
}


uint8_t RHWirePiSPI::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    selectSlave();
    byte spibuf[1];
    spibuf[0] = reg & 0x7F; // Send the start address with the write mask off
    wiringPiSPIDataRW(_spiChannel, spibuf, 1); 
    status = spibuf[0];
    wiringPiSPIDataRW(_spiChannel, dest, len); 
    unselectSlave();
    ATOMIC_BLOCK_END;
    return status;
}

uint8_t RHWirePiSPI::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    selectSlave();
    byte spibuf[1];
    spibuf[0] = reg | 0x80; // Send the start address with the write mask on
    wiringPiSPIDataRW(_spiChannel, spibuf, 1); 
    status = spibuf[0];

    while (len--) {
        spibuf[0] = *src++;
        wiringPiSPIDataRW(_spiChannel, spibuf, 1); 
    }
    unselectSlave();
    ATOMIC_BLOCK_END;
    return status;
}

void RHWirePiSPI::attachInterrupt(uint8_t gpio, void (*function)(void), int mode)
{
#ifdef RH_LINUX_SPI_DEBUG
	printf("RHLinuxSPI::attachInterrupt() : gpio=%d mode=%d\n", gpio, mode);
#endif
	pinMode(gpio, INPUT);
	if( wiringPiISR(gpio, mode, function) < 0 ){
		printf("RHLinuxSPI::attachInterrupt() : wiringPiIsr isr failed!\n");
	}
}

void RHWirePiSPI::detachInterrupt(uint8_t gpio, void (*function)(void))
{
#ifdef RH_LINUX_SPI_DEBUG
	printf("RHLinuxSPI::detachInterrupt()\n");
#endif
}

