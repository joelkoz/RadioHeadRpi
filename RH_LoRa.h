// RH_LoRa.h
//
// Definitions for SX1276 based radios in LoRa mode. 
// This code is a mixture of the original RH_RF95 RadioHead code
// as well as LoRa code found in ESP32 based LoRa code for Heltec's WiFi Lora 32 V2 MCU board.
// 
#ifndef RH_LORA_h
#define RH_LORA_h

#include "RHSPIDriver.h"

#if (RH_PLATFORM == RH_PLATFORM_RPI)
// Dragino Raspberry Pi LoRa HAT settings
// using WiringPi numbering scheme...
#define LORA_DEFAULT_SS_PIN     6
#define LORA_DEFAULT_RESET_PIN  0
#define LORA_DEFAULT_DIO0_PIN   7
#else
// Heltec LoRa settings...
#define LORA_DEFAULT_SS_PIN 18
#define LORA_DEFAULT_RESET_PIN  14
#define LORA_DEFAULT_DIO0_PIN   26
#endif


// Max number of octets the LORA Rx/Tx FIFO can hold
#define RH_LORA_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define RH_LORA_MAX_PAYLOAD_LEN RH_LORA_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RH_LORA_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RH_LORA_MAX_MESSAGE_LEN
 #define RH_LORA_MAX_MESSAGE_LEN (RH_LORA_MAX_PAYLOAD_LEN - RH_LORA_HEADER_LEN)
#endif


/*!
 * RegPaConfig
 */
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00 // Default

#define RF_PACONFIG_MAX_POWER_MASK                  0x8F

#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

/*!
 * RegPaDac
 */
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04  // Default


/////////////////////////////////////////////////////////////////////
/// \class RH_LoRa RH_LoRa.h
/// \brief Driver to send and receive unaddressed, unreliable datagrams via a LoRa 
/// capable radio transceiver.
///
/// For Semtech SX1276 chip found in Heltec ESP32 LoRa V2 and Dragino LoRa HAT
///

class RH_LoRa : public RHSPIDriver
{
public:
    RH_LoRa(uint8_t slaveSelectPin=LORA_DEFAULT_SS_PIN, uint8_t DIO0Pin=LORA_DEFAULT_DIO0_PIN, uint8_t resetPin=LORA_DEFAULT_RESET_PIN, RHGenericSPI& spi = hardware_spi);


    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool    init();

    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it wil be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool    available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool    recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// \return true if the message length was valid and it was correctly queued for transmit
    virtual bool    send(const uint8_t* data, uint8_t len);


    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();


    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF95/96/97/98.
    void setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF95/96/97/98.
    void setModeTx();

    void setModeSleep();

    // From Heltec LoRa code...
    void setTxPower(int8_t power, int8_t outputPin);
    void setTxPowerMax(int level);
    bool setFrequency(long frequency);
    void setSpreadingFactor(int sf);
    void setSignalBandwidth(long sbw);
    void setCodingRate4(int denominator);
    void setPreambleLength(long length);
    void setSyncWord(int sw);
    void enableCrc();
    void disableCrc();
    int packetRssi();
    float packetSnr();

protected:
    void explicitHeaderMode();
    void implicitHeaderMode();
    int beginPacket(int implicitHeader = false);
    int endPacket();
    int parsePacket(int size = 0);

    /// This is a low level function to handle the interrupts for one instance of RH_RF95.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    void handleInterrupt();

    /// Examine the revceive buffer to determine whether the message is for this node
    void validateRxBuf();

    /// Clear our local receive buffer
    void clearRxBuf();

private:
    /// Low level interrupt service routine for device connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for device connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for device connected to interrupt 2
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static RH_LoRa*     _deviceForInterrupt[];

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t      _interruptCount;

    /// The configured interrupt pin connected to this instance
    uint8_t             _dio0;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// Number of octets in the buffer
    volatile uint8_t    _bufLen;
    
    /// The receiver/transmitter buffer
    uint8_t             _buf[RH_LORA_MAX_PAYLOAD_LEN];

    /// True when there is a valid message in the buffer
    volatile bool       _rxBufValid;

    /// Pin connected to SX1276 reset pin
    uint8_t _reset;

    int _initCalled = 0;

    int _frequency;
    
};

#endif
