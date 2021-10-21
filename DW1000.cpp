/**
* @file    DW1000.cpp
* @version 0.0.2
* @author  Rafaella Neofytou (from Matthias Grob & Manuel Stalder - ETH Zürich - 2015)
* @brief   Cpp file of the DW1000 driver module. Changed to fit cav project. 
* 
*/

#include "DW1000.h"
#include "ThisThread.h"

DW1000::DW1000(PinName MOSI, PinName MISO, PinName SCLK, PinName CS) : 
                spi(MOSI, MISO, SCLK), cs(CS)
{
    deselect();                         // Chip must be deselected first
    spi.format(8,0);                    // Setup the spi for standard 8 bit data and SPI-Mode 0 (GPIO5, GPIO6 open circuit or ground on DW1000)
    spi.frequency(5000000);             // with a 1MHz clock rate (worked up to 49MHz in our Test)
    ThisThread::sleep_for(300ms);       //TODO: Needed?
    dw_on();
    
}

void DW1000::dw_on()
{
    resetAll();                         // we do a soft reset of the DW1000 everytime the driver starts
    //Those values are for the 110kbps mode (5, 16MHz, 1024 Symbols) and are quite complete
    writeRegister16(DW1000_AGC_CTRL, 0x04, 0x8870);             //AGC_TUNE1 for 16MHz PRF, for best performance
    writeRegister32(DW1000_AGC_CTRL, 0x0C, 0x2502A907);         //AGC_TUNE2 (Universal - DO NOT WRITE ANY OTHER VALUE)
    writeRegister16(DW1000_AGC_CTRL, 0x12, 0x0035);             //AGC_TUNE3 (Universal - DO NOT WRITE ANY OTHER VALUE)
 
    //DRX_TUNE
    writeRegister16(DW1000_DRX_CONF, 0x02, 0x000A);             //DRX_TUNE0b for 110kbps
    writeRegister16(DW1000_DRX_CONF, 0x04, 0x0087);             //DRX_TUNE1a for 16MHz PRF
    writeRegister16(DW1000_DRX_CONF, 0x06, 0x0064);             //DRX_TUNE1b for 110kbps & > 1024 symbols
    writeRegister32(DW1000_DRX_CONF, 0x08, 0x371A011D);               //PAC size for 2048 symbols preamble
 
    //LDE CONFIGURATION. Default is set to 0x0000
    writeRegister8 (DW1000_LDE_CTRL, 0x0806, 0xD);              //LDE_CFG1 0xD (better performance)
    writeRegister16(DW1000_LDE_CTRL, 0x1806, 0x1607);           //LDE_CFG2 for 16MHz PRF
    
    //TX POWER (When the message is smaller the tx power decreases)
    writeRegister32(DW1000_TX_POWER, 0X0, 0x0E082848);

    //RF_TXCTRL 
    writeRegister8(DW1000_RF_CONF, 0x0B, 0xD8);                 //RF_RXCTRLH for channel 5
    writeRegister32(DW1000_RF_CONF, 0x0C, 0x001E3FE0);          //RF_TXCTRL for channel 5
    
    //TC_PGDELAY 
    writeRegister8 (DW1000_TX_CAL, 0x0B, 0xC0);                 //TC_PGDELAY for channel 5
    
    //FS_PLLTUNE
    writeRegister32 (DW1000_FS_CTRL, 0x07, 0x0800041D);         //FS_PLLCFG for channel 5
    writeRegister8 (DW1000_FS_CTRL, 0x0B, 0xBE);                //FS_PLLTUNE for channel 5
 
    loadLDE();                                                      // important everytime DW1000 initialises/awakes otherwise the LDE algorithm must be turned off or there's receiving malfunction see User Manual LDELOAD on p22 & p158

    // 110kbps CAUTION: a lot of other registers have to be set for an optimized operation on 110kbps
    writeRegister16(DW1000_TX_FCTRL, 1, 0x0800 | 0x0100 | 0x0080);  // use 1024 symbols preamble (0x0800) (previously 2048 - 0x2800), 16MHz pulse repetition frequency (0x0100), 110kbps bit rate (0x0080) see p.69 of DW1000 User Manual
    writeRegister8(DW1000_SYS_CFG, 2, 0x44);                        // enable special receiving option for 110kbps (disable smartTxPower)!! (0x44) see p.64 of DW1000 User Manual [DO NOT enable 1024 byte frames (0x03) becuase it generates disturbance of ranging don't know why...]
 
    writeRegister16(DW1000_TX_ANTD, 0, 16384);                      // set TX and RX Antenna delay to neutral because we calibrate afterwards
    writeRegister16(DW1000_LDE_CTRL, 0x1804, 16384);                // = 2^14 a quarter of the range of the 16-Bit register which corresponds to zero calibration in a round trip (TX1+RX2+TX2+RX1)
 
    writeRegister8(DW1000_SYS_CFG, 3, 0x20);                        // enable auto reenabling receiver after error
  
    writeRegister8(DW1000_AON, 0x0A, 0x00);                         //TODO: Check manual for Aon configuration

}

void DW1000::wakeup_init()
{
    deselect();                         // Chip must be deselected first
    dw_on();
}

/* Reads the device id it should be 0xdeca0130
 */
uint32_t DW1000::getDeviceID()
{
    uint32_t result;
    readRegister(DW1000_DEV_ID, 0, (uint8_t*)&result, 4);
    return result;
}

/* Reads the device EUI
 */
uint64_t DW1000::getEUI()
{
    uint64_t result;
    readRegister(DW1000_EUI, 0, (uint8_t*)&result, 8);
    return result;
}

/* Set your EUI
 */
void DW1000::setEUI(uint64_t EUI) 
{
    writeRegister(DW1000_EUI, 0, (uint8_t*)&EUI, 8);
}

/* Reads the device EUI
 * User Manual p57
 * 8-Bit readings for Voltage and Temperature
 */
float DW1000::getVoltage() 
{
    uint8_t buffer[7] = {0x80, 0x0A, 0x0F, 0x01, 0x00};
    writeRegister(DW1000_RF_CONF, 0x11, buffer, 2);
    writeRegister(DW1000_RF_CONF, 0x12, &buffer[2], 1);
    writeRegister(DW1000_TX_CAL, 0x00, &buffer[3], 1);
    writeRegister(DW1000_TX_CAL, 0x00, &buffer[4], 1);
    readRegister(DW1000_TX_CAL, 0x03, &buffer[5], 2);
    float Voltage = buffer[5] * 0.0057 + 2.3;
    //float Temperature = buffer[6] * 1.13 - 113.0;                 // TODO: getTemperature was always ~35 degree with better formula/calibration
    return Voltage;
}

/* Read System Event Status Register
 */
uint64_t DW1000::getStatus()
{
    return readRegister40(DW1000_SYS_STATUS, 0);
}
 
uint64_t DW1000::getRXTimestamp()
{
    return readRegister40(DW1000_RX_TIME, 0);
}
 
uint64_t DW1000::getTXTimestamp()
{
    return readRegister40(DW1000_TX_TIME, 0);
}

void DW1000::sendString(char* message)
{
    sendFrame((uint8_t*)message, strlen(message) + 1);
}

void DW1000::receiveString(char* message)
{
    readRegister(DW1000_RX_BUFFER, 0, (uint8_t*)message, getFramelength());
}
 
void DW1000::sendFrame(uint8_t* message, uint16_t length)
{
   // if (length >= 1021) length = 1021;                            // check for maximim length a frame can have with 1024 Byte frames [not used, see constructor]
    if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
    writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer
    
    uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
    length += 2;                                                    // including 2 CRC Bytes
    length = ((backup & 0xFC) << 8) | (length & 0x03FF);
    writeRegister16(DW1000_TX_FCTRL, 0, length);
    stopTRX();                                                      // stop receiving
    writeRegister8(DW1000_SYS_CTRL, 0, 0x02);                       // trigger sending process by setting the TXSTRT bit
    startRX();                                                      // enable receiver again
}

void DW1000::sendDelayedFrame(uint8_t* message, uint16_t length, uint64_t TxTimestamp)
{
    //if (length >= 1021) length = 1021;                            // check for maximim length a frame can have with 1024 Byte frames [not used, see constructor]
    if (length >= 125) length = 125;                                // check for maximim length a frame can have with 127 Byte frames
    writeRegister(DW1000_TX_BUFFER, 0, message, length);            // fill buffer
 
    uint8_t backup = readRegister8(DW1000_TX_FCTRL, 1);             // put length of frame
    length += 2;                                                    // including 2 CRC Bytes
    length = ((backup & 0xFC) << 8) | (length & 0x03FF);
    writeRegister16(DW1000_TX_FCTRL, 0, length);

    writeRegister40(DW1000_DX_TIME, 0, TxTimestamp);                //write the timestamp on which to send the message

    stopTRX();                                                      // stop receiving
    writeRegister8(DW1000_SYS_CTRL, 0, 0x02 | 0x04);                // trigger sending process by setting the TXSTRT and TXDLYS bit
    startRX();                                                      // enable receiver again
}
 
void DW1000::startRX()
{
    writeRegister8(DW1000_SYS_CTRL, 0x01, 0x01);                    // start listening for preamble by setting the RXENAB bit
}
 
void DW1000::stopTRX()
{
    writeRegister8(DW1000_SYS_CTRL, 0, 0x40);                       // disable tranceiver go back to idle mode
}

/*
 *      mode: the array and LDE code (OTP/ROM) and LDO tune, and set sleep persist
 *      DWT_PRESRV_SLEEP 0x0100 - preserve sleep
 *      DWT_LOADOPSET    0x0080 - load operating parameter set on wakeup
 *      DWT_CONFIG       0x0040 - download the AON array into the HIF (configuration download)
 *      DWT_LOADEUI      0x0008
 *      DWT_GOTORX       0x0002
 *      DWT_TANDV        0x0001
 *
 *      wake: wake up parameters
 *      DWT_XTAL_EN      0x10 - keep XTAL running during sleep
 *      DWT_WAKE_SLPCNT  0x8 - wake up after sleep count
 *      DWT_WAKE_CS      0x4 - wake up on chip select
 *      DWT_WAKE_WK      0x2 - wake up on WAKEUP PIN
 *      DWT_SLP_EN       0x1 - enable sleep/deep sleep functionality
 */
void DW1000::configure_sleep()
{
    writeRegister16(DW1000_AON, 0x00, 0x01B8); //mode
    writeRegister32(DW1000_AON, 0x06, 0x05);
}

void DW1000::deepsleep()
{
    configure_sleep();
    writeRegister8(DW1000_AON, 0x02, 0x00);                       // disable tranceiver go back to idle mode
    writeRegister8(DW1000_AON, 0x02, 0x02); 
    deselect();
  
}

void DW1000::spi_wakeup()
{
  // ThisThread::sleep_for(30ms);
    DigitalOut spicss(p17);
    uint8_t buffer[22] = {0x00, 0x00, 0x00, 0x00, 0x00};
    if (getDeviceID() == 0)
    {
        spicss = 0; 
        ThisThread::sleep_for(10ms);
        spicss = 1;

        readRegister(0x0, 0x0, &buffer[20], 10); 
       ThisThread::sleep_for(1s);
    }
    writeRegister8(DW1000_AON, 0x02, 0x01); 
}
 
//  PRIVATE Methods ------------------------------------------------------------------------------------

/*  LDELOAD is reset to 0 by default. This needs to be set as part of DW1000 initialisation and before receiver
    enable, if it is important to get timestamp and diagnostic information from received frames
 */
void DW1000::loadLDE()
{   
    //Default values p24 (manual)
    writeRegister16(DW1000_PMSC, 0x0, 0x0301);                        // set clock to XTAL so OTP is reliable
    writeRegister16(DW1000_OTP_IF, 0x06, 0x8000);                   // set LDELOAD bit in OTP
    ThisThread::sleep_for(10ms);
    writeRegister16(DW1000_PMSC, 0, 0x0200);                        // recover to PLL clock
}

void DW1000::resetRX()
{
    writeRegister8(DW1000_PMSC, 3, 0xE0);   // set RX reset
    writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear RX reset
}

void DW1000::resetAll()
{
    writeRegister8(DW1000_PMSC, 0, 0x01);   // set clock to XTAL
    writeRegister8(DW1000_PMSC, 3, 0x00);   // set All reset
    writeRegister8(DW1000_PMSC, 3, 0xF0);   // clear All reset
}


void DW1000::setInterrupt(bool RX, bool TX)
{
    writeRegister16(DW1000_SYS_MASK, 0, RX*0x4000 | TX*0x0080);  // RX good frame 0x4000, TX done 0x0080
}

void DW1000::ISR()
{
    uint64_t status = getStatus();
    if (status & 0x4000)                                            // a frame was received
    {                                          
    
        writeRegister16(DW1000_SYS_STATUS, 0, 0x6F00);              // clearing of receiving status bits
    }
    if (status & 0x80) {                                            // sending complete
        writeRegister8(DW1000_SYS_STATUS, 0, 0xF8);                 // clearing of sending status bits
    }
}

uint16_t DW1000::getFramelength()
{
    uint16_t framelength = readRegister16(DW1000_RX_FINFO, 0);      // get framelength
    framelength = (framelength & 0x03FF) - 2;                       // take only the right bits and subtract the 2 CRC Bytes
    return framelength;
}
 
// SPI Interface ------------------------------------------------------------------------------------
uint8_t DW1000::readRegister8(uint8_t reg, uint16_t subaddress)
{
    uint8_t result;
    readRegister(reg, subaddress, &result, 1);
    return result;
}
 
uint16_t DW1000::readRegister16(uint8_t reg, uint16_t subaddress)
{
    uint16_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 2);
    return result;
}
 
uint64_t DW1000::readRegister40(uint8_t reg, uint16_t subaddress)
{
    uint64_t result;
    readRegister(reg, subaddress, (uint8_t*)&result, 5);
    result &= 0xFFFFFFFFFF;                                 // only 40-Bit
    return result;
}
 
void DW1000::writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer)
{
    writeRegister(reg, subaddress, &buffer, 1);
}
 
void DW1000::writeRegister16(uint8_t reg, uint16_t subaddress, uint16_t buffer)
{
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 2);
}
 
void DW1000::writeRegister32(uint8_t reg, uint16_t subaddress, uint32_t buffer)
{
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 4);
}
 
void DW1000::writeRegister40(uint8_t reg, uint16_t subaddress, uint64_t buffer)
{
    writeRegister(reg, subaddress, (uint8_t*)&buffer, 5);
}
 
void DW1000::readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length)
{
    setupTransaction(reg, subaddress, false);
    for(int i=0; i<length; i++)                             // get data
        buffer[i] = spi.write(0x00);
    deselect();
}
 
void DW1000::writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length)
{
    setupTransaction(reg, subaddress, true);
    for(int i=0; i<length; i++)                             // put data
        spi.write(buffer[i]);
    deselect();
}
 
void DW1000::setupTransaction(uint8_t reg, uint16_t subaddress, bool write)
{
    reg |=  (write * DW1000_WRITE_FLAG);                                        // set read/write flag
    select();
    if (subaddress > 0) {                                                       // there's a subadress, we need to set flag and send second header byte
        spi.write(reg | DW1000_SUBADDRESS_FLAG);
        if (subaddress > 0x7F) {                                                // sub address too long, we need to set flag and send third header byte
            spi.write((uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG); // and 
            spi.write((uint8_t)(subaddress >> 7));
        } else {
            spi.write((uint8_t)subaddress);
        }
    } else {
        spi.write(reg);                                                         // say which register address we want to access
    }
}
 
void DW1000::select()       // always called to start an SPI transmission
{
    cs = 0;                 // set Cable Select pin low to start transmission
}

void DW1000::deselect()     // always called to end an SPI transmission
{
    cs = 1;                 // set Cable Select pin high to stop transmission
}

uint8_t DW1000::getPMSCState()
{
    uint8_t sys_state[4];
    uint8_t pmsc_state;
    readRegister(DW1000_SYS_STATE, 0, sys_state, 32);
    pmsc_state = sys_state[1];
    return pmsc_state;
}