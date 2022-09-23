/*
 * common_tempi2c.c
 *
 *  Created on: 22 sept. 2022
 *      Author: BEL-LinPat
 */

#include "common_tempi2c.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "app_assert.h"
#include "limits.h"

#include "common_debug.h"

// SI7021_Config_Settings Si7021 Configuration Settings
#define TMP116_I2C_DEVICE                   (sl_i2cspm_temp_sensor) /**< I2C device used to control the Si7021  */
#define TMP116_I2C_BUS_ADDRESS              0x49               /**< I2C bus address                        */

/* REGISTER: Define */

#define TMP116_REG_TEMP                     0x00    /* Temperature register      (Read only)       */
#define TMP116_REG_CONFIG                   0x01    /* Configuration register    (Read and Write)  */
#define TMP116_REG_LIMITHIGH                0x02    /* Temp. limit High register (Read and Write)  */
#define TMP116_REG_LIMITLOW                 0x03    /* Temp. limit Low register  (Read and Write)  */

#define TMP116_REG_EEPROM_UNLOCK            0x04    /* EEPROM Unlock Register    (Read and Write)  */
#define TMP116_REG_EEPROM_1                 0x05    /* EEPROM1 Register          (Read and Write)  */
#define TMP116_REG_EEPROM_2                 0x06    /* EEPROM2 Register          (Read and Write)  */
#define TMP116_REG_EEPROM_3                 0x07    /* EEPROM3 Register          (Read and Write)  */
#define TMP116_REG_EEPROM_4                 0x08    /* EEPROM4 Register          (Read and Write)  */

#define TMP116_REG_CHIP_ID                  0x0F    /* Device ID Register        (Read only)       */

/* REGISTER: Configuration */

#define TMP116_PIN_ALERT_SRC_TEMPERATURE    0x0000  /* ALERT pin reflects the status of the limit Temp flag */
#define TMP116_PIN_ALERT_SRC_DATA_READY     0x0004  /* ALERT pin reflects the status of the data ready flag */

/* Polarity */
#define TMP116_PIN_ALERT_ACTIVE_LOW         0x0000  /* Polarity of Alert pin will be active LOW    */
#define TMP116_PIN_ALERT_ACTIVE_HIGH        0x0008  /* Polarity of Alert pin will be active HIGH   */

/* Alert pin mode */
#define TMP116_ALERT_MODE_COMPARATOR        0x0010  /* The Limit High and Low are Comparator mode  */
#define TMP116_ALERT_MODE_INTERRUPT         0x0000  /* The Limit High and Low are Interrupt  mode  */

/* Shutdown mode */
#define TMP116_MODE_CONVERSION              0x0000  /* The device maintains a continuous conversion state */
#define TMP116_MODE_SHUTDOWN                0x0400  /* Shutting down all device circuitry                 */
#define TMP116_MODE_ONESHOT                 0x0C00  /* The device selects mode One-shot conversion        */

/* Conversion cycle time (min is 15.5ms),
 * After conversion period, followed by a standby period */
#define TMP116_CYCLE_TIME_15_5ms            0x0000  /* Time between two conversion =>  15.5ms*/
#define TMP116_CYCLE_TIME_125ms             0x0080  /* Time between two conversion => 125 ms */
#define TMP116_CYCLE_TIME_250ms             0x0100  /* Time between two conversion => 250 ms */
#define TMP116_CYCLE_TIME_500ms             0x0180  /* Time between two conversion => 500 ms */
#define TMP116_CYCLE_TIME_1s                0x0200  /* Time between two conversion =>   1 s  */
#define TMP116_CYCLE_TIME_4s                0x0280  /* Time between two conversion =>   4 s  */
#define TMP116_CYCLE_TIME_8s                0x0300  /* Time between two conversion =>   8 s  */
#define TMP116_CYCLE_TIME_16s               0x0380  /* Time between two conversion =>  16 s  */

/* Nomber of conversion in cycle time */
#define TMP116_CONV_1_SAMPLE                0x0000  /*  1 conversion succesive =>  15.5 ms   */
#define TMP116_CONV_8_SAMPLE                0x0020  /*  8 conversion succesive => 125 ms     */
#define TMP116_CONV_32_SAMPLE               0x0040  /* 32 conversion succesive => 500 ms     */
#define TMP116_CONV_64_SAMPLE               0x0060  /* 64 conversion succesive =>   1 s      */

/* --------------------------- Factor Hardware of chipTMP116 ------------------------------- */
/*
 * Conversion Time:  (16 bits)     15.5 ms   =>   0.0078125 °C/bit
 *
 *                         Temp[°C] * FACT   =    ADC value
 * ----------------------------------------------------------------------------------------- */
#define kTMP116_FACT                        (1.0F / 0.0078125F)
#define kTMP116_revFACT                     (       0.0078125F)

static TMP116ConfigStruct sTmpCfgTab[kMaxDevice] = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};

/* -----------------------------------------------------------------------------------------
 * Private functions
   -----------------------------------------------------------------------------------------*/
/*
 * Internal I2C transaction management
 *
 */
static I2C_TransferReturn_TypeDef common_tempi2c_transaction(uint16_t flag,
                                                     uint8_t *writeCmd,
                                                     size_t writeLen,
                                                     uint8_t *readCmd,
                                                     size_t readLen)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = TMP116_I2C_BUS_ADDRESS << 1;
  seq.flags = flag;

  switch (flag) {
    // Send the write command from writeCmd
    case I2C_FLAG_WRITE:
      seq.buf[0].data = writeCmd;
      seq.buf[0].len  = writeLen;

      break;

    // Receive data into readCmd of readLen
    case I2C_FLAG_READ:
      seq.buf[0].data = readCmd;
      seq.buf[0].len  = readLen;

      break;

    // Send the write command from writeCmd
    // and receive data into readCmd of readLen
    case I2C_FLAG_WRITE_READ:
      seq.buf[0].data = writeCmd;
      seq.buf[0].len  = writeLen;

      seq.buf[1].data = readCmd;
      seq.buf[1].len  = readLen;

      break;

    default:
      return i2cTransferUsageFault;
  }

  // Perform the transfer and return status from the transfer
  ret = I2CSPM_Transfer(TMP116_I2C_DEVICE, &seq);

  return ret;
}

/* -----------------------------------------------------------------------------------------
 * Public functions
   -----------------------------------------------------------------------------------------*/

/***************************************************************************//**
 * @brief
 *   Fill in configuration structuer for all tmp116 devices (max 4). Do not send any i2C messages.
 *
 * @note
 *   Do not send any i2C messages.
 *   Use common_tempi2cSetup to send config to devices
 *
 * @param[in] index Device index
 * @param[in] chipAdr I2C device address
 * @param[in] limTempHigh Temp hysteresis high (set error output pin to high)
 * @param[in] limTempLow Temp hysteresis low (set error output pin to low)
 *
 ******************************************************************************/
void common_tempi2cConfig(uint8_t index, uint8_t chipAdr, float limTempHigh, float limTempLow)
{
    app_assert(index < kMaxDevice, "Invalid device index: %d", index);

    sTmpCfgTab[index].ChipAddr = chipAdr;
    sTmpCfgTab[index].ConfigReg = TMP116_PIN_ALERT_SRC_TEMPERATURE |
                                  TMP116_PIN_ALERT_ACTIVE_LOW |
                                  TMP116_ALERT_MODE_COMPARATOR |
                                  TMP116_MODE_CONVERSION |
                                  TMP116_CYCLE_TIME_15_5ms |
                                  TMP116_CONV_1_SAMPLE;
    sTmpCfgTab[index].LastReg = 0;

    sTmpCfgTab[index].HystLimTempHigh = (uint16_t)(limTempHigh * kTMP116_FACT);
    sTmpCfgTab[index].HystLimTempLow = (uint16_t)(limTempLow * kTMP116_FACT);
}

/***************************************************************************//**
 * @brief
 *   Send any i2C configuration messages to device(s)
 *
 * @note
 *   Send config register data and LimHigh and LimLow values
 *
 * @return  true if successful
 ******************************************************************************/
bool common_tempi2cSetup()
{
    bool success = true;
    I2C_TransferReturn_TypeDef ret;
    uint8_t tab[3];

    DEBUG_PIN_I2C_CFG_SET;

    for (int i = 0; i < kMaxDevice; i++)
    {
        if (sTmpCfgTab[i].ChipAddr != 0x00)
        {
            sTmpCfgTab[i].LastReg = TMP116_REG_CONFIG;
            tab[0] = sTmpCfgTab[i].LastReg;
            tab[1] = (uint8_t)(sTmpCfgTab[i].ConfigReg & 0xFF00) >> 8;
            tab[2] = (uint8_t)(sTmpCfgTab[i].ConfigReg & 0x00FF);

            ret = common_tempi2c_transaction(I2C_FLAG_WRITE, tab, 3, NULL, 0);
            if (ret != i2cTransferDone)
            {
                success = false;
                PrintError(ret, "I2C error write : TMP116_REG_CONFIG");
                break;
            }

            sTmpCfgTab[i].LastReg = TMP116_REG_LIMITHIGH;
            tab[0] = sTmpCfgTab[i].LastReg;
            tab[1] = (uint8_t)(sTmpCfgTab[i].HystLimTempHigh & 0xFF00) >> 8;
            tab[2] = (uint8_t)(sTmpCfgTab[i].HystLimTempHigh & 0x00FF);
            ret = common_tempi2c_transaction(I2C_FLAG_WRITE,(uint8_t*)&sTmpCfgTab[i].HystLimTempHigh, 3, NULL, 0);
            if (ret != i2cTransferDone)
            {
                success = false;
                PrintError(ret, "I2C error write : TMP116_REG_LIMITHIGH");
                break;
            }

            sTmpCfgTab[i].LastReg = TMP116_REG_LIMITLOW;
            tab[0] = sTmpCfgTab[i].LastReg;
            tab[1] = (uint8_t)(sTmpCfgTab[i].HystLimTempLow & 0xFF00) >> 8;
            tab[2] = (uint8_t)(sTmpCfgTab[i].HystLimTempLow & 0x00FF);
            ret = common_tempi2c_transaction(I2C_FLAG_WRITE,(uint8_t*)&sTmpCfgTab[i].HystLimTempLow, 3, NULL, 0);
            if (ret != i2cTransferDone)
            {
                success = false;
                PrintError(ret, "I2C error write : TMP116_REG_LIMITLOW");
                break;
            }
        }
    }
    DEBUG_PIN_I2C_CFG_RESET;

    return success;
}

/***************************************************************************//**
 * @brief
 *   Get temperature value from TMP116  device
 *
 * @note
 *   Do not send any i2C messages.
 *   Use common_tempi2cSetup to send config to devices
 *
 * @param[in] index i2C interface number (0...kMaxDevice)
 * @param[in] temp temperature value (not converted)
 *
 * @return True if successful
 ******************************************************************************/
bool common_tempi2cReadTemp(uint8_t index, uint16_t* temp)
{
    bool status = true;
    I2C_TransferReturn_TypeDef ret;
    uint8_t readData[2] = {0,0};

    app_assert(index < kMaxDevice, "Invalid device index: %d", index);

    if (sTmpCfgTab[index].LastReg != TMP116_REG_TEMP)
    {
        DEBUG_PIN_I2C_RW_TEMP_SET;

        uint8_t reg = TMP116_REG_TEMP;

        sTmpCfgTab[index].LastReg = TMP116_REG_TEMP;
        ret = common_tempi2c_transaction(I2C_FLAG_WRITE, (uint8_t*)&reg, 1, NULL, 0);
        if (ret != i2cTransferDone)
        {
            status = false;
            PrintError(ret, "I2C error write : TMP116_REG_CONFIG");
        }
        else
        {
            ret = common_tempi2c_transaction(I2C_FLAG_READ, NULL, 0, readData, 2);
            if (ret != i2cTransferDone)
            {
                status = false;
                PrintError(ret, "I2C error write : TMP116_REG_CONFIG");
            }
        }

        DEBUG_PIN_I2C_RW_TEMP_RESET;
    }
    else
    {
        DEBUG_PIN_I2C_RO_TEMP_SET;

        ret = common_tempi2c_transaction(I2C_FLAG_READ, NULL, 0, readData, 2);
        if (ret != i2cTransferDone)
        {
            status = false;
            PrintError(ret, "I2C error write : TMP116_REG_CONFIG");
        }

        DEBUG_PIN_I2C_RO_TEMP_RESET;
    }

    *temp = (readData[0] << 8) + readData[1];

    return status;
}
