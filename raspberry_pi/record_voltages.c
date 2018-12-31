
#include <bcm2835.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "record_voltages.h"

#define TIMING_PIN 2

int init_adc(double t) {
  uint8_t id;

  if (!bcm2835_init())
    return 1;

  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);    // default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);                 // default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256); // default

  bcm2835_gpio_fsel(SPICS, BCM2835_GPIO_FSEL_OUTP); //
  bcm2835_gpio_write(SPICS, HIGH);
  bcm2835_gpio_fsel(DRDY, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(DRDY, BCM2835_GPIO_PUD_UP);
  // ADS1256_WriteReg(REG_MUX,0x01);
  // ADS1256_WriteReg(REG_ADCON,0x20);
  // ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_15SPS);
  id = ADS1256_ReadChipID();
  if (id != 3) {
    printf("Error, ASD1256 Chip ID = 0x%d\r\n", (int)id);
    return -1;
  }

  ADS1256_CfgADC(ADS1256_GAIN_1, ADS1256_1000SPS);
  ADS1256_StartScan(0);
  enable_buffer(false);

  return 0;
}

int get_voltage(int COIL_PIN) {
  int32_t adc[2];
  int32_t volt[2];
  uint8_t buf[3];

  while ((ADS1256_Scan() == 0))
    ;
  adc[0] = ADS1256_GetAdc(COIL_PIN);
  volt[0] = (adc[0] * 100) / 167;
  adc[1] = ADS1256_GetAdc(TIMING_PIN);
  volt[1] = (adc[1] * 100) / 167;

  buf[0] = ((uint32_t)adc >> 16) & 0xFF;
  buf[1] = ((uint32_t)adc >> 8) & 0xFF;
  buf[2] = ((uint32_t)adc >> 0) & 0xFF;

  return volt[0];
}

int kill_adc(){

  bcm2835_spi_end();
  bcm2835_close();

  return 0;
}

/*
*********************************************************************************************************
*	name: ADS1256_StartScan
*	function: Configuration DRDY PIN for external interrupt is triggered
*	parameter: _ucDiffMode : 0  Single-ended input  8 channel�� 1
*Differential input  4 channe
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_StartScan(uint8_t _ucScanMode) {
  g_tADS1256.ScanMode = _ucScanMode;
  {
    uint8_t i;

    g_tADS1256.Channel = 0;

    for (i = 0; i < 8; i++) {
      g_tADS1256.AdcNow[i] = 0;
    }
  }
}

/*
*********************************************************************************************************
*	name: ADS1256_Send8Bit
*	function: SPI bus to send 8 bit data
*	parameter: _data:  data
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_Send8Bit(uint8_t _data) {

  bsp_DelayUS(2);
  bcm2835_spi_transfer(_data);
}

/*
*********************************************************************************************************
*	name: ADS1256_CfgADC
*	function: The configuration parameters of ADC, gain and data rate
*	parameter: _gain:gain 1-64
*                      _drate:  data  rate
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate) {
  g_tADS1256.Gain = _gain;
  g_tADS1256.DataRate = _drate;

  ADS1256_WaitDRDY();

  {
    uint8_t buf[4]; /* Storage ads1256 register configuration parameters */

    /*Status register define
            Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits
       (Read Only)

            Bit 3 ORDER: Data Output Bit Order
                    0 = Most Significant Bit First (default)
                    1 = Least Significant Bit First
            Input data  is always shifted in most significant byte and bit
       first. Output data is always shifted out most significant
            byte first. The ORDER bit only controls the bit order of the output
       data within the byte.

            Bit 2 ACAL : Auto-Calibration
                    0 = Auto-Calibration Disabled (default)
                    1 = Auto-Calibration Enabled
            When Auto-Calibration is enabled, self-calibration begins at the
       completion of the WREG command that changes
            the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE
       register) or BUFEN (bit 1 in the STATUS register)
            values.

            Bit 1 BUFEN: Analog Input Buffer Enable
                    0 = Buffer Disabled (default)
                    1 = Buffer Enabled

            Bit 0 DRDY :  Data Ready (Read Only)
                    This bit duplicates the state of the DRDY pin.

            ACAL=1  enable  calibration
    */
    // buf[0] = (0 << 3) | (1 << 2) | (1 << 1);//enable the internal buffer
    buf[0] =
        (0 << 3) | (1 << 2) | (0 << 1); // The internal buffer is prohibited

    // ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));

    buf[1] = 0x08;

    /*	ADCON: A/D Control Register (Address 02h)
            Bit 7 Reserved, always 0 (Read Only)
            Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
                    00 = Clock Out OFF
                    01 = Clock Out Frequency = fCLKIN (default)
                    10 = Clock Out Frequency = fCLKIN/2
                    11 = Clock Out Frequency = fCLKIN/4
                    When not using CLKOUT, it is recommended that it be turned
       off. These bits can only be reset using the RESET pin.

            Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
                    00 = Sensor Detect OFF (default)
                    01 = Sensor Detect Current = 0.5 �� A
                    10 = Sensor Detect Current = 2 �� A
                    11 = Sensor Detect Current = 10�� A
                    The Sensor Detect Current Sources can be activated to verify
       the integrity of an external sensor supplying a signal to the
                    ADS1255/6. A shorted sensor produces a very small signal
       while an open-circuit sensor produces a very large signal.

            Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
                    000 = 1 (default)
                    001 = 2
                    010 = 4
                    011 = 8
                    100 = 16
                    101 = 32
                    110 = 64
                    111 = 64
    */
    buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
    // ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));
    // /*choose 1: gain 1 ;input 5V/
    buf[3] = s_tabDataRate[_drate]; // DRATE_10SPS;

    CS_0(); /* SPIƬѡ = 0 */
    ADS1256_Send8Bit(CMD_WREG |
                     0); /* Write command register, send the register address */
    ADS1256_Send8Bit(0x03); /* Register number 4,Initialize the number  -1*/

    ADS1256_Send8Bit(buf[0]); /* Set the status register */
    ADS1256_Send8Bit(buf[1]); /* Set the input channel parameters */
    ADS1256_Send8Bit(buf[2]); /* Set the ADCON control register,gain */
    ADS1256_Send8Bit(buf[3]); /* Set the output rate */

    CS_1(); /* SPI  cs = 1 */
  }

  bsp_DelayUS(50);
}

/*
*********************************************************************************************************
*	name: ADS1256_DelayDATA
*	function: delay
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void) {
  /*
          Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT:
     RDATA, RDATAC,RREG Commands
          min  50   CLK = 50 * 0.13uS = 6.5uS
  */
  bsp_DelayUS(10); /* The minimum time delay 6.5us */
}

/*
*********************************************************************************************************
*	name: ADS1256_Recive8Bit
*	function: SPI bus receive function
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void) {
  uint8_t read = 0;
  read = bcm2835_spi_transfer(0xff);
  return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_WriteReg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue) {
  CS_0();                              /* SPI  cs  = 0 */
  ADS1256_Send8Bit(CMD_WREG | _RegID); /*Write command register */
  ADS1256_Send8Bit(0x00);              /*Write the register number */

  ADS1256_Send8Bit(_RegValue); /*send register value */
  CS_1();                      /* SPI   cs = 1 */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadReg
*	function: Read  the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID) {
  uint8_t read;

  CS_0();                              /* SPI  cs  = 0 */
  ADS1256_Send8Bit(CMD_RREG | _RegID); /* Write command register */
  ADS1256_Send8Bit(0x00);              /* Write the register number */

  ADS1256_DelayDATA(); /*delay time */

  read = ADS1256_Recive8Bit(); /* Read the register values */
  CS_1();                      /* SPI   cs  = 1 */

  return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_WriteCmd
*	function: Sending a single byte order
*	parameter: _cmd : command
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd) {
  CS_0(); /* SPI   cs = 0 */
  ADS1256_Send8Bit(_cmd);
  CS_1(); /* SPI  cs  = 1 */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadChipID
*	function: Read the chip ID
*	parameter: _cmd : NULL
*	The return value: four high status register
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void) {
  uint8_t id;

  ADS1256_WaitDRDY();
  id = ADS1256_ReadReg(REG_STATUS);
  return (id >> 4);
}

/*
*********************************************************************************************************
*	name: ADS1256_SetChannal
*	function: Configuration channel number
*	parameter:  _ch:  channel number  0--7
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch) {
  /*
  Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
          0000 = AIN0 (default)
          0001 = AIN1
          0010 = AIN2 (ADS1256 only)
          0011 = AIN3 (ADS1256 only)
          0100 = AIN4 (ADS1256 only)
          0101 = AIN5 (ADS1256 only)
          0110 = AIN6 (ADS1256 only)
          0111 = AIN7 (ADS1256 only)
          1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t
  care��)

          NOTE: When using an ADS1255 make sure to only select the available
  inputs.

  Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
          0000 = AIN0
          0001 = AIN1 (default)
          0010 = AIN2 (ADS1256 only)
          0011 = AIN3 (ADS1256 only)
          0100 = AIN4 (ADS1256 only)
          0101 = AIN5 (ADS1256 only)
          0110 = AIN6 (ADS1256 only)
          0111 = AIN7 (ADS1256 only)
          1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t
  care��)
  */
  if (_ch > 7) {
    return;
  }
  ADS1256_WriteReg(
      REG_MUX, (_ch << 4) | (1 << 3)); /* Bit3 = 1, AINN connection AINCOM */
}

/*
*********************************************************************************************************
*	name: ADS1256_SetDiffChannal
*	function: The configuration difference channel
*	parameter:  _ch:  channel number  0--3
*	The return value:  four high status register
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(uint8_t _ch) {
  /*
  Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
          0000 = AIN0 (default)
          0001 = AIN1
          0010 = AIN2 (ADS1256 only)
          0011 = AIN3 (ADS1256 only)
          0100 = AIN4 (ADS1256 only)
          0101 = AIN5 (ADS1256 only)
          0110 = AIN6 (ADS1256 only)
          0111 = AIN7 (ADS1256 only)
          1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ��don��t
  care��)

          NOTE: When using an ADS1255 make sure to only select the available
  inputs.

  Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
          0000 = AIN0
          0001 = AIN1 (default)
          0010 = AIN2 (ADS1256 only)
          0011 = AIN3 (ADS1256 only)
          0100 = AIN4 (ADS1256 only)
          0101 = AIN5 (ADS1256 only)
          0110 = AIN6 (ADS1256 only)
          0111 = AIN7 (ADS1256 only)
          1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ��don��t
  care��)
  */
  if (_ch == 0) {
    ADS1256_WriteReg(REG_MUX, (0 << 4) | 1); /* DiffChannal  AIN0�� AIN1 */
  } else if (_ch == 1) {
    ADS1256_WriteReg(REG_MUX, (2 << 4) | 3); /*DiffChannal   AIN2�� AIN3 */
  } else if (_ch == 2) {
    ADS1256_WriteReg(REG_MUX, (4 << 4) | 5); /*DiffChannal    AIN4�� AIN5 */
  } else if (_ch == 3) {
    ADS1256_WriteReg(REG_MUX, (6 << 4) | 7); /*DiffChannal   AIN6�� AIN7 */
  }
}

/*
*********************************************************************************************************
*	name: ADS1256_WaitDRDY
*	function: delay time  wait for automatic calibration
*	parameter:  NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void) {
  uint32_t i;

  for (i = 0; i < 400000; i++) {
    if (DRDY_IS_LOW()) {
      break;
    }
  }
  if (i >= 400000) {
    printf("ADS1256_WaitDRDY() Time Out ...\r\n");
  }
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadData
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void) {
  uint32_t read = 0;
  static uint8_t buf[3];

  CS_0(); /* SPI   cs = 0 */

  ADS1256_Send8Bit(CMD_RDATA); /* read ADC command  */

  ADS1256_DelayDATA(); /*delay time  */

  /*Read the sample results 24bit*/
  buf[0] = ADS1256_Recive8Bit();
  buf[1] = ADS1256_Recive8Bit();
  buf[2] = ADS1256_Recive8Bit();

  read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
  read |= ((uint32_t)buf[1]
           << 8); /* Pay attention to It is wrong   read |= (buf[1] << 8) */
  read |= buf[2];

  CS_1(); /* SPIƬѡ = 1 */

  /* Extend a signed number*/
  if (read & 0x800000) {
    read |= 0xFF000000;
  }

  return (int32_t)read;
}

/*
*********************************************************************************************************
*	name: ADS1256_GetAdc
*	function: read ADC value
*	parameter:  channel number 0--7
*	The return value:  ADC vaule (signed number)
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch) {
  int32_t iTemp;

  if (_ch > 7) {
    return 0;
  }

  iTemp = g_tADS1256.AdcNow[_ch];

  return iTemp;
}

/*
*********************************************************************************************************
*	name: ADS1256_ISR
*	function: Collection procedures
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
void ADS1256_ISR(void) {
  if (g_tADS1256.ScanMode == 0) /*  0  Single-ended input  8 channel�� 1
                                   Differential input  4 channe */
  {

    ADS1256_SetChannal(g_tADS1256.Channel); /*Switch channel mode */
    bsp_DelayUS(5);

    ADS1256_WriteCmd(CMD_SYNC);
    bsp_DelayUS(5);

    ADS1256_WriteCmd(CMD_WAKEUP);
    bsp_DelayUS(25);

    if (g_tADS1256.Channel == 0) {
      g_tADS1256.AdcNow[7] = ADS1256_ReadData();
    } else {
      g_tADS1256.AdcNow[g_tADS1256.Channel - 1] = ADS1256_ReadData();
    }

    if (++g_tADS1256.Channel >= 8) {
      g_tADS1256.Channel = 0;
    }
  } else /*DiffChannal*/
  {

    ADS1256_SetDiffChannal(g_tADS1256.Channel); /* change DiffChannal */
    bsp_DelayUS(5);

    ADS1256_WriteCmd(CMD_SYNC);
    bsp_DelayUS(5);

    ADS1256_WriteCmd(CMD_WAKEUP);
    bsp_DelayUS(25);

    if (g_tADS1256.Channel == 0) {
      g_tADS1256.AdcNow[3] = ADS1256_ReadData();
    } else {
      g_tADS1256.AdcNow[g_tADS1256.Channel - 1] = ADS1256_ReadData();
    }

    if (++g_tADS1256.Channel >= 4) {
      g_tADS1256.Channel = 0;
    }
  }
}

/*
*********************************************************************************************************
*	name: ADS1256_Scan
*	function:
*	parameter:NULL
*	The return value:  1
*********************************************************************************************************
*/
uint8_t ADS1256_Scan(void) {
  if (DRDY_IS_LOW()) {
    ADS1256_ISR();
    return 1;
  }

  return 0;
}

/*
*********************************************************************************************************
*	name: main
*	function:
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/

// Set the internal buffer (True - enable, False - disable).
uint8_t enable_buffer(bool val)
{
	CS_0();
	bcm2835_spi_transfer(CMD_WREG | REG_STATUS);
	bcm2835_spi_transfer((0 << 3) | (1 << 2) | (val << 1));
	CS_1();
}
