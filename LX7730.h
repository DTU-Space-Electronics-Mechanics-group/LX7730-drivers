/*!
   \file
   \brief Header file for LX7730 device driver.
   \author Denis Tcherniak
   \date 12/11/2018
   \copyright Copyright © [2023] Technical University of Denmark 
   \copyright This code is licensed under BSD 3-Clause License (see LICENSE.txt for details)
*/

#ifndef _LX7730_H__
#define _LX7730_H__

#include <stdint.h>
#include <assert.h>

#include "port_expander.h"

#define LX7730_MAX_CH_COUNT                             (64)      /*!< Defines the number of LX7730 channels. */

/* Registers and bitmask definitions */
#define LX7730_REG_FUNCTION_EN_ADDR                     (1)       /*!< Address of the LX7730 functional enable register. */
#define LX7730_MASK_CHIP_EN                             (1 << 7)  /*!< Bitmask of the chip enable bit in the LX7730 function enable register.  */
#define LX7730_MASK_SENSOR_MUX                          (1 << 6)  /*!< Bitmask of the sensor mux enable bit in the LX7730 function enable register. */
#define LX7730_MASK_CURRENT_SRC_DIS                     (1 << 5)  /*!< Bitmask of the current source disable bit in the LX7730 function enable register. */
#define LX7730_MASK_BILEVEL_COMP                        (1 << 4)  /*!< Bitmask of the bilevel comparator enable bit in the LX7730 function enable register. */
#define LX7730_MASK_ANALOG_AMP                          (1 << 3)  /*!< Bitmask of the analog amplifier enable bit in the LX7730 function enable register. */
#define LX7730_MASK_DAC                                 (1 << 2)  /*!< Bitmask of the DAC enable bit in the LX7730 function enable register. */
#define LX7730_MASK_FIXED_BILEVEL                       (1 << 1)  /*!< Bitmask of the fixed bilevel converters enable bit in the LX7730 function enable register. */
#define LX7730_MASK_ADC                                 (1 << 0)  /*!< Bitmask of the ADC enabled bit in the LX7730 function enable register. */

#define LX7730_REG_POWER_STATUS_ADDR                    (2)       /*!< Address of the LX7730 power status register. */
#define LX7730_USE_IREF2_SHIFT							(7)		  /*!< Specifies the bit shift of the USE_IREF2 bit in the LX7730 power status register. */
#define LX7730_MASK_USE_IREF2                           (1 << LX7730_USE_IREF2_SHIFT)  /*!< Bitmask of the use IREF2 enable bit in the LX7730 power status register. */
#define LX7730_MASK_MONITOR_VCC                         (1 << 6)  /*!< Bitmask of the monitor VCC enable bit in the LX7730 power status register. */
#define LX7730_MASK_MONITOR_VEE                         (1 << 5)  /*!< Bitmask of the monitor VEE enable bit in the LX7730 power status register. */
#define LX7730_MASK_MONITOR_5V0                         (1 << 4)  /*!< Bitmask of the monitor +5V enable bit in the LX7730 power status register. */
#define LX7730_MASK_MONITOR_VREF                        (1 << 3)  /*!< Bitmask of the monitor VREF enable bit in the LX7730 power status register. */
#define LX7730_MASK_VCC_UVLO                            (1 << 2)  /*!< Bitmask of the VCC UVLO bit in the LX7730 power status register. */
#define LX7730_MASK_VEE_UVLO                            (1 << 1)  /*!< Bitmask of the VEE UVLO bit in the LX7730 power status register. */
#define LX7730_MASK_5V0_UVLO                            (1 << 0)  /*!< Bitmask of the +5V UVLO bit in the LX7730 power status register. */

#define LX7730_REG_NON_INV_MUX_CH_SEL_ADDR              (3)                            /*!< Address of the LX7730 non-inverting MUX channel select register */
#define LX7730_BANK_SEL_SHIFT                           (3)                            /*!< Specifies the bit shift of the bank selection bits in the LX7730 inverting and non-inverting MUX channel select registers. */
#define LX7730_MASK_BANK_SEL                            (7 << LX7730_BANK_SEL_SHIFT)   /*!< Specifies the bitmask of the bank selection bits in the LX7730 inverting and non-inverting MUX channel select registers. */
#define LX7730_POS_SEL_SHIFT                            (0)                            /*!< Specifies the bit shift of the position selection bits in the LX7730 inverting and non-inverting MUX channel select registers. */
#define LX7730_MASK_POS_SEL                             (7 << LX7730_POS_SEL_SHIFT)    /*!< Specifies the bitmask of the position selection bits in the LX7730 inverting and non-inverting MUX channel select registers. */

#define LX7730_REG_INV_MUX_CH_SEL_ADDR                  (4)       /*!< Address of the LX7730 inverting MUX channel select register. */
#define LX7730_MASK_USE_SE_RTN                          (1 << 6)  /*!< Bitmask of the use SE_RTN bit in the LX7730 inverting MUX channel select register. */
/* Note:
The bank and the position select registers have the same masks as for the
non-inverting channel .
*/

#define LX7730_REG_CURRENT_MUX_LEVEL_ADDR               (5)                              /*!< Address of the LX7730 current MUX level register. */
#define LX7730_USE_DAC_SHIFT                            (7)                              /*!< Specifies the bit shift of the use DAC bit in the LX7730 MUX level register. */
#define LX7730_MASK_USE_DAC                             (1 << LX7730_USE_DAC_SHIFT)      /*!< Bitmask of the use DAC bit in the LX7730 current MUX level register. */
#define LX7730_DOUBLE_WIGHT_SHIFT                       (3)                              /*!< Specifies the bit shift of the double weight bit in the LX7730 MUX level register. */
#define LX7730_MASK_DOUBLE_WEIGHT                       (1 << LX7730_DOUBLE_WIGHT_SHIFT) /*!< Bitmask of the double weight bit in the LX7730 current MUX level register. */
#define LX7730_MASK_CURRENT_AMP                         (7)                     /*!< Bitmask of the current amplitude select bits in the LX7730 current MUX level register. */

#define LX7730_REG_CURRENT_MUX_CH_SEL_ADDR              (6)                              /*!< Address of the LX7730 current MUX channel select register. */
#define LX7730_MASK_CURRENT_MUX_CH_SEL                  (0b00111111)                     /*!< Bitmask of the current channel select bits in the LX7730 current MUX channel select register. */

#define LX7730_REG_SIGNAL_COND_AMP_ADDR                 (7)                              /*!< Address of the LX7730 signal conditioning amplifier register. */
#define LX7730_AAF_OFF_SHIFT                            (6)                              /*!< Specifies the bit shift of the AAF off bit in the LX7730 signal conditioning amplifier register. */
#define LX7730_MASK_AAF_OFF                             (1 << LX7730_AAF_OFF_SHIFT)      /*!< Bitmask of the AAF off bit in the LX7730 signal conditioning amplifier register. */
#define LX7730_SECOND_POLE_SHIFT                        (4)                              /*!< Specifies the bit shift of the second pole frequency selection bits in the LX7730 signal conditioning amplifier register. */
#define LX7730_MASK_SECOND_POLE_FREQ                    (3 << LX7730_SECOND_POLE_SHIFT)  /*!< Bitmask of the second pole frequency selection bits in the LX7730 signal conditioning amplifier register. */
#define LX7730_FIRST_POLE_SHIFT                         (2)                              /*!< Specifies the bit shift of the first pole frequency selection bits in the LX7730 signal conditioning amplifier register. */
#define LX7730_MASK_FIRST_POLE_FREQ                     (3 << LX7730_FIRST_POLE_SHIFT)   /*!< Bitmask of the first pole frequency selection bits in the LX7730 signal conditioning amplifier register. */
#define LX7730_MASK_GAIN_SETTING                        (3)                     /*!< Bitmask of the gain setting selection bits in the LX7730 signal conditioning amplifier register. */

#define LX7730_REG_ADC_CON_ADDR                         (8)                              /*!< Address of the LX7730 ADC control register. */
#define LX7730_AUTO_SAMPLE_RATE_SHIFT                   (5)                              /*!< Specifies the bit shift of the auto sample rate bits in the LX7730 ADC control register. */
#define LX7730_MASK_AUTO_SAMPLE_RATE                    (7 << LX7730_AUTO_SAMPLE_RATE_SHIFT) /*!< Bitmask of the auto sample rate control bits in the LX7730 ADC control register. */
#define LX7730_MASK_AUTO_CONV                           (1 << 4)                         /*!< Bitmask of the auto convert bit in the LX7730 ADC control register. */
#define LX7730_DATA_READY_SHIFT                         (3)                              /*!< Specifies the bit shift of the data ready bit in the LX7730 ADC control register. */
#define LX7730_MASK_DATA_READY                          (1 << LX7730_DATA_READY_SHIFT)   /*!< Bitmask of the data ready bit in the LX7730 ADC control register. */
#define LX7730_DATA_BUSY_SHIFT                         	(2)                              /*!< Specifies the bit shift of the busy bit in the LX7730 ADC control register. */
#define LX7730_MASK_BUSY                                (1 << LX7730_DATA_BUSY_SHIFT)    /*!< Bitmask of the busy bit in the LX7730 ADC control register. */
#define LX7730_MASK_START_CONVERSION                    (1 << 1)                         /*!< Bitmask of the start convertsion bit in the LX7730 ADC control register. */
#define LX7730_MASK_ADC_IN_HIZ                          (1 << 0)                         /*!< Bitmask of ADC_IN HI-Z bit in the LX7730 ADC control register. */

#define LX7730_REG_ADC_UPPER_ADDR                       (9)                              /*!< Address of the LX7730 ADC upper byte regsiter. */

#define LX7730_REG_ADC_LOWER_ADDR                       (10)                             /*!< Address of the LX7730 ADC lower bits regsiter. */
#define LX7730_MASK_ADC_LOWER_BITS                      (0b00001111)                     /*!< Bitmask of the ADC lower bits in the LX7730 ADC lower bits regsiter. */

#define LX7730_REG_BILEVEL_THRESHOLD_DAC_ADDR           (11)          /*!< Address of the LX7730 bi-level threshold DAC register. */

#define LX7730_REG_BILEVEL_BSP_FIXED_OPT_IN_ADDR        (12)          /*!< Address of the LX7730 bi-level bank switch position and fixed bi-level optional input register. */
#define LX7730_MASK_USE_BL_TH                           (1 << 7)      /*!< Bitmask of the use BL-TH bits in the LX7730 bi-level bank switch position and fixed bi-level optional input register. */
#define LX7730_MASK_EN_BL_SW_POS                        (1 << 3)      /*!< Bitmask of the en BL SW pos bits in the LX7730 bi-level bank switch position and fixed bi-level optional input register. */
#define LX7730_MASK_BILEVEL_MUX_COMMON_SW_POS           (0b00000111)  /*!< Bitmask of the common switch position selection bits for the eight bi-level multiplexers in the LX7730 bi-level bank switch position and fixed bi-level optional input register. */

#define LX7730_REG_BILEVEL_STATUS_ADDR                  (13)          /*!< Address of the LX7730 bi-level status register. */

#define LX7730_REG_DAC_UPPER_BYTE_ADDR                  (14)          /*!< Address of the LX7730 DAC upper byte regsiter */

#define LX7730_REG_DAC_LOWER_BYTE_ADDR                  (15)          /*!< Address of the LX7730 lower bits register. */
#define LX7730_MASK_DAC_LOWER_BITS                      (0b00000011)  /*!< Bitmask of the DAC lower bits in the LX7730 ADC lower bits register. */

#define LX7730_REG_CALIBRATION_ADDR                     (16)          /*!< Address of the LX7730 calibration register. */
#define LX7730_MASK_IA_SHORT                            (1 << 7)      /*!< Bitmask of the IA short bit in the LX7730 calibration register. */
#define LX7730_MASK_CONT_CHECK                          (1 << 4)      /*!< Bitmask of the cont check bit in the LX7730 calibration register. */
#define LX7730_MASK_NP_CONT_CHECK                       (1 << 3)      /*!< Bitmask of the NP cont check bit in the LX7730 calibration register. */
#define LX7730_MASK_I_GND                               (1 << 1)      /*!< Bitmask of the I GND bit in the LX7730 calibration register. */

#define LX7730_REG_OTP_ADDR                             (17)          /*!< Address of the LX7730 OTP register. */
#define LX7730_MASK_OTP_OUT_SELECT                      (1 << 1)      /*!< Bitmask of the OTP out select bit in the LX7730 OTP register. */
#define LX7730_MASK_OTP_IN_SELECT                       (1 << 0)      /*!< Bitmask of the OTP in select bit in the LX7730 OTP register. */

//TODO: Define trimming and test registers
#define LX7730_REG_TRIM1_ADDR							(18)		  /*!< Address of the LX7730 first trimming register. */
#define LX7730_REG_TRIM2_ADDR							(19)		  /*!< Address of the LX7730 second trimming register. */
#define LX7730_REG_TRIM3_ADDR							(20)		  /*!< Address of the LX7730 third trimming register. */
#define LX7730_REG_TRIM4_ADDR							(21)		  /*!< Address of the LX7730 fourth trimming register. */
#define LX7730_REG_TRIM5_ADDR							(22)		  /*!< Address of the LX7730 fifth trimming register. */
#define LX7730_REG_TRIM6_ADDR							(23)		  /*!< Address of the LX7730 sixth trimming register. */

#define LX7730_CMUX_SHIFT								(5)									/*!< Bit shift for the CMUX trimming bits. */
#define LX7730_MASK_CMUX								(0x07 << LX7730_CMUX_SHIFT)			/*!< Bitmask of the CMUX trimming bits. */
#define LX7730_MASK_VREF								(0x1f)								/*!< Bitmask of the VREF trimming bits. */
#define LX7730_VBGTC_SHIFT								(4)									/*!< Bit shift for the VBGTC trimming bits. */
#define LX7730_MASK_VBGTC								(0x0f << LX7730_VBGTC_SHIFT)		/*!< Bitmask of the VBGTC trimming bits. */
#define LX7730_MASK_OFFS								(0x0f)								/*!< Bitmask of the OFFS trimming bits. */
#define LX7730_VBG_SHIFT								(3)									/*!< Bit shift for the VBG trimming bits. */
#define LX7730_MASK_VBG									(0x1f << LX7730_VBG_SHIFT)			/*!< Bitmask of the VBG trimming bits. */
#define LX7730_MASK_VTOI_H								(0x07)								/*!< Bitmask of the VTOI high trimming bits. */
#define LX7730_VTOI_L_SHIFT								(6)									/*!< Bit shift for the VTOI low trimming bits. */
#define LX7730_MASK_VTOI_L								(0x03 << LX7730_VTOI_L_SHIFT)		/*!< Bitmask of the VTOI low trimming bits. */
#define LX7730_OSC_SHIFT								(2)									/*!< Bit shift for the OSC trimming bits. */
#define LX7730_MASK_OSC									(0x0f << LX7730_OSC_SHIFT)			/*!< Bitmask of the OSC trimming bits. */
#define LX7730_MASK_ADC_VTOI_H							(0x03)								/*!< Bitmask of the ADC VTOI high trimming bits. */
#define LX7730_ADC_VTOI_L_SHIFT							(5)									/*!< Bit shift for the ADC VTOI low trimming bits. */
#define LX7730_MASK_ADC_VTOI_L							(0x07 << LX7730_ADC_VTOI_L_SHIFT)	/*!< Bitmask of the ADC VTOI low trimming bits. */
#define LX7730_MASK_LO_DIS								(0x80)								/*!< Bitmask of the lo_dis bit. */

/*! Defines the return codes for LX7730 functions. */
typedef enum LX7730_return_code
{
  TX_ERROR,           /*!< Interface transmit error return code. */
  RX_ERROR,           /*!< Interface receive error return code. */
  CH_ERROR,
  PARITY_ERROR,       /*!< Error during parity check of the received byte. */
  CMD_NACK,           /*!< LX7730 command not acknowledge error return code. */
  RESULT_OK                  /*!< OK return code. */
}LX7730_return_code_e;

/*! Defines the default current MUX output current amplitudes. */
typedef enum LX7730_current_mux_amplitude
{
   CURRENT_242uA5 = 0,
   CURRENT_478uA75 = 8,
   CURRENT_485uA = 1,
   CURRENT_727uA5 = 2,
   CURRENT_957uA5 = 9,
   CURRENT_970uA = 3,
   CURRENT_1212uA5 = 4,
   CURRENT_1436uA25 = 10,
   CURRENT_1455uA = 5,
   CURRENT_1697uA5 = 6,
   CURRENT_1915uA = 11,
   CURRENT_1940uA = 7,
   CURRENT_2393uA75 = 12,
   CURRENT_2872uA5 = 13,
   CURRENT_3351uA25 = 14,
   CURRENT_3830uA = 15,
   DISABLED = -1
}LX7730_current_mux_amplitude_e;

/*! Defines the conditioning amplifier gain settings. */
typedef enum LX7730_aaf_gain
{
  GAIN_SETTING_04 = 0,     /*!< Sets the conditioning amplifier gain to 0.4. */
  GAIN_SETTING_2 = 1,      /*!< Sets the conditioning amplifier gain to 2. */
  GAIN_SETTING_10 = 2      /*!< Sets the conditioning amplifier gain to 10. */
}LX7730_aaf_gain_e;

/*! Defines the conditioning amplifier filter frequency settings. */
typedef enum LX7730_aaf_filter_frequency
{
  POLE_FREQ_400HZ = 0,     /*!< Sets the conditioning amplifier pole frequency to 400 Hz for either the first or the second pole. */
  POLE_FREQ_2KHZ  = 1,     /*!< Sets the conditioning amplifier pole frequency to 2 kHz for either the first or the second pole. */
  POLE_FREQ_10KHZ          /*!< Sets the conditioning amplifier pole frequency to 10 kHz for either the first or the second pole. */
}LX7730_aaf_filter_frequency_e;

/* Defines the available monitor settings */
typedef enum LX7730_monitor
{
	MONITOR_VCC,
	MONITOR_VEE,
	MONITOR_5V0,
	MONITOR_VREF
}LX7730_monitor_e;

/* Defines the available OTP sources */
typedef enum LX7730_otp_source
{
	USER_REGISTERS,			/*!< Sets the trimming data to be read from the user accessible registers 18-22. */
	FACTORY_REGISTERS		/*!< Sets the trimming data to be read from the one time programmable factory registers. */
}LX7730_otp_source_e;

/*! Defines the OTP trimming registers */
typedef struct LX7730_otp
{
	uint8_t cmux;
	uint8_t vref;
	uint8_t vbgtc;
	uint8_t offs;
	uint8_t vbg;
	uint8_t vtoi;
	uint8_t osc;
	uint8_t ADCvtoi;
}LX7730_otp_t;

/*!
   \brief Transmits bytes from the specified pointer over the HW interface. This function must be declared externally.
   \param data Pointer to the data buffer.
   \param len Length of the buffer.
   \return The result code for the operation.
*/
LX7730_return_code_e if_write(uint8_t * data, uint8_t len);

/*!
   \brief Receives bytes into the specified pointer over the HW interface. This function must be declared externally.
   \param data Pointer to the data buffer.
   \param len Number of bytes to receive.
   \return The result code for the operation.
*/
LX7730_return_code_e if_read(uint8_t * data, uint8_t len);

/*!
   \brief Calculates even parity bit for the provided data.
   \param data Pointer to the array in which the data is contained.
   \param data_len Length of the data array.
   \param data_bits Number of bits to perform the parity check over, starting from MSB.
   \return The parity bit
*/
uint8_t calculate_even_parity(uint8_t * data, uint8_t data_len, uint8_t data_bits);

/**
   \brief Writes a byte to the specified LX7730 register.
   \param address Address of the register to write the byte to.
   \param data Data to write.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_write_register(uint8_t address, uint8_t data);

/*!
   \brief Reads a byte from the specified LX7730 register.
   \param address Address of the register to read the byte from
   \param data Pointer to the buffer in which to place the read register.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_read_register(uint8_t address, uint8_t * data);


/**
   \brief Sets a bit in the specified LX7730 register.
   \param address Address of the register.
   \param bitmask Bitmask of the bit.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_set_bit(uint8_t address, uint8_t bitmask);

/**
   \brief Clears a bit in the specified LX7730 register.
   \param address Address of the register.
   \param bitmask Bitmask of the bit.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_clear_bit(uint8_t address, uint8_t bitmask);

/*!
   \brief Sets the LX7730 function enable register.
   \param data The functions to enable/disable. Must consist of the bitmasks defined for this register.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_set_function_enable(uint8_t data);

/*!
   \brief Enables the chip.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_chip_enable(void);

/*!
   \brief Disables the chip.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_chip_disable(void);

/*!
   \brief Enables the sensor mux.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_sensor_mux_enable(void);

/*!
   \brief Disables the sensor mux.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_sensor_mux_disable(void);

/*!
   \brief Enables the current source.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_current_source_enable(void);

/*!
   \brief Disables the current source.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_current_source_disable(void);

/*!
   \brief Enables the bi-level comparator.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_bi_level_comp_enable(void);

/*!
   \brief Disables the bi-level comparator.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_bi_level_comp_disable(void);

/*!
   \brief Enables the analog amplifier.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_analog_amplifier_enable(void);

/*!
   \brief Disables the analog amplifier.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_analog_amplifier_disable(void);

/*!
   \brief Enables the DAC.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_dac_enable(void);

/*!
   \brief Disables the DAC.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_dac_disable(void);

/*!
   \brief Enables the fixed bi-level comparator.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_fixed_bilevel_enable(void);

/*!
   \brief Disables the fixed bi-level comparator.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_fixed_bilevel_disable(void);

/*!
   \brief Disables the ADC.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_adc_enable(void);

/*!
   \brief Disables the ADC.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_adc_disable(void);

/*!
   \brief Sets the LX7730 power status register.
   \param data The power status functions to enable or disable. Must consist of the bitmasks defined for this register.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_set_power_status(uint8_t data);

/*!
   \brief Gets the LX7730 power status register.
   \param data Pointer to where to place the read register.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_get_power_status(uint8_t * data);

/*!
   \brief Gets the UVLO read only bits of the LX7730 power register.
   \param uvlo_mask The bitmask consisting of ORed LX7730_MASK_VCC_UVLO, LX7730_MASK_VEE_UVLO and LX7730_MASK_5V0_UVLO bitmasks.
   \param uvlo_status Pointer to where to place the read status bits.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_get_uvlo_status(uint8_t uvlo_mask, uint8_t * uvlo_status);

/*!
   \brief Sets the source channel of the instrumentation amplifier non-inverting input.
   \param ch The source channel (1..64).
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_select_non_inv_ch(uint8_t ch);

/*!
   \brief Sets the source channel of the instrumentation amplifier inverting input.
   \param ch The source channel (1..64).
   \param use_se_rtn Specifies whether to use the SE_RTN input instead of one of the channel pins. 1 = Use SE_RTN, 0 = use the specified channel.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_select_inv_ch(uint8_t ch, uint8_t use_se_rtn);

/*!
   \brief Sets the current amplitude of the current MUX to the specified default value.
   \param current One of the eight available amplitude values, defined by LX7730_current_mux_amplitude_e enum.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_current_mux_set_amplitude(LX7730_current_mux_amplitude_e current);

/*!
   \brief Specifies whether to use the DAC to control the amplitude of the current MUX current.
   \param use_dac Specifies whether to use the DAC to control the amplitude of the current MUX current. 1 = use the DAC, 0 = don't use the DAC.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_current_mux_use_dac(uint8_t use_dac);

/*!
   \brief Specifies whether to use multiply the amplitude of the current MUX current by 2.
   \param use_double_weight Specifies whether to use multiply the amplitude of the current MUX current by 2. 1 = multiply by 2, 0 = don't multiply by 2.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_current_mux_double_weight(uint8_t use_double_weight);

/*!
   \brief Specifies the channel on which to output the current MUX current.
   \param ch The channel on which to output the current. Valid values 1 - 64.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_current_mux_set_output_channel(uint8_t ch);

/*!
   \brief Turns on the signal conditioning amplifier
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_enable_AAF(void);

/*!
   \brief Turns off the signal conditioning amplifier
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_disable_AAF(void);

/*!
   \brief Sets the second pole frequency of the AAF.
   \param frequency The pole frequency defined by LX7730_aaf_filter_frequency_e enum.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_set_aaf_second_pole(LX7730_aaf_filter_frequency_e frequency);

/*!
   \brief Sets the first pole frequency of the AAF.
   \param frequency The pole frequency defined by LX7730_aaf_filter_frequency_e enum.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_set_aaf_first_pole(LX7730_aaf_filter_frequency_e frequency);

/*!
   \brief Sets the gain of the AAF.
   \param gain The gain defined by LX7730_aaf_gain_e enum.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_set_aaf_gain(LX7730_aaf_gain_e gain);

/*!
   \brief Sets the auto sample rate bits of the ADC control register
   \param asr Auto sample rate bits, see LX7730 datasheet rev. 1.3 p.25
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_set_adc_auto_sample_rate(uint8_t asr);

/*!
   \brief Enables ADC continuous conversion.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_enable_adc_auto_conversion(void);

/*!
   \brief Disables ADC continuous conversion.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_disable_adc_auto_conversion(void);

/*!
   \brief Gets the ADC data ready bit.
   \param data_ready_p pointer to where to store the data ready bit.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_get_adc_data_ready(uint8_t * data_ready_p);

/*!
   \brief Gets the ADC busy bit.
   \param busy_p pointer to where to store the busy bit.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_get_adc_busy(uint8_t * busy_p);

/*!
   \brief Starts the ADC conversion
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_adc_start_conversion();

/*!
   \brief Configures LX7730 to either use IREF1 or IREF2 current reference.
   \param use_iref2 Set to 1 to use IREF2 set to 0 to use IREF1.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_use_iref2(uint8_t use_iref2);

/*!
   \brief Enables or disables one of the built in voltage monitors.
   \param monitor Specifies which monitor to enable/disable, defined by LX7730_monitor_e enum.
   \param monitor Specifies whether to enable or to disable the monitor.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_monitor_en(LX7730_monitor_e monitor, uint8_t enable);

/*!
   \brief Enables shorting of the IA inputs.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_cal_iashort_enable();

/*!
   \brief Disables shorting of the IA inputs.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_cal_iashort_disable();

/*!
   \brief Applies current source to the non-inverting input of the IA.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_cal_cont_check_enable();

/*!
   \brief Disconnects the current source from the non-inverting input of the IA.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_cal_cont_check_disable();

/*!
   \brief Applies current source to the non-inverting input of the IA and connects the inverting input to VREF.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_cal_np_cont_check_enable();

/*!
   \brief Removes the current source from the non-inverting input of the IA and disconnects the inverting input from VREF.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_cal_np_cont_check_disable();

/*!
   \brief Shorts the inverting terminal of the IA to GND.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_cal_ignd_enable();

/*!
   \brief Un-shorts the inverting terminal of the IA to GND.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_cal_ignd_disable();


/*!
   \brief Loads the OTP registers from ROM.
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_otp_load_regs();

/*!
   \brief Sets the source of the trimming registers to either user provided or factory values.
   \param source Specifies the source of the trimming registers, defined by LX7730_otp_source_e enum./
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_otp_set_source(LX7730_otp_source_e source);

/*!
   \brief Reads and returns the OTP trimming registers.
   \param otp pointer to the LX7730_otp_t struct which will conain the read values;
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_otp_read_regs(LX7730_otp_t * otp);

/*!
   \brief Writes the provided OTP trimming registers.
   \param otp pointer to the LX7730_otp_t struct which will conain the values to write;
   \return The result code for the operation.
*/
LX7730_return_code_e LX7730_otp_write_regs(LX7730_otp_t * otp);

#endif /* _LX7730_H__ */
