/*!
   \file
   \brief Device driver for Microsemi LX7730 rad-hard telemetry controller.
   \author Denis Tcherniak
   \date 12/11/2018
   \copyright Copyright © [2023] Technical University of Denmark 
   \copyright This code is licensed under BSD 3-Clause License (see LICENSE.txt for details)
*/

#include "lx7730.h"
#include <stdio.h>

uint8_t calculate_even_parity(uint8_t * data, uint8_t data_len, uint8_t data_bits)
{
  uint8_t parity = 0;
  uint8_t processed_bits = 0;
  uint8_t current_byte = 0;

  while(data_len > 0 && data_bits > 0)
  {
    current_byte = data[data_len - 1];
    while(processed_bits < 8 && data_bits > 0)
    {
      parity ^= current_byte & 0x80;
      current_byte <<= 1;
      data_bits--;
      processed_bits++;
    }
    processed_bits = 0;
    data_len--;
  }

  return parity >> 7;
}

LX7730_return_code_e LX7730_write_register(uint8_t address, uint8_t data)
{
    LX7730_return_code_e ret = RESULT_OK;
    uint8_t out_frame[2];
    uint8_t parity;

    /*
     * The frame consists of 15 bits, we divide thm up over two bytes as follows:

     High byte: Dummy, R/W 1 = Write, A4 A3 A2 A1 A0, D7
     Low byte: D6 D5 D4 D3 D2 D1 D0, parity
    */
    out_frame[1] = (0x40 | (address << 1) | (data >> 7));
    out_frame[0] = (data << 1);

    /* Append parity */
    parity = calculate_even_parity(out_frame, 2, 15);
    out_frame[0] |= parity;

    ret = if_write(out_frame, 2);

    return ret;
}

LX7730_return_code_e LX7730_read_register(uint8_t address, uint8_t * data_p)
{
  LX7730_return_code_e ret = RESULT_OK;
  uint8_t frame[2];
  uint8_t parity = 0;

  /* Configure the frame with the register address for the "read" command" */
  frame[1] = (address << 1);
  frame[0] = 0x00;
  parity = calculate_even_parity(frame, 2, 15);
  frame[0] |= parity;

  /* Transmit command and receive data */
  ret = if_read(frame, 2);

  if(ret != RESULT_OK)
  {
	  return ret;
  }else
  {
	  /* Check parity */
	  parity = calculate_even_parity(frame, 2, 15);

	  if((frame[0] & 0x01) != parity)
	  {
		  ret = PARITY_ERROR;
		  return ret;
	  }else
	  {
		  *data_p = ((frame[1] & 0x01) << 7) | (frame[0] >> 1);
	  }

  }

  return ret;
}

inline LX7730_return_code_e LX7730_set_bit(uint8_t address, uint8_t bitmask)
{
	LX7730_return_code_e ret = RESULT_OK;
	uint8_t reg = 0;

	ret = LX7730_read_register(address, &reg);

	if(ret != RESULT_OK){return ret;}

	ret = LX7730_write_register(address, (reg & ~bitmask) | bitmask);
	return ret;
}

inline LX7730_return_code_e LX7730_clear_bit(uint8_t address, uint8_t bitmask)
{
	LX7730_return_code_e ret = RESULT_OK;
	uint8_t reg = 0;

	ret = LX7730_read_register(address, &reg);

	if(ret != RESULT_OK){return ret;}

	ret = LX7730_write_register(address, (reg & ~bitmask));
	return ret;
}

LX7730_return_code_e LX7730_set_function_enable(uint8_t data)
{
	return LX7730_write_register(LX7730_REG_FUNCTION_EN_ADDR, data);
}

inline LX7730_return_code_e LX7730_chip_enable(void)
{
	LX7730_return_code_e ret = RESULT_OK;

	ret = LX7730_write_register(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_CHIP_EN);

	return ret;
}

inline LX7730_return_code_e LX7730_chip_disable(void)
{
	return LX7730_clear_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_CHIP_EN);
}

inline LX7730_return_code_e LX7730_sensor_mux_enable(void)
{
	return LX7730_set_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_SENSOR_MUX);
}

inline LX7730_return_code_e LX7730_sensor_mux_disable(void)
{
	return LX7730_clear_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_SENSOR_MUX);
}

inline LX7730_return_code_e LX7730_current_source_enable(void)
{
	return LX7730_clear_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_CURRENT_SRC_DIS);
}

inline LX7730_return_code_e LX7730_current_source_disable(void)
{
	return LX7730_set_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_CURRENT_SRC_DIS);
}

inline LX7730_return_code_e LX7730_bi_level_comp_enable(void)
{
	return LX7730_set_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_BILEVEL_COMP);
}

inline LX7730_return_code_e LX7730_bi_level_comp_disable(void)
{
	return LX7730_clear_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_BILEVEL_COMP);
}

inline LX7730_return_code_e LX7730_analog_amplifier_enable(void)
{
	return LX7730_set_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_ANALOG_AMP);
}

inline LX7730_return_code_e LX7730_analog_amplifier_disable(void)
{
	return LX7730_clear_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_ANALOG_AMP);
}

inline LX7730_return_code_e LX7730_dac_enable(void)
{
	return LX7730_set_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_DAC);
}

inline LX7730_return_code_e LX7730_dac_disable(void)
{
	return LX7730_clear_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_DAC);
}

inline LX7730_return_code_e LX7730_fixed_bilevel_enable(void)
{
	return LX7730_set_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_FIXED_BILEVEL);
}

inline LX7730_return_code_e LX7730_fixed_bilevel_disable(void)
{
	return LX7730_clear_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_FIXED_BILEVEL);
}

inline LX7730_return_code_e LX7730_adc_enable(void)
{
	return LX7730_set_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_ADC);
}

inline LX7730_return_code_e LX7730_adc_disable(void)
{
	return LX7730_clear_bit(LX7730_REG_FUNCTION_EN_ADDR, LX7730_MASK_ADC);
}

LX7730_return_code_e LX7730_set_power_status(uint8_t data)
{
	return LX7730_write_register(LX7730_REG_POWER_STATUS_ADDR, data);
}

LX7730_return_code_e LX7730_get_power_status(uint8_t * data)
{
	LX7730_return_code_e ret = RESULT_OK;
	ret =  LX7730_read_register(LX7730_REG_POWER_STATUS_ADDR, data);
	return ret;
}

LX7730_return_code_e LX7730_get_uvlo_status(uint8_t uvlo_mask, uint8_t * uvlo_status)
{
  uint8_t power_status;
  LX7730_return_code_e ret;

  ret = LX7730_get_power_status(&power_status);
  if(ret != RESULT_OK)
  {
      return ret;
  }else
  {
      *uvlo_status = power_status & uvlo_mask;
      return ret;
  }
}

LX7730_return_code_e LX7730_use_iref2(uint8_t use_iref2)
{
	LX7730_return_code_e ret = RESULT_OK;
	if(use_iref2)
	{
		ret = LX7730_set_bit(LX7730_REG_POWER_STATUS_ADDR, LX7730_MASK_USE_IREF2);
	}else
	{
		ret = LX7730_clear_bit(LX7730_REG_POWER_STATUS_ADDR, LX7730_MASK_USE_IREF2);
	}
	return ret;
}

LX7730_return_code_e LX7730_monitor_en(LX7730_monitor_e monitor, uint8_t enable)
{
	uint8_t reg = 0;
	uint8_t bitmask = 0;
	LX7730_return_code_e ret = RESULT_OK;

	ret = LX7730_get_power_status(&reg);
	if(ret != RESULT_OK)
	{
		return ret;
	}

	// Only apply a bitmask, if we want to enable the monitor. If we want to disable it bitmask = 0 is fine.
	if(enable)
	{
		switch(monitor)
		{
		case MONITOR_VCC:
			bitmask = LX7730_MASK_MONITOR_VCC;
			break;
		case MONITOR_VEE:
			bitmask = LX7730_MASK_MONITOR_VEE;
			break;
		case MONITOR_5V0:
			bitmask = LX7730_MASK_MONITOR_5V0;
			break;
		case MONITOR_VREF:
			bitmask = LX7730_MASK_MONITOR_VREF;
			break;
		}
	}

	// Clear all other monitor flags in the register and or in the bitmask.
	reg = (reg & ~(LX7730_MASK_MONITOR_VCC | LX7730_MASK_MONITOR_VEE | LX7730_MASK_MONITOR_5V0 | LX7730_MASK_MONITOR_VREF)) | bitmask;

	ret = LX7730_set_power_status(reg);
	return ret;
}


/* ------------ Functions for controlling the amplifier inputs channel selection -------------*/

LX7730_return_code_e LX7730_select_non_inv_ch(uint8_t ch)
{
  if(ch <= 1 && ch <= LX7730_MAX_CH_COUNT){ return CH_ERROR; }

  uint8_t pos = 0;
  uint8_t bank = 0;
  uint8_t channel_select = 0;

  // Calculate the position and the bank (note that the input channel is from 1..64, while banks and positions are 0 indexed.
  pos = (ch - 1) / 8;
  bank = ch - (pos * 8) - 1;

  channel_select = (bank << LX7730_BANK_SEL_SHIFT) | (pos << LX7730_POS_SEL_SHIFT);

  return LX7730_write_register(LX7730_REG_NON_INV_MUX_CH_SEL_ADDR, channel_select);
}

LX7730_return_code_e LX7730_select_inv_ch(uint8_t ch, uint8_t use_se_rtn)
{
  if(ch <= 0 && ch <= LX7730_MAX_CH_COUNT){ return CH_ERROR; }

  uint8_t pos = 0;
  uint8_t bank = 0;
  uint8_t channel_select = 0;

  if(use_se_rtn)
  {
	  channel_select |= LX7730_MASK_USE_SE_RTN;
  }else
  {
	  pos = (ch - 1) / 8;
	  bank = ch - (pos * 8) - 1;

	  channel_select = (bank << LX7730_BANK_SEL_SHIFT) | (pos << LX7730_POS_SEL_SHIFT);
  }

  return LX7730_write_register(LX7730_REG_INV_MUX_CH_SEL_ADDR, channel_select);
}

/* ---------------------- Functions for controlling the current MUX --------------------------*/

LX7730_return_code_e LX7730_current_mux_set_amplitude(LX7730_current_mux_amplitude_e current)
{
  uint8_t current_mux;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_CURRENT_MUX_LEVEL_ADDR, &current_mux);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
    current_mux = (current_mux & (~LX7730_MASK_CURRENT_AMP)) | ((uint8_t)current & LX7730_MASK_CURRENT_AMP);
    return LX7730_write_register(LX7730_REG_CURRENT_MUX_LEVEL_ADDR, current_mux);
  }
}

LX7730_return_code_e LX7730_current_mux_use_dac(uint8_t use_dac)
{
  uint8_t current_mux;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_CURRENT_MUX_LEVEL_ADDR, &current_mux);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
    current_mux = (current_mux & (~LX7730_MASK_USE_DAC)) | ((use_dac << LX7730_USE_DAC_SHIFT) & LX7730_MASK_USE_DAC);
    return LX7730_write_register(LX7730_REG_CURRENT_MUX_LEVEL_ADDR, current_mux);
  }
}

LX7730_return_code_e LX7730_current_mux_double_weight(uint8_t use_double_weight)
{
  uint8_t current_mux;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_CURRENT_MUX_LEVEL_ADDR, &current_mux);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
    current_mux = (current_mux & (~LX7730_MASK_DOUBLE_WEIGHT)) | ((use_double_weight << LX7730_DOUBLE_WIGHT_SHIFT) & LX7730_MASK_DOUBLE_WEIGHT);
    return LX7730_write_register(LX7730_REG_CURRENT_MUX_LEVEL_ADDR, current_mux);
  }
}

LX7730_return_code_e LX7730_current_mux_set_output_channel(uint8_t ch)
{
  assert(ch > 0 && ch <= LX7730_MAX_CH_COUNT);

  ch = ch - 1; // In the register the channels are 0 indexed.
  return LX7730_write_register(LX7730_REG_CURRENT_MUX_CH_SEL_ADDR, ch);
}

/*-------------- Functions for controlling the signal conditioning amplifier -----------------*/
LX7730_return_code_e LX7730_enable_AAF(void)
{
  uint8_t aaf_register;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, &aaf_register);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
    aaf_register = (aaf_register & (~LX7730_MASK_AAF_OFF));
    return LX7730_write_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, aaf_register);
  }
}

LX7730_return_code_e LX7730_disable_AAF(void)
{
  uint8_t aaf_register;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, &aaf_register);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
    aaf_register = (aaf_register & (~LX7730_MASK_AAF_OFF)) | LX7730_MASK_AAF_OFF;
    return LX7730_write_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, aaf_register);
  }
}

LX7730_return_code_e LX7730_set_aaf_second_pole(LX7730_aaf_filter_frequency_e frequency)
{
  uint8_t aaf_register;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_CURRENT_MUX_LEVEL_ADDR, &aaf_register);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
    aaf_register = (aaf_register & (~LX7730_MASK_SECOND_POLE_FREQ)) | (((uint8_t)frequency) << LX7730_SECOND_POLE_SHIFT);
    return LX7730_write_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, aaf_register);
  }
}

LX7730_return_code_e LX7730_set_aaf_first_pole(LX7730_aaf_filter_frequency_e frequency)
{
  uint8_t aaf_register;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, &aaf_register);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
    aaf_register = (aaf_register & (~LX7730_MASK_FIRST_POLE_FREQ)) | (((uint8_t)frequency) << LX7730_FIRST_POLE_SHIFT);
    return LX7730_write_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, aaf_register);
  }
}

LX7730_return_code_e LX7730_set_aaf_gain(LX7730_aaf_gain_e gain)
{
  uint8_t aaf_register;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, &aaf_register);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
    aaf_register = (aaf_register & (~LX7730_MASK_GAIN_SETTING)) | (((uint8_t)gain));
    return LX7730_write_register(LX7730_REG_SIGNAL_COND_AMP_ADDR, aaf_register);
  }
}

/*-------------- Functions for controlling the ADC -----------------*/

LX7730_return_code_e LX7730_set_adc_auto_sample_rate(uint8_t asr)
{
  assert(asr <= 7);

  uint8_t adc_register;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_ADC_CON_ADDR, &adc_register);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
	adc_register = (adc_register & (~LX7730_MASK_AUTO_SAMPLE_RATE)) | (((uint8_t)asr << LX7730_AUTO_SAMPLE_RATE_SHIFT));
    return LX7730_write_register(LX7730_REG_ADC_CON_ADDR, adc_register);
  }
}

LX7730_return_code_e LX7730_enable_adc_auto_conversion(void)
{
  uint8_t adc_register;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_ADC_CON_ADDR, &adc_register);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
	  adc_register = (adc_register & (~LX7730_MASK_AUTO_CONV)) | LX7730_MASK_AUTO_CONV;
    return LX7730_write_register(LX7730_REG_ADC_CON_ADDR, adc_register);
  }
}

LX7730_return_code_e LX7730_disable_adc_auto_conversion(void)
{
  uint8_t adc_register;
  LX7730_return_code_e ret;

  ret = LX7730_read_register(LX7730_REG_ADC_CON_ADDR, &adc_register);
  if(ret != RESULT_OK)
  {
    return ret;
  }else
  {
	adc_register = (adc_register & (~LX7730_MASK_AUTO_CONV));
    return LX7730_write_register(LX7730_REG_ADC_CON_ADDR, adc_register);
  }
}

LX7730_return_code_e LX7730_get_adc_data_ready(uint8_t * data_ready_p)
{
  LX7730_return_code_e ret;
  ret = LX7730_read_register(LX7730_REG_ADC_CON_ADDR, data_ready_p);
  *data_ready_p = (*data_ready_p & LX7730_MASK_DATA_READY) >> LX7730_DATA_READY_SHIFT;

  return ret;
}

LX7730_return_code_e LX7730_get_adc_busy(uint8_t * busy_p)
{
  LX7730_return_code_e ret;
  ret = LX7730_read_register(LX7730_REG_ADC_CON_ADDR, busy_p);
  *busy_p = (*busy_p & LX7730_MASK_BUSY) >> LX7730_DATA_BUSY_SHIFT;

  return ret;
}

LX7730_return_code_e LX7730_adc_start_conversion()
{
  LX7730_return_code_e ret;

  ret = LX7730_set_bit(LX7730_REG_ADC_CON_ADDR, LX7730_MASK_START_CONVERSION);

  return ret;
}

/*-------------- Functions for handling calibration register -----------------*/

LX7730_return_code_e LX7730_cal_iashort_enable()
{
	LX7730_return_code_e ret;
	ret = LX7730_set_bit(LX7730_REG_CALIBRATION_ADDR, LX7730_MASK_IA_SHORT);
	return ret;
}

LX7730_return_code_e LX7730_cal_iashort_disable()
{
	LX7730_return_code_e ret;
	ret = LX7730_clear_bit(LX7730_REG_CALIBRATION_ADDR, LX7730_MASK_IA_SHORT);
	return ret;
}

LX7730_return_code_e LX7730_cal_cont_check_enable()
{
	LX7730_return_code_e ret;
	ret = LX7730_set_bit(LX7730_REG_CALIBRATION_ADDR, LX7730_MASK_CONT_CHECK);
	return ret;
}

LX7730_return_code_e LX7730_cal_cont_check_disable()
{
	LX7730_return_code_e ret;
	ret = LX7730_set_bit(LX7730_REG_CALIBRATION_ADDR, LX7730_MASK_CONT_CHECK);
	return ret;
}

LX7730_return_code_e LX7730_cal_np_cont_check_enable()
{
	LX7730_return_code_e ret;
	ret = LX7730_set_bit(LX7730_REG_CALIBRATION_ADDR, LX7730_MASK_NP_CONT_CHECK);
	return ret;
}

LX7730_return_code_e LX7730_cal_np_cont_check_disable()
{
	LX7730_return_code_e ret;
	ret = LX7730_set_bit(LX7730_REG_CALIBRATION_ADDR, LX7730_MASK_NP_CONT_CHECK);
	return ret;
}

LX7730_return_code_e LX7730_cal_ignd_enable()
{
	LX7730_return_code_e ret;
	ret = LX7730_set_bit(LX7730_REG_CALIBRATION_ADDR, LX7730_MASK_I_GND);
	return ret;
}

LX7730_return_code_e LX7730_cal_ignd_disable()
{
	LX7730_return_code_e ret;
	ret = LX7730_set_bit(LX7730_REG_CALIBRATION_ADDR, LX7730_MASK_I_GND);
	return ret;
}

/*-------------- Functions for handling trimming registers -----------------*/

LX7730_return_code_e LX7730_otp_load_regs()
{
	LX7730_return_code_e ret = RESULT_OK;
	ret = LX7730_set_bit(LX7730_REG_OTP_ADDR, LX7730_MASK_OTP_IN_SELECT);
	return ret;
}

LX7730_return_code_e LX7730_otp_set_source(LX7730_otp_source_e source)
{
	LX7730_return_code_e ret = RESULT_OK;

	if(source == USER_REGISTERS)
	{
		ret = LX7730_set_bit(LX7730_REG_OTP_ADDR, LX7730_MASK_OTP_OUT_SELECT);
	}else
	{
		ret = LX7730_clear_bit(LX7730_REG_OTP_ADDR, LX7730_MASK_OTP_OUT_SELECT);
	}
	return ret;
}

LX7730_return_code_e LX7730_otp_read_regs(LX7730_otp_t * otp)
{
	uint8_t trim_address = 0;
	LX7730_return_code_e ret = RESULT_OK;
	uint8_t trim_regs_p[6];

	for(trim_address = LX7730_REG_TRIM1_ADDR; trim_address < LX7730_REG_TRIM6_ADDR; trim_address++)
	{
		ret = LX7730_read_register(trim_address, trim_regs_p + (trim_address - LX7730_REG_TRIM1_ADDR));
		if(ret != RESULT_OK){return ret;}
	}

	otp->cmux = (trim_regs_p[0] & LX7730_MASK_CMUX) >> LX7730_CMUX_SHIFT;
	otp->vref = (trim_regs_p[0] & LX7730_MASK_VREF);
	otp->vbgtc = (trim_regs_p[1] & LX7730_MASK_VBGTC) >> LX7730_VBGTC_SHIFT;
	otp->offs = (trim_regs_p[1] & LX7730_MASK_OFFS);
	otp->vbg = (trim_regs_p[2] & LX7730_MASK_VBG) >> LX7730_VBG_SHIFT;
	otp->vtoi = ((trim_regs_p[2] & LX7730_MASK_VTOI_H) << 2) | ((trim_regs_p[3] & LX7730_MASK_VTOI_L) >> LX7730_VTOI_L_SHIFT);
	otp->osc =(trim_regs_p[3] & LX7730_MASK_OSC) >> LX7730_OSC_SHIFT;
	otp->ADCvtoi = ((trim_regs_p[3] & LX7730_MASK_ADC_VTOI_H) << 3) | ((trim_regs_p[4] & LX7730_MASK_ADC_VTOI_L) >> LX7730_ADC_VTOI_L_SHIFT);

	return ret;
}

LX7730_return_code_e LX7730_otp_write_regs(LX7730_otp_t * otp)
{
	uint8_t trim_address = 0;
	LX7730_return_code_e ret = RESULT_OK;
	uint8_t trim_regs_p[6];

	// Populate the trim regs
	trim_regs_p[0] = (otp->cmux << LX7730_CMUX_SHIFT) | (otp->vref);
	trim_regs_p[1] = (otp->vbgtc << LX7730_VBGTC_SHIFT) | (otp->offs);
	trim_regs_p[2] = (otp->vbg << LX7730_VBG_SHIFT) | (otp->vtoi >> 2);
	trim_regs_p[3] = (otp->vtoi << LX7730_VTOI_L_SHIFT) | (otp->osc << LX7730_OSC_SHIFT) | (otp->ADCvtoi >> 3);
	trim_regs_p[4] = (otp->ADCvtoi << LX7730_ADC_VTOI_L_SHIFT);

	for(trim_address = LX7730_REG_TRIM1_ADDR; trim_address < LX7730_REG_TRIM6_ADDR; trim_address++)
	{
		ret = LX7730_write_register(trim_address, trim_regs_p[trim_address - LX7730_REG_TRIM1_ADDR]);
		if(ret != RESULT_OK){return ret;}
	}

	return ret;
}
