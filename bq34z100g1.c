/*
 * bq34z100g1.c
 *
 *  Created on: Nov 5, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */


#include "bq34z100g1.h"
#include "main.h"


BQ34Z100_SOC_t hsoc;
bq34z100_status_e Control_status;
bq34z100_flags_e flags;
bq34z100_flags_b_e flags_b;
uint8_t flash_block_data[32];

void bq34z100g1_process(void)
{
  hsoc.control_status = bq34z100g1_control_status();
  hsoc.flags = bq34z100g1_flags();
  hsoc.flags_b = bq34z100g1_flags_b();
  hsoc.voltage = bq34z100g1_voltage();
  hsoc.current = bq34z100g1_current();
  hsoc.state_of_charge = bq34z100g1_state_of_charge();
  hsoc.state_of_health = bq34z100g1_state_of_health();

}


uint32_t bq34z100g1_read_register(uint8_t address, uint8_t length)
{
  uint8_t returnBuf[4];
  uint32_t returnVal = 0;

  HAL_I2C_Mem_Read(I2C_BMS, BQ34Z100_G1_ADDRESS, address, 1, returnBuf, length, 1000);

  for(uint8_t i = 0; i < length; i++)
    returnVal = returnVal + (returnBuf[i] << (8 * i));

  return returnVal;
}

uint16_t bq34z100g1_read_control(uint8_t address)
{
  uint8_t pData[2] = {address, 0x00};
  //HAL_I2C_Master_Transmit(I2C_BMS, BQ34Z100_G1_ADDRESS, pData, 3, 100);
  HAL_I2C_Mem_Write(I2C_BMS, BQ34Z100_G1_ADDRESS, 0x00, 1, pData, 2, 1000);
  HAL_I2C_Mem_Read(I2C_BMS, BQ34Z100_G1_ADDRESS, 0x00, 1, pData, 2, 100);
  uint16_t returnValue = pData[0] + (pData[1] << 8);
  return returnValue;
}

void bq34z100g1_read_flash_block(uint8_t sub_class, uint8_t offset)
{
  uint8_t pData[2];
  pData[0] = 0x61;
  pData[1] = 0x00;
  HAL_I2C_Master_Transmit(I2C_BMS, BQ34Z100_G1_ADDRESS, pData, 2, 100);
  HAL_Delay(30);//mimic bq2300

  pData[0] = 0x3e;
  pData[1] = sub_class;
  HAL_I2C_Master_Transmit(I2C_BMS, BQ34Z100_G1_ADDRESS, pData, 2, 100);
  HAL_Delay(30);//mimic bq2300

  pData[0] = 0x3f;
  pData[1] = offset/32;
  HAL_I2C_Master_Transmit(I2C_BMS, BQ34Z100_G1_ADDRESS, pData, 2, 100);
  HAL_Delay(30);//mimic bq2300

  HAL_I2C_Mem_Read(I2C_BMS, BQ34Z100_G1_ADDRESS, 0x40, 1, flash_block_data, 32, 100);
  HAL_Delay(10);
}

void bq34z100g1_write_reg(uint8_t addr, uint8_t val)
{
    HAL_I2C_Mem_Write(I2C_BMS, BQ34Z100_G1_ADDRESS, addr, sizeof(addr), &val, sizeof(val), 100);
}

void bq34z100g1_write_flash_block(uint8_t sub_class, uint8_t offset)
{
    uint8_t pData[2];
    pData[0] = 0x61;
    pData[1] = 0x00;
    HAL_I2C_Master_Transmit(I2C_BMS, BQ34Z100_G1_ADDRESS, pData, 2, 100);
    HAL_Delay(30);//mimic bq2300

    pData[0] = 0x3e;
    pData[1] = sub_class;
    HAL_I2C_Master_Transmit(I2C_BMS, BQ34Z100_G1_ADDRESS, pData, 2, 100);
   HAL_Delay(30);//mimic bq2300

    pData[0] = 0x3f;
    pData[1] = offset / 32;
    HAL_I2C_Master_Transmit(I2C_BMS, BQ34Z100_G1_ADDRESS, pData, 2, 100);
   HAL_Delay(30);//mimic bq2300

    HAL_I2C_Mem_Write(I2C_BMS, BQ34Z100_G1_ADDRESS, 0x40, 1, flash_block_data, 32, 100);
}

uint8_t bq34z100g1_flash_block_checksum()
{
    uint8_t temp = 0;
    for (uint8_t i = 0; i < 32; i++) {
        temp += flash_block_data[i];
    }
    return 255 - temp;
}

double bq34z100g1_xemics_to_double(uint32_t value)
{
    bool is_negetive = false;
    if (value & 0x800000) {
        is_negetive = true;
    }
    int16_t exp_gain = (value >> 24) - 128 - 24;
    double exponent = pow(2, exp_gain);
    double mantissa = (int32_t)((value & 0xffffff) | 0x800000);
    if (is_negetive) {
        return mantissa * exponent * -1;
    }
    return mantissa * exponent;
}

uint32_t bq34z100g1_double_to_xemics(double value)
{
    bool is_negetive = false;
    if (value < 0) {
        is_negetive = true;
        value *= -1;
    }
    int8_t exponent;
    if (value > 1) {
        exponent = (log(value) / log(2)) + 1;
    } else {
        exponent = (log(value) / log(2));
    }
    double mantissa = value / (pow(2, (double)exponent) * pow(2, -24));
    if (is_negetive) {
        return (((exponent + 128) << 24) | (uint32_t)mantissa) | 0x800000;
    }
    return ((exponent + 128) << 24) | ((uint32_t)mantissa & 0x7fffff);
}

HAL_StatusTypeDef bq34z100g1_unsealed()
{
  HAL_StatusTypeDef result = HAL_ERROR;
  uint8_t pData[2];

  /*send first 2 bytes of UNSEAL key using Control(0x0414)*/
  pData[0] = 0x14;
  pData[1] = 0x04;
  result = HAL_I2C_Mem_Write(I2C_BMS, BQ34Z100_G1_ADDRESS, 0x00, 1, pData, 2, 1000);

  /*send second 2 bytes of UNSEAL key using Control(0x3672)*/
  pData[0] = 0x72;
  pData[1] = 0x36;
  result = HAL_I2C_Mem_Write(I2C_BMS, BQ34Z100_G1_ADDRESS, 0x00, 1, pData, 2, 1000);

  return result;
}

void bq34z100g1_enter_calibration()
{
    bq34z100g1_unsealed();
    do
    {
      bq34z100g1_cal_enable();
      bq34z100g1_enter_cal();
      HAL_Delay(1000);
    } while (!(bq34z100g1_control_status() & 0x1000)); // CALEN
}

void bq34z100g1_exit_calibration()
{
    do
    {
      bq34z100g1_exit_cal();
      HAL_Delay(1000);
    } while (!(bq34z100g1_control_status() &~ 0x1000)); // CALEN

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);
}
bool bq34z100g1_update_design_capacity(int16_t capacity)
{
    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(48, 0);

    flash_block_data[6] = 0; // Cycle Count
    flash_block_data[7] = 0;

    flash_block_data[8] = capacity >> 8; // CC Threshold
    flash_block_data[9] = capacity & 0xff;

    flash_block_data[11] = capacity >> 8; // Design Capacity
    flash_block_data[12] = capacity & 0xff;

    for (uint8_t i = 6; i <= 9; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    for (uint8_t i = 11; i <= 12; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(48, 0);
    int16_t updated_cc_threshold = flash_block_data[8] << 8;
    updated_cc_threshold |= flash_block_data[9];

    int16_t updated_capacity = flash_block_data[11] << 8;
    updated_capacity |= flash_block_data[12];

    if (flash_block_data[6] != 0 || flash_block_data[7] != 0) {
        return false;
    }
    if (capacity != updated_cc_threshold) {
        return false;
    }
    if (capacity != updated_capacity) {
        return false;
    }
    return true;
}

bool bq34z100g1_update_q_max(int16_t capacity)
{
    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(82, 0);
    flash_block_data[0] = capacity >> 8; // Q Max
    flash_block_data[1] = capacity & 0xff;

    flash_block_data[2] = 0; // Cycle Count
    flash_block_data[3] = 0;

    for (uint8_t i = 0; i <= 3; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(82, 0);
    int16_t updated_q_max = flash_block_data[0] << 8;
    updated_q_max |= flash_block_data[1];

    if (capacity != updated_q_max) {
        return false;
    }
    return true;
}

bool bq34z100g1_update_design_energy(int16_t energy)
{
    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(48, 0);
    flash_block_data[13] = energy >> 8; // Design Energy
    flash_block_data[14] = energy & 0xff;

    for (uint8_t i = 13; i <= 14; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(48, 0);
    int16_t updated_energy = flash_block_data[13] << 8;
    updated_energy |= flash_block_data[14];

    if (energy != updated_energy) {
        return false;
    }
    return true;
}

bool bq34z100g1_update_cell_charge_voltage_range(uint16_t t1_t2, uint16_t t2_t3, uint16_t t3_t4)
{
    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(48, 0);

    flash_block_data[17] = t1_t2 >> 8; // Cell Charge Voltage T1-T2
    flash_block_data[18] = t1_t2 & 0xff;

    flash_block_data[19] = t2_t3 >> 8; // Cell Charge Voltage T2-T3
    flash_block_data[20] = t2_t3 & 0xff;

    flash_block_data[21] = t3_t4 >> 8; // Cell Charge Voltage T3-T4
    flash_block_data[22] = t3_t4 & 0xff;

    for (uint8_t i = 17; i <= 22; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(48, 0);
    uint16_t updated_t1_t2 = flash_block_data[17] << 8;
    updated_t1_t2 |= flash_block_data[18];

    uint16_t updated_t2_t3 = flash_block_data[19] << 8;
    updated_t2_t3 |= flash_block_data[20];

    uint16_t updated_t3_t4 = flash_block_data[21] << 8;
    updated_t3_t4 |= flash_block_data[22];

    if (t1_t2 != updated_t1_t2 || t2_t3 != updated_t2_t3 || t3_t4 != updated_t3_t4) {
        return false;
    }
    return true;
}

bool bq34z100g1_update_number_of_series_cells(uint8_t cells)
{
    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(64, 0);

    flash_block_data[7] = cells; // Number of Series Cell

    bq34z100g1_write_reg(0x40 + 7, flash_block_data[7]);

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(64, 0);

    if (cells != flash_block_data[7]) {
        return false;
    }
    return true;
}

bool bq34z100g1_update_pack_configuration(uint16_t config)
{
    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(64, 0);

    flash_block_data[0] = config >> 8; // Pack Configuration
    flash_block_data[1] = config & 0xff;

    for (uint8_t i = 0; i <= 1; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(64, 0);
    uint16_t updated_config = flash_block_data[0] << 8;
    updated_config |= flash_block_data[1];
    if (config != updated_config) {
        return false;
    }
    return true;
}

bool bq34z100g1_update_charge_termination_parameters(int16_t taper_current, int16_t min_taper_capacity, int16_t cell_taper_voltage, uint8_t taper_window, int8_t tca_set, int8_t tca_clear, int8_t fc_set, int8_t fc_clear)
{
    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(36, 0);

    flash_block_data[0] = taper_current >> 8; // Taper Current
    flash_block_data[1] = taper_current & 0xff;

    flash_block_data[2] = min_taper_capacity >> 8; // Min Taper Capacity
    flash_block_data[3] = min_taper_capacity & 0xff;

    flash_block_data[4] = cell_taper_voltage >> 8; // Cell Taper Voltage
    flash_block_data[5] = cell_taper_voltage & 0xff;

    flash_block_data[6] = taper_window; // Current Taper Window

    flash_block_data[7] = tca_set & 0xff; // TCA Set %

    flash_block_data[8] = tca_clear & 0xff; // TCA Clear %

    flash_block_data[9] = fc_set & 0xff; // FC Set %

    flash_block_data[10] = fc_clear & 0xff; // FC Clear %

    for (uint8_t i = 0; i <= 10; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(36, 0);
    int16_t updated_taper_current, updated_min_taper_capacity, updated_cell_taper_voltage;
    uint8_t updated_taper_window;
    int8_t updated_tca_set, updated_tca_clear, updated_fc_set, updated_fc_clear;

    updated_taper_current = flash_block_data[0] << 8;
    updated_taper_current |= flash_block_data[1];

    updated_min_taper_capacity = flash_block_data[2] << 8;
    updated_min_taper_capacity |= flash_block_data[3];

    updated_cell_taper_voltage = flash_block_data[4] << 8;
    updated_cell_taper_voltage |= flash_block_data[5];

    updated_taper_window = flash_block_data[6];

    updated_tca_set = flash_block_data[7] & 0xff;

    updated_tca_clear = flash_block_data[8] & 0xff;

    updated_fc_set = flash_block_data[9] & 0xff;

    updated_fc_clear = flash_block_data[10] & 0xff;

    if (taper_current != updated_taper_current) {
        return false;
    }
    if (min_taper_capacity != updated_min_taper_capacity) {
        return false;
    }
    if (cell_taper_voltage != updated_cell_taper_voltage) {
        return false;
    }
    if (taper_window != updated_taper_window) {
        return false;
    }
    if (tca_set != updated_tca_set) {
        return false;
    }
    if (tca_clear != updated_tca_clear) {
        return false;
    }
    if (fc_set != updated_fc_set) {
        return false;
    }
    if (fc_clear != updated_fc_clear) {
        return false;
    }
    return true;
}

void bq34z100g1_calibrate_cc_offset()
{
    bq34z100g1_enter_calibration();
    do {
        bq34z100g1_cc_offset();
        HAL_Delay(1000);
    } while (!(bq34z100g1_control_status() & 0x0800)); // CCA

    do {
        HAL_Delay(1000);
    } while (!(bq34z100g1_control_status() &~ 0x0800)); // CCA

    bq34z100g1_cc_offset_save();
    bq34z100g1_exit_calibration();
}

void bq34z100g1_calibrate_board_offset() {
    bq34z100g1_enter_calibration();
    do {
        bq34z100g1_board_offset();
        HAL_Delay(1000);
    } while (!(bq34z100g1_control_status() & 0x0c00)); // CCA + BCA

    do {
        HAL_Delay(1000);
    } while (!(bq34z100g1_control_status() &~ 0x0c00)); // CCA + BCA

    bq34z100g1_cc_offset_save();
    bq34z100g1_exit_calibration();
}

void bq34z100g1_calibrate_voltage_divider(uint16_t applied_voltage, uint8_t cells_count)
{
    double volt_array[50];
    for (uint8_t i = 0; i < 50; i++) {
        volt_array[i] = bq34z100g1_voltage();
        HAL_Delay(150);
    }
    double volt_mean = 0;
    for (uint8_t i = 0; i < 50; i++) {
        volt_mean += volt_array[i];
    }
    volt_mean /= 50.0;

    double volt_sd = 0;
    for (uint8_t i = 0; i < 50; i++) {
        volt_sd += pow(volt_array[i] - volt_mean, 2);
    }
    volt_sd /= 50.0;
    volt_sd = sqrt(volt_sd);

    if (volt_sd > 100) {
        return;
    }

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(104, 0);

    uint16_t current_voltage_divider = flash_block_data[14] << 8;
    current_voltage_divider |= flash_block_data[15];

    uint16_t new_voltage_divider = ((double)applied_voltage / volt_mean) * (double)current_voltage_divider;

    flash_block_data[14] = new_voltage_divider >> 8;
    flash_block_data[15] = new_voltage_divider & 0xff;

    for (uint8_t i = 14; i <= 15; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());
    HAL_Delay(150);

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(68, 0);

    int16_t flash_update_of_cell_voltage = (double)(2800 * cells_count * 5000) / (double)new_voltage_divider;

    flash_block_data[0] = flash_update_of_cell_voltage << 8;
    flash_block_data[1] = flash_update_of_cell_voltage & 0xff;

    for (uint8_t i = 0; i <= 1; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);
}

void bq34z100g1_calibrate_sense_resistor(int16_t applied_current)
{
    double current_array[50];
    for (uint8_t i = 0; i < 50; i++) {
        current_array[i] = bq34z100g1_current();
        HAL_Delay(150);
    }
    double current_mean = 0;
    for (uint8_t i = 0; i < 50; i++) {
        current_mean += current_array[i];
    }
    current_mean /= 50.0;

    double current_sd = 0;
    for (uint8_t i = 0; i < 50; i++) {
        current_sd += pow(current_array[i] - current_mean, 2);
    }
    current_sd /= 50.0;
    current_sd = sqrt(current_sd);

    if (current_sd > 100) {
        return;
    }

    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(104, 0);

    uint32_t cc_gain = flash_block_data[0] << 24;
    cc_gain |= flash_block_data[1] << 16;
    cc_gain |= flash_block_data[2] << 8;
    cc_gain |= flash_block_data[3];

    double gain_resistence = 4.768 / bq34z100g1_xemics_to_double(cc_gain);

    double temp = (current_mean * gain_resistence) / (double)applied_current;

    uint32_t new_cc_gain = bq34z100g1_double_to_xemics(4.768 / temp);
    flash_block_data[0] = new_cc_gain >> 24;
    flash_block_data[1] = new_cc_gain >> 16;
    flash_block_data[2] = new_cc_gain >> 8;
    flash_block_data[3] = new_cc_gain & 0xff;

    new_cc_gain = bq34z100g1_double_to_xemics(5677445.6 / temp);
    flash_block_data[4] = new_cc_gain >> 24;
    flash_block_data[5] = new_cc_gain >> 16;
    flash_block_data[6] = new_cc_gain >> 8;
    flash_block_data[7] = new_cc_gain & 0xff;


    for (uint8_t i = 0; i <= 3; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    for (uint8_t i = 4; i <= 7; i++) {
        bq34z100g1_write_reg(0x40 + i, flash_block_data[i]);
    }

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());
    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);
}

void bq34z100g1_set_current_deadband(uint8_t deadband)
{
    bq34z100g1_unsealed();
    bq34z100g1_read_flash_block(107, 0);

    flash_block_data[1] = deadband;

    bq34z100g1_write_reg(0x40 + 1, flash_block_data[1]);

    bq34z100g1_write_reg(0x60, bq34z100g1_flash_block_checksum());

    HAL_Delay(150);
    bq34z100g1_reset();
    HAL_Delay(150);
}

void bq34z100g1_ready()
{
    bq34z100g1_unsealed();
    bq34z100g1_it_enable();
    bq34z100g1_sealed();
}

uint16_t bq34z100g1_control_status()
{
    return bq34z100g1_read_control(0x00);
}

uint16_t bq34z100g1_device_type()
{
    return bq34z100g1_read_control(0x01);
}

uint16_t bq34z100g1_fw_version()
{
    return bq34z100g1_read_control(0x02);
}

uint16_t bq34z100g1_hw_version()
{
    return bq34z100g1_read_control(0x03);
}

uint16_t bq34z100g1_reset_data()
{
    return bq34z100g1_read_control(0x05);
}

uint16_t bq34z100g1_prev_macwrite()
{
    return bq34z100g1_read_control(0x07);
}

uint16_t bq34z100g1_chem_id()
{
    return bq34z100g1_read_control(0x08);
}

uint16_t bq34z100g1_board_offset()
{
    return bq34z100g1_read_control(0x09);
}

uint16_t bq34z100g1_cc_offset()
{
    return bq34z100g1_read_control(0x0a);
}

uint16_t bq34z100g1_cc_offset_save()
{
    return bq34z100g1_read_control(0x0b);
}

uint16_t bq34z100g1_df_version()
{
    return bq34z100g1_read_control(0x0c);
}

uint16_t bq34z100g1_set_fullsleep()
{
    return bq34z100g1_read_control(0x10);
}

uint16_t bq34z100g1_static_chem_chksum()
{
    return bq34z100g1_read_control(0x17);
}

uint16_t bq34z100g1_sealed()
{
    return bq34z100g1_read_control(0x20);
}

uint16_t bq34z100g1_it_enable()
{
    return bq34z100g1_read_control(0x21);
}

uint16_t bq34z100g1_cal_enable()
{
    return bq34z100g1_read_control(0x2d);
}

uint16_t bq34z100g1_reset()
{
    return bq34z100g1_read_control(0x41);
}

uint16_t bq34z100g1_exit_cal()
{
    return bq34z100g1_read_control(0x80);
}

uint16_t bq34z100g1_enter_cal()
{
    return bq34z100g1_read_control(0x81);
}

uint16_t bq34z100g1_offset_cal()
{
    return bq34z100g1_read_control(0x82);
}

uint8_t bq34z100g1_state_of_charge()
{
    return (uint8_t)bq34z100g1_read_register(0x02, 1);
}

uint8_t bq34z100g1_state_of_charge_max_error()
{
    return (uint8_t)bq34z100g1_read_register(0x03, 1);
}

uint16_t bq34z100g1_remaining_capacity()
{
    return bq34z100g1_read_register(0x04, 2);
}

uint16_t bq34z100g1_full_charge_capacity()
{
    return bq34z100g1_read_register(0x06, 2);
}

uint16_t bq34z100g1_voltage()
{
    return bq34z100g1_read_register(0x08, 2);
}

int16_t bq34z100g1_average_current()
{
    return (int16_t)bq34z100g1_read_register(0x0a, 2);
}

uint16_t bq34z100g1_temperature()
{
    return bq34z100g1_read_register(0x0c, 2);
}

uint16_t bq34z100g1_flags()
{
    return bq34z100g1_read_register(0x0e, 2);
}

uint16_t bq34z100g1_flags_b()
{
    return bq34z100g1_read_register(0x12, 2);
}

int16_t bq34z100g1_current()
{
    return (int16_t)bq34z100g1_read_register(0x10, 2);
}

uint16_t bq34z100g1_average_time_to_empty()
{
    return bq34z100g1_read_register(0x18, 2);
}

uint16_t bq34z100g1_average_time_to_full()
{
    return bq34z100g1_read_register(0x1a, 2);
}

int16_t bq34z100g1_passed_charge()
{
    return bq34z100g1_read_register(0x1c, 2);
}

uint16_t bq34z100g1_do_d0_time()
{
    return bq34z100g1_read_register(0x1e, 2);
}

uint16_t bq34z100g1_available_energy()
{
    return bq34z100g1_read_register(0x24, 2);
}

uint16_t bq34z100g1_average_power()
{
    return bq34z100g1_read_register(0x26, 2);
}

uint16_t bq34z100g1_serial_number()
{
    return bq34z100g1_read_register(0x28, 2);
}

uint16_t bq34z100g1_internal_temperature()
{
    return bq34z100g1_read_register(0x2a, 2);
}

uint16_t bq34z100g1_cycle_count()
{
    return bq34z100g1_read_register(0x2c, 2);
}

uint16_t bq34z100g1_state_of_health()
{
    return bq34z100g1_read_register(0x2e, 2);
}

uint16_t bq34z100g1_charge_voltage()
{
    return bq34z100g1_read_register(0x30, 2);
}

uint16_t bq34z100g1_charge_current()
{
    return bq34z100g1_read_register(0x32, 2);
}

uint16_t bq34z100g1_pack_configuration()
{
    return bq34z100g1_read_register(0x3a, 2);
}

uint16_t bq34z100g1_design_capacity()
{
    return bq34z100g1_read_register(0x3c, 2);
}

uint8_t bq34z100g1_grid_number()
{
    return (uint8_t)bq34z100g1_read_register(0x62, 1);
}

uint8_t bq34z100g1_learned_status()
{
    return (uint8_t)bq34z100g1_read_register(0x63, 1);
}

uint16_t bq34z100g1_dod_at_eoc()
{
    return bq34z100g1_read_register(0x64, 2);
}

uint16_t bq34z100g1_q_start()
{
    return bq34z100g1_read_register(0x66, 2);
}

uint16_t bq34z100g1_true_fcc()
{
    return bq34z100g1_read_register(0x6a, 2);
}

uint16_t bq34z100g1_state_time()
{
    return bq34z100g1_read_register(0x6c, 2);
}

uint16_t bq34z100g1_q_max_passed_q()
{
    return bq34z100g1_read_register(0x6e, 2);
}

uint16_t bq34z100g1_dod_0()
{
    return bq34z100g1_read_register(0x70, 2);
}

uint16_t bq34z100g1_q_max_dod_0()
{
    return bq34z100g1_read_register(0x72, 2);
}

uint16_t bq34z100g1_q_max_time()
{
    return bq34z100g1_read_register(0x74, 2);
}
