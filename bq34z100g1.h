/*
 * bq34z100g1.h
 *
 *  Created on: Nov 5, 2024
 *      Author: Dmitrij Sarychev
 *
 *	Description:
 * Copyright © 2024 - Dmitrij Sarychev, RAWLPLUG.
 */

#ifndef INC_BQ34Z100G1_H_
#define INC_BQ34Z100G1_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <main.h>
#include <math.h>
#include "i2c.h"


#define BQ34Z100_G1_ADDRESS 0xAA
//#define BQ34Z100_G1_ADDRESS 0x55

typedef struct
{
  int       current;
  uint16_t  voltage;
  uint8_t   state_of_charge;
  uint16_t  state_of_health;

  int       temp[10];
  int       capacity;
  uint16_t  status;
  uint16_t  flags;
  uint16_t  flags_b;
  int       remaining_charge;
  uint16_t  chem_id;
  uint16_t  control_status;
}BQ34Z100_SOC_t;

typedef enum
{
  QEN = 0,
  VOK,
  RUP_DIS,
  LDMD,
  SLEEP,
  FULL_SLEEP,

  CSV = 9,
  BCA,
  CCA,
  CALEN,
  SS,
  FAS
}bq34z100_status_e;

typedef enum
{
  DSG = 0,
  SOCF,
  SOC1,
  CF = 4,
  OCV = 7,
  CHG,
  FC,
  XCHG,
  CHG_INH,
  BATLOW,
  BATHI,
  OTD,
  OTC
}bq34z100_flags_e;

typedef enum
{
  DTRC = 9,
  DODEOC,
  FIRSTDOD = 13,
  LIFE,
  SOH
}bq34z100_flags_b_e;

extern BQ34Z100_SOC_t hsoc;
extern bq34z100_status_e Control_status;
extern bq34z100_flags_e flags;
extern bq34z100_flags_b_e flags_b;

/*
 1. Update design capacity.
 2. Update Q max.
 3. Update design energy.
 4. Update cell charge voltage range.
 5. Update number of series cells.
 6. Update pack configuration.
 7. Update charge termination parameters.
 8. Calibrate cc offset. (No current applied)
 9. Calibrate board offset. (No currrent applied)
 10. Calibrate voltage divider. (Apply known voltage and use that)
 11. Calibrate sense resistor. (Apply known current, current from battery is negetive)
 12. Set current deadband if current is non zero without load.
 13. Set ready and start learning cycle.
 */

    void bq34z100g1_process(void);
    uint32_t bq34z100g1_read_register(uint8_t address, uint8_t length);
    uint16_t bq34z100g1_read_control(uint8_t address);
    void bq34z100g1_read_flash_block(uint8_t sub_class, uint8_t offset);
    void bq34z100g1_write_reg(uint8_t address, uint8_t value);
    void bq34z100g1_write_flash_block(uint8_t sub_class, uint8_t offset);

    uint8_t bq34z100g1_flash_block_checksum(void);

    double bq34z100g1_xemics_to_double(uint32_t value);
    uint32_t bq34z100g1_double_to_xemics(double value);

    HAL_StatusTypeDef bq34z100g1_unsealed(void);
    void bq34z100g1_enter_calibration(void);
    void bq34z100g1_exit_calibration(void);



    bool bq34z100g1_update_design_capacity(int16_t capacity);
    bool bq34z100g1_update_q_max(int16_t capacity);
    bool bq34z100g1_update_design_energy(int16_t energy);
    bool bq34z100g1_update_cell_charge_voltage_range(uint16_t t1_t2, uint16_t t2_t3, uint16_t t3_t4);
    bool bq34z100g1_update_number_of_series_cells(uint8_t cells);
    bool bq34z100g1_update_pack_configuration(uint16_t config);
    bool bq34z100g1_update_charge_termination_parameters(int16_t taper_current, int16_t min_taper_capacity, int16_t cell_taper_voltage, uint8_t taper_window, int8_t tca_set, int8_t tca_clear, int8_t fc_set, int8_t fc_clear);
    void bq34z100g1_calibrate_cc_offset(void);
    void bq34z100g1_calibrate_board_offset(void);
    void bq34z100g1_calibrate_voltage_divider(uint16_t applied_voltage, uint8_t cells_count);
    void bq34z100g1_calibrate_sense_resistor(int16_t applied_current);
    void bq34z100g1_set_current_deadband(uint8_t deadband);
    void bq34z100g1_ready(void);

    uint16_t bq34z100g1_control_status(void);
    uint16_t bq34z100g1_device_type(void);
    uint16_t bq34z100g1_fw_version(void);
    uint16_t bq34z100g1_hw_version(void);
    uint16_t bq34z100g1_reset_data(void);
    uint16_t bq34z100g1_prev_macwrite(void);
    uint16_t bq34z100g1_chem_id(void);
    uint16_t bq34z100g1_board_offset(void);
    uint16_t bq34z100g1_cc_offset(void);
    uint16_t bq34z100g1_cc_offset_save(void);
    uint16_t bq34z100g1_df_version(void);
    uint16_t bq34z100g1_set_fullsleep(void);
    uint16_t bq34z100g1_static_chem_chksum(void);
    uint16_t bq34z100g1_sealed(void);
    uint16_t bq34z100g1_it_enable(void);
    uint16_t bq34z100g1_cal_enable(void);
    uint16_t bq34z100g1_reset(void);
    uint16_t bq34z100g1_exit_cal(void);
    uint16_t bq34z100g1_enter_cal(void);
    uint16_t bq34z100g1_offset_cal(void);

    uint8_t bq34z100g1_state_of_charge(void); // 0 to 100%
    uint8_t bq34z100g1_state_of_charge_max_error(void); // 1 to 100%
    uint16_t bq34z100g1_remaining_capacity(void); // mAh
    uint16_t bq34z100g1_full_charge_capacity(void); // mAh
    uint16_t bq34z100g1_voltage(void); // mV
    int16_t bq34z100g1_average_current(void); // mA
    uint16_t bq34z100g1_temperature(void); // Unit of x10 K
    uint16_t bq34z100g1_flags(void);
    uint16_t bq34z100g1_flags_b(void);
    int16_t bq34z100g1_current(void); // mA

    uint16_t bq34z100g1_average_time_to_empty(void); // Minutes
    uint16_t bq34z100g1_average_time_to_full(void); // Minutes
    int16_t bq34z100g1_passed_charge(void); // mAh
    uint16_t bq34z100g1_do_d0_time(void); // Minutes
    uint16_t bq34z100g1_available_energy(void); // 10 mWh
    uint16_t bq34z100g1_average_power(void); // 10 mW
    uint16_t bq34z100g1_serial_number(void);
    uint16_t bq34z100g1_internal_temperature(void); // Unit of x10 K
    uint16_t bq34z100g1_cycle_count(void); // Counts
    uint16_t bq34z100g1_state_of_health(void); // 0 to 100%
    uint16_t bq34z100g1_charge_voltage(void); // mV
    uint16_t bq34z100g1_charge_current(void); // mA
    uint16_t bq34z100g1_pack_configuration(void);
    uint16_t bq34z100g1_design_capacity(void); // mAh
    uint8_t bq34z100g1_grid_number(void);
    uint8_t bq34z100g1_learned_status(void);
    uint16_t bq34z100g1_dod_at_eoc(void);
    uint16_t bq34z100g1_q_start(void); // mAh
    uint16_t bq34z100g1_true_fcc(void); // mAh
    uint16_t bq34z100g1_state_time(void); // s
    uint16_t bq34z100g1_q_max_passed_q(void); // mAh
    uint16_t bq34z100g1_dod_0(void);
    uint16_t bq34z100g1_q_max_dod_0(void);
    uint16_t bq34z100g1_q_max_time(void);



#endif /* INC_BQ34Z100G1_H_ */
