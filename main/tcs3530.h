/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/

/*!
 * @file tcs3530.h
 * @brief This file contains API for TCS3530 Click Driver.
 */

#ifndef TCS3530_H
#define TCS3530_H

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief TCS3530 register list.
 * @details Specified register list of TCS3530 Click driver.
 */
#define TCS3530_REG_CONTROL_SCL                     0x24
#define TCS3530_REG_MOD_OFFSET0_LSB                 0x40
#define TCS3530_REG_MOD_OFFSET0_MSB                 0x41
#define TCS3530_REG_MOD_OFFSET1_LSB                 0x42
#define TCS3530_REG_MOD_OFFSET1_MSB                 0x43
#define TCS3530_REG_MOD_OFFSET2_LSB                 0x44
#define TCS3530_REG_MOD_OFFSET2_MSB                 0x45
#define TCS3530_REG_MOD_OFFSET3_LSB                 0x46
#define TCS3530_REG_MOD_OFFSET3_MSB                 0x47
#define TCS3530_REG_MOD_OFFSET4_LSB                 0x48
#define TCS3530_REG_MOD_OFFSET4_MSB                 0x49
#define TCS3530_REG_MOD_OFFSET5_LSB                 0x4A
#define TCS3530_REG_MOD_OFFSET5_MSB                 0x4B
#define TCS3530_REG_MOD_OFFSET6_LSB                 0x4C
#define TCS3530_REG_MOD_OFFSET6_MSB                 0x4D
#define TCS3530_REG_MOD_OFFSET7_LSB                 0x4E
#define TCS3530_REG_MOD_OFFSET7_MSB                 0x4F
#define TCS3530_REG_OSCEN                           0x7F
#define TCS3530_REG_ENABLE                          0x80
#define TCS3530_REG_MEAS_MODE0                      0x81
#define TCS3530_REG_MEAS_MODE1                      0x82
#define TCS3530_REG_SAMPLE_TIME0                    0x83
#define TCS3530_REG_SAMPLE_TIME1                    0x84
#define TCS3530_REG_SAMPLE_TIME_ALTERNATIVE0        0x85
#define TCS3530_REG_SAMPLE_TIME_ALTERNATIVE1        0x86
#define TCS3530_REG_ALS_NR_SAMPLES0                 0x87
#define TCS3530_REG_ALS_NR_SAMPLES1                 0x88
#define TCS3530_REG_ALS_NR_SAMPLES_ALTERNATIVE0     0x89
#define TCS3530_REG_ALS_NR_SAMPLES_ALTERNATIVE1     0x8A
#define TCS3530_REG_FD_NR_SAMPLES0                  0x8B
#define TCS3530_REG_FD_NR_SAMPLES1                  0x8C
#define TCS3530_REG_FD_NR_SAMPLES_ALTERNATIVE0      0x8D
#define TCS3530_REG_FD_NR_SAMPLES_ALTERNATIVE1      0x8E
#define TCS3530_REG_WTIME                           0x8F
#define TCS3530_REG_AUX_ID                          0x90
#define TCS3530_REG_REV_ID                          0x91
#define TCS3530_REG_ID                              0x92
#define TCS3530_REG_AILT0                           0x93
#define TCS3530_REG_AILT1                           0x94
#define TCS3530_REG_AILT2                           0x95
#define TCS3530_REG_AIHT0                           0x96
#define TCS3530_REG_AIHT1                           0x97
#define TCS3530_REG_AIHT2                           0x98
#define TCS3530_REG_AGC_NR_SAMPLES0                 0x99
#define TCS3530_REG_AGC_NR_SAMPLES1                 0x9A
#define TCS3530_REG_STATUS                          0x9B
#define TCS3530_REG_STATUS2                         0x9C
#define TCS3530_REG_STATUS3                         0x9D
#define TCS3530_REG_STATUS4                         0x9E
#define TCS3530_REG_STATUS5                         0x9F
#define TCS3530_REG_STATUS6                         0xA0
#define TCS3530_REG_CFG0                            0xA1
#define TCS3530_REG_CFG1                            0xA2
#define TCS3530_REG_CFG2                            0xA3
#define TCS3530_REG_CFG3                            0xA4
#define TCS3530_REG_CFG4                            0xA5
#define TCS3530_REG_CFG5                            0xA6
#define TCS3530_REG_CFG6                            0xA7
#define TCS3530_REG_CFG7                            0xA8
#define TCS3530_REG_CFG8                            0xA9
#define TCS3530_REG_CFG9                            0xAA
#define TCS3530_REG_MOD_CHANNEL_CTRL                0xAB
#define TCS3530_REG_TRIGGER_MODE                    0xAD
#define TCS3530_REG_OSC_TUNE                        0xAE
#define TCS3530_REG_VSYNC_GPIO_INT                  0xB0
#define TCS3530_REG_INTENAB                         0xBA
#define TCS3530_REG_SIEN                            0xBB
#define TCS3530_REG_CONTROL                         0xBC
#define TCS3530_REG_ALS_DATA_STATUS                 0xBD
#define TCS3530_REG_ALS_DATA_FIRST                  0xBE
#define TCS3530_REG_ALS_DATA                        0xBF
#define TCS3530_REG_MEAS_SEQR_STEP0_MOD_GAINX_0     0xC0
#define TCS3530_REG_MEAS_SEQR_STEP0_MOD_GAINX_1     0xC1
#define TCS3530_REG_MEAS_SEQR_STEP0_MOD_GAINX_2     0xC2
#define TCS3530_REG_MEAS_SEQR_STEP0_MOD_GAINX_3     0xC3
#define TCS3530_REG_MEAS_SEQR_STEP1_MOD_GAINX_0     0xC4
#define TCS3530_REG_MEAS_SEQR_STEP1_MOD_GAINX_1     0xC5
#define TCS3530_REG_MEAS_SEQR_STEP1_MOD_GAINX_2     0xC6
#define TCS3530_REG_MEAS_SEQR_STEP1_MOD_GAINX_3     0xC7
#define TCS3530_REG_MEAS_SEQR_STEP2_MOD_GAINX_0     0xC8
#define TCS3530_REG_MEAS_SEQR_STEP2_MOD_GAINX_1     0xC9
#define TCS3530_REG_MEAS_SEQR_STEP2_MOD_GAINX_2     0xCA
#define TCS3530_REG_MEAS_SEQR_STEP2_MOD_GAINX_3     0xCB
#define TCS3530_REG_MEAS_SEQR_STEP3_MOD_GAINX_0     0xCC
#define TCS3530_REG_MEAS_SEQR_STEP3_MOD_GAINX_1     0xCD
#define TCS3530_REG_MEAS_SEQR_STEP3_MOD_GAINX_2     0xCE
#define TCS3530_REG_MEAS_SEQR_STEP3_MOD_GAINX_3     0xCF
#define TCS3530_REG_MEAS_SEQR_STEP0_FD              0xD0
#define TCS3530_REG_MEAS_SEQR_STEP1_FD              0xD1
#define TCS3530_REG_MEAS_SEQR_STEP2_FD              0xD2
#define TCS3530_REG_MEAS_SEQR_STEP3_FD              0xD3
#define TCS3530_REG_MEAS_SEQR_STEP0_RESIDUAL        0xD4
#define TCS3530_REG_MEAS_SEQR_STEP1_RESIDUAL        0xD5
#define TCS3530_REG_MEAS_SEQR_STEP2_RESIDUAL        0xD6
#define TCS3530_REG_MEAS_SEQR_STEP3_RESIDUAL        0xD7
#define TCS3530_REG_MEAS_SEQR_STEP0_ALS             0xD8
#define TCS3530_REG_MEAS_SEQR_STEP1_ALS             0xD9
#define TCS3530_REG_MEAS_SEQR_STEP2_ALS             0xDA
#define TCS3530_REG_MEAS_SEQR_STEP3_ALS             0xDB
#define TCS3530_REG_MEAS_SEQR_APERS_AND_VSYNC_WAIT  0xDC
#define TCS3530_REG_MEAS_SEQR_AGC                   0xDD
#define TCS3530_REG_MEAS_SEQR_SMUX_AND_SAMPLE_TIME  0xDE
#define TCS3530_REG_MEAS_SEQR_WAIT_AND_TS_ENABLE    0xDF
#define TCS3530_REG_MOD_CALIB_CFG0                  0xE0
#define TCS3530_REG_MOD_CALIB_CFG2                  0xE2
#define TCS3530_REG_MOD_CALIB_CFG3                  0xE3
#define TCS3530_REG_MOD_COMP_CFG2                   0xE7
#define TCS3530_REG_MOD_RESIDUAL_CFG0               0xE8
#define TCS3530_REG_MOD_RESIDUAL_CFG1               0xE9
#define TCS3530_REG_MOD_RESIDUAL_CFG2               0xEA
#define TCS3530_REG_VSYNC_DELAY_CFG0                0xEB
#define TCS3530_REG_VSYNC_DELAY_CFG1                0xEC
#define TCS3530_REG_VSYNC_PERIOD0                   0xED
#define TCS3530_REG_VSYNC_PERIOD1                   0xEE
#define TCS3530_REG_VSYNC_PERIOD_TARGET0            0xEF
#define TCS3530_REG_VSYNC_PERIOD_TARGET1            0xF0
#define TCS3530_REG_VSYNC_CONTROL                   0xF1
#define TCS3530_REG_VSYNC_CFG                       0xF2
#define TCS3530_REG_FIFO_THR                        0xF3
#define TCS3530_REG_MOD_FIFO_DATA_CFG0              0xF4
#define TCS3530_REG_MOD_FIFO_DATA_CFG1              0xF5
#define TCS3530_REG_MOD_FIFO_DATA_CFG2              0xF6
#define TCS3530_REG_MOD_FIFO_DATA_CFG3              0xF7
#define TCS3530_REG_MOD_FIFO_DATA_CFG4              0xF8
#define TCS3530_REG_MOD_FIFO_DATA_CFG5              0xF9
#define TCS3530_REG_MOD_FIFO_DATA_CFG6              0xFA
#define TCS3530_REG_MOD_FIFO_DATA_CFG7              0xFB
#define TCS3530_REG_FIFO_STATUS0                    0xFC
#define TCS3530_REG_FIFO_STATUS1                    0xFD
#define TCS3530_REG_FIFO_DATA_PROTOCOL              0xFE
#define TCS3530_REG_FIFO_DATA                       0xFF

/**
 * @brief TCS3530 CONTROL_SCL register setting.
 * @details Specified setting for CONTROL_SCL register of TCS3530 Click driver.
 */
#define TCS3530_CONTROL_SCL_SOFT_RESET              0x01

/**
 * @brief TCS3530 OSCEN register setting.
 * @details Specified setting for OSCEN register of TCS3530 Click driver.
 */
#define TCS3530_OSCEN_PON_INIT                      0x04
#define TCS3530_OSCEN_OSCEN_STATUS                  0x02
#define TCS3530_OSCEN_OSCEN                         0x01

/**
 * @brief TCS3530 ENABLE register setting.
 * @details Specified setting for ENABLE register of TCS3530 Click driver.
 */
#define TCS3530_ENABLE_FDEN                         0x40
#define TCS3530_ENABLE_AEN                          0x02
#define TCS3530_ENABLE_PON                          0x01

/**
 * @brief TCS3530 MEAS_MODE0 register setting.
 * @details Specified setting for MEAS_MODE0 register of TCS3530 Click driver.
 */
#define TCS3530_MEAS_MODE0_STOP_AFTER_NTH_ITERATION                 0x80
#define TCS3530_MEAS_MODE0_ENABLE_AGC__ASAT_DOUBLE_STEP_DOWN        0x40
#define TCS3530_MEAS_MODE0_MEASUREMENT_SEQUENCER_SINGLE_SHOT_MODE   0x20
#define TCS3530_MEAS_MODE0_MOD_FIFO_ALS_STATUS_WRITE_ENABLE         0x10
#define TCS3530_MEAS_MODE0_ALS_SCALE                                0x0F


/**
 * @brief TCS3530 WTIME register setting.
 * @details Specified setting for WTIME register of TCS3530 Click driver.
 */
#define TCS3530_WTIME_DEFAULT                       0x46

/**
 * @brief TCS3530 TRIGGER_MODE register setting.
 * @details Specified setting for TRIGGER_MODE register of TCS3530 Click driver.
 */
#define TCS3530_TRIGGER_MODE_OFF                    0x00
#define TCS3530_TRIGGER_MODE_NORMAL                 0x01
#define TCS3530_TRIGGER_MODE_LONG                   0x02
#define TCS3530_TRIGGER_MODE_FAST                   0x03
#define TCS3530_TRIGGER_MODE_FASTLONG               0x04
#define TCS3530_TRIGGER_MODE_VSYNC                  0x05
#define TCS3530_TRIGGER_MODE_MASK                   0x07

/**
 * @brief TCS3530 CFG4 register setting.
 * @details Specified setting for CFG4 register of TCS3530 Click driver.
 */
#define TCS3530_CFG4_MEASUREMENT_SEQUENCER_MAX_MOD_GAIN                     0xf0
#define TCS3530_CFG4_MEASUREMENT_SEQUENCER_AGC_PREDICT_MOD_GAIN_REDUCTION   0x0f

/**
 * @brief TCS3530 INTENAB register setting.
 * @details Specified setting for INTENAB register of TCS3530 Click driver.
 */
#define TCS3530_INTENAB_MIEN                        0x80
#define TCS3530_INTENAB_AIEN                        0x08
#define TCS3530_INTENAB_FIEN                        0x04
#define TCS3530_INTENAB_SIEN                        0x01

/**
 * @brief TCS3530 FIFO interrupt threshold setting.
 * @details Specified setting for FIFO interrupt threshold of TCS3530 Click driver.
 */
#define TCS3530_FIFO_THR_DEFAULT                    15

/**
 * @brief TCS3530 CONTROL register setting.
 * @details Specified setting for CONTROL register of TCS3530 Click driver.
 */
#define TCS3530_CONTROL_FIFO_CLR                    0x02
#define TCS3530_CONTROL_CLEAR_SAI_ACTIVE            0x01

/**
 * @brief TCS3530 device ID value.
 * @details Specified device ID value of TCS3530 Click driver.
 */
#define TCS3530_DEVICE_ID                           0x68

/**
 * @brief TCS3530 device address setting.
 * @details Specified setting for device slave address selection of
 * TCS3530 Click driver.
 */
#define TCS3530_DEVICE_ADDRESS                      0x39

/**
 * @brief TCS3530 Click data object.
 * @details Data object definition of TCS3530 Click driver.
 */
typedef struct
{
    uint16_t x;                 /**< X/RED (MOD0) data counts. */
    uint16_t y;                 /**< Y/GREEN (MOD1) data counts. */
    uint16_t z;                 /**< Z/BLUE (MOD2) data counts. */
    uint16_t ir;                /**< IR (MOD3) data counts. */
    uint16_t hgl;               /**< HgL (MOD4) data counts. */
    uint16_t hgh;               /**< HgH (MOD5) data counts. */
    uint16_t clear;             /**< Clear (MOD6) data counts. */
    uint16_t flicker;           /**< Flicker (MOD7) data counts. */
    
} tcs3530_data_t;

#ifdef __cplusplus
}
#endif
#endif // TCS3530_H

/*! @} */ // tcs3530

// ------------------------------------------------------------------------ END
