/**************************************************************************************************
* Copyright (C) 2019-2021 Maxim Integrated Products, Inc. All Rights Reserved.
*
* Maxim Integrated Products, Inc. Default Copyright Notice:
* https://www.maximintegrated.com/en/aboutus/legal/copyrights.html
**************************************************************************************************/

/*
 * This header file was automatically @generated for the imu_fixed_inputs_no_softmax network from a template.
 * Please do not edit; instead, edit the template and regenerate.
 */

#ifndef __CNN_H__
#define __CNN_H__

#include <stdint.h>
typedef int32_t q31_t;
typedef int16_t q15_t;

/* Return codes */
#define CNN_FAIL 0
#define CNN_OK 1

/*
  SUMMARY OF OPS
  Hardware: 1,715,382 ops (1,703,817 macc; 11,565 comp; 0 add; 0 mul; 0 bitwise)
    Layer 0 (trans_conv1_ConvTranspose_8): 622,080 ops (622,080 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 1 (trans_conv2_ConvTranspose_6): 663,552 ops (663,552 macc; 0 comp; 0 add; 0 mul; 0 bitwise)
    Layer 2 (conv1_Conv_6): 336,384 ops (331,776 macc; 4,608 comp; 0 add; 0 mul; 0 bitwise)
    Layer 3 (conv2_Conv_6): 88,704 ops (82,944 macc; 5,760 comp; 0 add; 0 mul; 0 bitwise)
    Layer 4 (conv3_Conv_6): 4,437 ops (3,240 macc; 1,197 comp; 0 add; 0 mul; 0 bitwise)
    Layer 5 (fc_Gemm_4): 225 ops (225 macc; 0 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 7,209 bytes out of 442,368 bytes total (1.6%)
  Bias memory:   50 bytes out of 2,048 bytes total (2.4%)
*/

/* Number of outputs for this network */
#define CNN_NUM_OUTPUTS 3

/* Port pin actions used to signal that processing is active */

#define CNN_START LED_On(1)
#define CNN_COMPLETE LED_Off(1)
#define SYS_START LED_On(0)
#define SYS_COMPLETE LED_Off(0)

/* Run software SoftMax on unloaded data */
void softmax_q17p14_q15(const q31_t * vec_in, const uint16_t dim_vec, q15_t * p_out);
/* Shift the input, then calculate SoftMax */
void softmax_shift_q17p14_q15(q31_t * vec_in, const uint16_t dim_vec, uint8_t in_shift, q15_t * p_out);

/* Stopwatch - holds the runtime when accelerator finishes */
extern volatile uint32_t cnn_time;

/* Custom memcopy routines used for weights and data */
void memcpy32(uint32_t *dst, const uint32_t *src, int n);
void memcpy32_const(uint32_t *dst, int n);

/* Enable clocks and power to accelerator, enable interrupt */
int cnn_enable(uint32_t clock_source, uint32_t clock_divider);

/* Disable clocks and power to accelerator */
int cnn_disable(void);

/* Perform minimum accelerator initialization so it can be configured */
int cnn_init(void);

/* Configure accelerator for the given network */
int cnn_configure(void);

/* Load accelerator weights */
int cnn_load_weights(void);

/* Verify accelerator weights (debug only) */
int cnn_verify_weights(void);

/* Load accelerator bias values (if needed) */
int cnn_load_bias(void);

/* Start accelerator processing */
int cnn_start(void);

/* Force stop accelerator */
int cnn_stop(void);

/* Continue accelerator after stop */
int cnn_continue(void);

/* Unload results from accelerator */
int cnn_unload(uint32_t *out_buf);

/* Turn on the boost circuit */
int cnn_boost_enable(mxc_gpio_regs_t *port, uint32_t pin);

/* Turn off the boost circuit */
int cnn_boost_disable(mxc_gpio_regs_t *port, uint32_t pin);

#endif // __CNN_H__
