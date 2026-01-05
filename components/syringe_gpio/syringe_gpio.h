#ifndef _SYRINGE_GPIO_
#define _SYRINGE_GPIO_

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"

#define STEP_MOTOR_ENABLE_LEVEL 0 // DM420 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 1 // DIR 1 means dispense
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

void syringe_gpio_init (const char *TAG, const gpio_num_t step, const gpio_num_t dir, const gpio_num_t en, const float feedrate);

bool syringe_gpio_readSetVolume (uint16_t *data);
bool syringe_gpio_writeSetVolume (const uint16_t msg);
bool syringe_gpio_cmdAspire (const uint16_t msg); 
bool syringe_gpio_cmdDispense (const uint16_t msg); 
bool syringe_gpio_cmdDispenseStepVolume (const uint16_t msg); 

/*
https://github.com/espressif/esp-idf/blob/v5.2.5/examples/peripherals/rmt/stepper_motor/main/stepper_motor_encoder.h
*/
#include <stdint.h>
#include "driver/rmt_encoder.h"

/**
 * @brief Stepper motor curve encoder configuration
 */
typedef struct {
    uint32_t resolution;    // Encoder resolution, in Hz
    uint32_t sample_points; // Sample points used for deceleration phase. Note: |end_freq_hz - start_freq_hz| >= sample_points
    uint32_t start_freq_hz; // Start frequency on the curve, in Hz
    uint32_t end_freq_hz;   // End frequency on the curve, in Hz
} stepper_motor_curve_encoder_config_t;

/**
 * @brief Stepper motor uniform encoder configuration
 */
typedef struct {
    uint32_t resolution; // Encoder resolution, in Hz
} stepper_motor_uniform_encoder_config_t;

/**
 * @brief Create stepper motor curve encoder
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating step motor encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_stepper_motor_curve_encoder(const stepper_motor_curve_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

/**
 * @brief Create RMT encoder for encoding step motor uniform phase into RMT symbols
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating step motor encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_stepper_motor_uniform_encoder(const stepper_motor_uniform_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

#endif // _SYRINGE_GPIO_