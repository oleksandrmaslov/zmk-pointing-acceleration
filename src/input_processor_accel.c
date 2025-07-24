#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>  // For abs() function

#define DT_DRV_COMPAT zmk_input_processor_acceleration
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Forward declaration of the event handler */
static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state);

/* Maximum number of event codes this processor can handle (e.g. REL_X, REL_Y). */
#define ACCEL_MAX_CODES 4

/* Configuration from devicetree (constant for each instance) */
struct accel_config {
    uint8_t input_type;                  /* Event type to process (e.g. INPUT_EV_REL) */
    const uint16_t *codes;               /* Array of event code values to accelerate (e.g. REL_X, REL_Y) */
    uint32_t codes_count;                /* Number of codes in the array */
    bool track_remainders;               /* Whether to accumulate fractional movement remainders */
    uint16_t min_factor;                 /* Minimum acceleration factor (scaled by 1000, e.g. 500 = 0.5x) */
    uint16_t max_factor;                 /* Maximum acceleration factor (scaled by 1000, e.g. 3500 = 3.5x) */
    uint32_t speed_threshold;            /* Speed (counts per second) at which factor reaches 1.0 */
    uint32_t speed_max;                  /* Speed (counts per second) at which factor reaches max_factor */
    uint8_t  acceleration_exponent;      /* Exponent for acceleration curve (1=linear, 2=quadratic, etc.) */
};

/* Runtime state for each instance (mutable data) */
struct accel_data {
    int64_t last_time;                   /* Timestamp of last processed event (ms) */
    int32_t last_phys_dx;                /* Last physical X delta (for direction check) */
    int32_t last_phys_dy;                /* Last physical Y delta (for direction check) */
    uint16_t last_code;                  /* Last event code processed (e.g. REL_X or REL_Y) */
    int16_t remainders[ACCEL_MAX_CODES]; /* Remainder values for fractional movements per code */
};

/* Populate config and data for each instance from devicetree */
#define ACCEL_INST_INIT(inst)                                                  \
static const uint16_t accel_codes_##inst[] = { INPUT_REL_X, INPUT_REL_Y };     \
static const struct accel_config accel_config_##inst = {                       \
    .input_type = DT_INST_PROP_OR(inst, input_type, INPUT_EV_REL),             \
    .codes = accel_codes_##inst,                                               \
    .codes_count = ARRAY_SIZE(accel_codes_##inst),                                                          \
    .track_remainders = DT_INST_NODE_HAS_PROP(inst, track_remainders),         \
    .min_factor = DT_INST_PROP_OR(inst, min_factor, 1000),                     \
    .max_factor = DT_INST_PROP_OR(inst, max_factor, 3500),                     \
    .speed_threshold = DT_INST_PROP_OR(inst, speed_threshold, 1000),           \
    .speed_max = DT_INST_PROP_OR(inst, speed_max, 6000),                       \
    .acceleration_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, 1)   \
};                                                                             \
static struct accel_data accel_data_##inst = {0};                              \
DEVICE_DT_INST_DEFINE(inst,                                                    \
                      NULL,                                                    \
                      NULL,                                                    \
                      &accel_data_##inst,                                      \
                      &accel_config_##inst,                                    \
                      POST_KERNEL,                                             \
                      CONFIG_INPUT_PROCESSOR_ACCELERATION_INIT_PRIORITY,       \
                      &(const struct zmk_input_processor_driver_api){          \
                          .handle_event = accel_handle_event                   \
                      });

/* Instantiate for each DT node matching our compatible */
DT_INST_FOREACH_STATUS_OKAY(ACCEL_INST_INIT)

/* Event handler implementation */
static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    /* Process only events of the specified type */
    if (event->type != cfg->input_type) {
        return 0;
    }

    /* Process only the specified event codes */
    bool code_matched = false;
    uint32_t code_index = 0;
    for (uint32_t i = 0; i < cfg->codes_count; ++i) {
        if (event->code == cfg->codes[i]) {
            code_index = i;
            code_matched = true;
            break;
        }
    }
    if (!code_matched) {
        return 0;
    }

        /* Skip zero-value events */
    if (event->value == 0) {
        return 0;
    }

    /* Validate code_index bounds */
    if (code_index >= ACCEL_MAX_CODES) {
        return 0;
    }

    /* Get current timestamp */
    int64_t current_time = k_uptime_get();
    
    /* Calculate time delta and speed */
    int64_t time_delta = current_time - data->last_time;
    if (time_delta <= 0) {
        time_delta = 1; /* Avoid division by zero */
    }
    
    /* Calculate speed in counts per second */
    uint32_t speed = (abs(event->value) * 1000) / time_delta;
    
    /* Calculate acceleration factor based on speed */
    uint16_t factor = cfg->min_factor;
    
    if (speed > cfg->speed_threshold) {
        if (speed >= cfg->speed_max) {
            factor = cfg->max_factor;
        } else {
            /* Interpolate between min and max factor based on speed */
            uint32_t speed_range = cfg->speed_max - cfg->speed_threshold;
            uint32_t factor_range = cfg->max_factor - cfg->min_factor;
            uint32_t speed_offset = speed - cfg->speed_threshold;
            
            /* Apply acceleration exponent */
            uint32_t normalized_speed = (speed_offset * 1000) / speed_range;
            uint32_t accelerated_speed = normalized_speed;
            
            /* Simple exponent implementation for common cases */
            if (cfg->acceleration_exponent == 2) {
                accelerated_speed = (normalized_speed * normalized_speed) / 1000;
            } else if (cfg->acceleration_exponent == 3) {
                accelerated_speed = (normalized_speed * normalized_speed * normalized_speed) / (1000 * 1000);
            }
            
            factor = cfg->min_factor + ((factor_range * accelerated_speed) / 1000);
            if (factor > cfg->max_factor) {
                factor = cfg->max_factor;
            }
        }
    }
    
    /* Apply acceleration factor */
    int32_t accelerated_value = (event->value * factor) / 1000;
    
    /* Handle remainders if enabled */
    if (cfg->track_remainders && code_index < ACCEL_MAX_CODES) {
        int32_t remainder = ((event->value * factor) % 1000) / 100; /* Scale to avoid overflow */
        data->remainders[code_index] += remainder;
        
        /* Add accumulated remainder to output */
        if (abs(data->remainders[code_index]) >= 10) {
            int32_t remainder_contribution = data->remainders[code_index] / 10;
            accelerated_value += remainder_contribution;
            data->remainders[code_index] -= remainder_contribution * 10;
        }
    }
    
    /* Update tracking data */
    data->last_time = current_time;
    data->last_code = event->code;
    if (event->code == INPUT_REL_X) {
        data->last_phys_dx = event->value;
    } else if (event->code == INPUT_REL_Y) {
        data->last_phys_dy = event->value;
    }
    
    /* Update event value with accelerated result */
    event->value = accelerated_value;

    return 0;
}
#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)