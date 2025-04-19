#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>  // For abs() function

#define DT_DRV_COMPAT zmk_input_processor_acceleration

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
    int64_t last_time[ACCEL_MAX_CODES];         /* Timestamp per axis */
    int16_t remainders[ACCEL_MAX_CODES];        /* Remainder values per axis */
};

/* Populate config and data for each instance from devicetree */
#define ACCEL_INST_INIT(inst)                                                  \
static const uint16_t accel_codes_##inst[] = { INPUT_REL_X, INPUT_REL_Y };     \
static const struct accel_config accel_config_##inst = {                       \
    .input_type = DT_INST_PROP_OR(inst, input_type, INPUT_EV_REL),             \
    .codes = accel_codes_##inst,                                               \
    .codes_count = 2,                                                          \
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

    /* Get the current timestamp */
    int64_t current_time = k_uptime_get();
    int64_t delta_time = current_time - data->last_time[code_index];

    /* Skip processing if delta_time is too small (e.g., first event or noise) */
    if (delta_time <= 0) {
        data->last_time[code_index] = current_time;
        return 0;
    }

    /* Calculate speed (counts per second) */
    int32_t delta = event->value;
    float speed = (float)abs(delta) / ((float)delta_time / 1000.0f); // Speed in counts/sec

    /* Determine acceleration factor */
    float factor = 1.0f; // Default factor (no acceleration)
    if (speed > cfg->speed_threshold) {
        float normalized_speed = (speed - cfg->speed_threshold) /
                                 (cfg->speed_max - cfg->speed_threshold);
        normalized_speed = normalized_speed > 1.0f ? 1.0f : normalized_speed;

        factor = cfg->min_factor / 1000.0f +
                 (cfg->max_factor / 1000.0f - cfg->min_factor / 1000.0f) *
                     powf(normalized_speed, cfg->acceleration_exponent);
    }

    /* Apply acceleration factor */
    float adjusted_delta = delta * factor;

    /* Track remainders if enabled */
    if (cfg->track_remainders) {
        adjusted_delta += data->remainders[code_index];
        data->remainders[code_index] = (int16_t)(adjusted_delta - (int32_t)adjusted_delta);
    }

    /* Update the event value */
    event->value = (int32_t)adjusted_delta;

    /* Update the last timestamp for this axis */
    data->last_time[code_index] = current_time;

    return 0;
}
