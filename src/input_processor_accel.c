#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

/* Maximum number of event codes this processor can handle (e.g. REL_X, REL_Y). */
#define ACCEL_MAX_CODES 4

/* Configuration from devicetree (constant for each instance) */
struct accel_config {
    uint8_t input_type;                  /* Event type to process (e.g. INPUT_EV_REL) */
    const uint16_t *codes;         /* Array of event code values to accelerate (e.g. REL_X, REL_Y) */
    uint32_t codes_count;          /* Number of codes in the array */
    bool track_remainders;         /* Whether to accumulate fractional movement remainders */
    uint16_t min_factor;           /* Minimum acceleration factor (scaled by 1000, e.g. 500 = 0.5x) */
    uint16_t max_factor;           /* Maximum acceleration factor (scaled by 1000, e.g. 3500 = 3.5x) */
    uint32_t speed_threshold;      /* Speed (counts per second) at which factor reaches 1.0 */
    uint32_t speed_max;            /* Speed (counts per second) at which factor reaches max_factor */
    uint8_t  acceleration_exponent;/* Exponent for acceleration curve (1=linear, 2=quadratic, etc.) */
};

/* Runtime state for each instance (mutable data) */
struct accel_data {
    int64_t last_time;                     /* Timestamp of last processed event (ms) */
    int32_t last_phys_dx;                  /* Last physical X delta (for direction check) */
    int32_t last_phys_dy;                  /* Last physical Y delta (for direction check) */
    uint16_t last_code;                    /* Last event code processed (e.g. REL_X or REL_Y) */
    int16_t remainders[ACCEL_MAX_CODES];   /* Remainder values for fractional movements per code */
};

/* Instantiate for each DT node matching our compatible */
DT_INST_FOREACH_STATUS_OKAY(ACCEL_INST_INIT)

/* Event handler implementation */
static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);

    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    /* Only handle the configured event type (e.g. relative events) */
    if (event->type != cfg->type) {
        return 0;  /* Not our event type, leave it unchanged */
    }
    /* Only handle specified codes (e.g. REL_X, REL_Y); others pass through */
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
        return 0;  /* Not an event code we process */
    }

    /* If acceleration exponent is 0, feature is disabled – pass event through */
    if (cfg->acceleration_exponent == 0) {
        data->last_time = k_uptime_get();        /* update timestamp for completeness */
        data->last_phys_dx = (event->code == INPUT_REL_X) ? event->value : data->last_phys_dx;
        data->last_phys_dy = (event->code == INPUT_REL_Y) ? event->value : data->last_phys_dy;
        data->last_code = event->code;
        return 0;
    }

    /* Calculate time since last event (in milliseconds) */
    int64_t now = k_uptime_get();
    int64_t dt_ms;
    if (data->last_time == 0) {
        dt_ms = now;  /* first event – treat as a long delay to avoid initial acceleration */
    } else {
        dt_ms = now - data->last_time;
    }
    if (dt_ms < 1) {
        dt_ms = 1;    /* avoid division by zero, clamp minimal dt to 1ms */
    }

    /* Determine movement distance for velocity calculation */
    int32_t orig_val = event->value;  /* preserve original input movement value */
    uint32_t dist;
    if (dt_ms <= 1 && data->last_code != event->code) {
        /* If events for different axes come almost simultaneously (same frame), 
           combine their deltas for velocity calculation (approximate diagonal speed). */
        if (event->code == INPUT_REL_X) {
            dist = abs(orig_val) + abs(data->last_phys_dy);
        } else {  /* event->code == INPUT_REL_Y */
            dist = abs(orig_val) + abs(data->last_phys_dx);
        }
    } else {
        dist = abs(orig_val);
    }

    /* Compute velocity in counts per second (scaled) */
    uint32_t velocity_cps = (uint64_t)dist * 1000U / (uint32_t)dt_ms;

    /* Determine acceleration factor based on velocity */
    uint16_t min_fact = cfg->min_factor;
    uint16_t max_fact = cfg->max_factor;
    uint32_t v1 = cfg->speed_threshold;
    uint32_t v2 = cfg->speed_max;
    uint8_t exp = cfg->acceleration_exponent;
    uint32_t factor_int = 1000;  /* default factor = 1.000 */

    if (velocity_cps <= v1) {
        /* Low speeds: decelerate linearly down to min_factor */
        if (min_fact < 1000) {
            if (v1 > 0) {
                uint32_t interp = (uint32_t)velocity_cps * 1000U / v1;
                // interp is 0–1000 representing velocity_cps/v1
                factor_int = min_fact + ((1000U - min_fact) * interp) / 1000U;
            } else {
                factor_int = 1000;
            }
        } else {
            factor_int = 1000;
        }
    } else if (velocity_cps >= v2) {
        /* High speeds: cap at max_factor */
        factor_int = max_fact;
    } else {
        /* Mid-to-high speeds: accelerate from 1.0 to max_factor */
        uint32_t ratio = (uint32_t)(velocity_cps - v1) * 1000U / (v2 - v1);
        if (exp <= 1) {
            /* Linear interpolation */
            factor_int = 1000 + ((max_fact - 1000) * ratio) / 1000U;
        } else {
            /* Exponential (polynomial) curve: (ratio/1000)^exp, scaled to 1000 */
            uint64_t scaled = ratio;
            uint64_t result = ratio;
            for (uint8_t i = 2; i <= exp; i++) {
                result = (result * scaled) / 1000U;
            }
            uint32_t ratio_exp = (uint32_t)result;  /* (ratio^exp)*1000^(1-exp) result in [0,1000] */
            factor_int = 1000 + ((max_fact - 1000) * ratio_exp) / 1000U;
        }
    }

    /* Detect direction reversal: if this axis changed sign relative to last motion on same axis */
    bool reversed = false;
    if ((event->code == INPUT_REL_X && data->last_phys_dx != 0 && (int64_t)data->last_phys_dx * orig_val < 0) ||
        (event->code == INPUT_REL_Y && data->last_phys_dy != 0 && (int64_t)data->last_phys_dy * orig_val < 0)) {
        reversed = true;
    }
    if (reversed && factor_int > 1000) {
        /* On immediate direction reversal, suppress acceleration (limit factor to 1x for this event) */
        factor_int = 1000;
    }

    /* Apply acceleration factor to the movement value */
    if (cfg->track_remainders) {
        /* Use remainder accumulation to preserve fractional movement */
        int64_t total = (int64_t)orig_val * factor_int + data->remainders[code_index];
        int32_t output = total / 1000;         /* integer output delta */
        int32_t rem = (int32_t)(total - (int64_t)output * 1000);  /* new remainder (carry-over) */
        /* Store updated output and remainder */
        event->value = output;
        data->remainders[code_index] = (int16_t)rem;
    } else {
        /* Simple scaling (fractional part truncated) */
        event->value = (int32_t)(((int64_t)orig_val * factor_int) / 1000);
        /* (No remainder tracking, small motions under factor may be lost) */
    }

    /* Update last event state for next time */
    data->last_time = now;
    if (event->code == INPUT_REL_X) {
        data->last_phys_dx = orig_val;
    } else if (event->code == INPUT_REL_Y) {
        data->last_phys_dy = orig_val;
    }
    data->last_code = event->code;

    return 0;
}


/* Forward declaration of event handler function */
static int accel_handle_event(const struct device *dev, struct input_event *event,
    uint32_t param1, uint32_t param2,
    struct zmk_input_processor_state *state);

/* Populate config and data for each instance from devicetree */
#define ACCEL_INST_INIT(inst)                                                    \
BUILD_ASSERT(DT_INST_PROP_LEN(inst, codes) <= ACCEL_MAX_CODES,               \
"Too many codes in acceleration processor instance " #inst);    \
static const uint16_t accel_codes_##inst[] = DT_INST_PROP(inst, codes);      \
static const struct accel_config accel_config_##inst = {                     \
.type = DT_INST_PROP(inst, type),                                        \
.codes = accel_codes_##inst,                                             \
.codes_count = DT_INST_PROP_LEN(inst, codes),                            \
.track_remainders = DT_INST_NODE_HAS_PROP(inst, track_remainders),       \
.min_factor = DT_INST_PROP_OR(inst, min_factor, 1000),   /* default 1.0 */\
.max_factor = DT_INST_PROP_OR(inst, max_factor, 3500),   /* default 3.5 */\
.speed_threshold = DT_INST_PROP_OR(inst, speed_threshold, 1000),         \
.speed_max = DT_INST_PROP_OR(inst, speed_max, 6000),                     \
.acceleration_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, 1) \
};                                                                          \
static struct accel_data accel_data_##inst;                                  \
DEVICE_DT_INST_DEFINE(inst, NULL, NULL,                                      \
&accel_data_##inst, &accel_config_##inst,              \
APPLICATION, CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY, \
&(const struct zmk_input_processor_driver_api){        \
    .handle_event = accel_handle_event                \
});

