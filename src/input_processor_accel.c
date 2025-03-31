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
    ARG_UNUSED(param2);
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    /* Обрабатываем только события заданного типа */
    if (event->type != cfg->input_type {
        return 0;
    }

    /* Обрабатываем только указанные коды событий */
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

    /* Если ускорение отключено (exponent==0), просто обновляем состояния и выходим */
    if (cfg->acceleration_exponent == 0) {
        data->last_phys_dx = (event->code == INPUT_REL_X) ? event->value : data->last_phys_dx;
        data->last_phys_dy = (event->code == INPUT_REL_Y) ? event->value : data->last_phys_dy;
        data->last_code = event->code;
        return 0;
    }

    /* Вместо k_uptime_get() используем param1 как dt в мс, передаваемое извне */
    int64_t dt_ms = param1;
    if (dt_ms < 1) {
        dt_ms = 1;    /* защита от деления на 0 */
    }

    int32_t orig_val = event->value;  /* оригинальное значение перемещения */
    uint32_t dist;
    if (dt_ms <= 1 && data->last_code != event->code) {
        if (event->code == INPUT_REL_X) {
            dist = abs(orig_val) + abs(data->last_phys_dy);
        } else {  /* INPUT_REL_Y */
            dist = abs(orig_val) + abs(data->last_phys_dx);
        }
    } else {
        dist = abs(orig_val);
    }

    uint32_t velocity_cps = (uint64_t)dist * 1000U / (uint32_t)dt_ms;
    uint16_t min_fact = cfg->min_factor;
    uint16_t max_fact = cfg->max_factor;
    uint32_t v1 = cfg->speed_threshold;
    uint32_t v2 = cfg->speed_max;
    uint8_t exp = cfg->acceleration_exponent;
    uint32_t factor_int = 1000;  /* базовый коэффициент 1.000 */

    if (velocity_cps <= v1) {
        if (min_fact < 1000) {
            if (v1 > 0) {
                uint32_t interp = velocity_cps * 1000U / v1;
                factor_int = min_fact + ((1000U - min_fact) * interp) / 1000U;
            } else {
                factor_int = 1000;
            }
        } else {
            factor_int = 1000;
        }
    } else if (velocity_cps >= v2) {
        factor_int = max_fact;
    } else {
        uint32_t ratio = (velocity_cps - v1) * 1000U / (v2 - v1);
        if (exp <= 1) {
            factor_int = 1000 + ((max_fact - 1000) * ratio) / 1000U;
        } else {
            uint64_t scaled = ratio;
            uint64_t result = ratio;
            for (uint8_t i = 2; i <= exp; i++) {
                result = (result * scaled) / 1000U;
            }
            uint32_t ratio_exp = (uint32_t)result;
            factor_int = 1000 + ((max_fact - 1000) * ratio_exp) / 1000U;
        }
    }

    bool reversed = false;
    if ((event->code == INPUT_REL_X && data->last_phys_dx != 0 && (int64_t)data->last_phys_dx * orig_val < 0) ||
        (event->code == INPUT_REL_Y && data->last_phys_dy != 0 && (int64_t)data->last_phys_dy * orig_val < 0)) {
        reversed = true;
    }
    if (reversed && factor_int > 1000) {
        factor_int = 1000;
    }

    if (cfg->track_remainders) {
        int64_t total = (int64_t)orig_val * factor_int + data->remainders[code_index];
        int32_t output = total / 1000;
        int32_t rem = (int32_t)(total - (int64_t)output * 1000);
        event->value = output;
        data->remainders[code_index] = (int16_t)rem;
    } else {
        event->value = (int32_t)(((int64_t)orig_val * factor_int) / 1000);
    }

    /* Обновляем предыдущие значения для осей */
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
static const uint16_t accel_codes_##inst[] = { INPUT_REL_X, INPUT_REL_Y };      \
static const struct accel_config accel_config_##inst = {                     \
    .type = DT_INST_PROP_OR(inst, type, INPUT_EV_REL),                       \
    .codes = accel_codes_##inst,                                               \
    .codes_count = 2,                                                          \
    .track_remainders = DT_INST_NODE_HAS_PROP(inst, track_remainders),         \
    .min_factor = DT_INST_PROP_OR(inst, min_factor, 1000),   /* default 1.0 */   \
    .max_factor = DT_INST_PROP_OR(inst, max_factor, 3500),   /* default 3.5 */   \
    .speed_threshold = DT_INST_PROP_OR(inst, speed_threshold, 1000),           \
    .speed_max = DT_INST_PROP_OR(inst, speed_max, 6000),                       \
    .acceleration_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, 1)   \
};                                                                             \
static struct accel_data accel_data_##inst;                                    \
DEVICE_DT_INST_DEFINE(inst, NULL, (struct pm_device *)DEVICE_DT_GET(DT_NODELABEL(glidepoint)),    \
                      &accel_data_##inst, &accel_config_##inst,                \
                      APPLICATION, CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY,   \
                      &(const struct zmk_input_processor_driver_api){          \
                          .handle_event = accel_handle_event                   \
                      })

