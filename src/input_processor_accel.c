#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>  // abs()

#define DT_DRV_COMPAT zmk_input_processor_acceleration

#ifndef CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY
#define CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY 50
#endif

#define ACCEL_MAX_CODES 4
#define SCALE 1000

static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state);

struct accel_config {
    uint8_t  input_type;
    const uint16_t *codes;          /* e.g. REL_X, REL_Y */
    uint32_t codes_count;
    bool     track_remainders;
    uint16_t min_factor;            /* x1000 (500 = 0.5x, 1000 = 1.0x) */
    uint16_t max_factor;            /* x1000 */
    uint32_t speed_threshold;       /* cps: factor достигает 1.0 (или min_factor, если >1) */
    uint32_t speed_max;             /* cps: factor достигает max_factor */
    uint8_t  acceleration_exponent; /* 1=linear, 2=quadratic, ... */
};

struct accel_data {
    int64_t last_time_ms[ACCEL_MAX_CODES];  /* per-axis timestamp */
    int32_t last_phys[ACCEL_MAX_CODES];     /* last raw delta per axis */
    int16_t remainders[ACCEL_MAX_CODES];    /* thousandths remainder per axis */
};

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline bool code_to_index(const struct accel_config *cfg, uint16_t code, uint32_t *out_idx) {
    for (uint32_t i = 0; i < cfg->codes_count; ++i) {
        if (cfg->codes[i] == code) {
            if (out_idx) *out_idx = i;
            return true;
        }
    }
    return false;
}

static uint32_t pow_scaled(uint32_t t, uint8_t exp) {
    t = clamp_u32(t, 0, SCALE);
    if (exp <= 1) return t;
    uint64_t acc = t;
    for (uint8_t i = 1; i < exp; ++i) {
        acc = (acc * t) / SCALE;
    }
    if (acc > UINT32_MAX) acc = UINT32_MAX;
    return (uint32_t)acc;
}

static uint32_t compute_factor_scaled(const struct accel_config *cfg, uint32_t cps) {
    const uint32_t f_min = clamp_u32(cfg->min_factor, 100, 20000);  /* 0.1x..20x */
    const uint32_t f_max = clamp_u32(cfg->max_factor, f_min, 20000);
    const uint32_t v1 = cfg->speed_threshold;
    const uint32_t v2 = (cfg->speed_max > v1) ? cfg->speed_max : (v1 + 1);
    const uint8_t  e  = cfg->acceleration_exponent ? cfg->acceleration_exponent : 1;

    const uint32_t base = (f_min > 1000) ? f_min : 1000;

    if (cps <= v1) {
        uint32_t t = (v1 == 0) ? SCALE : (uint32_t)((uint64_t)cps * SCALE / v1);
        uint32_t shaped = pow_scaled(t, e);
        int32_t span = (int32_t)base - (int32_t)f_min;
        int32_t f = (int32_t)f_min + (int32_t)((int64_t)span * shaped / SCALE);
        return clamp_u32((uint32_t)f, f_min, base);
    } else if (cps >= v2) {
        return f_max;
    } else {
        uint32_t t = (uint32_t)((uint64_t)(cps - v1) * SCALE / (v2 - v1));
        uint32_t shaped = pow_scaled(t, e);
        int32_t span = (int32_t)f_max - (int32_t)base;
        int32_t f = (int32_t)base + (int32_t)((int64_t)span * shaped / SCALE);
        return clamp_u32((uint32_t)f, base, f_max);
    }
}

static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    if (event->type != cfg->input_type) {
        return 0;
    }

    uint32_t idx = 0;
    if (!code_to_index(cfg, event->code, &idx)) {
        return 0;
    }

    const int32_t raw = event->value;
    const int64_t now = k_uptime_get();

    if (raw == 0) {
        data->last_time_ms[idx] = now;
        return 0;
    }

    uint32_t dt_ms = 1;
    if (data->last_time_ms[idx] > 0 && now > data->last_time_ms[idx]) {
        int64_t diff = now - data->last_time_ms[idx];
        if (diff > 100) diff = 100;        
        dt_ms = (uint32_t)diff;
    }

    uint32_t cps = (uint32_t)(( (uint64_t)abs(raw) * 1000ULL ) / dt_ms);

    uint32_t factor = compute_factor_scaled(cfg, cps);

    if ((int64_t)data->last_phys[idx] * (int64_t)raw < 0 && factor > 1000) {
        factor = 1000;
    }

    if (cfg->track_remainders) {
        int64_t total = (int64_t)raw * (int64_t)factor + (int64_t)data->remainders[idx];
        int32_t out = (int32_t)(total / SCALE);
        int32_t rem = (int32_t)(total - (int64_t)out * SCALE);  /* [-999..999] */
        if (out > 32767) out = 32767;
        if (out < -32768) out = -32768;
        event->value = out;
        data->remainders[idx] = (int16_t)rem;
    } else {
        event->value = (int32_t)(((int64_t)raw * (int64_t)factor) / SCALE);
    }

    data->last_phys[idx] = raw;
    data->last_time_ms[idx] = now;
    return 0;
}

#define ACCEL_INST_INIT(inst)                                                      \
    static const uint16_t accel_codes_##inst[] = { INPUT_REL_X, INPUT_REL_Y };    \
    static const struct accel_config accel_config_##inst = {                      \
        .input_type = DT_INST_PROP_OR(inst, input_type, INPUT_EV_REL),            \
        .codes = accel_codes_##inst,                                              \
        .codes_count = 2,                                                         \
        .track_remainders = DT_INST_NODE_HAS_PROP(inst, track_remainders),        \
        .min_factor = DT_INST_PROP_OR(inst, min_factor, 1000),                    \
        .max_factor = DT_INST_PROP_OR(inst, max_factor, 3500),                    \
        .speed_threshold = DT_INST_PROP_OR(inst, speed_threshold, 1000),          \
        .speed_max = DT_INST_PROP_OR(inst, speed_max, 6000),                      \
        .acceleration_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, 1), \
    };                                                                            \
    static struct accel_data accel_data_##inst = {0};                             \
    DEVICE_DT_INST_DEFINE(inst,                                                   \
                          NULL,                                                   \
                          NULL,                                                   \
                          &accel_data_##inst,                                     \
                          &accel_config_##inst,                                   \
                          POST_KERNEL,                                            \
                          CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY,               \
                          &(const struct zmk_input_processor_driver_api){         \
                              .handle_event = accel_handle_event,                 \
                          });

DT_INST_FOREACH_STATUS_OKAY(ACCEL_INST_INIT)
