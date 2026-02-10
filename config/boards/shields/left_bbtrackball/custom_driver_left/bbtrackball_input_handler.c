/*
 * bbtrackball_input_handler.c - BB Trackball (严格限速模式)
 *
 * 算法：每个脉冲触发1个字，有最小间隔限制，防止跳太多
 * 类似黑莓手机：一格一格走，不加速，只限速
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_bbtrackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_REGISTER(bbtrackball_input_handler, LOG_LEVEL_INF);

/* ==== GPIO Pins (黑莓原版轨迹球) ==== */
#define DOWN_GPIO_PIN 9
#define LEFT_GPIO_PIN 12
#define UP_GPIO_PIN 5
#define RIGHT_GPIO_PIN 27

#define GPIO0_DEV DT_NODELABEL(gpio0)
#define GPIO1_DEV DT_NODELABEL(gpio1)

/* ==== 限速参数 ==== */
#define MIN_STEP_INTERVAL_MS 55     /* 最小步进间隔，越小越灵敏但越容易跳 */
#define KEY_PRESS_MS 25             /* 按键按下时间，模拟真实按键 */
#define SCROLL_INTERVAL_MS 80       /* 滚动模式的间隔 */

/* ==== 方向定义 ==== */
enum {
    DIR_LEFT = 0,
    DIR_RIGHT,
    DIR_UP,
    DIR_DOWN,
    DIR_COUNT
};

/* ==== 状态 ==== */
static bool space_pressed = false;
static const struct device *trackball_dev_ref = NULL;

/* ==== 每个方向的状态 ==== */
typedef struct {
    const struct device *gpio_dev;
    int pin;
    int last_state;
    uint32_t last_trigger;   /* 上次触发时间 */
    int pending_steps;       /* 待执行的步数 */
    bool key_pressed;        /* 当前是否按住 */
} DirState;

/* 方向修正：往左划→左，往右划→右，往上划→上，往下划→下 */
static DirState dir_states[DIR_COUNT] = {
    [DIR_LEFT]  = {DEVICE_DT_GET(GPIO0_DEV), RIGHT_GPIO_PIN, 1, 0, 0, false},
    [DIR_RIGHT] = {DEVICE_DT_GET(GPIO0_DEV), LEFT_GPIO_PIN, 1, 0, 0, false},
    [DIR_UP]    = {DEVICE_DT_GET(GPIO1_DEV), DOWN_GPIO_PIN, 1, 0, 0, false},
    [DIR_DOWN]  = {DEVICE_DT_GET(GPIO0_DEV), UP_GPIO_PIN, 1, 0, 0, false},
};

static struct gpio_callback gpio_cbs[DIR_COUNT];

/* ==== 定时器 ==== */
static struct k_work_delayable step_work;
static struct k_work_delayable key_release_works[DIR_COUNT];

/* ==== Device Config/Data ==== */
struct bbtrackball_dev_config {
    uint16_t x_input_code;
    uint16_t y_input_code;
};

struct bbtrackball_data {
    const struct device *dev;
};

/* ==== 发送方向键 ==== */
static void send_arrow_key(uint8_t keycode, bool pressed) {
    if (pressed) {
        zmk_hid_keyboard_press(keycode);
    } else {
        zmk_hid_keyboard_release(keycode);
    }
    zmk_endpoints_send_report(0x07);
}

/* ==== 触发一次方向键 (按下+释放) ==== */
static void trigger_key_press(uint8_t dir) {
    uint8_t keycode;
    switch (dir) {
        case DIR_LEFT:  keycode = 0x50; break; /* LEFT */
        case DIR_RIGHT: keycode = 0x4F; break; /* RIGHT */
        case DIR_UP:    keycode = 0x52; break; /* UP */
        case DIR_DOWN:  keycode = 0x51; break; /* DOWN */
        default: return;
    }

    send_arrow_key(keycode, true);
    send_arrow_key(keycode, false);
    LOG_DBG("Dir %d triggered", dir);
}

/* ==== 释放按键的work handler ==== */
static void key_release_handler(struct k_work *work) {
    for (int i = 0; i < DIR_COUNT; i++) {
        if (work == &key_release_works[i].work) {
            dir_states[i].key_pressed = false;
            return;
        }
    }
}

/* ==== Space Listener (滚动模式) ==== */
static int space_listener_cb(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev) return 0;

    if (ev->position == 60) {
        space_pressed = ev->state;
        LOG_INF("Space %s", space_pressed ? "HELD (scroll mode)" : "RELEASED");
    }
    return 0;
}

ZMK_LISTENER(space_listener, space_listener_cb);
ZMK_SUBSCRIPTION(space_listener, zmk_position_state_changed);

/* ==== GPIO 中断回调 (脉冲检测) ==== */
static void dir_edge_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t now = k_uptime_get_32();

    for (int i = 0; i < DIR_COUNT; i++) {
        DirState *d = &dir_states[i];
        if ((dev == d->gpio_dev) && (pins & BIT(d->pin))) {
            int val = gpio_pin_get(dev, d->pin);
            if (val != d->last_state) {
                d->last_state = val;
                /* 下降沿 = 一个脉冲 */
                if (val == 0) {
                    /* 检查是否已经过了最小间隔 */
                    if ((now - d->last_trigger) >= MIN_STEP_INTERVAL_MS) {
                        d->pending_steps++;
                        d->last_trigger = now;
                        LOG_DBG("Dir %d step pending (interval=%d)",
                                i, now - d->last_trigger);
                    }
                }
            }
            break;
        }
    }
}

/* ==== 步进处理 (定时执行) ==== */
static void step_handler(struct k_work *work) {
    /* X轴 (左右) */
    for (int axis = 0; axis < 2; axis++) {
        int dir_neg = axis == 0 ? DIR_LEFT : DIR_UP;
        int dir_pos = axis == 0 ? DIR_RIGHT : DIR_DOWN;
        DirState *d_neg = &dir_states[dir_neg];
        DirState *d_pos = &dir_states[dir_pos];

        /* 优先处理有pending的方向，避免同时触发 */
        if (d_neg->pending_steps > 0 && d_pos->pending_steps > 0) {
            /* 两个方向都有，先处理pending多的 */
            if (d_neg->pending_steps >= d_pos->pending_steps) {
                if (space_pressed) {
                    input_report_rel(trackball_dev_ref, INPUT_REL_HWHEEL, -1, false, K_FOREVER);
                } else {
                    trigger_key_press(DIR_LEFT);
                }
                d_neg->pending_steps--;
            } else {
                if (space_pressed) {
                    input_report_rel(trackball_dev_ref, INPUT_REL_HWHEEL, 1, false, K_FOREVER);
                } else {
                    trigger_key_press(DIR_RIGHT);
                }
                d_pos->pending_steps--;
            }
        } else if (d_neg->pending_steps > 0) {
            if (space_pressed) {
                input_report_rel(trackball_dev_ref, INPUT_REL_HWHEEL, -1, false, K_FOREVER);
            } else {
                trigger_key_press(DIR_LEFT);
            }
            d_neg->pending_steps--;
        } else if (d_pos->pending_steps > 0) {
            if (space_pressed) {
                input_report_rel(trackball_dev_ref, INPUT_REL_HWHEEL, 1, false, K_FOREVER);
            } else {
                trigger_key_press(DIR_RIGHT);
            }
            d_pos->pending_steps--;
        }
    }

    /* Y轴 (上下) - 同样逻辑 */
    DirState *d_up = &dir_states[DIR_UP];
    DirState *d_down = &dir_states[DIR_DOWN];

    if (d_up->pending_steps > 0 && d_down->pending_steps > 0) {
        if (d_up->pending_steps >= d_down->pending_steps) {
            if (space_pressed) {
                input_report_rel(trackball_dev_ref, INPUT_REL_WHEEL, 1, false, K_FOREVER);
            } else {
                trigger_key_press(DIR_UP);
            }
            d_up->pending_steps--;
        } else {
            if (space_pressed) {
                input_report_rel(trackball_dev_ref, INPUT_REL_WHEEL, -1, false, K_FOREVER);
            } else {
                trigger_key_press(DIR_DOWN);
            }
            d_down->pending_steps--;
        }
    } else if (d_up->pending_steps > 0) {
        if (space_pressed) {
            input_report_rel(trackball_dev_ref, INPUT_REL_WHEEL, 1, false, K_FOREVER);
        } else {
            trigger_key_press(DIR_UP);
        }
        d_up->pending_steps--;
    } else if (d_down->pending_steps > 0) {
        if (space_pressed) {
            input_report_rel(trackball_dev_ref, INPUT_REL_WHEEL, -1, false, K_FOREVER);
        } else {
            trigger_key_press(DIR_DOWN);
        }
        d_down->pending_steps--;
    }

    /* 继续调度 */
    k_work_schedule(&step_work, K_MSEC(MIN_STEP_INTERVAL_MS));
}

/* ==== 初始化 ==== */
static int bbtrackball_init(const struct device *dev) {
    struct bbtrackball_data *data = dev->data;

    LOG_INF("Initializing BBtrackball (step-limited mode)...");
    LOG_INF("  Step interval: %dms, Key press: %dms",
            MIN_STEP_INTERVAL_MS, KEY_PRESS_MS);

    /* 初始化GPIO */
    for (int i = 0; i < DIR_COUNT; i++) {
        DirState *d = &dir_states[i];
        gpio_pin_configure(d->gpio_dev, d->pin,
                          GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_BOTH);
        d->last_state = gpio_pin_get(d->gpio_dev, d->pin);

        gpio_init_callback(&gpio_cbs[i], dir_edge_cb, BIT(d->pin));
        gpio_add_callback(d->gpio_dev, &gpio_cbs[i]);
        gpio_pin_interrupt_configure(d->gpio_dev, d->pin, GPIO_INT_EDGE_BOTH);

        /* 初始化释放work */
        k_work_init_delayable(&key_release_works[i], key_release_handler);
    }

    data->dev = dev;
    trackball_dev_ref = dev;

    /* 启动步进定时器 */
    k_work_init_delayable(&step_work, step_handler);
    k_work_schedule(&step_work, K_MSEC(MIN_STEP_INTERVAL_MS));

    return 0;
}

/* ==== 驱动实例注册 ==== */
#define BBTRACKBALL_INIT_PRIORITY CONFIG_INPUT_INIT_PRIORITY
#define BBTRACKBALL_DEFINE(inst)                                                                   \
    static struct bbtrackball_data bbtrackball_data_##inst;                                        \
    static const struct bbtrackball_dev_config bbtrackball_config_##inst = {                       \
        .x_input_code = DT_PROP_OR(DT_DRV_INST(inst), x_input_code, INPUT_REL_X),                  \
        .y_input_code = DT_PROP_OR(DT_DRV_INST(inst), y_input_code, INPUT_REL_Y),                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, bbtrackball_init, NULL, &bbtrackball_data_##inst,                  \
                          &bbtrackball_config_##inst, POST_KERNEL, BBTRACKBALL_INIT_PRIORITY,      \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(BBTRACKBALL_DEFINE);
