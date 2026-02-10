/*
 * bbtrackball_input_handler.c - BB Trackball (默认方向键模式，脉冲触发)
 *
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

/* ==== GPIO Pins ==== */
#define DOWN_GPIO_PIN 9
#define LEFT_GPIO_PIN 12
#define UP_GPIO_PIN 5
#define RIGHT_GPIO_PIN 27

#define GPIO0_DEV DT_NODELABEL(gpio0)
#define GPIO1_DEV DT_NODELABEL(gpio1)

/* ==== Config ==== */
#define KEY_HOLD_MS 35      /* 方向键按住时间 */
#define DEBOUNCE_MS 80      /* 消抖时间，防止同一方向连续触发过快 */
#define SCROLL_DEBOUNCE_MS 40  /* 滚动模式的消抖时间 */

/* ==== 状态 ==== */
static bool space_pressed = false;
static const struct device *trackball_dev_ref = NULL;

/* ==== 每个方向的状态 ==== */
typedef struct {
    const struct device *gpio_dev;
    int pin;
    int last_state;
    uint8_t dir;           /* 0=left, 1=right, 2=up, 3=down */
    uint32_t last_trigger; /* 上次触发时间 */
    bool key_pressed;      /* 当前是否正在按住 */
    int pulse_count;       /* 脉冲计数 */
} DirState;

static DirState dir_states[] = {
    {DEVICE_DT_GET(GPIO0_DEV), LEFT_GPIO_PIN, 1, 0, 0, false, 0},
    {DEVICE_DT_GET(GPIO0_DEV), RIGHT_GPIO_PIN, 1, 1, 0, false, 0},
    {DEVICE_DT_GET(GPIO0_DEV), UP_GPIO_PIN, 1, 2, 0, false, 0},
    {DEVICE_DT_GET(GPIO1_DEV), DOWN_GPIO_PIN, 1, 3, 0, false, 0},
};

static struct gpio_callback gpio_cbs[ARRAY_SIZE(dir_states)];

/* ==== 释放work ==== */
static struct k_work_delayable release_works[4];

/* ==== Device Config/Data ==== */
struct bbtrackball_dev_config {
    uint16_t x_input_code;
    uint16_t y_input_code;
};

struct bbtrackball_data {
    const struct device *dev;
    struct k_work_delayable scroll_work;
};

/* ==== 外部接口 ==== */
bool trackball_is_moving(void) {
    for (int i = 0; i < 4; i++) {
        if (dir_states[i].pulse_count > 0 || dir_states[i].key_pressed) {
            return true;
        }
    }
    return false;
}

/* ==== 发送方向键 ==== */
static void send_key(uint8_t keycode, bool pressed) {
    if (pressed) {
        zmk_hid_keyboard_press(keycode);
    } else {
        zmk_hid_keyboard_release(keycode);
    }
    zmk_endpoints_send_report(0x07);
}

/* ==== 方向键释放 handler ==== */
static void key_release_handler(struct k_work *work) {
    for (int i = 0; i < 4; i++) {
        if (work == &release_works[i].work) {
            uint8_t keycode;
            switch (i) {
                case 0: keycode = 0x50; break; /* LEFT */
                case 1: keycode = 0x4F; break; /* RIGHT */
                case 2: keycode = 0x52; break; /* UP */
                case 3: keycode = 0x51; break; /* DOWN */
                default: return;
            }
            if (dir_states[i].key_pressed) {
                send_key(keycode, false);
                dir_states[i].key_pressed = false;
                LOG_DBG("Dir %d released", i);
            }
            return;
        }
    }
}

/* ==== 触发方向键 ==== */
static bool try_trigger_arrow(uint8_t dir) {
    DirState *d = &dir_states[dir];
    uint32_t now = k_uptime_get_32();

    /* 检查消抖 - 如果上次触发时间太近，忽略本次 */
    if ((now - d->last_trigger) < DEBOUNCE_MS) {
        return false;
    }

    /* 如果已经在按住，先释放 */
    if (d->key_pressed) {
        uint8_t keycode;
        switch (dir) {
            case 0: keycode = 0x50; break;
            case 1: keycode = 0x4F; break;
            case 2: keycode = 0x52; break;
            case 3: keycode = 0x51; break;
            default: return false;
        }
        send_key(keycode, false);
        d->key_pressed = false;
    }

    /* 发送按下 */
    uint8_t keycode;
    switch (dir) {
        case 0: keycode = 0x50; break; /* LEFT */
        case 1: keycode = 0x4F; break; /* RIGHT */
        case 2: keycode = 0x52; break; /* UP */
        case 3: keycode = 0x51; break; /* DOWN */
        default: return false;
    }

    send_key(keycode, true);
    d->key_pressed = true;
    d->last_trigger = now;
    d->pulse_count = 0;

    /* 安排释放 */
    k_work_schedule(&release_works[dir], K_MSEC(KEY_HOLD_MS));

    LOG_DBG("Dir %d triggered", dir);
    return true;
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

/* ==== GPIO 中断回调 ==== */
static void dir_edge_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    for (size_t i = 0; i < ARRAY_SIZE(dir_states); i++) {
        DirState *d = &dir_states[i];
        if ((dev == d->gpio_dev) && (pins & BIT(d->pin))) {
            int val = gpio_pin_get(dev, d->pin);

            /* 状态变化时处理 */
            if (val != d->last_state) {
                d->last_state = val;

                /* 下降沿计数 */
                if (val == 0) {
                    d->pulse_count++;

                    if (space_pressed) {
                        /* 滚动模式：累积脉冲 */
                        /* 在主循环中处理 */
                    } else {
                        /* 方向键模式：立即触发 */
                        try_trigger_arrow(d->dir);
                    }
                }
            }
            break; /* 找到匹配的方向就退出 */
        }
    }
}

/* ==== 滚动处理 work ==== */
static void scroll_work_handler(struct k_work *work) {
    if (!space_pressed) return;

    static uint32_t last_scroll[4] = {0, 0, 0, 0};
    uint32_t now = k_uptime_get_32();

    int scroll_x = 0;
    int scroll_y = 0;

    for (int i = 0; i < 4; i++) {
        DirState *d = &dir_states[i];

        if (d->pulse_count > 0 && (now - last_scroll[i]) >= SCROLL_DEBOUNCE_MS) {
            switch (i) {
                case 0: scroll_x -= d->pulse_count; break; /* LEFT */
                case 1: scroll_x += d->pulse_count; break; /* RIGHT */
                case 2: scroll_y -= d->pulse_count; break; /* UP */
                case 3: scroll_y += d->pulse_count; break; /* DOWN */
            }
            d->pulse_count = 0;
            last_scroll[i] = now;
        }
    }

    if (scroll_x != 0 || scroll_y != 0) {
        input_report_rel(trackball_dev_ref, INPUT_REL_HWHEEL, scroll_x, false, K_FOREVER);
        input_report_rel(trackball_dev_ref, INPUT_REL_WHEEL, -scroll_y, true, K_FOREVER);
    }

    /* 如果还在滚动模式，继续调度 */
    if (space_pressed) {
        struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
        k_work_schedule(dwork, K_MSEC(SCROLL_DEBOUNCE_MS));
    }
}

/* ==== 初始化 ==== */
static int bbtrackball_init(const struct device *dev) {
    struct bbtrackball_data *data = dev->data;

    LOG_INF("Initializing BBtrackball (optimized arrow key mode)...");

    /* 初始化GPIO */
    for (size_t i = 0; i < ARRAY_SIZE(dir_states); i++) {
        DirState *d = &dir_states[i];
        gpio_pin_configure(d->gpio_dev, d->pin, GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_BOTH);
        d->last_state = gpio_pin_get(d->gpio_dev, d->pin);

        gpio_init_callback(&gpio_cbs[i], dir_edge_cb, BIT(d->pin));
        gpio_add_callback(d->gpio_dev, &gpio_cbs[i]);
        gpio_pin_interrupt_configure(d->gpio_dev, d->pin, GPIO_INT_EDGE_BOTH);

        /* 初始化释放work */
        k_work_init_delayable(&release_works[i], key_release_handler);
    }

    data->dev = dev;
    trackball_dev_ref = dev;

    /* 初始化滚动work */
    k_work_init_delayable(&data->scroll_work, scroll_work_handler);

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
