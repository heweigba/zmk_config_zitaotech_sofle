/*
 * bbtrackball_input_handler.c - BB Trackball (黑莓原版物理惯性模拟)
 *
 * 算法：软件模拟黑莓轨迹球的物理阻尼感
 * - 脉冲加速，无脉冲时速度衰减（模拟摩擦力）
 * - 速度决定跳跃距离，自然流畅
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_bbtrackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <math.h>
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

/* ==== 物理惯性参数 ==== */
#define ACCEL_PER_PULSE 1.0f       /* 每个脉冲增加的速度 */
#define FRICTION_COEFF 0.88f       /* 摩擦系数(衰减率)，0.85-0.95之间调试 */
#define MIN_VELOCITY 0.5f          /* 速度低于此值停止 */
#define MAX_JUMP 8                 /* 最大跳跃字数，防止失控 */
#define VELOCITY_SCALE 0.4f        /* 速度转跳跃距离的系数 */
#define TICK_INTERVAL_MS 12        /* 物理 tick 间隔 */

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

/* ==== 每个方向的速度和脉冲计数 ==== */
typedef struct {
    const struct device *gpio_dev;
    int pin;
    int last_state;
    float velocity;         /* 当前速度(可正负) */
    int pulse_accum;        /* 脉冲累积 */
    bool active;            /* 是否正在运动 */
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
static struct k_work_delayable physics_work;

/* ==== Device Config/Data ==== */
struct bbtrackball_dev_config {
    uint16_t x_input_code;
    uint16_t y_input_code;
};

struct bbtrackball_data {
    const struct device *dev;
};

/* ==== 外部接口 ==== */
bool trackball_is_moving(void) {
    for (int i = 0; i < DIR_COUNT; i++) {
        if (dir_states[i].active || dir_states[i].velocity != 0) {
            return true;
        }
    }
    return false;
}

/* ==== 发送方向键 ==== */
static void send_arrow_key(uint8_t keycode, bool pressed) {
    if (pressed) {
        zmk_hid_keyboard_press(keycode);
    } else {
        zmk_hid_keyboard_release(keycode);
    }
    zmk_endpoints_send_report(0x07);
}

/* ==== 触发一次方向键 ==== */
static void trigger_single_step(uint8_t dir) {
    uint8_t keycode;
    switch (dir) {
        case DIR_LEFT:  keycode = 0x50; break; /* LEFT */
        case DIR_RIGHT: keycode = 0x4F; break; /* RIGHT */
        case DIR_UP:    keycode = 0x52; break; /* UP */
        case DIR_DOWN:  keycode = 0x51; break; /* DOWN */
        default: return;
    }
    /* 按下并立即释放，模拟一次按键 */
    send_arrow_key(keycode, true);
    send_arrow_key(keycode, false);
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
    for (int i = 0; i < DIR_COUNT; i++) {
        DirState *d = &dir_states[i];
        if ((dev == d->gpio_dev) && (pins & BIT(d->pin))) {
            int val = gpio_pin_get(dev, d->pin);
            if (val != d->last_state) {
                d->last_state = val;
                /* 下降沿 = 一个脉冲 */
                if (val == 0) {
                    d->pulse_accum++;
                    d->active = true;
                    LOG_DBG("Dir %d pulse, accum=%d", i, d->pulse_accum);
                }
            }
            break;
        }
    }
}

/* ==== 物理引擎：速度累积与衰减 ==== */
static void physics_handler(struct k_work *work) {
    /* X轴处理 (左右) */
    for (int axis = 0; axis < 2; axis++) {
        int dir_neg = axis == 0 ? DIR_LEFT : DIR_UP;
        int dir_pos = axis == 0 ? DIR_RIGHT : DIR_DOWN;
        DirState *d_neg = &dir_states[dir_neg];
        DirState *d_pos = &dir_states[dir_pos];

        /* 脉冲加速：每个脉冲增加速度 */
        if (d_neg->pulse_accum > 0) {
            d_neg->velocity += ACCEL_PER_PULSE * d_neg->pulse_accum;
            d_neg->pulse_accum = 0;
            d_pos->velocity = 0; /* 反向清零 */
        }
        if (d_pos->pulse_accum > 0) {
            d_pos->velocity += ACCEL_PER_PULSE * d_pos->pulse_accum;
            d_pos->pulse_accum = 0;
            d_neg->velocity = 0; /* 反向清零 */
        }

        /* 计算净速度 */
        float net_velocity = d_pos->velocity - d_neg->velocity;

        /* 速度衰减 (摩擦力) */
        d_pos->velocity *= FRICTION_COEFF;
        d_neg->velocity *= FRICTION_COEFF;

        /* 低于阈值清零 */
        if (d_pos->velocity < MIN_VELOCITY) d_pos->velocity = 0;
        if (d_neg->velocity < MIN_VELOCITY) d_neg->velocity = 0;

        /* 根据速度计算跳跃距离 */
        if (fabsf(net_velocity) >= MIN_VELOCITY) {
            int jump = (int)(fabsf(net_velocity) * VELOCITY_SCALE);
            if (jump < 1) jump = 1;
            if (jump > MAX_JUMP) jump = MAX_JUMP;

            uint8_t dir = net_velocity > 0 ? dir_pos : dir_neg;

            if (space_pressed) {
                /* 滚动模式 */
                int scroll = net_velocity > 0 ? jump : -jump;
                if (axis == 0) {
                    input_report_rel(trackball_dev_ref, INPUT_REL_HWHEEL, scroll, false, K_FOREVER);
                } else {
                    input_report_rel(trackball_dev_ref, INPUT_REL_WHEEL, -scroll, true, K_FOREVER);
                }
            } else {
                /* 方向键模式：发送jump次方向键 */
                for (int k = 0; k < jump; k++) {
                    trigger_single_step(dir);
                }
            }

            LOG_DBG("Axis %d: vel=%.2f, jump=%d, dir=%s",
                    axis, net_velocity, jump, net_velocity > 0 ? "pos" : "neg");
        }

        /* 更新active状态 */
        d_pos->active = (d_pos->velocity >= MIN_VELOCITY);
        d_neg->active = (d_neg->velocity >= MIN_VELOCITY);
    }

    /* 继续调度 */
    k_work_schedule(&physics_work, K_MSEC(TICK_INTERVAL_MS));
}

/* ==== 初始化 ==== */
static int bbtrackball_init(const struct device *dev) {
    struct bbtrackball_data *data = dev->data;

    LOG_INF("Initializing BBtrackball (BlackBerry physics simulation)...");
    LOG_INF("  Friction: %.2f, Accel: %.2f, Scale: %.2f",
            FRICTION_COEFF, ACCEL_PER_PULSE, VELOCITY_SCALE);

    /* 初始化GPIO */
    for (int i = 0; i < DIR_COUNT; i++) {
        DirState *d = &dir_states[i];
        gpio_pin_configure(d->gpio_dev, d->pin,
                          GPIO_INPUT | GPIO_PULL_UP | GPIO_INT_EDGE_BOTH);
        d->last_state = gpio_pin_get(d->gpio_dev, d->pin);

        gpio_init_callback(&gpio_cbs[i], dir_edge_cb, BIT(d->pin));
        gpio_add_callback(d->gpio_dev, &gpio_cbs[i]);
        gpio_pin_interrupt_configure(d->gpio_dev, d->pin, GPIO_INT_EDGE_BOTH);
    }

    data->dev = dev;
    trackball_dev_ref = dev;

    /* 启动物理引擎 */
    k_work_init_delayable(&physics_work, physics_handler);
    k_work_schedule(&physics_work, K_MSEC(TICK_INTERVAL_MS));

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
