/*
 * mouse_layer_controller.c - 左手层控制器
 *
 * 功能：监听右手小红点输入，检测到移动时自动进入鼠标层
 * 停止移动后自动退出鼠标层
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <zmk/keymap.h>
#include <zmk/events/layer_state_changed.h>

LOG_MODULE_REGISTER(mouse_layer_ctrl, LOG_LEVEL_INF);

#define MOUSE_LAYER_ID 2
#define AUTO_MOUSE_TIMEOUT_MS 400
#define MOVEMENT_THRESHOLD 2  /* 最小移动阈值 */

static bool mouse_layer_active = false;
static uint32_t last_movement_time = 0;
static struct k_work_delayable timeout_work;

/* 超时处理：停止移动后退出鼠标层 */
static void timeout_handler(struct k_work *work) {
    uint32_t now = k_uptime_get_32();

    if (mouse_layer_active && (now - last_movement_time) > AUTO_MOUSE_TIMEOUT_MS) {
        zmk_keymap_layer_deactivate(MOUSE_LAYER_ID);
        mouse_layer_active = false;
        LOG_INF("Mouse layer OFF (timeout)");
    }
}

/* 输入事件回调 - 监听小红点移动 */
static void trackpoint_input_cb(struct input_event *evt, void *user_data) {
    if (evt->type != INPUT_EV_REL) {
        return;
    }

    /* 检查是否是 X 或 Y 轴移动 */
    if (evt->code == INPUT_REL_X || evt->code == INPUT_REL_Y) {
        int16_t value = evt->value;

        /* 过滤微小抖动 */
        if (abs(value) >= MOVEMENT_THRESHOLD) {
            uint32_t now = k_uptime_get_32();
            last_movement_time = now;

            /* 激活鼠标层 */
            if (!mouse_layer_active) {
                zmk_keymap_layer_activate(MOUSE_LAYER_ID);
                mouse_layer_active = true;
                LOG_INF("Mouse layer ON (movement)");
            }

            /* 重新调度超时检查 */
            k_work_reschedule(&timeout_work, K_MSEC(AUTO_MOUSE_TIMEOUT_MS));
        }
    }
}

/* 层变化监听 - 同步状态 */
static int layer_listener_cb(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (!ev) return 0;

    if (ev->layer == MOUSE_LAYER_ID) {
        mouse_layer_active = ev->state;
        LOG_DBG("Layer %d state: %d", MOUSE_LAYER_ID, ev->state);
    }
    return 0;
}

ZMK_LISTENER(mouse_layer_listener, layer_listener_cb);
ZMK_SUBSCRIPTION(mouse_layer_listener, zmk_layer_state_changed);

/* 注册输入回调 */
INPUT_CALLBACK_DEFINE(NULL, trackpoint_input_cb, NULL);

/* 初始化 */
static int mouse_layer_ctrl_init(const struct device *dev) {
    LOG_INF("Mouse layer controller init");

    k_work_init_delayable(&timeout_work, timeout_handler);

    return 0;
}

SYS_INIT(mouse_layer_ctrl_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
