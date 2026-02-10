/*
 * mouse_layer_controller.c - 左手层控制器
 *
 * 功能：监听右手小红点输入，检测到移动时自动进入鼠标层
 * 停止移动后自动退出鼠标层
 *
 * 注意：通过 INPUT_CALLBACK_DEFINE 注册到 trackpoint_listener 设备
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

/* 输入事件处理函数 - 会被 trackpoint_listener 调用 */
static void mouse_layer_input_handler(struct input_event *evt) {
    LOG_DBG("Input: type=%d code=%d val=%d", evt->type, evt->code, evt->value);

    if (evt->type != INPUT_EV_REL) {
        return;
    }

    /* 检查是否是 X 或 Y 轴移动 */
    if (evt->code == INPUT_REL_X || evt->code == INPUT_REL_Y) {
        int16_t value = evt->value;

        /* 过滤微小抖动 */
        if (value >= 2 || value <= -2) {
            uint32_t now = k_uptime_get_32();
            last_movement_time = now;

            LOG_DBG("Movement detected: %d", value);

            /* 激活鼠标层 */
            if (!mouse_layer_active) {
                int ret = zmk_keymap_layer_activate(MOUSE_LAYER_ID);
                if (ret == 0) {
                    mouse_layer_active = true;
                    LOG_INF("Mouse layer ON");
                } else {
                    LOG_WRN("Failed to activate layer: %d", ret);
                }
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

/* 初始化 */
static int mouse_layer_ctrl_init(const struct device *dev) {
    LOG_INF("Mouse layer controller init");
    k_work_init_delayable(&timeout_work, timeout_handler);
    return 0;
}

/* 使用 SYS_INIT 初始化 */
SYS_INIT(mouse_layer_ctrl_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* 注册 input 回调到 trackpoint_listener 设备
 * 注意：trackpoint_listener 是在 zitaotech_sofle.dtsi 中定义的节点
 */
#define TRACKPOINT_LISTENER_NODE DT_NODELABEL(trackpoint_listener)

/* 这个回调会在每次 trackpoint_listener 接收到输入事件时被调用 */
void trackpoint_listener_input_handler(struct input_event *evt, void *user_data) {
    mouse_layer_input_handler(evt);
}

/* 注册回调 - 绑定到 trackpoint_listener 设备 */
INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(TRACKPOINT_LISTENER_NODE),
                      trackpoint_listener_input_handler, NULL);
