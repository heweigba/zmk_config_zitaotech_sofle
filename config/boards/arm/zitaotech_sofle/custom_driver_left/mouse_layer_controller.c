/*
 * mouse_layer_controller.c - 左手层控制器
 *
 * 功能：通过 input-processor 监听小红点输入，检测到移动时自动进入鼠标层
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>

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

/* 层切换监听 - 同步状态 */
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

/* input-processor 处理函数
 * 这个函数会被 trackpoint_listener 的 input-processors 调用
 */
static int mouse_layer_processor(const struct device *dev, struct input_event *event, void *user_data) {
    /* 只处理 REL_X 和 REL_Y 事件 */
    if (event->type == INPUT_EV_REL && (event->code == INPUT_REL_X || event->code == INPUT_REL_Y)) {
        /* 过滤微小抖动 */
        if (event->value >= 2 || event->value <= -2) {
            uint32_t now = k_uptime_get_32();
            last_movement_time = now;

            LOG_DBG("Trackpoint movement: %d", event->value);

            /* 激活鼠标层 */
            if (!mouse_layer_active) {
                int ret = zmk_keymap_layer_activate(MOUSE_LAYER_ID);
                if (ret == 0) {
                    mouse_layer_active = true;
                    LOG_INF("Mouse layer ON");
                }
            }

            /* 重新调度超时检查 */
            k_work_reschedule(&timeout_work, K_MSEC(AUTO_MOUSE_TIMEOUT_MS));
        }
    }

    /* 继续传递事件给下一个处理器 */
    return 0;
}

/* 注册为 sys_init */
SYS_INIT(mouse_layer_ctrl_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* 导出处理函数供 device tree 引用 */
#define MOUSE_LAYER_PROCESSOR_INST(inst) \
    static const struct device *mouse_layer_processor_##inst = DEVICE_DT_INST_GET(inst);

/* 定义 input-processor */
#define ZMK_INPUT_PROCESSOR_DEFINE(name, handler) \
    static int _CONCAT(z_mk_input_processor_, name)(const struct device *dev, \
                                                    struct input_event *event, \
                                                    void *user_data) { \
        return handler(dev, event, user_data); \
    }

/* 实际的处理函数入口 */
int zmk_input_processor_mouse_layer_handle(const struct device *dev,
                                           struct input_event *event,
                                           void *user_data) {
    return mouse_layer_processor(dev, event, user_data);
}
