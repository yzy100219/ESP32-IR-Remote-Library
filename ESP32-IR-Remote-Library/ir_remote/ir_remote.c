/**
  * @file    ir_remote.c
  * @brief   红外遥控接收模块底层实现
  * @note    采用 RMT + FreeRTOS 队列 + 后台任务 的架构，高效且不阻塞主循环。
  */

#include "ir_remote.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "esp_check.h"

/* 日志 TAG 定义 */
static const char *TAG = "IR_Remote";

/* 参数宏定义*********************/
#define IR_RESOLUTION_HZ      1000000   // RMT 时钟分辨率：1MHz (即 1 tick = 1 微秒)
#define RMT_RX_MAX_SYMBOLS    256       // 硬件接收缓冲区大小，扩容至256以兼容空调等长帧协议
#define DEBUG_IR_RAW          0         // 调试开关：1开启底层波形打印(会刷屏)，0关闭(释放性能)

/* 内部数据类型定义*********************/

/**
  * @brief 内部传递的原始脉冲包 (硬件中断 -> 解析任务)
  */
typedef struct {
    rmt_symbol_word_t symbols[RMT_RX_MAX_SYMBOLS]; // 存放高低电平持续时间的数组
    size_t num_symbols;                            // 实际接收到的符号(高低电平对)数量
} ir_raw_packet_t;

/* 全局变量*********************/
static rmt_channel_handle_t rx_channel = NULL; // RMT 接收通道句柄
static QueueHandle_t rx_queue = NULL;          // 硬件中断到后台任务的原始数据队列
static QueueHandle_t result_queue = NULL;      // 后台任务到主程序的解析结果队列
static ir_raw_packet_t rx_packet;              // 全局缓冲，用于中断中快速拷贝数据

/* 函数声明*********************/
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);
static bool IR_ParseNEC(rmt_symbol_word_t *symbols, size_t num_symbols, IR_Data_t *out_data);
static void ir_rx_task(void *pvParameters);

/* 函数实现*********************/

/**
  * 函    数：RMT 接收完成中断回调函数
  * 参    数：edata 包含接收到的数据指针和数量
  * 参    数：user_data 用户数据(此处传入了队列句柄)
  * 返 回 值：是否需要唤醒高优先级任务
  * 说    明：在中断中仅做数据拷贝和入队，绝对不进行耗时操作（如解析和打印）
  */
static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    
    // 快速拷贝数据到全局变量，并发送到队列
    rx_packet.num_symbols = edata->num_symbols;
    memcpy(rx_packet.symbols, edata->received_symbols, edata->num_symbols * sizeof(rmt_symbol_word_t));
    xQueueSendFromISR(queue, &rx_packet, &high_task_wakeup);
    
    return high_task_wakeup == pdTRUE;
}

/**
  * 函    数：NEC 协议解析算法 (自适应版本)
  * 参    数：symbols 原始脉冲数据
  * 参    数：num_symbols 脉冲数量
  * 参    数：out_data 解析结果输出
  * 返 回 值：解析成功返回 true，失败返回 false
  * 说    明：不依赖严格的高低电平起始状态，只根据脉宽时长(长短)来判定数据位
  */
static bool IR_ParseNEC(rmt_symbol_word_t *symbols, size_t num_symbols, IR_Data_t *out_data)
{
    if (num_symbols < 2) return false;

    // 1. 优先处理长按重复码 (标准 NEC 重复码极短，约为 9ms+2.25ms 或 2.25ms+0.56ms)
    if (num_symbols <= 3) {
        uint32_t d0 = symbols[0].duration0;
        uint32_t d1 = symbols[0].duration1;
        uint32_t d2 = (num_symbols > 1) ? symbols[1].duration0 : 0;
        
        // 只要捕捉到 2.25ms 左右的脉冲，即判定为长按
        if ((d0 > 2000 && d0 < 2500) || (d1 > 2000 && d1 < 2500) || (d2 > 2000 && d2 < 2500)) {
            out_data->IsRepeat = true;
            return true; 
        }
    }

    // 2. 将二维的 rmt_symbol_word_t 拉平为一维的脉宽数组
    uint32_t durations[RMT_RX_MAX_SYMBOLS * 2];
    int cnt = 0;
    for (int i = 0; i < num_symbols; i++) {
        durations[cnt++] = symbols[i].duration0;
        durations[cnt++] = symbols[i].duration1;
    }

    // 3. 自适应寻找引导码起始点 (防止因遮挡导致引导码前段丢失)
    int data_start = -1;
    for (int i = 0; i < cnt; i++) {
        // 情况 A：标准完整引导码 (9ms 低电平)
        if (durations[i] > 8000 && durations[i] < 10000) {
            data_start = i + 2; // 数据位从引导码后的第2个脉宽开始
            break;
        }
        // 情况 B：截断引导码 (直接抓到了 4.5ms 的高电平)
        if (durations[i] > 4000 && durations[i] < 5000) {
            if (i + 1 < cnt && durations[i+1] < 1000) { // 确保后面跟着一个短脉冲
                data_start = i + 2; 
                break;
            }
        }
    }

    if (data_start == -1) {
        // 没找到引导码，视为环境杂波，直接静默丢弃
        ESP_LOGD(TAG, "未找到引导码，过滤杂波");
        return false;
    }

    // 4. 解析 32 位有效数据 (NEC 协议包含 Address, ~Address, Command, ~Command)
    uint32_t data = 0;
    for (int i = 0; i < 32; i++) {
        int idx = data_start + i * 2; // 每个 bit 占据 2 个脉宽 (高+低)
        if (idx >= cnt - 1) return false; // 数据不足 32 位
        
        // 取高电平和低电平中时间较长的那个作为判断依据
        uint32_t d1 = durations[idx];
        uint32_t d2 = durations[idx + 1];
        uint32_t max_d = (d1 > d2) ? d1 : d2;

        if (max_d > 1000 && max_d < 2000) {
            data |= (1 << i); // 脉宽约 1.69ms，代表逻辑 1
        } else if (max_d < 1000) {
            // 脉宽约 0.56ms，代表逻辑 0，无需操作
        } else {
            ESP_LOGD(TAG, "第 %d 位数据异常 (%lu us)", i, (unsigned long)max_d);
            return false;
        }
    }

    // 5. 提取 Address 和 Command
    out_data->Address = data & 0xFF;
    out_data->Command = (data >> 16) & 0xFF;
    out_data->IsRepeat = false;
    
    return true;
}

/**
  * 函    数：红外解析后台任务
  * 参    数：pvParameters 未使用
  * 返 回 值：无
  * 说    明：无限循环，从 rx_queue 读取原始数据，解析后存入 result_queue，并重启接收。
  */
static void ir_rx_task(void *pvParameters)
{
    ir_raw_packet_t received;
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,        // 过滤短于 1.25us 的毛刺噪声
        .signal_range_max_ns = 12000000,    // 12ms 超时，超过此时间认为一帧结束
    };

    while (1) {
        // 阻塞等待硬件中断发送过来的原始波形
        if (xQueueReceive(rx_queue, &received, portMAX_DELAY) == pdPASS) {
            
            // 调试模式下打印底层波形
            #if DEBUG_IR_RAW
            ESP_LOGI("IR_RAW", "收到脉冲！共 %d 个符号", received.num_symbols);
            for(int i = 0; i < received.num_symbols && i < 5; i++) {
                ESP_LOGI("IR_RAW", "  [%d] L0:%d T0:%lu us | L1:%d T1:%lu us", 
                         i, received.symbols[i].level0, (unsigned long)received.symbols[i].duration0,
                         received.symbols[i].level1, (unsigned long)received.symbols[i].duration1);
            }
            #endif

            IR_Data_t ir_data;
            // 调用解析函数
            if (IR_ParseNEC(received.symbols, received.num_symbols, &ir_data)) {
                // 解析成功，将结果放入结果队列，供外部读取
                if (xQueueSend(result_queue, &ir_data, 0) != pdPASS) {
                    ESP_LOGW(TAG, "结果队列已满，丢弃数据");
                }
            }
            
            // 关键：处理完一帧后，必须重新开启 RMT 接收，等待下一帧
            rmt_receive(rx_channel, rx_packet.symbols, sizeof(rx_packet.symbols), &receive_config);
        }
    }
}

/**
  * 函    数：红外接收模块初始化
  * 参    数：rx_gpio 接收引脚编号
  * 返 回 值：esp_err_t 错误码
  * 说    明：初始化底层硬件、创建队列和后台任务
  */
esp_err_t IR_Remote_Init(int rx_gpio)
{
    // 1. 创建队列
    rx_queue = xQueueCreate(1, sizeof(ir_raw_packet_t));
    result_queue = xQueueCreate(5, sizeof(IR_Data_t)); // 深度为 5，可缓存 5 次按键
    
    if (rx_queue == NULL || result_queue == NULL) {
        ESP_LOGE(TAG, "队列创建失败，内存不足");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "初始化 RMT 接收通道...");

    // 2. 配置 RMT 接收通道
    rmt_rx_channel_config_t rx_channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = rx_gpio,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = RMT_RX_MAX_SYMBOLS, // 设置足够大的硬件缓冲区
        .flags.invert_in = false,                // 不反转输入信号
    };
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_channel_config, &rx_channel), TAG, "RMT 创建失败");

    // 3. 注册中断回调并传入队列句柄
    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_done_callback };
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(rx_channel, &cbs, rx_queue), TAG, "回调注册失败");
    
    // 4. 使能通道
    ESP_RETURN_ON_ERROR(rmt_enable(rx_channel), TAG, "使能失败");

    // 5. 启动第一次接收
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,
        .signal_range_max_ns = 12000000,
    };
    ESP_RETURN_ON_ERROR(rmt_receive(rx_channel, rx_packet.symbols, sizeof(rx_packet.symbols), &receive_config), TAG, "接收开启失败");

    // 6. 创建后台解析任务
    xTaskCreate(ir_rx_task, "ir_rx_task", 4096, NULL, 10, NULL);
    
    ESP_LOGI(TAG, "红外初始化完毕, GPIO: %d", rx_gpio);
    return ESP_OK;
}

/**
  * 函    数：读取红外数据结构体 (阻塞式)
  * 参    数：data 输出数据指针
  * 参    数：timeout_ms 超时时间
  * 返 回 值：是否成功读取到数据
  */
bool IR_Remote_Read(IR_Data_t *data, uint32_t timeout_ms)
{
    if (result_queue == NULL) return false;
    TickType_t ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xQueueReceive(result_queue, data, ticks) == pdPASS);
}

/**
  * 函    数：极简获取 Command 键值
  * 参    数：无
  * 返 回 值：键值，0xFF 表示无按键
  * 说    明：非阻塞，适合在主循环中不断轮询
  */
uint8_t IR_Remote_GetCommand(void)
{
    IR_Data_t data;
    // 超时时间填 0，表示立刻返回
    if (IR_Remote_Read(&data, 0)) {
        return data.Command;
    }
    return 0xFF; 
}