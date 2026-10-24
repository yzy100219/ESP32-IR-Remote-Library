/**
  * @file    ir_remote.h
  * @brief   红外遥控接收模块 (纯 C 语言版本，基于 ESP-IDF RMT 外设)
  * @note    支持 NEC 协议的自适应解析，兼容长按重复码及部分截断杂波。
  */

#ifndef __IR_REMOTE_H
#define __IR_REMOTE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* 参数宏定义*********************/

/**
  * @brief 红外接收数据结构体
  */
typedef struct {
    uint8_t Address;    // 遥控器地址码 (通常用于区分不同设备)
    uint8_t Command;    // 按键命令码 (如 0x19 代表上)
    bool IsRepeat;      // 是否为长按重复码 (true: 长按连发, false: 正常单次按下)
} IR_Data_t;

/* 函数声明*********************/

/**
  * 函    数：红外接收模块初始化
  * 参    数：rx_gpio 红外接收头 OUT 引脚连接的 GPIO 编号
  * 返 回 值：ESP_OK 表示初始化成功，其他表示失败
  * 说    明：内部会创建 RMT 接收通道、队列以及后台解析任务
  */
esp_err_t IR_Remote_Init(int rx_gpio);

/**
  * 函    数：读取红外数据结构体
  * 参    数：data 用于接收数据的结构体指针
  * 参    数：timeout_ms 阻塞等待超时时间 (毫秒)，填 0 表示非阻塞，填 portMAX_DELAY 表示一直等
  * 返 回 值：true 读到数据；false 未读到数据
  * 说    明：提供给需要同时获取 Address、Command 和 IsRepeat 的高级应用使用
  */
bool IR_Remote_Read(IR_Data_t *data, uint32_t timeout_ms);

/**
  * 函    数：极简获取按键值
  * 参    数：无
  * 返 回 值：返回解析出的 Command 键值；如果当前无按键，返回 0xFF
  * 说    明：专为主程序轮询设计，无需传参，直接获取结果
  */
uint8_t IR_Remote_GetCommand(void); 

#endif /* __IR_REMOTE_H */