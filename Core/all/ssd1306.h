#ifndef __SSD1306_H
#define __SSD1306_H

#include "i2c.h"
#include "main.h"

#define SSD1306_ADDR 0x78  /* 8 位写地址（HAL 风格）= 0x3C << 1                     */
#define SSD1306_ADDR7 0x3C /* 7 位地址，bsp_i2c_* 内部会左移一位后使用               */
#define SSD1306_CMD 0x00   /* I2C 控制字：后面跟的是命令                            */
#define SSD1306_DATA 0x40  /* I2C 控制字：后面跟的是显存数据                        */

#define SSD1306_WIDTH 128U /* 面板宽度（像素列），同时是每页的显存字节数            */
#define SSD1306_PAGES 8U   /* 页数（每页 8 像素高）：8 页 x 8 = 64 行，共 1024 字节 */

/**
 * @brief  初始化面板（上电阶段调用，此时还没有 RTOS，不需要加锁）
 */
void ssd1306_Init(void);

/**
 * @brief  清空 VRAM 并把 8 页全部置脏（不立即产生 I2C 流量）
 */
void ssd1306_Clear(void);

/**
 * @brief  设置光标（字符坐标：x 像素 0..127，page 页号 0..7）
 */
void ssd1306_SetCursor(uint8_t x, uint8_t page);

/**
 * @brief  按当前光标写一个字节到 VRAM
 */
void ssd1306_WriteData(uint8_t data);

/**
 * @brief  按当前光标写字符串（ASCII，6x8 字库，自动换行）
 */
void ssd1306_WriteString(const char* str);

/**
 * @brief  写一条命令到面板（立即产生 I2C 流量）
 */
void ssd1306_WriteCommand(uint8_t cmd);

/**
 * @brief  把脏页刷到面板，没有脏页就什么都不做
 */
void ssd1306_UpdateScreen(void);

/**
 * @brief  强制刷新所有页（面板复位、总线恢复之后用）
 */
void ssd1306_FlushAll(void);

/**
 * @brief  对比度设置（0x00..0xFF）
 */
void ssd1306_SetContrast(uint8_t val);

/**
 * @brief  最近一次面板访问是否成功
 * @retval 1 = 成功，0 = 失败（多半是总线被拉死，见 i2c 统计）
 */
uint8_t ssd1306_IsPanelOk(void);

/**
 * @brief  累计已刷新的页数
 */
uint32_t ssd1306_GetFlushCount(void);

#endif
