#include "ssd1306.h"

#include <string.h>

#include "font5x8.h"
#include "i2c.h"

/* 128 x 64 分辨率对应 128 列 x 8 页 = 1024 字节显存 */
static uint8_t s_vram[SSD1306_WIDTH * SSD1306_PAGES];
/* 当前光标：像素列 0..127 */
static uint8_t s_cur_x = 0U;
/* 当前光标：页号 0..7（一页 = 8 像素高） */
static uint8_t s_cur_page     = 0U;
static uint8_t s_dirty        = 0U; /* bit0..bit7 对应 page0..page7    */
static uint8_t s_panel_ok     = 1U; /* 最近一次面板访问是否成功        */
static uint32_t s_flush_pages = 0U; /* 累计刷新页数                    */

/* 写命令缓冲长度：控制字 + 命令 */
#define SSD1306_CTRL_LEN 2U

/**
 * @brief  写 VRAM 一个字节，只有数值变了才置脏
 */
static void vram_put(uint16_t idx, uint8_t val)
{
        if (s_vram[idx] != val)
        {
                s_vram[idx] = val;
                s_dirty |= (uint8_t)(1U << (idx / SSD1306_WIDTH));
        }
}

/**
 * @brief  写一条命令到面板
 * @param  cmd 命令字节（不含控制字，控制字 0x00 由本函数自动补）
 */
void ssd1306_WriteCommand(uint8_t cmd)
{
        /* 一帧 I2C 数据：控制字(0x00) + 命令，共 2 字节 */
        uint8_t buf[SSD1306_CTRL_LEN];

        buf[0] = SSD1306_CMD;
        buf[1] = cmd;

        if (bsp_i2c_write(SSD1306_ADDR7, buf, SSD1306_CTRL_LEN, BSP_I2C_TIMEOUT_MS) != HAL_OK)
        {
                s_panel_ok = 0U;
                return;
        }

        s_panel_ok = 1U;
}

/**
 * @brief  把某一页的 128 字节 VRAM 写进面板
 * @param  page 页号 0..7
 */
static void flush_page(uint8_t page)
{
        /* 页地址+列地址低4位/高4位 */
        ssd1306_WriteCommand((uint8_t)(0xB0U + page));
        ssd1306_WriteCommand(0x00U);
        ssd1306_WriteCommand(0x10U);

        if (bsp_i2c_mem_write(SSD1306_ADDR7, SSD1306_DATA,
                              &s_vram[(uint16_t)page * SSD1306_WIDTH],
                              (uint16_t)SSD1306_WIDTH, BSP_I2C_TIMEOUT_MS) != HAL_OK)
        {
                s_panel_ok = 0U;
                return;
        }

        s_panel_ok = 1U;
        s_flush_pages++;
}

/**
 * @brief  上电初始化面板：等 100 ms + 下发整串配置 + 清屏并整屏刷一次
 */
void ssd1306_Init(void)
{
        /* 等面板内部上电复位完成（手册要求上电后 >100 ms 再发命令） */
        HAL_Delay(100U);

        ssd1306_WriteCommand(0xAE); /* 关显示            */
        ssd1306_WriteCommand(0xD5); /* 时钟分频          */
        ssd1306_WriteCommand(0x80);
        ssd1306_WriteCommand(0xA8); /* 多路复用比        */
        ssd1306_WriteCommand(0x3F); /* 64 行             */
        ssd1306_WriteCommand(0xD3); /* 显示偏移          */
        ssd1306_WriteCommand(0x00);
        ssd1306_WriteCommand(0x40); /* 起始行 0          */

        ssd1306_WriteCommand(0x8D); /* 电荷泵            */
        ssd1306_WriteCommand(0x14); /* 内部 DC/DC 打开   */

        ssd1306_WriteCommand(0xA1); /* 段重映射（左右）  */
        ssd1306_WriteCommand(0xC8); /* 扫描方向（上下）  */
        ssd1306_WriteCommand(0xDA); /* COM 引脚配置      */
        ssd1306_WriteCommand(0x12);
        ssd1306_WriteCommand(0x81); /* 对比度            */
        ssd1306_WriteCommand(0xCF);
        ssd1306_WriteCommand(0xD9); /* 预充电周期        */
        ssd1306_WriteCommand(0xF1);
        ssd1306_WriteCommand(0xDB); /* VCOMH 电压        */
        ssd1306_WriteCommand(0x30);
        ssd1306_WriteCommand(0x20); /* 寻址模式：页模式  */
        ssd1306_WriteCommand(0x00);
        ssd1306_WriteCommand(0xA4); /* 输出跟随 RAM      */
        ssd1306_WriteCommand(0xA6); /* 正常显示          */
        ssd1306_WriteCommand(0xAF); /* 开显示            */

        ssd1306_Clear();
        ssd1306_FlushAll(); /* 上电先整屏刷一遍  */
}

/**
 * @brief  清空显存并把 8 页全部置脏
 */
void ssd1306_Clear(void)
{
        memset(s_vram, 0, sizeof(s_vram));
        s_cur_x    = 0U;
        s_cur_page = 0U;
        s_dirty    = 0xFFU; /* 8 页全部置脏，但本函数里不发 I2C */
}

/**
 * @brief  设置字符光标（写显存的起点）
 * @param  x    像素列 0..127（越界自动到 127）
 * @param  page 页号 0..7（越界自动到 7）
 */
void ssd1306_SetCursor(uint8_t x, uint8_t page)
{
        s_cur_x    = (x < SSD1306_WIDTH) ? x : (uint8_t)(SSD1306_WIDTH - 1U);
        s_cur_page = (page < SSD1306_PAGES) ? page : (uint8_t)(SSD1306_PAGES - 1U);
}

/**
 * @brief  把一个字节写进显存，并自动前进光标
 * @param  data 一个字节：8 个 bit 对应该列上的 8 个像素
 */
void ssd1306_WriteData(uint8_t data)
{
        /* 该字节在 VRAM 里的一维下标 = 页号 * 128 + x */
        uint16_t idx;

        if (s_cur_page >= SSD1306_PAGES)
        {
                return;
        }

        idx = (uint16_t)((uint16_t)s_cur_page * SSD1306_WIDTH) + s_cur_x;
        if (idx < (uint16_t)sizeof(s_vram))
        {
                vram_put(idx, data);
        }

        s_cur_x++;
        if (s_cur_x >= SSD1306_WIDTH)
        {
                s_cur_x = 0U;
                s_cur_page++;
        }
}

/**
 * @brief  从当前光标开始写一个 ASCII 字符串（6x8 字库，自动换行/翻页）
 * @param  str 以 '\0' 结尾的字符串；NULL 时直接返回
 */
void ssd1306_WriteString(const char* str)
{
        /* 指向字库中一个字符的 6 字节点阵（5 列点阵 + 1 列间隔） */
        const uint8_t* glyph;
        /* 点阵列下标 0..5 */
        uint8_t i;
        /* 当前写入位置的 VRAM 一维下标 */
        uint16_t idx;

        if (str == NULL)
        {
                return;
        }

        while (*str != '\0')
        {
                /* 行末自动换页 / 到底停止 */
                if ((uint16_t)s_cur_x + 6U > SSD1306_WIDTH)
                {
                        s_cur_x = 0U;
                        s_cur_page++;
                }
                if (s_cur_page >= SSD1306_PAGES)
                {
                        break;
                }

                /* 字库只有 32..127；超出范围（如中文 UTF-8 字节）画成空格 */
                if (((uint8_t)*str < 32U) || ((uint8_t)*str > 127U))
                {
                        glyph = font5x8[0];
                }
                else
                {
                        glyph = font5x8[(uint8_t)(*str) - 32U];
                }

                for (i = 0U; i < 6U; i++)
                {
                        idx = (uint16_t)((uint16_t)s_cur_page * SSD1306_WIDTH) +
                              (uint16_t)s_cur_x + (uint16_t)i;
                        vram_put(idx, glyph[i]);
                }

                s_cur_x = (uint8_t)(s_cur_x + 6U);
                str++;
        }
}

/**
 * @brief  把脏页发到面板，没有脏页时不发I2C
 */
void ssd1306_UpdateScreen(void)
{
        /* 逐个检查 page0..page7 的脏位 */
        uint8_t page;

        for (page = 0U; page < SSD1306_PAGES; page++)
        {
                if ((s_dirty & (uint8_t)(1U << page)) != 0U)
                {
                        flush_page(page);
                        s_dirty &= (uint8_t)(~(uint8_t)(1U << page));
                }
        }
}

/**
 * @brief  把 8 页全部置脏并立即刷新（面板复位、I2C 总线恢复之后用）
 */
void ssd1306_FlushAll(void)
{
        s_dirty = 0xFFU;
        ssd1306_UpdateScreen();
}

/**
 * @brief  设置对比度
 * @param  val 0x00..0xFF，越大越亮（上电初始化用的是 0xCF）
 */
void ssd1306_SetContrast(uint8_t val)
{
        ssd1306_WriteCommand(0x81); /* 对比度指令 */
        ssd1306_WriteCommand(val);  /* 0..255      */
}

/**
 * @brief  查询最近一次面板访问是否成功
 * @retval 1 = 成功；0 = 失败（通常是总线被拉死，见 i2c 统计）
 */
uint8_t ssd1306_IsPanelOk(void)
{
        return s_panel_ok;
}

/**
 * @brief  取累计已刷新页数计数
 * @retval 从上电起一共发过多少页，数值越小说明脏页机制越省 I2C
 */
uint32_t ssd1306_GetFlushCount(void)
{
        return s_flush_pages;
}
