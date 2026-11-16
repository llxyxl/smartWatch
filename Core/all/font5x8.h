#ifndef __FONT5X8_H
#define __FONT5X8_H

#include <stdint.h>
// ASCII 32~126
/* 96 x 6 字节：每个字符 5 列点阵 + 1 列间隔，索引 = 字符ASCII - 32 */
extern const uint8_t font5x8[96][6];

#endif
