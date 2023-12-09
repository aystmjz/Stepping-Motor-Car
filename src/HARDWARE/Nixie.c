#include "Nixie.h"

#define Max7219_pinCLK PGout(4)
#define Max7219_pinCS  PGout(2)
#define Max7219_pinDIN PDout(14)

void GPIOMax7219_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

}

/**
 * @brief 发送命令字
 *
 * @param DATA
 */
void Write_Max7219_byte(unsigned char DATA)
{
    unsigned char i;
    Max7219_pinCS = 0;
    for (i = 8; i >= 1; i--) {
        Delay_us(1);
        Max7219_pinCLK = 0;
        Max7219_pinDIN = (DATA & 0x80) >> 7;
        // 先将高位数据发送
        Delay_us(1);
        DATA           = DATA << 1;
        Max7219_pinCLK = 1;
    }
}

/**
 * @brief 写寄存器
 *
 * @param address 写地址
 * @param dat 写数据
 */
void Write_Max7219(unsigned char address, unsigned char dat)
{
    Max7219_pinCS = 0;
    Write_Max7219_byte(address); // 写入地址，即数码管编号
    Write_Max7219_byte(dat);     // 写入数据，即数码管显示数字
    Max7219_pinCS = 1;
}

/**
 * @brief 初始化芯片
 *
 */
void MAX7219_Init()
{
    GPIOMax7219_Init();
    NIXIE_UP;
    Write_Max7219(0x09, 0x00); // 译码方式: no decode
    Write_Max7219(0x0B, 0x07); // 模式: 8个数码管显示
    NIXIE_LIGHT(0x03);
    Nixie_reset(); // 清空显示
}

/**
 * @brief 显示数字(11和11比较特殊)
 *
 * @param index 下标
 * @param num 数字
 */
void Nixie_showNum(unsigned char index, unsigned char num, unsigned char dot)
{
    if ((index < 1) || (index > 8)) {
        /* 错误的下标 */
        return;
    }

    unsigned char ifdot = 0x00;

    if (dot) {
        ifdot = 0x80;
    }

    switch (num) {
        case 0:
            num = 0x7E | ifdot;
            break;
        case 1:
            num = 0x30 | ifdot;
            break;
        case 2:
            num = 0x6D | ifdot;
            break;
        case 3:
            num = 0x79 | ifdot;
            break;
        case 4:
            num = 0x33 | ifdot;
            break;
        case 5:
            num = 0x5B | ifdot;
            break;
        case 6:
            num = 0x5F | ifdot;
            break;
        case 7:
            num = 0x70 | ifdot;
            break;
        case 8:
            num = 0x7F | ifdot;
            break;
        case 9:
            num = 0x7B | ifdot;
            break;
        case 10:
            num = 0x01 | ifdot;
            break;
        case 11:
            num = 0x07 | ifdot;
            break;
        default:
            num = 0x00 | ifdot;
    }
    Write_Max7219(9 - index, num);
}

/**
 * @brief 显示加号（占两个数码管）
 *
 * @param index 下标
 */
void Nixie_showPlus(unsigned char index)
{
    Write_Max7219(9 - index, 0x01);
    Write_Max7219(8 - index, 0x07);
}

/**
 * @brief 清空显示
 *
 */
void Nixie_reset(void)
{
    for (int i = 0; i <= 8; i++) {
        Nixie_showNum(i, -1, 0);
    }
}
