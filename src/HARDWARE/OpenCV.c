#include "OpenCV.h"

uint8_t Serial3_RxData;
uint8_t Serial3_RxFlag;
static uint8_t databuf[ReceivedArrLength]; // 接收数组，用来存储OpenCV传递过来的数据

/**
 * @brief 简单的memcpy函数
 *
 */
void OpenCV_memcpy(void *dest, void *src, int n)
{
    uint8_t *d = (uint8_t *)dest;
    uint8_t *s = (uint8_t *)src;
    for (int i = 0; i < n; i++) {
        d[i] = s[i];
    }
}

/**
 * @brief 初始化OpenCV所需要的串口
 * @retval 无
 */
void OpenCV_Init()
{
    Usart3SerialInit(); // 初始化串口3
}

/**
 * @brief  接收OpenCV传来的数据并存储
 * @retval 无
 */
void OpenCV_Calc()
{
    static unsigned char index = 0; // 接收数组的下标
    static uint8_t rdata       = 0; // 从串口中接收到的数据
    rdata                      = Usart3SerialGetRxData();

    static enum {
        FrameHeaderFlag,
        CalcFlag,
        FrameTailFlag
    } state = FrameHeaderFlag;

    // 串口传输中信息的处理
    if (state == FrameHeaderFlag && rdata == FrameHeaderData) { // 处理帧头
        state = CalcFlag;
    } else if (state == CalcFlag) { // 处理数据
        databuf[index++] = rdata;
        if (index == ReceivedArrLength) {
            state = FrameTailFlag;
        }
    } else if (state == FrameTailFlag && rdata == FrameTailData) { // 判断数据处理结束
        state = FrameHeaderFlag;
        index = 0;
    }
}

/**
 * @brief 从串口中取得8位数据
 * @param index 数组里的下标为index的数据
 * @retval uint8_t
 */
uint8_t getUsartBuf(unsigned char index)
{
    return databuf[index];
}

/**
 * @brief 从串口中取得16位数据
 *
 * @param index
 * @return uint16_t
 */
uint16_t getUsartBuf_16(unsigned char index)
{
    uint16_t recevdata;
    OpenCV_memcpy(&recevdata, databuf + index, sizeof(uint16_t));
    return recevdata;
}

/**
 * @brief 从串口中取得32位数据
 *
 * @param index
 * @return uint32_t
 */
uint32_t getUsartBuf_32(unsigned char index)
{
    uint32_t recevdata;
    OpenCV_memcpy(&recevdata, databuf + index, sizeof(uint32_t));
    return recevdata;
}

/**
 * @brief 从串口中取得float类型数据
 *
 * @param index
 * @return float
 */
float getUsartBuf_float(unsigned char index)
{
    float recevdata;
    OpenCV_memcpy(&recevdata, databuf + index, sizeof(float));
    return recevdata;
}

/**
 * @brief Get the Signed Usart Buf object
 *
 * @param index
 * @return int8_t
 */
int8_t
getSignedUsartBuf(unsigned char index)
{
    return (int8_t)databuf[index];
}

/**
 * @brief USART3的中断处理函数
 * @retval 无
 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET) {
        Serial3_RxFlag = 1;
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        OpenCV_Calc();
    }
}

uint8_t Usart3SerialGetRxFlag(void)
{
    if (Serial3_RxFlag == 1) {
        Serial3_RxFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t Usart3SerialGetRxData(void)
{
    Serial3_RxData = USART_ReceiveData(USART3);
    return Serial3_RxData;
}

// void DataTest()
// {
//     OLED_ShowNum(1, 1, getUsartBuf(0), 2);
//     OLED_ShowNum(2, 1, getUsartBuf(1), 2);
//     OLED_ShowNum(3, 1, getUsartBuf(2), 2);
// }

#ifdef WSDC2412D

#define target 100

int8_t Get_Excursion()
{
    static int8_t last_data = 0;
    int8_t A1, B1, C1, D1, E1, F1, A2, B2, C2, D2, E2, F2, Excursion1 = 0, Excursion2 = 0;
    if (getUsartBuf(1) == 0xff || getUsartBuf(2) == 0xff || getUsartBuf(1) == 200 || getUsartBuf(2) == 200 || getUsartBuf(1) == 0 || getUsartBuf(2) == 0) {
        return 0;
    }
    Excursion1 = (getUsartBuf(2) - getUsartBuf(1)) * 2;
    Excursion2 = (getUsartBuf(2) + getUsartBuf(1)) / 2 - target;

#if 0
    A1=getUsartBuf(1)&0x20;
    B1=getUsartBuf(1)&0x10;
    C1=getUsartBuf(1)&0x08;
    D1=getUsartBuf(1)&0x04;
    E1=getUsartBuf(1)&0x02;
    F1=getUsartBuf(1)&0x01;
    A2=getUsartBuf(2)&0x20;
    B2=getUsartBuf(2)&0x10;
    C2=getUsartBuf(2)&0x08;
    D2=getUsartBuf(2)&0x04;
    E2=getUsartBuf(2)&0x02;
    F2=getUsartBuf(2)&0x01;
    if(!(A1_||B1_||C1_||D1 ||E1 ||F1 ))Excursion1=0;
    if(!(A1_||B1_||C1 ||D1 ||E1 ||F1 ))Excursion1=1;
    if(!(A1_||B1_||C1_||D1_||E1 ||F1 ))Excursion1=-1;
    if(!(A1_||B1 ||C1 ||D1 ||E1 ||F1 ))Excursion1=2;
    if(!(A1_||B1_||C1_||D1_||E1_||F1 ))Excursion1=-2;
    if(!(A1 ||B1 ||C1 ||D1 ||E1 ||F1 ))Excursion1=3;
    if(!(A1_||B1_||C1_||D1_||E1_||F1_))Excursion1=-3;
    if(!(A2_||B2_||C2_||D2 ||E2 ||F2 ))Excursion2=0;
    if(!(A2_||B2_||C2 ||D2 ||E2 ||F2 ))Excursion2=-1;
    if(!(A2_||B2_||C2_||D2_||E2 ||F2 ))Excursion2=1;
    if(!(A2_||B2 ||C2 ||D2 ||E2 ||F2 ))Excursion2=-2;
    if(!(A2_||B2_||C2_||D2_||E2_||F2 ))Excursion2=2;
    if(!(A2 ||B2 ||C2 ||D2 ||E2 ||F2 ))Excursion1=3;
    if(!(A2_||B2_||C2_||D2_||E2_||F2_))Excursion1=-3;
#endif
    last_data = Excursion1 + Excursion2;
    return Excursion1 + Excursion2;
}

#else
int8_t Get_Excursion()
{
    int8_t A, B, C, D, E, F, Excursion = 0;
    A = getUsartBuf(2) & 0x20;
    B = getUsartBuf(2) & 0x10;
    C = getUsartBuf(2) & 0x08;
    D = getUsartBuf(2) & 0x04;
    E = getUsartBuf(2) & 0x02;
    F = getUsartBuf(2) & 0x01;
    if (!(A || B || C || D || E || F))
        Excursion = 0;
    if (!(A || B || C_ || D_ || E || F))
        Excursion = 0;
    if (!(A || B_ || C_ || D_ || E_ || F))
        Excursion = 0;
    if (!(A_ || B_ || C_ || D_ || E_ || F_))
        Excursion = 0;
    if (!(A || B || C_ || D || E || F))
        Excursion = 0;
    if (!(A || B || C || D_ || E || F))
        Excursion = 0;
    if (!(A || B || C_ || D_ || E_ || F))
        Excursion = 1;
    if (!(A || B || C || D_ || E_ || F))
        Excursion = 1;
    if (!(A || B || C || D || E_ || F))
        Excursion = 1;
    if (!(A || B_ || C_ || D_ || E || F))
        Excursion = -1;
    if (!(A || B_ || C_ || D || E || F))
        Excursion = -1;
    if (!(A || B_ || C || D || E || F))
        Excursion = -1;
    if (!(A || B || C_ || D_ || E_ || F_))
        Excursion = 2;
    if (!(A || B || C || D_ || E_ || F_))
        Excursion = 2;
    if (!(A || B || C || D || E_ || F_))
        Excursion = 2;
    if (!(A || B || C || D || E || F_))
        Excursion = 2;
    if (!(A_ || B_ || C_ || D_ || E || F))
        Excursion = -2;
    if (!(A_ || B_ || C_ || D || E || F))
        Excursion = -2;
    if (!(A_ || B_ || C || D || E || F))
        Excursion = -2;
    if (!(A_ || B || C || D || E || F))
        Excursion = -2;

    return Excursion;
}
#endif

uint8_t Get_Condition()
{
    uint8_t A, B, C, D, E, Condition = 0;
    E = getUsartBuf(1) & 0x10;
    D = getUsartBuf(1) & 0x08;
    C = getUsartBuf(1) & 0x04;
    B = getUsartBuf(1) & 0x02;
    A = getUsartBuf(1) & 0x01;

    if (!(A_ || E_ || B_ || D || C))
        Condition = 1;
    if (!(A_ || E_ || B_ || D || C_))
        Condition = 1;
    if (!(A_ || E_ || B_ || D_ || C))
        Condition = 1;
    if (!(A_ || E_ || B_ || D_ || C_))
        Condition = 1;

    if (!(A || E_ || B_ || D || C))
        Condition = 1;
    if (!(A || E_ || B_ || D || C_))
        Condition = 1;
    if (!(A || E_ || B_ || D_ || C))
        Condition = 1;
    if (!(A || E_ || B_ || D_ || C_))
        Condition = 1;

    return Condition;
}
uint8_t Get_Color()
{
    return 0;
}
