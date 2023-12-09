#include "QR_code.h"

static uint8_t Serial2_RxData;
static uint8_t Serial2_RxFlag;

static uint8_t recevBuf[QRBUFLEN];
static int recevBufindex = 0;

uint8_t Block_Data[10];

void Clear_Buffer(void) // 清空缓存
{
    u8 i;
    for (i = 0; i < recevBufindex; i++)
        recevBuf[i] = 0; // 缓存
    recevBufindex = 0;
}

void QR_Init(uint32_t bound)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = bound;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void Serial2_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        ;
}

void Serial2_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) {
        Serial2_SendByte(Array[i]);
    }
}

void Serial2_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        Serial2_SendByte(String[i]);
    }
}

uint32_t Serial2_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void Serial2_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        Serial2_SendByte(Number / Serial2_Pow(10, Length - i - 1) % 10 + '0');
    }
}

int fputc(int ch, FILE *f)
{
    Serial2_SendByte(ch);
    return ch;
}

void Serial2_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial2_SendString(String);
}

uint8_t Serial2_GetRxFlag(void)
{
    if (Serial2_RxFlag == 1) {
        Serial2_RxFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t Serial2_GetRxData(void)
{
    return Serial2_RxData;
}

uint8_t Usart2SerialGetRxData(void)
{
    Serial2_RxData = USART_ReceiveData(USART1);
    return Serial2_RxData;
}

/**
 * @brief 开始扫码
 *
 * @param ms 延迟等待的时间
 * @param if_calibrate 是否开启回应校准
 * @mode:
 *          9600 比特率
 *          8位数据位
 *          1位停止位
 *          命令触发模式
 *          有前缀后缀
 */
#define Wait_Angle 20
extern double SMOTOR_Angle;
void start_scan_QRCode(uint16_t Wait_Time)
{
    uint16_t i, Times, timeout = 0, flag = 0;
    for (Times = 1; Times <= 100; Times++) {
        Clear_Buffer();
        uint8_t outbuf[9] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
        for (i = 0; i < 9; i++) { Serial2_SendByte(outbuf[i]); }
        while (!flag) {
            for (i = 0; i <= recevBufindex; i++) {
                if (recevBuf[i] == PREIX &&(recevBuf[i + 8] == SUFIX||recevBuf[i + 9] == SUFIX) ) {
                    flag = 1;
                    for (uint8_t j = 1; j <= 7; j++) { Block_Data[j] = recevBuf[j + i] - '0'; }
                }
            }
            Delay_ms(1);
            timeout++;
            if (timeout > Wait_Time) {
                timeout = 0;
                break;
            }
        }
        if (flag) {
            if (SMOTOR_Angle) SMOTOR_Angle_Adjust(0, 100);
            return;
        }
        if (Times % 2)
            SMOTOR_Angle_Adjust(Wait_Angle, 15);
        else
            SMOTOR_Angle_Adjust(-Wait_Angle, 15);
        for (i = 0; i <= recevBufindex; i++) {
            if (recevBuf[i] == PREIX &&(recevBuf[i + 8] == SUFIX||recevBuf[i + 9] == SUFIX)) {
                for (uint8_t j = 1; j <= 7; j++) { Block_Data[j] = recevBuf[j + i] - '0'; }
                if (SMOTOR_Angle) SMOTOR_Angle_Adjust(0, 100);
                return;
            }
        }
    }
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
        // 接收字符串
        Serial2_RxFlag = 1;
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        recevBuf[recevBufindex++] = Usart2SerialGetRxData();
    }
}
