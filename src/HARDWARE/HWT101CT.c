#include "HWT101CT.h"

uint8_t HWT_Serial_RxData;
uint8_t HWT_Serial_RxFlag;

static uint8_t angle_speed_buf[Angle_Speed_Length]; // 角速度的接收数组
static uint8_t angle_buf[Angle_Length];             // 角度的接收数组

/**
 * @brief 获取角速度(°/s)
 *
 * @return
 */
float HWT_getAngleSpeed()
{
    return (float)((short)(((short)angle_speed_buf[5] << 8) | angle_speed_buf[4])) * 2000 / 32768;
}

/**
 * @brief 获取角度(°)
 *
 * @return
 */
float HWT_getAngle()
{
    float Angle;
    Angle=(float)((short)(((short)angle_buf[5] << 8) | angle_buf[4])) * 180 / 32768;
    if(Angle>-180&&Angle<0)
    {
        Angle=-Angle;
    }
    else if(Angle>0&&Angle<180)
    {
        Angle=360.0-Angle;
    }
    else if(Angle==180||Angle==-180){
        Angle=180.0;
    }
    return Angle;
}

/**
 * @brief Z轴归零
 *
 */
void HWT_setZero()
{
    uint8_t arr[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
    HWT_Serial_SendArray(arr, 5);
}

/**
 * @brief 设置波特率(默认9600)
 *
 * @param bitrate
 */
void HWT_setBaud(uint8_t bitrate)
{
    uint8_t arr[5] = {0xFF, 0xAA, 0x04, bitrate, 0x00};
    HWT_Serial_SendArray(arr, 5);
}

/**
 * @brief 检验和验证
 *
 * @return signed char
 */
int8_t HWT_verifySUM(uint8_t state)
{
    uint8_t sum = 0;
    if (state == ANGLE_SPEED_STATE) {
        sum = 0x55 + 0x52 + angle_speed_buf[2] + angle_speed_buf[3] + angle_speed_buf[4] + angle_speed_buf[5];
        return sum == angle_speed_buf[8];
    } else if (state == ANGLE_STATE) {
        sum = 0x55 + 0x53 + angle_buf[4] + angle_buf[5] + angle_buf[6] + angle_buf[7];
        return sum == angle_buf[8];
    }
    return 0;
}

/**
 * @brief 处理HWT101CT发来的数据
 *
 */
void HWTCalc()
{
    static unsigned char angle_speed_index = 0; // 角速度数组的下标
    static unsigned char angle_index       = 0; // 角度数组的下标
    static uint8_t rdata                   = 0; // 从串口中接收到的数据
    rdata                                  = HWT_Serial_GetRxData();

    const uint8_t HWTHeader      = 0x55; // 帧头
    const uint8_t HWTangle_speed = 0x52; // 角速度
    const uint8_t HWTangle       = 0x53; // 角度

    static enum {
        FrameHeaderFlag,
        FrameOutPut,
        CalcAngleSpeed,
        CalcAngle,
        AngleSpeedTailFlag,
        AngleTailFlag,
    } state = FrameHeaderFlag;

    switch (state) {
        case FrameHeaderFlag: // 帧头
            if (rdata == HWTHeader) {
                state = FrameOutPut;
            }
            break;
        case FrameOutPut: // 分支

            if (rdata == HWTangle_speed) { // 角速度
                state = CalcAngleSpeed;
            } else if (rdata == HWTangle) { // 角度
                state = CalcAngle;
            }
            break;
        case CalcAngleSpeed: // 处理角速度
            angle_speed_buf[angle_speed_index++] = rdata;
            if (angle_speed_index == Angle_Speed_Length - 1) {
                state = AngleSpeedTailFlag;
            }
            break;
        case CalcAngle: // 处理角度的帧尾
            angle_buf[angle_index++] = rdata;
            if (angle_index == Angle_Length - 1) {
                state = AngleTailFlag;
            }
            break;
        case AngleSpeedTailFlag: // 处理角速度的帧尾
            angle_speed_buf[angle_speed_index++] = rdata;
            state                                = FrameHeaderFlag;
            angle_speed_index                    = 0;
            // if (HWT_verifySUM(ANGLE_SPEED_STATE) == 0) {
            //     memset(angle_speed_buf, 0, sizeof(uint8_t) * 9);
            // }
            break;
        case AngleTailFlag: // 处理角度的帧尾
            angle_buf[angle_index++] = rdata;
            state                    = FrameHeaderFlag;
            angle_index              = 0;
            // if (HWT_verifySUM(ANGLE_STATE) == 0) {
            //     memset(angle_buf, 0, sizeof(uint8_t) * 9);
            // }
            break;
        default:
            break;
    }
}

void HWT_Serial_Init()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

     GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10; // TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11; // RX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(UART4, &USART_InitStructure);

    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStructure);


    USART_Cmd(UART4, ENABLE);
}

/**
 * @brief 初始化芯片
 *
 */
void HWT101CT_Init(void)
{
    HWT_Serial_Init();
    Delay_ms(100);
    //HWT_setBaud(HWT_BIRATE_9600);
    HWT_setZero();
}

void HWT_Serial_SendByte(uint8_t Byte)
{
    USART_SendData(UART4, Byte);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
        ;
}

void HWT_Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) {
        HWT_Serial_SendByte(Array[i]);
    }
}

void HWT_Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        HWT_Serial_SendByte(String[i]);
    }
}

uint32_t HWT_Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void HWT_Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        HWT_Serial_SendByte(Number / HWT_Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

uint8_t HWT_Serial_GetRxFlag(void)
{
    if (HWT_Serial_RxFlag == 1) {
        HWT_Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}

uint8_t HWT_Serial_GetRxData(void)
{
    return HWT_Serial_RxData;
}

void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET) {
        HWT_Serial_RxData = USART_ReceiveData(UART4);
        HWT_Serial_RxFlag = 1;
        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
        HWTCalc();
    }
}
