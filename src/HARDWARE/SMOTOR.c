#include "SMOTOR.h"

uint8_t SMOTOR_L_flag = 0, SMOTOR_R_flag = 0, SMOTOR_B_flag = 0;
uint32_t SMOTOR_L_target = 0, SMOTOR_R_target = 0, SMOTOR_B_target = 0;
uint32_t SMOTOR_L_step = 0, SMOTOR_R_step = 0, SMOTOR_B_step = 0;
int32_t SMOTOR_L_Location = 0, SMOTOR_R_Location = 0, SMOTOR_B_Location = 0;
double SMOTOR_L_Angle = 0, SMOTOR_R_Angle = 0, SMOTOR_B_Angle = 0;
double SMOTOR_Long = 0, SMOTOR_Height = 0, SMOTOR_Angle = 0;

double SMOTOR_SPEED=80;

void clock_config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 使能AFIO外设时钟,AFIO配置io口
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void NVIC_IQR_Confing(uint8_t nvic_IRQChannel, uint8_t nvic_PreemptionPriority, uint8_t nvic_SubPriority)
{ // 初始化中断控制器NVIC
    NVIC_InitTypeDef nvic_Init;
    nvic_Init.NVIC_IRQChannel                   = nvic_IRQChannel;         // 中断号
    nvic_Init.NVIC_IRQChannelPreemptionPriority = nvic_PreemptionPriority; // 抢占优先级
    nvic_Init.NVIC_IRQChannelSubPriority        = nvic_SubPriority;        // 子优先级
    nvic_Init.NVIC_IRQChannelCmd                = ENABLE;                  // 启用中断优先级
    NVIC_Init(&nvic_Init);
}

void nvic_config(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 配置中断优先级分组
    NVIC_IQR_Confing(TIM2_IRQn, 1, 0);
    NVIC_IQR_Confing(TIM3_IRQn, 1, 0);
    NVIC_IQR_Confing(TIM4_IRQn, 1, 0);
}

void SMOTOR_TIM_Init(void)
{
    TIM_TimeBaseInitTypeDef motor_TimeInit;
    TIM_OCInitTypeDef motor_OCInit;

    // 1.引脚初始化
    GPIO_InitTypeDef motor_gpio_init;
    motor_gpio_init.GPIO_Pin   = DIR_GPIO_A;
    motor_gpio_init.GPIO_Mode  = GPIO_Mode_Out_PP;
    motor_gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &motor_gpio_init); // DIR ENABLE 通用推挽输出模式
    motor_gpio_init.GPIO_Pin   = DIR_GPIO_B;
    motor_gpio_init.GPIO_Mode  = GPIO_Mode_Out_PP;
    motor_gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &motor_gpio_init); // DIR ENABLE 通用推挽输出模式
    motor_gpio_init.GPIO_Pin  = STEP_GPIO_A;
    motor_gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &motor_gpio_init); // STEP 复用推挽输出
    motor_gpio_init.GPIO_Pin  = STEP_GPIO_B;
    motor_gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &motor_gpio_init); // STEP 复用推挽输出

    // 2.定时器初始化配置
    TIM_DeInit(TIM2);
    TIM_DeInit(TIM3);
    TIM_DeInit(TIM4);
    motor_TimeInit.TIM_ClockDivision     = TIM_CKD_DIV1;
    motor_TimeInit.TIM_CounterMode       = TIM_CounterMode_Up; // 向上计数模式
    motor_TimeInit.TIM_Period            = 3999;               // 重装载值
    motor_TimeInit.TIM_Prescaler         = 31;                 // 预分频值
    motor_TimeInit.TIM_RepetitionCounter = 0;                  // 重复计数值
    TIM_TimeBaseInit(TIM2, &motor_TimeInit);                   // 基本计数模式
    TIM_TimeBaseInit(TIM3, &motor_TimeInit);                   // 基本计数模式
    TIM_TimeBaseInit(TIM4, &motor_TimeInit);                   // 基本计数模式

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // 配置更新中断
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);      // 清除更新中断位
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); // 配置更新中断
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);      // 清除更新中断位
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); // 配置更新中断
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);      // 清除更新中断位

    TIM_OCStructInit(&motor_OCInit);
    motor_OCInit.TIM_OCMode      = TIM_OCMode_PWM1;
    motor_OCInit.TIM_OutputState = TIM_OutputState_Enable;
    motor_OCInit.TIM_Pulse       = 1499;
    TIM2_OCInit(TIM2, &motor_OCInit);
    TIM3_OCInit(TIM3, &motor_OCInit);
    TIM4_OCInit(TIM4, &motor_OCInit);

    TIM_Cmd(TIM2, DISABLE);
    TIM_Cmd(TIM3, DISABLE);
    TIM_Cmd(TIM4, DISABLE);
}

// 步进电机初始化
void SMOTOR_Init(void)
{
    clock_config();    // 配置RCC时钟
    nvic_config();     // 配置中断优先级
    SMOTOR_TIM_Init(); // motor初始化
}

void SET_TIM(Tim Vale, uint16_t ARR, uint16_t CCR)
{
    switch (Vale) {
        case Tim2:
            TIM_SetAutoreload(TIM2, ARR);
            TIM2_SetCompare(TIM2, CCR);
            break;
        case Tim3:
            TIM_SetAutoreload(TIM3, ARR);
            TIM3_SetCompare(TIM3, CCR);
            break;
        case Tim4:
            TIM_SetAutoreload(TIM4, ARR);
            TIM4_SetCompare(TIM4, CCR);
            break;
    }
}

void SET_Direction(uint8_t SMOTOR, Direction Dir)
{
    switch (SMOTOR) {
        case SMOTOR_L:
            GPIO_WriteBit(GPIOA, GPIO_Pin_0, (BitAction)(Dir));
            break;
        case SMOTOR_R:
            GPIO_WriteBit(GPIOB, GPIO_Pin_0, (BitAction)(Dir));
            break;
        case SMOTOR_B:
            GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(Dir));
            break;
    }
}

void SMOTOR_STOP(uint8_t SMOTOR)
{
    switch (SMOTOR) {
        case SMOTOR_L:
            TIM_Cmd(TIM2, DISABLE);
            break;
        case SMOTOR_R:
            TIM_Cmd(TIM3, DISABLE);
            break;
        case SMOTOR_B:
            TIM_Cmd(TIM4, DISABLE);
            break;
    }
}

void SMOTOR_START(uint8_t SMOTOR)
{
    switch (SMOTOR) {
        case SMOTOR_L:
            TIM_Cmd(TIM2, ENABLE);
            break;
        case SMOTOR_R:
            TIM_Cmd(TIM3, ENABLE);
            break;
        case SMOTOR_B:
            TIM_Cmd(TIM4, ENABLE);
            break;
    }
}

/// @brief 获取步进电机当前状态
/// @param SMOTOR 目标电机（支持SMOTOR_L|SMOTOR_R写法）
/// @return 目标电机全部到达指定位置时返回0
uint8_t Get_State(uint8_t SMOTOR)
{
    uint8_t State;
    switch (SMOTOR) {
        case SMOTOR_L:
            State = SMOTOR_L_flag;
            break;
        case SMOTOR_R:
            State = SMOTOR_R_flag;
            break;
        case SMOTOR_B:
            State = SMOTOR_B_flag;
            break;
        case SMOTOR_L | SMOTOR_R:
            State = SMOTOR_L_flag || SMOTOR_R_flag;
            break;
        case SMOTOR_R | SMOTOR_B:
            State = SMOTOR_R_flag || SMOTOR_B_flag;
            break;
        case SMOTOR_L | SMOTOR_B:
            State = SMOTOR_L_flag || SMOTOR_B_flag;
            break;
        case SMOTOR_L | SMOTOR_R | SMOTOR_B:
            State = SMOTOR_L_flag || SMOTOR_R_flag || SMOTOR_B_flag;
            break;
    }
    return State;
}

/// @brief 初始化步进电机
/// @param SMOTOR 目标电机（支持SMOTOR_L|SMOTOR_R写法）
void SMOTOR_RESET(uint8_t SMOTOR)
{
    switch (SMOTOR) {
        case SMOTOR_L:
            SMOTOR_L_Location = 0;
            SMOTOR_L_flag     = 0;
            SMOTOR_STOP(SMOTOR_L);
            break;
        case SMOTOR_R:
            SMOTOR_R_Location = 0;
            SMOTOR_R_flag     = 0;
            SMOTOR_STOP(SMOTOR_R);
            break;
        case SMOTOR_B:
            SMOTOR_B_Location = 0;
            SMOTOR_B_flag     = 0;
            SMOTOR_STOP(SMOTOR_B);
            break;
        case SMOTOR_L | SMOTOR_R:
            SMOTOR_L_Location = 0;
            SMOTOR_R_Location = 0;
            SMOTOR_L_flag     = 0;
            SMOTOR_STOP(SMOTOR_L);
            SMOTOR_R_flag = 0;
            SMOTOR_STOP(SMOTOR_R);
            break;
        case SMOTOR_R | SMOTOR_B:
            SMOTOR_R_Location = 0;
            SMOTOR_B_Location = 0;
            SMOTOR_R_flag     = 0;
            SMOTOR_STOP(SMOTOR_R);
            SMOTOR_B_flag = 0;
            SMOTOR_STOP(SMOTOR_B);
            break;
        case SMOTOR_L | SMOTOR_B:
            SMOTOR_L_Location = 0;
            SMOTOR_B_Location = 0;
            SMOTOR_L_flag     = 0;
            SMOTOR_STOP(SMOTOR_L);
            SMOTOR_B_flag = 0;
            SMOTOR_STOP(SMOTOR_B);
            break;
        case SMOTOR_L | SMOTOR_R | SMOTOR_B:
            SMOTOR_L_Location = 0;
            SMOTOR_R_Location = 0;
            SMOTOR_B_Location = 0;
            SMOTOR_L_flag     = 0;
            SMOTOR_STOP(SMOTOR_L);
            SMOTOR_R_flag = 0;
            SMOTOR_STOP(SMOTOR_R);
            SMOTOR_B_flag = 0;
            SMOTOR_STOP(SMOTOR_B);
            break;
    }
}

/// @brief 步进电机控制
/// @param Spead 速度（）
/// @param Location 角度（500对应转动22.5度）
/// @param SMOTOR 目标步进电机
void SMOTOR_CONTROL(uint32_t Spead, int32_t Location, uint8_t SMOTOR)
{
    int32_t Error = 0;
    if (Spead <= 10) return;
    switch (SMOTOR) {
        case SMOTOR_L:
            if (!SMOTOR_L_flag) {
                SMOTOR_L_flag     = 1;
                SMOTOR_L_step     = 0;
                Error             = Location - SMOTOR_L_Location;
                SMOTOR_L_Location = Location;
                if (Error < 0) {
                    Error = -Error;
                    SET_Direction(SMOTOR, Backward);
                } else
                    SET_Direction(SMOTOR, Forward);
                SMOTOR_L_target = Error;
                SET_TIM(Tim2, Spead - 1, Spead / 2 - 1);
                SMOTOR_START(SMOTOR_L);
            }
            break;
        case SMOTOR_R:
            if (!SMOTOR_R_flag) {
                SMOTOR_R_flag     = 1;
                SMOTOR_R_step     = 0;
                Error             = Location - SMOTOR_R_Location;
                SMOTOR_R_Location = Location;
                if (Error < 0) {
                    Error = -Error;
                    SET_Direction(SMOTOR, Backward);
                } else
                    SET_Direction(SMOTOR, Forward);
                SMOTOR_R_target = Error;
                SET_TIM(Tim3, Spead - 1, Spead / 2 - 1);
                SMOTOR_START(SMOTOR_R);
            }
            break;
        case SMOTOR_B:
            if (!SMOTOR_B_flag) {
                SMOTOR_B_flag     = 1;
                SMOTOR_B_step     = 0;
                Error             = Location - SMOTOR_B_Location;
                SMOTOR_B_Location = Location;
                if (Error < 0) {
                    Error = -Error;
                    SET_Direction(SMOTOR, Backward);
                } else
                    SET_Direction(SMOTOR, Forward);
                SMOTOR_B_target = Error;
                SET_TIM(Tim4, Spead - 1, Spead / 2 - 1);
                SMOTOR_START(SMOTOR_B);
            }
            break;
    }
}

/// @brief 移动步进电机到指定位置
/// @param Long 机械臂伸出长度
/// @param Height 机械臂底端高度
/// @param Angle 机械臂旋转角度
void SMOTOR_MOVE(double Long, double Height, double Angle,double Speed)
{
    SMOTOR_SPEED=Speed;
    angleTypeDef result;
    result = SMOTOR_ANGLE(Long, Height, Angle);
    SMOTOR_CONTROL(result.speed_L, result.angle_L * 200 / 9, SMOTOR_L);
    SMOTOR_CONTROL(result.speed_R, result.angle_R * 200 / 9, SMOTOR_R);
    SMOTOR_CONTROL(result.speed_B, result.angle_B * 200 / 9, SMOTOR_B);
    while (Get_State(SMOTOR_L | SMOTOR_R | SMOTOR_B)) {}
}

/// @brief 移动步进电机到相对位置
/// @param Delta_Long 机械臂伸出相对长度
/// @param Delta_Height 机械臂底端相对高度
/// @param Delta_Angle 机械臂旋转相对角度
void SMOTOR_Delta_MOVE(double Delta_Long, double Delta_Height, double Delta_Angle,double Speed)
{
    SMOTOR_SPEED=Speed;
    angleTypeDef result;
    result = SMOTOR_ANGLE(SMOTOR_Long + Delta_Long, SMOTOR_Height + Delta_Height, SMOTOR_Angle + Delta_Angle);
    SMOTOR_CONTROL(result.speed_L, result.angle_L * 200 / 9, SMOTOR_L);
    SMOTOR_CONTROL(result.speed_R, result.angle_R * 200 / 9, SMOTOR_R);
    SMOTOR_CONTROL(result.speed_B, result.angle_B * 200 / 9, SMOTOR_B);
    while (Get_State(SMOTOR_L | SMOTOR_R | SMOTOR_B)) {}
}

void SMOTOR_XY_MOVE(double Location_X, double Location_Y, double Height,double Speed)
{
    double Long, Angle;
    SMOTOR_SPEED=Speed;
    Angle = Angle_Grasp;
    Long  = sqrt(Location_Y * Location_Y + Location_X * Location_X);
    Angle -= Angle_Clculate(Location_X, (Location_Y + Location_X));
    Long -= Camera_Distance;
    SMOTOR_MOVE(Long, Height, Angle,Speed);
}
/// @brief 复位步进电机
/// @param SMOTOR 目标电机（支持SMOTOR_L|SMOTOR_R写法）
void SMOTOR_ResetLocation(uint8_t SMOTOR)
{
    switch (SMOTOR) {
        case SMOTOR_L:
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_L);
            while (Get_State(SMOTOR_L)) {}
            break;
        case SMOTOR_R:
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_R);
            while (Get_State(SMOTOR_R)) {}
            break;
        case SMOTOR_B:
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_B);
            while (Get_State(SMOTOR_B)) {}
            break;
        case SMOTOR_L | SMOTOR_R:
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_L);
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_R);
            while (Get_State(SMOTOR_L | SMOTOR_R)) {}
            break;
        case SMOTOR_R | SMOTOR_B:
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_R);
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_B);
            while (Get_State(SMOTOR_R | SMOTOR_B)) {}
            break;
        case SMOTOR_L | SMOTOR_B:
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_L);
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_B);
            while (Get_State(SMOTOR_L | SMOTOR_B)) {}
            break;
        case SMOTOR_L | SMOTOR_R | SMOTOR_B:
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_L);
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_R);
            SMOTOR_CONTROL(SMOTOR_SPEED_K / SMOTOR_SPEED, 0, SMOTOR_B);
            while (Get_State(SMOTOR_L | SMOTOR_R | SMOTOR_B)) {}
            break;
    }
}

/// @brief 角度计算 (初始状态 Long=96.1  Height=0)
/// @param Long 机械臂伸出长度
/// @param Height 机械臂底端高度
/// @param Angle 机械臂旋转角度
/// @return 角度计算结果
angleTypeDef SMOTOR_ANGLE(double Long, double Height, double Angle)
{
    angleTypeDef angle;
    double Angle_L, Angle_R;
    double Delta_Angle_max, Delta_Angle_min, Delta_Angle_B;

    SMOTOR_Long   = Long;
    SMOTOR_Height = Height;
    SMOTOR_Angle  = Angle;
    if (!Height) {
        Angle_L = Clculate_Angle_L_Z(Long);
        Angle_R = Clculate_Angle_R_Z(Long);
    } else if (Height > 0) {
        Angle_L = Clculate_Angle_L_P(Long, Height);
        Angle_R = Clculate_Angle_R_P(Long, Height);

    } else {
        Angle_L = Clculate_Angle_L_N(Long, -Height);
        Angle_R = Clculate_Angle_R_N(Long, -Height);
    }

    angle.angle_L = Modify(Angle_L, Angle_R) - Modify(SMOTOR_L_Init, SMOTOR_R_Init);
    angle.angle_R = Angle_R - SMOTOR_R_Init;
    angle.angle_B = Angle - SMOTOR_B_Init;

    Delta_Angle_max = SMOTOR_L_Angle - angle.angle_L;
    Delta_Angle_min = SMOTOR_R_Angle - angle.angle_R;
    Delta_Angle_B   = SMOTOR_B_Angle - angle.angle_B;

    if (Delta_Angle_max < 0) Delta_Angle_max = -Delta_Angle_max;
    if (Delta_Angle_min < 0) Delta_Angle_min = -Delta_Angle_min;
    if (Delta_Angle_B < 0) Delta_Angle_B = -Delta_Angle_B;

    if (!Delta_Angle_max || !Delta_Angle_min) {
        angle.speed_L = SMOTOR_SPEED_K / SMOTOR_SPEED;
        angle.speed_R = SMOTOR_SPEED_K / SMOTOR_SPEED;
        angle.speed_B = SMOTOR_SPEED_K / SMOTOR_SPEED_B;

    } else if (Delta_Angle_max > Delta_Angle_min) {
        angle.speed_L = SMOTOR_SPEED_K / SMOTOR_SPEED;
        angle.speed_R = (Delta_Angle_max * SMOTOR_SPEED_K) / (Delta_Angle_min * SMOTOR_SPEED);
        angle.speed_B = (Delta_Angle_max * SMOTOR_SPEED_K) / (Delta_Angle_B * SMOTOR_SPEED);
    } else {
        angle.speed_R = SMOTOR_SPEED_K / SMOTOR_SPEED;
        angle.speed_L = (Delta_Angle_min * SMOTOR_SPEED_K) / (Delta_Angle_max * SMOTOR_SPEED);
        angle.speed_B = (Delta_Angle_min * SMOTOR_SPEED_K) / (Delta_Angle_B * SMOTOR_SPEED);
    }
    if (angle.speed_B < SMOTOR_SPEED_K / SMOTOR_SPEED_B_MAX) angle.speed_B = SMOTOR_SPEED_K / SMOTOR_SPEED_B_MAX;
    SMOTOR_L_Angle = angle.angle_L;
    SMOTOR_R_Angle = angle.angle_R;
    SMOTOR_B_Angle = angle.angle_B;
    Delay_ms(50);
    return angle;
}

/// @brief 摄像头粗略调整
/// @param Camera_x 摄像头识别x坐标
/// @param Camera_y 摄像头识别y坐标
/// @param Distance 机械臂伸出长度
/// @param Height 机械臂目标底端高度
void CAMERA_ANGLE(double Camera_x, double Camera_y,double Speed)
{
    Camera_x *= K_x;
    Camera_y *= K_y;
    SMOTOR_XY_MOVE(Camera_x,SMOTOR_Long + Camera_Distance+ Camera_y, SMOTOR_Height,Speed);
}


void CAMERA_ANGLE_(double Camera_x, double Camera_y, double Distance, double Height)
{
    double Long, Angle;
    Camera_x *= K_x;
    Camera_y *= K_y;
    Distance+=Camera_Distance;
    Angle=Angle_Grasp;
    if (Camera_x >= 0 && Camera_y >= 0) {
        Long  = sqrt((Distance + Camera_y) * (Distance + Camera_y) + Camera_x * Camera_x);
        Angle -= Angle_Clculate(Camera_x ,(Camera_y + Distance));
    } else if (Camera_x <= 0 && Camera_y <= 0) {
        Long  = sqrt((Distance + Camera_y) * (Distance + Camera_y) + Camera_x * Camera_x);
        Angle += Angle_Clculate(-Camera_x , (Camera_y + Distance));
    } else if (Camera_x < 0 && Camera_y > 0) {
        Long  = sqrt((Distance + Camera_y) * (Distance + Camera_y) + Camera_x * Camera_x);
        Angle += Angle_Clculate(-Camera_x, (Camera_y + Distance));
    } else {
        Long  = sqrt((Distance + Camera_y) * (Distance + Camera_y) + Camera_x * Camera_x);
        Angle -= Angle_Clculate(Camera_x , (Camera_y + Distance));
    }
    Long-=Camera_Distance;
    SMOTOR_Long   = Long;
    SMOTOR_Height = Height;
    SMOTOR_Angle  = Angle;
    angleTypeDef result;
    result = SMOTOR_ANGLE(Long, Height, Angle);
    SMOTOR_CONTROL(result.speed_L, result.angle_L * 200 / 9, SMOTOR_L);
    SMOTOR_CONTROL(result.speed_R, result.angle_R * 200 / 9, SMOTOR_R);
    SMOTOR_CONTROL(result.speed_B, result.angle_B * 200 / 9, SMOTOR_B);
    while (Get_State(SMOTOR_L | SMOTOR_R | SMOTOR_B)) {}

}
/**
 * @brief 校准机械臂
 *
 */
void smotor_calibrate()
{
    // 坐标
    float x;
    float y;
    // 误差范围
    float min_calibrate;
    float max_calibrate;
    // 在范围之外就继续校准
    while ((x > max_calibrate) || (x < min_calibrate) ||
           (y > max_calibrate) || (y < min_calibrate)) {
        // 不断接收数据并不断校准
    }
    // 校准完成
}

/**
 * @brief 抓物料
 *
 */
void smotor_grasp_obj()
{
    float datax = getUsartBuf_float(0);
    float datay = getUsartBuf_float(4);
    struct angles result1;
    struct angles result2;

    OLED_ShowSignedNum(1, 4, datax * 10000, 8);
    OLED_ShowSignedNum(2, 4, datay * 10000, 8);
    if (datax != START_DEFALTX || datay != START_DEFALTY) { // 扫描到圆
        // 开始移动
        result1 = SMOTOR_ANGLE(datax, datay, 11);
        OLED_ShowSignedNum(3, 4, result1.angle_L, 8);
        OLED_ShowSignedNum(4, 4, result1.angle_R, 8);

        SMOTOR_CONTROL(SMOTOR_SPEED, result1.angle_B * 200 / 9, SMOTOR_B);
        while (Get_State(SMOTOR_B)) {}
        SMOTOR_CONTROL(SMOTOR_SPEED, result1.angle_L * 200 / 9, SMOTOR_L);
        SMOTOR_CONTROL(SMOTOR_SPEED, result1.angle_R * 200 / 9, SMOTOR_R);
        while (Get_State(SMOTOR_L | SMOTOR_R)) {}
        Delay_ms(2000);

        // 移动到了目标地点，再次接收数据
        // float tdatax = datax;
        // float tdatay = datay;
        datax = START_DEFALTX;
        datay = START_DEFALTY;
        while (datax == START_DEFALTX && datay == START_DEFALTY) { // 接收数据到扫描到了圆为止
            datax = getUsartBuf_float(0);
            datay = getUsartBuf_float(4);
            OLED_ShowSignedNum(1, 4, datax * 10000, 8);
            OLED_ShowSignedNum(2, 4, datay * 10000, 8);
        }
        // 进行第二次校准
        result2 = SMOTOR_ANGLE(datax, datay, 11);

        SMOTOR_CONTROL(SMOTOR_SPEED, (result2.angle_B - result1.angle_B) * 200 / 9, SMOTOR_B);
        while (Get_State(SMOTOR_B)) {}
        SMOTOR_CONTROL(SMOTOR_SPEED, (result2.angle_L - result1.angle_L) * 200 / 9, SMOTOR_L);
        SMOTOR_CONTROL(SMOTOR_SPEED, (result2.angle_R - result1.angle_R) * 200 / 9, SMOTOR_R);
        while (Get_State(SMOTOR_L | SMOTOR_R)) {}
        OLED_ShowSignedNum(3, 4, result2.angle_L - result1.angle_L, 8);
        OLED_ShowSignedNum(4, 4, result2.angle_R - result1.angle_R, 8);
    }
    for (;;)
        ;
}

// 中断服务函数
void TIM2_IRQHandler(void)
{
    if (RESET != TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志位

        SMOTOR_L_flag = 1;
        SMOTOR_L_step++;
        if (SMOTOR_L_step >= SMOTOR_L_target) {
            SMOTOR_L_flag = 0;
            SMOTOR_STOP(SMOTOR_L);
        }
    }
}

void TIM3_IRQHandler(void)
{
    if (RESET != TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update); // 清除中断标志位

        SMOTOR_R_flag = 1;
        SMOTOR_R_step++;
        if (SMOTOR_R_step >= SMOTOR_R_target) {
            SMOTOR_R_flag = 0;
            SMOTOR_STOP(SMOTOR_R);
        }
    }
}

void TIM4_IRQHandler(void)
{
    if (RESET != TIM_GetITStatus(TIM4, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update); // 清除中断标志位

        SMOTOR_B_flag = 1;
        SMOTOR_B_step++;
        if (SMOTOR_B_step >= SMOTOR_B_target) {
            SMOTOR_B_flag = 0;
            SMOTOR_STOP(SMOTOR_B);
        }
    }
}

// 设置步进模式 （用于MS1、MS2、MS3与单片机IO连接时使用）
// 速度=(1/2M) * speed * (1.8/16)  度/秒
#if 0
void setMotorStepMod(uint8_t stepMod)
{
    switch(StepMode)
        {
            case MOTOR_FULL_STEP:
                                MOTOR_MS1(0);
                                MOTOR_MS2(0);
                                MOTOR_MS3(0);
                break;
            case MOTOR_HALF_STEP:
                                MOTOR_MS1(1);
                                MOTOR_MS2(0);
                                MOTOR_MS3(0);
                break;
            case MOTOR_QUARTER_STEP:
                                MOTOR_MS1(0);
                                MOTOR_MS2(1);
                                MOTOR_MS3(0);
                break;
            case MOTOR_EIGHTH_STEP:
                                MOTOR_MS1(1);
                                MOTOR_MS2(1);
                                MOTOR_MS3(0);
                break;
            case MOTOR_SIXTEENTH_STEP:
                                MOTOR_MS1(1);
                                MOTOR_MS2(1);
                                MOTOR_MS3(1);
                break;
            default :
                                MOTOR_MS1(1);
                                MOTOR_MS2(1);
                                MOTOR_MS3(1);
                break;
        }
}
#endif
