#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_HC-SR04.h"
#include "stm32f10x_exti.h"
#include "time.h"

// UART �÷���
char flagUART1 = 0;   // USART1���� ������ ���� �� Ȯ���ϴ� �÷���
char flagUART2 = 0;   // USART2���� ������ ���� �� Ȯ���ϴ� �÷���

char wordFromUART1;   // USART1���� ���� ������ ���� ����
char wordFromUART2;   // USART2���� ���� ������ ���� ����

// �з� ���� ���� ����
volatile double pressureValue = 0; // �з� ����  ��
int flag = 0;             // �з� ���� �÷���
double pressureThreshold1 = 500; // �з� �Ӱ谪 1 - �������� ���� �ʿ�
double pressureThreshold2 = 1000; // �з� �Ӱ谪 2
double dist = 0;          // �Ÿ� ������


// �Լ� ������Ÿ�� ����
void RCC_Configure(void);      // Ŭ�� ����
void GPIO_Configure(void);     // GPIO ����  
void USART1_Init(void);        // USART1 �ʱ�ȭ
void USART2_Init(void);        // USART2 �ʱ�ȭ
void sendDataUART1(uint16_t data); // USART1���� ������ ����
void sendDataUART2(uint16_t data); // USART2���� ������ ����
void NVIC_Configure(void);     // ���ͷ�Ʈ ����
void ADC_Configure(void);      // ADC ����
void ADC1_2_IRQHandler(void);  // ADC ���ͷ�Ʈ �ڵ鷯
void USART1_IRQHandler(void);  // USART1 ���ͷ�Ʈ �ڵ鷯
void USART2_IRQHandler(void);  // USART2 ���ͷ�Ʈ �ڵ鷯
void InitHCSR04(void);         // ������ ���� �ʱ�ȭ
void setRGBLED(int flag);      // RGB LED ����
void TIM4_Configure(void);     // TIM4 Ÿ�̸� ����
void moveMotor(uint16_t var); // ���� ���� �Լ�

void open(void);  // �� ���� �Լ�
void close(void); // �� �ݱ� �Լ�

// Ŭ�� ���� �Լ�
void RCC_Configure(void) {
    // GPIO ��Ʈ Ŭ�� Ȱ��ȭ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Port A Ȱ��ȭ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Port B Ȱ��ȭ  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // Port C Ȱ��ȭ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // Port D Ȱ��ȭ
    
    // ADC Ŭ�� Ȱ��ȭ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    
    // USART Ŭ�� Ȱ��ȭ
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE); // USART1
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE); // USART2
    
    // AFIO Ŭ�� Ȱ��ȭ 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // �������� TIM3 Ȱ��ȭ
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  // Timer4 enable
} 

// GPIO ���� �Լ�
void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // �з� ���� 1 (PC0, ADC ä�� 10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ���׳�ƽ ���� (������ ��)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // USART1 TX(PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // USART1 TX(PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART2 TX(PA2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // USART2 RX(PA3)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RGB LED �� ���� (GPIOB)
    // ������ (PD2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // �ʷϻ� (PB2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // �Ķ��� (PC5)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // ��������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// USART1 �ʱ�ȭ �Լ�
void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    // Enable the USART1 peripheral
    USART_Cmd(USART1, ENABLE);

    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_WordLength = (uint16_t) USART_WordLength_8b;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART1_InitStructure.USART_Parity = (uint16_t) USART_Parity_No;
    USART1_InitStructure.USART_StopBits = (uint16_t) USART_StopBits_1;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART1, &USART1_InitStructure);
    // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

}

// USART2 �ʱ�ȭ �Լ�
void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    // Enable the USART2 peripheral
    USART_Cmd(USART2, ENABLE);

    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    // Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_WordLength = (uint16_t)USART_WordLength_8b;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_Parity = (uint16_t)USART_Parity_No;
    USART2_InitStructure.USART_StopBits = (uint16_t)USART_StopBits_1;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART2, &USART2_InitStructure);
    // TODO: Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

// USART1 ���ͷ�Ʈ �ڵ鷯
void USART1_IRQHandler() {

    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        flagUART1 = 1;
        // the most recent received data by the USART1 peripheral
        wordFromUART1 = USART_ReceiveData(USART1);
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

// USART2 ���ͷ�Ʈ �ڵ鷯
void USART2_IRQHandler() {

    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        flagUART2 = 1;
        // the most recent received data by the USART1 peripheral
        wordFromUART2 = USART_ReceiveData(USART2);
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
}

void sendDataUART1(uint16_t data) {
    USART_SendData(USART1, data);
}

void sendDataUART2(uint16_t data) {
    USART_SendData(USART2, data);
}

// ���ͷ�Ʈ �켱���� ����: TIM4 > USART > ADC
void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    // NVIC Priority Group ����
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
        
    // ADC1 ���ͷ�Ʈ
    NVIC_EnableIRQ(ADC1_2_IRQn);                  // ADC1_2 ���ͷ�Ʈ Ȱ��ȭ
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART1 ���ͷ�Ʈ
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2 ���ͷ�Ʈ
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // TIM4 ���ͷ�Ʈ
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

// ADC ���� �Լ�
void ADC_Configure(void) {
    ADC_DeInit(ADC1);
    ADC_InitTypeDef ADC_InitStructure;
    
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // �з¼��� 1,2 ä�� ����
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);
    
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    
    // ADC ����
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// ADC ��ȯ �Ϸ� ���ͷ�Ʈ �ڵ鷯
void ADC1_2_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
        // ADC ��ȯ �� �б�
        pressureValue = ADC_GetConversionValue(ADC1);

        // ���ͷ�Ʈ �÷��� Ŭ����
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

// TIM4 Ÿ�̸� �ʱ�ȭ �Լ� (PWM ����)
void TIM4_Configure(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint16_t prescale = (uint16_t) (SystemCoreClock / 1000000); // 72 * 10^6 / 10^6
    
    TIM_TimeBaseStructure.TIM_Period = 20000;                   // 72 * 10^6 / 72 / 20000 = 50 Hz
    TIM_TimeBaseStructure.TIM_Prescaler = prescale;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;   // us
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

// ���� ȸ�� �Լ� (PWM ����)
void moveMotor(uint16_t var)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = var;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
}

// ���׳�ƽ ������ �� ���� ����
int isOpen(void) {
    if(~GPIOD->IDR & GPIO_Pin_12) {
        return 0; // ���� �����ٰ� ǥ��
    }
    else {
        return 1; // ���� ���� �ִٰ� ǥ��
    }
}

// ������ ������ ��� ����
int detectPerson(void) {
  if(isOpen() == 1) { // ���� ���� ���� ���� ���� (��, ���׳�ƽ ���� ������ �̿��� ȣ��)
      int distanceThr = 150;  // ���� �Ÿ� �Ӱ谪
      dist = HCSR04GetDistance(); // ������ ������ ���� �Ÿ� ����
      if(distanceThr > dist) {
          return 1;  // ��� ������
      }
      else {
          return 0;  // ��� ����
      }
    }
}

// �� ���� ����
void open() {//�� ���� �Լ�
    if (isOpen() == 0) { //���� ���� ���� ���� ���� ����
        moveMotor(1000);  // ���� ȸ��
        for(int i=0; i < 23000000; i++) {}  // �����̴� ���������� ����
    }
    moveMotor(1500); //���� ����
}

void close() {
    if (isOpen() == 1 && !detectPerson()) { // ���� ���� �ְ�(detectPerson()���� Ȯ��) ����� ���� ���� ���� ����
        moveMotor(2000);  // ���� ȸ��
        for(int i=0; i < 23000000; i++) {}  // �����̴� ���������� ����
    }
    //���� off
    moveMotor(1500);
 }

void setRGBLED(int flag) {
    switch (flag) {
    case 1: // �ʷϻ�
        GPIOD->ODR |= (GPIO_Pin_2);  // ������ ��
        GPIOB->ODR &= ~(GPIO_Pin_2);   // �ʷϻ� ��
        GPIOC->ODR |= (GPIO_Pin_5);  // �Ķ��� ��
        break;
    case 2: // ����� (���� + �ʷ�)
        GPIOD->ODR &= ~(GPIO_Pin_2);  // ������ ��
        GPIOB->ODR &= ~(GPIO_Pin_2);  // �ʷϻ� ��
        GPIOC->ODR |= (GPIO_Pin_5);  // �Ķ��� ��
        break;
    case 3: // ������
        GPIOD->ODR &= ~(GPIO_Pin_2);  // ������ ��
        GPIOB->ODR |= (GPIO_Pin_2);  // �ʷϻ� ��
        GPIOC->ODR |= (GPIO_Pin_5);  // �Ķ��� ��
        break;
    default: // �Ͼ�� (����, �ʷ�, �Ķ��� ��� ��)
      GPIOD->ODR |= GPIO_Pin_2;
        GPIOB->ODR |= GPIO_Pin_2;
        GPIOC->ODR |= GPIO_Pin_5;
        break;
}
}


volatile uint16_t pressure_value = 0;

int main(void) {
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    NVIC_Configure();
    USART1_Init();
    USART2_Init();
    InitHCSR04();
    TIM4_Configure();

    setRGBLED(flag);
    
    while (1) {
        if (flagUART1 == 1){
            sendDataUART2(wordFromUART1);
            flagUART1 = 0;
        }
        else if (flagUART2 == 1){
            sendDataUART1(wordFromUART2);
            flagUART2 = 0;

            // 'o'�� ������ �� ����
            if(wordFromUART2 == 'o') {
                open();
            }
            // 'c'�� ������ �� �ݱ�
            else if(wordFromUART2 == 'c') {
                close();
            }
            else if(wordFromUART2 == 's') {
                moveMotor(1500);
            }

        }
        
        // �з� �������� ���� ���� ����
        if(pressureValue < pressureThreshold1) {
            flag = 1;
        }
        else if(pressureValue < pressureThreshold2) {
            flag = 2;
        }
        else {
            flag = 3;
        }
        
        setRGBLED(flag);
        
    }
     return 0;
}
