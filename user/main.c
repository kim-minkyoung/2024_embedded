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

// UART 플래그
char flagUART1 = 0;   // USART1에서 데이터 수신 시 확인하는 플래그
char flagUART2 = 0;   // USART2에서 데이터 수신 시 확인하는 플래그

char wordFromUART1;   // USART1으로 받은 데이터 저장 변수
char wordFromUART2;   // USART2으로 받은 데이터 저장 변수

// 압력 센서 관련 변수
volatile double pressureValue = 0; // 압력 센서  값
int flag = 0;             // 압력 상태 플래그
double pressureThreshold1 = 500; // 압력 임계값 1 - 실험으로 결정 필요
double pressureThreshold2 = 1000; // 압력 임계값 2
double dist = 0;          // 거리 측정값


// 함수 프로토타입 선언
void RCC_Configure(void);      // 클럭 설정
void GPIO_Configure(void);     // GPIO 설정  
void USART1_Init(void);        // USART1 초기화
void USART2_Init(void);        // USART2 초기화
void sendDataUART1(uint16_t data); // USART1으로 데이터 전송
void sendDataUART2(uint16_t data); // USART2으로 데이터 전송
void NVIC_Configure(void);     // 인터럽트 설정
void ADC_Configure(void);      // ADC 설정
void ADC1_2_IRQHandler(void);  // ADC 인터럽트 핸들러
void USART1_IRQHandler(void);  // USART1 인터럽트 핸들러
void USART2_IRQHandler(void);  // USART2 인터럽트 핸들러
void InitHCSR04(void);         // 초음파 센서 초기화
void setRGBLED(int flag);      // RGB LED 제어
void TIM4_Configure(void);     // TIM4 타이머 설정
void moveMotor(uint16_t var); // 모터 제어 함수

void open(void);  // 문 열기 함수
void close(void); // 문 닫기 함수

// 클럭 설정 함수
void RCC_Configure(void) {
    // GPIO 포트 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // Port A 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Port B 활성화  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // Port C 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // Port D 활성화
    
    // ADC 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    
    // USART 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN, ENABLE); // USART1
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_USART2EN, ENABLE); // USART2
    
    // AFIO 클럭 활성화 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // 서보모터 TIM3 활성화
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  // Timer4 enable
} 

// GPIO 설정 함수
void GPIO_Configure(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 압력 센서 1 (PC0, ADC 채널 10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 마그네틱 센서 (고정된 쪽)
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

    // RGB LED 핀 설정 (GPIOB)
    // 빨간색 (PD2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 초록색 (PB2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 파란색 (PC5)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // 서보모터
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// USART1 초기화 함수
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

// USART2 초기화 함수
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

// USART1 인터럽트 핸들러
void USART1_IRQHandler() {

    if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
        flagUART1 = 1;
        // the most recent received data by the USART1 peripheral
        wordFromUART1 = USART_ReceiveData(USART1);
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
    }
}

// USART2 인터럽트 핸들러
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

// 인터럽트 우선순위 설정: TIM4 > USART > ADC
void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    // NVIC Priority Group 설정
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
        
    // ADC1 인터럽트
    NVIC_EnableIRQ(ADC1_2_IRQn);                  // ADC1_2 인터럽트 활성화
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART1 인터럽트
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART2 인터럽트
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // TIM4 인터럽트
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

// ADC 설정 함수
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
    
    // 압력센서 1,2 채널 설정
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);
    
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    
    // ADC 보정
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// ADC 변환 완료 인터럽트 핸들러
void ADC1_2_IRQHandler(void) {
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET) {
        // ADC 변환 값 읽기
        pressureValue = ADC_GetConversionValue(ADC1);

        // 인터럽트 플래그 클리어
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

// TIM4 타이머 초기화 함수 (PWM 생성)
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

// 모터 회전 함수 (PWM 제어)
void moveMotor(uint16_t var)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = var;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
}

// 마그네틱 센서로 문 닫힘 감지
int isOpen(void) {
    if(~GPIOD->IDR & GPIO_Pin_12) {
        return 0; // 문이 닫혔다고 표시
    }
    else {
        return 1; // 문이 열려 있다고 표시
    }
}

// 초음파 센서로 사람 감지
int detectPerson(void) {
  if(isOpen() == 1) { // 문이 열려 있을 때만 감지 (즉, 마그네틱 값이 초음파 이용을 호출)
      int distanceThr = 150;  // 감지 거리 임계값
      dist = HCSR04GetDistance(); // 초음파 센서를 통한 거리 측정
      if(distanceThr > dist) {
          return 1;  // 사람 감지됨
      }
      else {
          return 0;  // 사람 없음
      }
    }
}

// 문 열기 동작
void open() {//문 여는 함수
    if (isOpen() == 0) { //문이 닫혀 있을 때만 모터 동작
        moveMotor(1000);  // 모터 회전
        for(int i=0; i < 23000000; i++) {}  // 딜레이는 실험적으로 지정
    }
    moveMotor(1500); //모터 정지
}

void close() {
    if (isOpen() == 1 && !detectPerson()) { // 문이 열려 있고(detectPerson()에서 확인) 사람이 없을 때만 모터 동작
        moveMotor(2000);  // 모터 회전
        for(int i=0; i < 23000000; i++) {}  // 딜레이는 실험적으로 지정
    }
    //모터 off
    moveMotor(1500);
 }

void setRGBLED(int flag) {
    switch (flag) {
    case 1: // 초록색
        GPIOD->ODR |= (GPIO_Pin_2);  // 빨간색 끔
        GPIOB->ODR &= ~(GPIO_Pin_2);   // 초록색 켬
        GPIOC->ODR |= (GPIO_Pin_5);  // 파란색 끔
        break;
    case 2: // 노란색 (빨간 + 초록)
        GPIOD->ODR &= ~(GPIO_Pin_2);  // 빨간색 켬
        GPIOB->ODR &= ~(GPIO_Pin_2);  // 초록색 켬
        GPIOC->ODR |= (GPIO_Pin_5);  // 파란색 끔
        break;
    case 3: // 빨간색
        GPIOD->ODR &= ~(GPIO_Pin_2);  // 빨간색 켬
        GPIOB->ODR |= (GPIO_Pin_2);  // 초록색 끔
        GPIOC->ODR |= (GPIO_Pin_5);  // 파란색 끔
        break;
    default: // 하얀색 (빨간, 초록, 파란색 모두 켬)
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

            // 'o'를 받으면 문 열기
            if(wordFromUART2 == 'o') {
                open();
            }
            // 'c'를 받으면 문 닫기
            else if(wordFromUART2 == 'c') {
                close();
            }
            else if(wordFromUART2 == 's') {
                moveMotor(1500);
            }

        }
        
        // 압력 센서값에 따른 상태 설정
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
