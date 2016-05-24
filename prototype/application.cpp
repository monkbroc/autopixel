#include "application.h"

#if defined(STM32F10X_MD) || !defined(PLATFORM_ID)		//Core
  #define SYSCORECLOCK	72000000UL
#elif defined(STM32F2XX) && defined(PLATFORM_ID)	//Photon
  #define SYSCORECLOCK	60000000UL		// Timer clock tree uses core clock / 2
#else
  #error "*** PARTICLE device not supported by this library. PLATFORM should be Core or Photon ***"
#endif

#define PIXEL_FREQUENCY 800000
#define PIXEL_0_HIGH_TIME_NS 350
#define PIXEL_1_HIGH_TIME_NS 700

// Hard code TIM4 for now
auto timer_peripheral = TIM4;

void enableTimClock() {
  // Enable TIM4 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void configureDebugPins() {
  // Pin D0 (from pinmap_hal.c)
  HAL_Pin_Mode(D0, AF_OUTPUT_PUSHPULL);
  auto gpio_peripheral = GPIOB;
  auto gpio_pin_source = GPIO_PinSource7;
  GPIO_PinAFConfig(gpio_peripheral, gpio_pin_source, GPIO_AF_TIM4);

  // Pin D1
  HAL_Pin_Mode(D1, AF_OUTPUT_PUSHPULL);
  gpio_pin_source = GPIO_PinSource6;
  GPIO_PinAFConfig(gpio_peripheral, gpio_pin_source, GPIO_AF_TIM4);
}

uint32_t baseClock()
{
    // for TIM4
    return SystemCoreClock / 2;
}

uint16_t calculatePeriod(uint32_t pwm_frequency)
{
    return (uint16_t) (baseClock() / pwm_frequency) - 1;
}

uint16_t calculatePulseWidth(uint32_t durationNs)
{
    return (uint16_t) (baseClock() / 1000 * durationNs / 1000000);
}

TIM_TimeBaseInitTypeDef calculateTimeBase() {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };

    uint16_t period = calculatePeriod(PIXEL_FREQUENCY);

    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = 0; // full speed clock
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    return TIM_TimeBaseStructure;
}

void configureTimeBase() {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = calculateTimeBase();
  TIM_TimeBaseInit(timer_peripheral, &TIM_TimeBaseStructure);
}

void configureChannel(uint16_t timerChannel, uint16_t pulseWidth) {
    // PWM1 Mode configuration
    // Initialize all 8 struct params to 0, fixes randomly inverted RX, TX PWM
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = pulseWidth;

    switch(timerChannel) {
      case TIM_Channel_1:
        TIM_OC1Init(timer_peripheral, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(timer_peripheral, TIM_OCPreload_Enable);
        break;
      case TIM_Channel_2:
        TIM_OC2Init(timer_peripheral, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(timer_peripheral, TIM_OCPreload_Enable);
        break;
      case TIM_Channel_3:
        TIM_OC3Init(timer_peripheral, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(timer_peripheral, TIM_OCPreload_Enable);
        break;
      case TIM_Channel_4:
        TIM_OC4Init(timer_peripheral, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(timer_peripheral, TIM_OCPreload_Enable);
        break;
    }
}

void configureChannels() {
  configureChannel(TIM_Channel_2, calculatePulseWidth(PIXEL_0_HIGH_TIME_NS));
  configureChannel(TIM_Channel_1, calculatePulseWidth(PIXEL_1_HIGH_TIME_NS));
}

void enableTimer() {
    // TIM enable counter
    TIM_Cmd(timer_peripheral, ENABLE);
}

void setup() {
  enableTimClock();
  configureDebugPins();
  configureTimeBase();
  configureChannels();
  enableTimer();
}

void loop() {

}
