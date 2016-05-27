#include "application.h"

// See
// hal/src/stm32f2xx/adc_hal.c
// hal/src/stm32f2xx/spi_hal.c
// hal/src/stm32/pwm_hal.c
// hal/src/stm32f2xx/pinmap_hal.c

#define PIXEL_FREQUENCY 800000
#define PIXEL_0_HIGH_DC 20
#define PIXEL_1_HIGH_DC 80

uint16_t pixel_0_pulse, pixel_1_pulse;

// Hard code TIM5 for now
auto timerPeripheral = TIM5;
auto timerPeripheralEnable = RCC_APB1Periph_TIM5;
auto timerChannel = TIM_Channel_1;
auto timerPulseRegister = &timerPeripheral->CCR1;
auto timerPinFunction = GPIO_AF_TIM1;

// DMA1, channel 2 for TIM4
auto dmaPeripheralEnable = RCC_AHB1Periph_DMA1;
auto dmaChannel = DMA_Channel_6;
auto dmaStream = DMA1_Stream6;
auto dmaSource = TIM_DMA_Update;
auto dmaFinishFlag = DMA_FLAG_TCIF6;

// Use A7 as output (GPIOA, GPIO_Pin_0)
pin_t outputPin = A7;
auto pinPeripheralEnable = RCC_AHB1Periph_GPIOA;

uint32_t pixels[] = {
    0xFF0000,
    0xFFFF00,
    0xFF00FF,
    0x00FF00,
    0x00FFFF,
    0x0000FF
};

const auto pixelCount = sizeof(pixels) / sizeof(pixels[0]);
const auto bitsPerPixel = 24;
const auto bitCount = pixelCount * bitsPerPixel;

const auto pulseCount = bitCount + 1; // extra pulse will be 0
uint16_t ledPulses[pulseCount];

void initializeTimer() {
    // Connect pin to timer
    auto &pinMap = PIN_MAP[outputPin];
    RCC_AHB1PeriphClockCmd(pinPeripheralEnable, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    GPIO_InitStructure.GPIO_Pin = pinMap.gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(pinMap.gpio_peripheral, &GPIO_InitStructure);

    GPIO_PinAFConfig(pinMap.gpio_peripheral, pinMap.gpio_pin_source, timerPinFunction);

    // Enable TIM5 clock
    RCC_APB1PeriphClockCmd(timerPeripheralEnable, ENABLE);

    // for TIM5
    uint32_t clock = SystemCoreClock / 2;

    uint16_t period = (uint16_t)(clock / PIXEL_FREQUENCY) - 1;
    pixel_0_pulse = period * PIXEL_0_HIGH_DC;
    pixel_1_pulse = period * PIXEL_1_HIGH_DC;

    // Time base configuration
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = 0; // full speed clock
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timerPeripheral, &TIM_TimeBaseStructure);

    // Configure PWM
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    switch(timerChannel) {
      case TIM_Channel_1:
        TIM_OC1Init(timerPeripheral, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(timerPeripheral, TIM_OCPreload_Enable);
        break;
      case TIM_Channel_2:
        TIM_OC2Init(timerPeripheral, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(timerPeripheral, TIM_OCPreload_Enable);
        break;
      case TIM_Channel_3:
        TIM_OC3Init(timerPeripheral, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(timerPeripheral, TIM_OCPreload_Enable);
        break;
      case TIM_Channel_4:
        TIM_OC4Init(timerPeripheral, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(timerPeripheral, TIM_OCPreload_Enable);
        break;
    }
}


void initializeDMA() {
    RCC_AHB1PeriphClockCmd(dmaPeripheralEnable, ENABLE);

    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(dmaStream);

    DMA_StructInit(&DMA_InitStructure);

    DMA_InitStructure.DMA_BufferSize = 42;
    DMA_InitStructure.DMA_Channel = dmaChannel;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ledPulses;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)timerPulseRegister;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    DMA_Init(dmaStream, &DMA_InitStructure);

    TIM_DMACmd(timerPeripheral, dmaSource, ENABLE);
}

// Fill an array with the pulse times
void fillBits() {
    uint16_t bit = 0;
    for(uint16_t i = 0; i < pixelCount; i++) {
        for(uint32_t mask = 0x800000; mask != 0; mask >>= 1) {
            ledPulses[bit] = (pixels[i] & mask) ? pixel_1_pulse : pixel_0_pulse;
            bit++;
        }
    }

    // Put back pulse to 0 at the end
    ledPulses[bit] = 0;
}

void send() {
    DMA_SetCurrDataCounter(dmaStream, pulseCount);
    TIM_SetCounter(timerPeripheral, 0);
    TIM_SetCompare1(timerPeripheral, 0);
    TIM_Cmd(timerPeripheral, ENABLE);

    DMA_Cmd(dmaStream, ENABLE);
}

void setup() {
    Serial.begin(9600);
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    pinMode(A7, OUTPUT);
    analogWrite(A7, 50);

    initializeTimer();
    initializeDMA();

    fillBits();
    send();
}

void loop() {
    Serial.printlnf("%d", DMA_GetCurrDataCounter(dmaStream));

    /*
  *          9. To control DMA events you can use one of the following
  *              two methods:
  *               a- Check on DMA Stream flags using the function DMA_GetFlagStatus().
  *               b- Use DMA interrupts through the function DMA_ITConfig() at initialization
  *                  phase and DMA_GetITStatus() function into interrupt routines in
  *                  communication phase.
  *              After checking on a flag you should clear it using DMA_ClearFlag()
  *              function. And after checking on an interrupt event you should
  *              clear it using DMA_ClearITPendingBit() function.
     */
    if(DMA_GetFlagStatus(dmaStream, dmaFinishFlag)) {
        digitalWrite(D7, HIGH);
    }
}
