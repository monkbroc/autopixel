#include "application.h"

// See
// hal/src/stm32f2xx/adc_hal.c
// hal/src/stm32f2xx/spi_hal.c
// hal/src/stm32/pwm_hal.c
// hal/src/stm32f2xx/pinmap_hal.c

#define PIXEL_FREQUENCY 800000
#define PIXEL_0_HIGH_TIME_NS 350
#define PIXEL_1_HIGH_TIME_NS 800

// Hard code TIM4 for now
auto timer_peripheral = TIM4;
auto pixel_0_channel = TIM_Channel_2;
auto pixel_1_channel = TIM_Channel_1;
// DMA1, channel 2 for TIM4
auto dma_peripheral_enable = RCC_AHB1Periph_DMA1;
auto dma_channel = DMA_Channel_2;
auto rising_dma_stream = DMA1_Stream6;
auto pixel_0_dma_stream = DMA1_Stream3;
auto pixel_1_dma_stream = DMA1_Stream0;
auto dma_finish_flag = DMA_FLAG_TCIF6;

// Use A7 as output (GPIOA, GPIO_Pin_0)
pin_t output_pin = A7;
uint16_t pin_bit = PIN_MAP[output_pin].gpio_pin;
volatile uint16_t *pin_set_register = &PIN_MAP[output_pin].gpio_peripheral->BSRRL;
volatile uint16_t *pin_reset_register = &PIN_MAP[output_pin].gpio_peripheral->BSRRH;

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
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // Change to Disable when debug is done
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
  configureChannel(pixel_0_channel, calculatePulseWidth(PIXEL_0_HIGH_TIME_NS));
  configureChannel(pixel_1_channel, calculatePulseWidth(PIXEL_1_HIGH_TIME_NS));
}

void enableTimer() {
    // TIM enable counter
    TIM_Cmd(timer_peripheral, ENABLE);
}

void initializeDMAChannel(uint16_t *source, volatile uint16_t *dest, uint16_t length, bool increment, uint32_t channel, DMA_Stream_TypeDef *stream) {
    DMA_InitTypeDef DMA_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;

    DMA_DeInit(stream);

    DMA_StructInit(&DMA_InitStructure);

    DMA_InitStructure.DMA_Channel = channel;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)source;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)dest;
    DMA_InitStructure.DMA_BufferSize = length;
    DMA_InitStructure.DMA_MemoryInc = increment ? DMA_MemoryInc_Enable : DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    DMA_Init(stream, &DMA_InitStructure);

  /*
  *          4. Enable the NVIC and the corresponding interrupt(s) using the function
  *             DMA_ITConfig() if you need to use DMA interrupts.
  */
    // /* Enable SPI TX DMA Stream Interrupt */
    // DMA_ITConfig(spiMap[spi].SPI_TX_DMA_Stream, DMA_IT_TC, ENABLE);
    // /* Enable SPI RX DMA Stream Interrupt */
    // DMA_ITConfig(spiMap[spi].SPI_RX_DMA_Stream, DMA_IT_TC, ENABLE);

    // NVIC_InitStructure.NVIC_IRQChannel = spiMap[spi].SPI_TX_DMA_Stream_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = spiState[spi].mode == SPI_MODE_MASTER ? 12 : 1;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);

    // NVIC_InitStructure.NVIC_IRQChannel = spiMap[spi].SPI_RX_DMA_Stream_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = spiState[spi].mode == SPI_MODE_MASTER ? 12 : 1;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);

    DMA_Cmd(stream, ENABLE);
}


void enableDMA(uint16_t *buffer, uint16_t length) {
    RCC_AHB1PeriphClockCmd(dma_peripheral_enable, ENABLE);
    RCC_AHB1PeriphResetCmd(dma_peripheral_enable, ENABLE);
    initializeDMAChannel(&pin_bit, pin_set_register, length, false, dma_channel, rising_dma_stream);
    initializeDMAChannel(buffer, pin_reset_register, length, true, dma_channel, pixel_0_dma_stream);
    initializeDMAChannel(&pin_bit, pin_reset_register, length, false, dma_channel, pixel_1_dma_stream);
}

void enableTimerDMA() {
    TIM_DMACmd(timer_peripheral, TIM_DMA_Update, ENABLE);
    TIM_DMACmd(timer_peripheral, TIM_DMA_CC1, ENABLE);
    TIM_DMACmd(timer_peripheral, TIM_DMA_CC2, ENABLE);
}

 /*
  *          8. Optionally, you can configure the number of data to be transferred
  *             when the Stream is disabled (ie. after each Transfer Complete event
  *             or when a Transfer Error occurs) using the function DMA_SetCurrDataCounter().
  *             And you can get the number of remaining data to be transferred using
  *             the function DMA_GetCurrDataCounter() at run time (when the DMA Stream is
  *             enabled and running).
  *
    */
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

uint16_t resetBits[bitCount];

// Fill an array with the bits that must be reset
void fillBits() {
    uint16_t bit = 0;
    for(uint16_t i = 0; i < pixelCount; i++) {
        for(uint32_t mask = 0x800000; mask != 0; mask >>= 1) {
            resetBits[bit] = (pixels[i] & mask) ? 0 : pin_bit;
            bit++;
        }
    }
}

FunctionalState status;
void setup() {
    pinMode(D2, OUTPUT);
    digitalWrite(D2, LOW);

    fillBits();
    pinMode(output_pin, OUTPUT);

    enableTimClock();
    configureDebugPins();
    configureTimeBase();
    configureChannels();

    // clear flags
    //DMA1->LISR = 0;
    //DMA1->HISR = 0;
    DMA_ClearFlag(rising_dma_stream, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);
    DMA_ClearFlag(pixel_0_dma_stream, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);
    DMA_ClearFlag(pixel_1_dma_stream, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);
    enableDMA(resetBits, bitCount);
    enableTimerDMA();
    enableTimer();
    status = DMA_GetCmdStatus(rising_dma_stream);

    Serial.begin(9600);
}

void loop() {
    Serial.printlnf("%d %d %d", DMA_GetCurrDataCounter(rising_dma_stream), DMA_GetCurrDataCounter(pixel_0_dma_stream), DMA_GetCurrDataCounter(pixel_1_dma_stream));

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
    if(DMA_GetFlagStatus(rising_dma_stream, dma_finish_flag)) {
        digitalWrite(D3, HIGH);
    }
}
