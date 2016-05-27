#include "application.h"

#define d2r (3.14159265/180)

// Use update signal
// PAP: This is better than the CC1 signal; using the CC1 signal causes the timing to shift
#define PWM_TIMER	TIM5
#define DMA_STREAM	DMA1_Stream6
#define DMA_TCIF	DMA_FLAG_TCIF6
#define DMA_CHANNEL	DMA_Channel_6
#define DMA_SOURCE	TIM_DMA_Update


#define TIM_PERIOD			74
#define TIM_COMPARE_HIGH	60
#define TIM_COMPARE_LOW		15


TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
DMA_InitTypeDef DMA_InitStructure;

/* Buffer that holds one complete DMA transmission
 * 
 * Ensure that this buffer is big enough to hold
 * all data bytes that need to be sent
 * 
 * The buffer size can be calculated as follows:
 * number of LEDs * 24 bytes + 42 bytes
 * 
 * This leaves us with a maximum string length of
 * (2^16 bytes per DMA stream - 42 bytes)/24 bytes per LED = 2728 LEDs
 */
uint32_t LED_BYTE_Buffer[100];	

void Timer3_init(void)
{
	uint16_t PrescalerValue;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* GPIOB Configuration: PWM_TIMER Channel 1 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	/* Compute the prescaler value */
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	PrescalerValue = 0;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD; // 800kHz 
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(PWM_TIMER, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(PWM_TIMER, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);

	/* configure DMA */
	/* DMA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	/* DMA1 Channel6 Config */
	DMA_DeInit(DMA_STREAM);

	DMA_InitStructure.DMA_BufferSize = 42;
	DMA_InitStructure.DMA_Channel = DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					// data shifted from memory to peripheral
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&LED_BYTE_Buffer;		// this is the buffer memory 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// automatically increase buffer index
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// stop DMA feed after buffer size is reached

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&PWM_TIMER->CCR1;	// physical address of Timer 3 CCR1
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	
	DMA_Init(DMA_STREAM, &DMA_InitStructure);
	
	/* PWM_TIMER CC1 DMA Request enable */
	TIM_DMACmd(PWM_TIMER, DMA_SOURCE, ENABLE);
}

/* This function sends data bytes out to a string of WS2812s
 * The first argument is a pointer to the first RGB triplet to be sent
 * The seconds argument is the number of LEDs in the chain
 * 
 * This will result in the RGB triplet passed by argument 1 being sent to 
 * the LED that is the furthest away from the controller (the point where
 * data is injected into the chain)
 */
void WS2812_send(const uint8_t *color, const uint16_t _len)
{
	int i, j;
	uint8_t led;
	uint16_t memaddr;
	uint16_t buffersize;
	uint16_t len = _len;

	// Byte order mapping. 0 is red, 1 is green, 2 is blue
	const uint8_t pix_map[3] = {0, 2, 1};

	buffersize = (len*24)+42;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;				// reset buffer memory index
	led = 0;					// reset led index

	// fill transmit buffer with correct compare values to achieve
	// correct pulse widths according to color values
	while (len)
	{
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 8; j++)					// GREEN data
			{
				if ( (color[3*led + pix_map[i]]<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
				{
					LED_BYTE_Buffer[memaddr] = TIM_COMPARE_HIGH;	// compare value for logical 1
				}
				else
				{
					LED_BYTE_Buffer[memaddr] = TIM_COMPARE_LOW;		// compare value for logical 0
				}
				memaddr++;
			}
		}
		
		led++;
		len--;
	}
	
	// add needed delay at end of byte cycle, pulsewidth = 0
	while(memaddr < buffersize)
	{
		LED_BYTE_Buffer[memaddr] = 0;
		memaddr++;
	}

	DMA_SetCurrDataCounter(DMA_STREAM, buffersize); 	// load number of bytes to be transferred

	// PAP: Clear the timer's counter and set the compare value to 0. This
	// sets the output low on start and gives us a full cycle to set up DMA.
	TIM_SetCounter(PWM_TIMER, 0);
	TIM_SetCompare1(PWM_TIMER, 0);
	TIM_Cmd(PWM_TIMER, ENABLE); 						// enable Timer 3
	
	// PAP: Start DMA transfer after starting the timer. This prevents the
	// DMA/PWM from dropping the first bit.
	DMA_Cmd(DMA_STREAM, ENABLE); 			// enable DMA channel 6
	while(!DMA_GetFlagStatus(DMA_STREAM, DMA_TCIF)); 	// wait until transfer complete
	TIM_Cmd(PWM_TIMER, DISABLE); 					// disable Timer 3
	DMA_Cmd(DMA_STREAM, DISABLE); 			// disable DMA channel 6
	DMA_ClearFlag(DMA_STREAM, DMA_TCIF); 				// clear DMA1 Channel 6 transfer complete flag
}


void blend(const uint8_t *colourA, const uint8_t *colourB, uint8_t *colourOut, float amount)
{
	float r, g, b;

	r = ((float)colourB[0] * amount) + ((float)colourA[0] * (1.0 - amount));
	g = ((float)colourB[1] * amount) + ((float)colourA[1] * (1.0 - amount));
	b = ((float)colourB[2] * amount) + ((float)colourA[2] * (1.0 - amount));

	colourOut[0] = (r > 255.0) ? 255.0 : (r < 0.0) ? 0.0 : r;
	colourOut[1] = (g > 255.0) ? 255.0 : (g < 0.0) ? 0.0 : g;
	colourOut[2] = (b > 255.0) ? 255.0 : (b < 0.0) ? 0.0 : b;
}

const uint8_t off[3]	= {0,0,0};
const uint8_t red[3]	= {255,0,0};
const uint8_t green[3]	= {0,255,0};
const uint8_t blue[3]	= {0,0,255};
const uint8_t white[3]	= {255,255,255};


void setup() {
    Timer3_init();
}

void loop() {
	int i, j;
    const uint8_t *order[] = {
        red,
        green,
        blue
    };

    // Blend colours
    const int MAXJ = (sizeof(order)/sizeof(order[0]));
    for (j = 0; j < MAXJ; j++) {
        for (i=0; i<100; i++) {
            uint8_t colourbuf[3];
            int k = (j == MAXJ-1) ? 0 : j+1;
            blend(order[j], order[k], colourbuf, ((float)i) / 100.0);
            WS2812_send(colourbuf, 1);
            delay(100);
        }
    }
}
