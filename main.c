// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
//                                 INCLUDES
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//                                  PRAGMA
// ----------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


// ----------------------------------------------------------------------------
//                                  DEFINE
// ----------------------------------------------------------------------------

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

#define DAC_MAX_VAL (4095)
#define DAC_MAX_VOLTS (2.92)
#define OPTO_DROP (1.0)

// ----------------------------------------------------------------------------
//                                PROTOTYPES
// ----------------------------------------------------------------------------

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
uint32_t readPot(void);
uint32_t toOhms(uint32_t adc_val);
void writeDAC(uint32_t dac_val);

// ----------------------------------------------------------------------------
//                                 GLOBALS
// ----------------------------------------------------------------------------
volatile uint32_t edge_num = 0;

// ----------------------------------------------------------------------------
//                                   MAIN
// ----------------------------------------------------------------------------

int main(int argc, char* argv[])
{
	uint32_t potVal = 0;
	uint32_t potOhm = 0;

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init(); // init port A
	myTIM2_Init();	// init timer and interrupts
	myEXTI_Init();	// init exti interrupts for PA1
	myADC_Init();	// init ADC with PA0 as analog input
	myDAC_Init();	// init DAC (automagically configured to PA4)

	while (1)
	{
		potVal = readPot();
		potOhm = toOhms(potVal);
		trace_printf("Potentiometer ADC value: %d\n", potVal);
		trace_printf("Potentiometer Resistance value: %d Ohms\n", potOhm);

		writeDAC(potVal);
	}

	return 0;

}

// ----------------------------------------------------------------------------
//                                INIT FUNCTIONS
// ----------------------------------------------------------------------------

void myGPIOA_Init()
{
	// Enable clock for GPIOA peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA1 as input for 555 timer
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	//no pull-up/pull-down for PA1
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	// Configure PA0 as analog input
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	//no pull-up or down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	// Configure PA4 as analog output
	GPIOA->MODER &= ~(GPIO_MODER_MODER4);
	GPIOA->MODER &= ~(GPIO_PUPDR_PUPDR4);
}


void myTIM2_Init()
{
	// Note that the max possible value for period counting is 1099511627775 ticks
	// Clock is 48 000 000 Hz
	// So the longest detectable period is max_period_ticks / clock = 22906.5 seconds
	// So the smallest possible detectable frequency is 0.00004366
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;
}


void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	SYSCFG->EXTICR[0] = ((uint32_t)0x0000);

	/* EXTI1 line interrupts: set rising-edge trigger */
	EXTI->RTSR = ((uint32_t)0x0002);

	/* Unmask interrupts from EXTI1 line */
	EXTI->IMR = ((uint32_t)0x0002);

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void myADC_Init(void)
{
	// enable clock input to ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// do some safety stuff found in the appendix of the reference manual
	if ((ADC1->CR & ADC_CR_ADEN) != 0)
		ADC1->CR |= ADC_CR_ADDIS;
	while ((ADC1->CR & ADC_CR_ADEN) != 0);
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

	// start calibration
	trace_printf("Start ADC Calibration\n");
	ADC1->CR = ADC_CR_ADCAL;
	//wait until calibration finished
	while (ADC1->CR == ADC_CR_ADCAL);
	trace_printf("Finished ADC calibration\n");

	// continuous conversion and overun
	ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD);

	//select channel 0 for PA0
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;

	// enable and wait for ready
	trace_printf("Star ADC Enable and wait for ack\n");
	ADC1->CR |= ADC_CR_ADEN;
	while (!(ADC1->ISR & ADC_ISR_ADRDY));
	trace_printf("ADC Enabled\n");
}

void myDAC_Init(void)
{
	// enable DAC clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	// enable dac
	DAC->CR |= DAC_CR_EN1;
}

// ----------------------------------------------------------------------------
//                               UTILITY FUNCTIONS
// ----------------------------------------------------------------------------

uint32_t readPot(void)
{
	// start conversion
	ADC1->CR |= ADC_CR_ADSTART;
	// wait for end of conversion flag
	while (!(ADC1->ISR & ADC_ISR_EOC));
	//reset end of conversion flag
	ADC1->ISR &= ~(ADC_ISR_EOC);
	return (uint32_t)((ADC1->DR) & ADC_DR_DATA);
}

uint32_t toOhms(uint32_t adc_val)
{
	return (uint32_t)((((float)adc_val)/4096.0) * 5000.0);
}

void writeDAC(uint32_t dac_val)
{
	float output_ratio = ((float)dac_val) / ((float)DAC_MAX_VAL);
	float volt_range = (float)DAC_MAX_VOLTS - OPTO_DROP;
	float dac_volts_out = ( (output_ratio * volt_range ) + OPTO_DROP );
	uint32_t dac_val_out = (uint32_t)((dac_volts_out / DAC_MAX_VOLTS)*((float)DAC_MAX_VAL));

	trace_printf("Writing to DAC: %d\n", dac_val_out);
	trace_printf("Equivalent of volts: %f\n\n\n", dac_volts_out);
	DAC->DHR12R1 = dac_val_out;
}

// ----------------------------------------------------------------------------
//                                  INTERRUPTS
// ----------------------------------------------------------------------------

void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}


void EXTI0_1_IRQHandler()
{
	// Your local variables...
	double period = 0.0;
	double freq = 0.0;
	uint32_t count = 0;

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		if (edge_num == 0)
		{
			TIM2->CNT = 0x0;
			TIM2->CR1 |= TIM_CR1_CEN; /* Restart stopped timer */
			edge_num++;
		}
		else if (edge_num >= 1)
		{
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			//Stop timer
			//Get the period in ticks
			count = TIM2->CNT;
			// Calculate period
			period = ((double)count) / ((double)SystemCoreClock);
			freq = ((double)1.0) / period;

			trace_printf("====== Signal Parameters: =====\n");
			trace_printf("\tPeriod:\t\t%f s\n", period);
			trace_printf(" seconds\n\tFrequency:\t\t%f Hz\n", freq);
			trace_printf("\n==========================\n\n");

			//Reset edges to 0
			edge_num = 0;
		}
		//	  NOTE: Function trace_printf does not work
		//	  with floating-point numbers: you must use
		//	  "unsigned int" type to print your signal
		//	  period and frequency.
		//
		EXTI->PR |= ((uint32_t)0x0002);
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

