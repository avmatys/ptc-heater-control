/*
 * PTC.c
 *
 * Created: 8/7/2023 11:56:44 PM
 * Author : Avmatys
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define CLM_DATA PORTA // Climate input data
#define PTC_CTRL PORTA // Output to control heater
#define BTN_CTRL PORTB // Control button

#define PIN_CLM_DATA PINA // Climate input data
#define PIN_PTC_CTRL PINA // Output to control heater
#define PIN_BTN_CTRL PINB // Control button


// Timer period
const uint8_t MAX_TIMER_COUNTER = 70; // approx 1 min 8 sec for frequency 8MHz and prescaler 1024
const uint8_t PTC_ON_STEPS  [3] = {0,  10, 15};
const uint8_t PTC_OFF_STEPS [3] = {50, 45, 40};
const uint8_t MAX_SIZE = 3;


// Const values for ADC input channels
const uint8_t CH_POT_L = 0b00000000; // ADC0 0b00000000
const uint8_t CH_POT_R = (1<<MUX0); // ADC1 0b00000001
const uint8_t CH_BAT = (1<<MUX1) | (1<<MUX0); // ADC3 0b00000010

// Min values for ADC
const uint8_t MIN_BAT_VAL = 220; // Max value 255 = 5V, current 4.4V
								 // Battery voltage should be divided using voltage divider (10k + 4.7k for example) 
const uint8_t MIN_POT_VAL = 60; // Max value 255 = 5V, current 1.17V 
								// Voltage range 0-4.7V, where 0 = fully closed, 4.7 = fully opened


// Const values for pins
const uint8_t IN_POT_L = PORTA0; // Potentiometer left
const uint8_t IN_POT_R = PORTA1; // Potentiometer right
const uint8_t IN_ELMOT = PORTA2; // Electric motor
const uint8_t IN_BAT = PORTA3; // Battery voltage
const uint8_t OUT_RELAYS [3] = {PORTA4, PORTA5, PORTA6};
const uint8_t OUT_LED = PORTA7;


// Value to start/stop timer
volatile uint8_t PTC_MODE = 0;


// Timer counter
volatile uint16_t timerCounter = 0;


// Calculate average value from the measurements excluding min and max values
uint8_t calcAverageAnalogValue(uint8_t measurements[], uint8_t size)
{
	uint8_t min = measurements[0];
	uint8_t max = measurements[0];
	uint16_t total = measurements[0];
	for (uint8_t i=1; i<size; i++)
	{
		if (measurements[i] < min)
		{
			min = measurements[i];
		}
		if (measurements[i] > max)
		{
			max = measurements[i];
		}
		total += measurements[i];
	}
	uint8_t average = (total - min - max) / (size - 2);
	return average;
}


// Get analog value using ADC
uint8_t getAnalogValue(uint8_t channel)
{
	// Disable all MUX channels
	ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3) | (1<<MUX4) | (1<<MUX5));
	// Enable channel
	ADMUX |= channel; 
	uint8_t measurements[5];
	for (uint8_t i=0; i<5; i++)
	{
		// Start conversion
		ADCSRA |= (1<<ADSC);
		// Wait until ADC conversion done
		while ((ADCSRA & (1<<ADSC)))
		{
			;
		}
		// Store into array
		measurements[i] = ADCH;
	}
	uint8_t adc = calcAverageAnalogValue(measurements, 5);
	return adc;
}


// Function provides digital 1 if 80/100 measurements where 1
uint8_t getDigitalValue(uint8_t pin, uint8_t channel)
{
	uint8_t sum = 0;
	for (uint8_t i=0; i<100; i++)
	{
		sum += (pin & (1<<channel));
	}
	if (sum >= 80)
	{
		return 1;
	}
	return 0;
}


// Check mandatory conditions to start/continue heating
uint8_t checkInputChannels()
{
	// Get current adc of the battery
	uint8_t batteryVoltageValue = getAnalogValue(CH_BAT);
	if (batteryVoltageValue < MIN_BAT_VAL)
	{
		return 0;
	}
	// Get adc of the potentiometers of the climate servos
	uint8_t potLeftVoltageValue = getAnalogValue(CH_POT_L);
	uint8_t potRightVoltageValue = getAnalogValue(CH_POT_R);
	if (potRightVoltageValue < MIN_POT_VAL && potLeftVoltageValue < MIN_POT_VAL)
	{
		return 0;
	}
	// Check if electric motor is active 
	if (getDigitalValue(PIN_CLM_DATA, IN_ELMOT) == 0)
	{
		return 0;
	}
	return 1;
}


// Prepare mask to enable relay on time schedule
// Returned mask should be used with | operator
uint8_t enableRelayOnTime(uint8_t pin, uint16_t timer, const uint8_t relays[], const uint8_t enableSteps[], uint8_t size)
{
	for (uint8_t step=0; step < size; step++)
	{
		if (timer == enableSteps[step] && !(pin & (1<<relays[step])))
		{
			return (1<<relays[step]);
		}
	}
	return pin;
}


// Prepare mask to disable relay on time schedule 
// Returned mask should be used with & operator. ~ is not needed
uint8_t disableRelayOnTime(uint8_t pin, uint16_t timer, const uint8_t relays[], const uint8_t disableSteps[], uint8_t size)
{
	for (uint8_t step=0; step < size; step++)
	{
		if (timer == disableSteps[step] && (pin & (1<<relays[step])))
		{
			return ~(1<<relays[step]);
		}
	}
	return pin;
}


// Disable all configured relays using button or when max time limit was exceed
// Returned mask should be used with & operator. ~ is not needed
uint8_t disableRelays(uint8_t pin, const uint8_t relays[], uint8_t size)
{
	uint8_t mask = 0b0;
	for(uint8_t i=0; i<size; i++)
	{
		if (pin && (1<<relays[i]))
		{
			mask |= (1<<relays[i]);
		}
	}
	return ~mask;
}


// Get current relays state
// If any relay is active - return 1
// If all relays are disabled - return 0
uint8_t getRelaysState(uint8_t pin, const uint8_t relays[], uint8_t size)
{
	uint8_t relaysState = 0;
	for (uint8_t i=0; i<size; i++)
	{
		if (pin & (1<<relays[i]))
		{
			relaysState =1;
			break;
		}
	}
	return relaysState;
}


// Check current led state and if it's active return bit mask to disable it
// Returned mast should be used with & operator. ~ is not needed
uint8_t disableLed(uint8_t pin, uint8_t led)
{
	if (pin & (1<<led))
	{
		return ~(1<<led);
	}
	return pin;
}


// Function to initialize i/o ports
void init()
{
	// Setup register for port A
	DDRA |= (1<<DDA7) | (1<<DDA6) | (1<<DDA5) | (1<<DDA4); // 1 = Out 0 = In
	// Setup register for port B
	DDRB |= (1<<DDB3) | (1<<DDB1) | (1<<DDB0); // 1 = Out 0 = In
	// Enable pull up resistor for unused ports
	PORTB |= (1<<PORTB3) | (1<<PORTB1) | (1<<PORTB0);

	// Setup interrupts for button
	MCUCR |= (1<<ISC01) | (1<<SE); // Falling edge + Enable Sleep + IDLE
	GIMSK |= (1<<INT0); // Enable global interrupts

	// Setup interrupts for timer
	TCCR0B |= (1<<CS00) | (1<<CS02); // Prescaler 1024
	TIMSK0 |= (1<<TOIE0); //Enable overflow interrupt for the timer

	// Enable interrupts after needed configuration
	sei();

	// Basic setup of ADC register
	ADCSRA |= (1<<ADEN) | (1<<ADPS1) | (1<<ADPS2) ; // Enable ADC (ADEN) + Prescaler 64 (125kHz)
	ADMUX |= (1<<REFS1); // Vref  = 1.1V
	ADCSRB |= (1<<ADLAR); // Enable left adjust result in order to use only 8 bits

}


// External interrupt INT0 handling - button
ISR(EXT_INT0_vect)
{
	// Change state of the PTC
	if (PTC_MODE == 0)
	{	
		// In case if ADC inputs are fine -> switch on
		PTC_MODE = checkInputChannels();
		// Reset timer
		timerCounter = 0;
	}
	else
	{
		PTC_MODE = 0; // Off
	}
}


// Timer external interrupt handling
ISR(TIM0_OVF_vect)
{
	if (PTC_MODE == 1){
		if (timerCounter <= MAX_TIMER_COUNTER) 
		{
			// Update counter
			timerCounter += 1;
			// Check input channels
			PTC_MODE = checkInputChannels();
		}
		else 
		{
			// Switch off PTC
			PTC_MODE = 0;	
		}
	}
	else 
	{
		timerCounter = 0;
	}
}


int main(void)
{
   
	// Initialize IOs, ADC, interrupts
	init();
	
	// Main program activity - enable relays in the sequence if input conditions are met
    while (1) 
    {
		// Switch on PTC ports
		if (PTC_MODE == 1)
		{
			// Switch on led
			PTC_CTRL |= (1<<OUT_LED);
			// Switch on relays in the sequence
			if (timerCounter <= PTC_ON_STEPS[MAX_SIZE-1]){ 
				PTC_CTRL |= enableRelayOnTime(PIN_PTC_CTRL, timerCounter, OUT_RELAYS, PTC_ON_STEPS, MAX_SIZE);
			}
			// Disable pins in a reverse sequence
			else if (timerCounter >= PTC_OFF_STEPS[MAX_SIZE-1]){
				PTC_CTRL &= disableRelayOnTime(PIN_PTC_CTRL, timerCounter, OUT_RELAYS, PTC_OFF_STEPS, MAX_SIZE);
			}	
			// Get current relays state
			uint8_t relaysState = getRelaysState(PIN_PTC_CTRL, OUT_RELAYS, MAX_SIZE);
			// Check input channels
			if (relaysState == 0)
			{
				PTC_MODE = 0;
			}
		}
		// Switch off everything
		if (PTC_MODE == 0) {
			// Get current relays state
			uint8_t relaysState = getRelaysState(PIN_PTC_CTRL, OUT_RELAYS, MAX_SIZE);
			// Disable relays
			if (relaysState == 1)
			{
				PTC_CTRL &= disableRelays(PIN_PTC_CTRL, OUT_RELAYS, MAX_SIZE);
			}
			// Disable led
			PTC_CTRL &= disableLed(PIN_PTC_CTRL, OUT_LED);
			// Sleep until next interruption
			sleep_enable();
		}
		
    }
}

