#include <stdint.h>
#include "main.h"
#include <string.h>
#include <stdlib.h>

// UART4_TX PC10
// UART4_RX PC11
// ADC PA1
// Ultrasonic Trig PA5
// Ultrasonic Echo PB4
// Led PA8
// Buzzer PA8
// Button PC8
// Servo Motor PB6
// 7 segment activation pins PE4-PE5-PE6
// 7 segment character pins PD1...PD7

void turn_on_led_buzzer(void){
	GPIOA->ODR |= 1<<8;
}

void turn_off_led_buzzer(void){
	GPIOA->ODR &= ~(1<<8);
}

void power_on_adc(void){
	ADC1->CR &= ~(1 << 29);	//deep-power down mode off
	ADC1->CR |= (1 << 28);	//voltage regulator on
	ADC1->CR |= 1;
}

void power_off_adc(void){
	ADC1->CR &= ~1;
	ADC1->CR &= ~(1<<28);
	ADC1->CR |= 1<<29;
}

uint32_t read_button(){
	return !(GPIOC->IDR & (1 << 8));
}

void init(){
	enable_bus_clocks();
    button_init();
    led_init();
	ultrasonic_trig_init();
	ultrasonic_echo_init();
	init_TIM5();
	init_TIM6();
	init_TIM7();
	init_TIM15();
	init_TIM16();
	adc_init();
	servo_init();
	uart4_init();
	init_7s();

	// Enable interrupts globally
	__asm volatile("cpsie i");
}

void enable_bus_clocks(){
	// Enable GPIOA, GPIOB, GPIOC, GPIOD clocks
	RCC_AHB2ENR |= 0b11111;

	// Enable TIM2, TIM3, TIM4, TIM5, TIM6, TIM7 clocks
	RCC_APB1ENR1 |= 0b111111;

	// Enable TIM15, TIM16 clocks
	RCC_APB2ENR |= (0b11 << 16);

	// Enable ADC clock
	RCC_AHB2ENR |= (1 << 13);

	// Enable UART4 clock
	RCC_APB1ENR1 |= (1 << 19);
}

void init_TIM16(void){
	TIM16->PSC = 3999;
	TIM16->ARR = 20;
	TIM16->DIER |= 1;              // enable update interrupt in TIM16
	ISER2 |= 1 << 6;               // enable TIM16 interrupt in ISER2
}

void button_init(void){
	GPIOC->MODER &= ~(3 << (8 * 2)); // Set PC8 as input mode

	EXTI->EXTICR3 |= (0x02 << 0);	// Map EXTI8 to PC8

	EXTI->FTSR1 |= (1 << 8);		// Falling edge trigger for line 8
	EXTI->IMR1 |= (1 << 8);			// Enable interrupt on line 8

	ISER0 |= (1 << 19);				// EXTI Line8 interrupt
}

void led_init(void){
    GPIOA->MODER &= ~(3 << (8*2));		//clear bits
    GPIOA->MODER |= 0b01 << (8*2); 		//set pa1 to analog mode
}

// Use TIM2_CHANNEL1 PA5 for output compare
void ultrasonic_trig_init() {
	GPIOA->MODER &= ~(3 << (5 * 2)); // Set mode as alternate function for PA5
	GPIOA->MODER |= (2 << (5 * 2));
    GPIOA->AFRL |= (1 << (5 * 4)); // Set AF1 for PA15
	TIM2->CR1 |= (1 << 2); // Set update event source as only counter overflow/underflow
	TIM2->PSC = 3999; // Set TIM2 frequency to 1 khz
	TIM2->ARR = 19; // Set period of output signal to 20 ms
	TIM2->CCMR1 |= (6 << 4); // output compare 1 mode = pwm mode 1
	TIM2->CCR1 = 9; // Set high time of pulse to 10ms Duty cycle = 50%
	TIM2->CCER |= (1 << 0); // Enable output compare 1
	TIM2->CNT = 0; // Initialize the counter
}

// Use TIM3_CHANNEL1 PB4 for output compare
void ultrasonic_echo_init() {

	// Set mode as alternate function for PB4
	GPIOB->MODER &= ~(3 << (4 * 2));
	GPIOB->MODER |= (2 << (4 * 2));

	// Set AF2 for PB4
	GPIOB->AFRL |= (2 << (4 * 4));

	// Enable input capture 1 interrupt
	TIM3->DIER |= (1 << 1);

	// Enable global signaling for TIM3 global interrupt		 46 = 32 * n(1) + m(14)
	ISER1 |= (1 << 14);

	// Set update event source as only counter overflow/underflow
	TIM3->CR1 |= (1 << 2);

	// Set TIM3 frequency to 1 Mhz
	TIM3->PSC = 3;

	// CC1 channel is configured as input, IC1 is mapped on TI1
	TIM3->CCMR1 |= (1 << 0);

	// Enable capture mode
	TIM3->CCER |= (1 << 0);
}

void init_TIM5(void) {
    TIM5->PSC = 3999;          // Set prescaler
    TIM5->ARR = ON_DURATION;   // Initial period
	TIM5->DIER |= 1;		  // enable update interrupt in TIM5
    ISER1 |= 1 << 16;
}

void init_TIM6(void) {
    TIM6->PSC = 3999;
    TIM6->ARR = 999;
    TIM6->DIER |= 1;                // enable update interrupt in TIM6
    ISER1 |= 1 << 17;               // enable TIM6 interrupt in ISER
}

void init_TIM7(void) {
    TIM7->PSC = 3999;
    TIM7->ARR = 4;
    TIM7->DIER |= 1;                // enable update interrupt in TIM7
    ISER1 |= 1 << 18;               // enable TIM7 interrupt in ISER
    TIM7->CR1 |= 1;             	// enable TIM7 by setting bit 0 in CR1
}

void init_TIM15(void) {
    TIM15->PSC = 3999;
    TIM15->ARR = 19;
    TIM15->DIER |= 1;                // enable update interrupt in TIM15
    ISER2 |= 1 << 5;               // enable TIM15 interrupt in ISER2
}

void adc_init(){
    GPIOA->MODER |= 0b11 << 2; //set pa1 to analog mode

    ADC1->CR &= ~(1 << 29);//deep-power down mode off
    ADC1->CR |= (1 << 28);//voltage regulator on
    RCC_CCIPR1 |= 3 << 28;//set system clock as as adc clock

    ADC->CCR |= 3 << 16;//choose HCLK/4
    ADC1->SMPR1 |= 0b111 << 18; //Sampling time = 640.5 for channel 6 for high precision
    ADC1->SQR1 |= 6 << 6;//set first convertion as 6

    ADC1->CR |= (1 << 31);//calibration
    while((ADC1->CR & (1 << 31)) != 0) {}//wait till it calibrates

    ADC1->CR |= 1;//adc enable ADEN
    while((ADC1->ISR & 1) == 0) {}//wait till ADRDY becomes 1

    //Enable EOCIE
    ADC1->IER |= 1 << 2;//as here ier should be set first but ps code
    //sets it later than cr which is wrong according to manual
    ISER1 |= 1 << 5;//enable  ADC1_2 interrupt
}

void servo_init(){
	// Set mode as alternate function for PB6
	GPIOB->MODER &= ~(3 << (6 * 2));
	GPIOB->MODER |= (2 << (6 * 2));

	// Set AF2 for PB6
	GPIOB->AFRL |= (2 << (6 * 4));

	// Enable TIM4 clock
	RCC_APB1ENR1 |= (1 << 2);

	// Set update event source as only counter overflow/underflow
	TIM4->CR1 |= (1 << 2);

	// Set TIM4 frequency to 1 Mhz
	TIM4->PSC = 3;

	// Set period of output signal to 20 ms
	TIM4->ARR = 19999;

	// output compare 1 mode = pwm mode 1
	TIM4->CCMR1 |= (6 << 4);

	// Set high time of pulse to 10ms Duty cycle = 50%
	TIM4->CCR1 = 700;

	// Enable output compare 1
	TIM4->CCER |= (1 << 0);

	// Initialize the counter
	TIM4->CNT = 0;

	TIM4->CR1 |= 1;
}

void uart4_init() {

    // Configure PC10-11 for UART4
    GPIOC->MODER &= ~(0b1111 << (10 * 2));
    GPIOC->MODER |= 0b1010 << (10 * 2);

	GPIOC->AFRH &= ~(0xFF << (4 * 2));
    GPIOC->AFRH |= (0x88 << (4 * 2));      // AF8 for both pins

	UART4->BRR = 417;  // For 4MHz clock and baud rate 9600

	UART4->CR1 |= (1 << 29);    // FIFO enable
	UART4->CR1 |= (0b11 << 2);  // TX/RX enable
	UART4->CR1 |= (1 << 5);     // RXNEIE enable

	ISER2 |= (1 << 0);

	UART4->CR1 |= 1;
}

void init_7s() {
	// 2^16 - 1 = mask of 16 ones for a-b-c-d-e-f-g-dp for 7 segments
	int mask = ((1 << 16) - 1);
	GPIOD->MODER &= ~(mask << 2);
	GPIOD->MODER |= (0b0101010101010101 << 2);
	GPIOE->MODER &= ~(0b111111<<(4*2));
	GPIOE->MODER |= 0b010101<<(4*2);
}

void write_to_7s(uint8_t number){
	static const uint8_t seven_segment[10] = {0x3FU,0x06U,0x5BU,0x4FU,0x66U,0x6DU,0x7DU,0x07U,0x7FU,0x6FU};
	GPIOD->ODR |= (0b11111111 << 1);
	GPIOD->ODR &= ~(seven_segment[number] << 1);
}

void activate_7s(uint8_t digit){
	switch(digit){
		case 1:
			GPIOE->ODR |= (1<<4);
			GPIOE->ODR &= ~(0b110<<4);
			break;
		case 2:
			GPIOE->ODR |= (1<<5);
			GPIOE->ODR &= ~(0b101<<4);
			break;

		case 3:
			GPIOE->ODR |= (1<<6);
			GPIOE->ODR &= ~(0b011<<4);
			break;
	}
}

void UART4_IRQHandler(void){
    if (UART4->ISR & (1 << 5)){  // Check RXNE flag
		char received_char = UART4->RDR;

		if (received_char == '\r' || received_char == '\n') {
			input_buffer[buffer_index] = '\0'; // Null-terminate the string
			received_value = atoi((const char*)input_buffer); // Convert to integer
			cycles_left_to_drop = received_value;
			state = 1; // Ready for next state
			buffer_index = 0; // Reset the buffer index
			UART4->TDR = 'O';
		} else {
			// Store the received digit in the buffer
			if (buffer_index < BUFFER_SIZE - 1) {
				input_buffer[buffer_index++] = received_char;
			} else {
				// Handle buffer overflow if necessary
				buffer_index = 0; // Reset the buffer
			}
		}
    }
}

void ADC1_2_IRQHandler(void){
	if((ADC1->ISR & (1 << 2)) != 0) {
        adc_value = ADC1->DR;//no need to delete ISR it deletes in hw when DR is read
        power_off_adc();
       /* if(state == 3 || state == 4)
        	ADC1->CR |= 1 << 2;  // Start next conversion*/
    }
}

void TIM3_IRQHandler(){
	if(TIM3->SR & (1 << 1)){
		if(!isFirstCaptured){
			time = TIM3->CCR1;

			// Set edge sensitivity to falling edge
			TIM3->CCER |= (1 << 1);
			isFirstCaptured = 1;
		}
		else{
			time = TIM3->CCR1 - time;
			distance = time * 10 / 58;
			// Set edge sensitivity to rising edge
			TIM3->CCER &= ~(1 << 1);
			isFirstCaptured = 0;

			if(!pillDropped) {
				if(distance < thresholdDistance){
					checkCounter++;
				}
				else{
					checkCounter = 0;
				}

				if(checkCounter >= 10){
					pillDropped = 1;
					TIM5->CR1 |= 1;
					checkCounter = 0;
				}
			}
			else {
				if(distance > thresholdDistance){
					checkCounter++;
				}
				else{
					checkCounter = 0;
				}

				if(checkCounter>=20){
					TIM2->CR1 &= ~(1 << 0); // disable OC
					TIM3->CR1 &= ~(1 << 0); // disable IC
					TIM5->CR1 &= ~(1<<0);
					TIM15->CR1 &= ~(1<<0);
					turn_off_led_buzzer();
					checkCounter = 0;
					pillDropped = 0;

					if(state == 4){
						UART4->TDR = 'C';
					}
					else{
						UART4->TDR = 'T';
					}

					if(state == 3) {
						state = 2;
					}
					else if(state == 4){
						state = 1;
						TIM4->CCR1 = 700;
					}
				}
			}

			TIM3->CNT = 0;
		}
		TIM3->SR &= ~(1 << 1);
	}
}

void TIM5_IRQHandler(void){ // interrupt handler for TIM5
	if (TIM5->SR & 1) {
        TIM5->SR &= ~1;  // Clear flag
        GPIOA->ODR ^= (1 << 8);  // Toggle LED

        if (GPIOA->ODR & (1 << 8)) {
            TIM5->ARR = ON_DURATION;
        } else {
        	//this mapping is inspired by textbook to map it between 0.5 sec to 5 sec
            off_duration = 500 + (adc_value * 4500) / 4095;
            TIM5->ARR = off_duration;
        }
    }
}

void TIM6_IRQHandler(void) // interrupt handler for TIM6
{
	cycles_left_to_drop--;
	if(cycles_left_to_drop==0){
	//display_minutes(cycles_left_to_drop);
	cycles_left_to_drop = received_value;
	UART4->TDR = 'D';
	switch(position){
			case 0:
				TIM4->CCR1 = 1250;
				position = 1;
				state = 3;
				break;
			case 1:
				TIM4->CCR1 = 1700;
				position = 2;
				state = 3;
				break;
			case 2:
				TIM4->CCR1 = 2150;
				position = 0;
				state = 4;
				TIM6->CR1 &=~1;
				break;
		}

		TIM2->CR1 |= (1 << 0);
		TIM3->CR1 |= (1 << 0);
    	//ADC1->CR |= 1 << 2;  // Start next conversion
		TIM15->CR1 |= (1<<0);
	}
    TIM6->SR = 0; // Clear interrupt flag

}

void TIM7_IRQHandler(void) {
	switch(digit) {
		case 1:
			digit = 2;
			activate_7s(1);
			write_to_7s(cycles_left_to_drop / 100);
			break;
		case 2:
			digit = 3;
			activate_7s(2);
			write_to_7s((cycles_left_to_drop % 100) / 10);
			break;
		case 3:
			digit = 1;
			activate_7s(3);
			write_to_7s(cycles_left_to_drop % 10);
			break;
	}
	TIM7->SR = 0; // Clear interrupt flag
}

void TIM15_IRQHandler(void){ // interrupt handler for TIM5
	if (TIM15->SR & 1) {
        TIM15->SR &= ~1;  // Clear flag
        power_on_adc();
        //adc_init();
    	ADC1->CR |= 1 << 2;  // Start next conversion
    }
}

void TIM16_IRQHandler(void) {
	TIM16->SR = 0;

    if(read_button()) {
    	if(state == 1){
			TIM4->CCR1 = 700;
			state++;
			TIM6->CR1 |= 1;
		}
    	else if((state==3) || (state==4) || (state==2)){
			state = 1;
			TIM6->CR1 &=~1;
			TIM4->CCR1 = 700;
			UART4->TDR = 'R';
			TIM2->CR1 &= ~(1 << 0); // disable OC
			TIM3->CR1 &= ~(1 << 0); // disable IC
			TIM5->CR1 &= ~(1<<0);
			TIM15->CR1 &= ~(1<<0);
			turn_off_led_buzzer();
			checkCounter = 0;
			pillDropped = 0;
			cycles_left_to_drop = received_value;
			position = 0;
		}
    }

    TIM16->CR1 &= ~(1 << 0);
}

void EXTI8_IRQHandler(void) {
    if (EXTI->FPR1 & (1 << 8)) {
        EXTI->FPR1 |=  (1 << 8);   // Clear pending flag

        TIM16->CNT = 0;
        TIM16->CR1 |= (1 << 0);
    }
}

int main(void){
	init();

	while(1){
		__asm volatile("wfi");
	}
	return 0;
}
