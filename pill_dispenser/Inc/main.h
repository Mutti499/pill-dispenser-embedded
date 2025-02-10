/*
 * main.h
 *
 *  Created on: Dec 18, 2024
 *      Author: onurc
 */

#ifndef MAIN_H_
#define MAIN_H_

typedef struct{
	volatile uint32_t CR1;       // Control register 1
	volatile uint32_t CR2;       // Control register 2
	volatile uint32_t SMCR;      // Slave mode control register
	volatile uint32_t DIER;      // DMA/Interrupt enable register
	volatile uint32_t SR;        // Status register
	volatile uint32_t EGR;       // Event generation register
	volatile uint32_t CCMR1;     // Capture/Compare mode register 1
	volatile uint32_t CCMR2;     // Capture/Compare mode register 2
	volatile uint32_t CCER;      // Capture/Compare enable register
	volatile uint32_t CNT;       // Counter register
	volatile uint32_t PSC;       // Prescaler register
	volatile uint32_t ARR;       // Auto-reload register
	volatile uint32_t reserved;  // Reserved space
	volatile uint32_t CCR1;      // Capture/Compare register 1
	volatile uint32_t CCR2;      // Capture/Compare register 2
	volatile uint32_t CCR3;      // Capture/Compare register 3
	volatile uint32_t CCR4;      // Capture/Compare register 4
	volatile uint32_t DCR;       // DMA control register
	volatile uint32_t DMAR;      // DMA address for full transfer
	volatile uint32_t OR1;       // Option register 1
	volatile uint32_t reserved1[3]; // Reserved space
	volatile uint32_t OR2;       // Option register 2
} TIM_TypeDef;

typedef struct {
	volatile uint32_t CR1; //0
	volatile uint32_t CR2; //4
	volatile uint32_t SMCR; //8
	volatile uint32_t DIER; //C
	volatile uint32_t SR; //10
	volatile uint32_t EGR; //14
	volatile uint32_t CCMR1; //18
	uint32_t reserved1; //1C
	volatile uint32_t CCER; //20
	volatile uint32_t CNT; //24
	volatile uint32_t PSC; //28
	volatile uint32_t ARR; //2C
	volatile uint32_t RCR; //30
	volatile uint32_t CCR1; //34
	volatile uint32_t CCR2; //38
	uint32_t reserved2[2]; //3C 40
	volatile uint32_t BDTR; //44
	volatile uint32_t DCR; //48
	volatile uint32_t DMAR; //4C
	volatile uint32_t OR1; //50
	uint32_t reserved3[3]; //54 58 5C
	volatile uint32_t OR2; //60
} TIM_General_Purpose_Type;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    uint32_t reserved;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    uint32_t reserved1[3];
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
} TIMxBasicType;

typedef struct{
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
    volatile uint32_t BRR;
    uint32_t reserved;
    volatile uint32_t SECCFGR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR1;  //0
    volatile uint32_t CR2;  //4
    volatile uint32_t CR3;  //8
    volatile uint32_t BRR;  //C
    volatile uint32_t GTPR; //10
    volatile uint32_t RTOR; //14
    volatile uint32_t RQR;  //18
    volatile uint32_t ISR;  //1C
    volatile uint32_t ICR;  //20
    volatile uint32_t RDR;  //24
    volatile uint32_t TDR;  //28
} USARTType;

typedef struct{
	volatile uint32_t ISR; //0
	volatile uint32_t IER; //4
	volatile uint32_t CR; //8
	volatile uint32_t CFGR; //C
	volatile uint32_t CFG2; //10
	volatile uint32_t SMPR1; //14
	volatile uint32_t SMPR2; //18
	uint32_t reserved2; //1C
	volatile uint32_t TR1; //20
	volatile uint32_t TR2; //24
	volatile uint32_t TR3; //28
	uint32_t reserved3; //2C
	volatile uint32_t SQR1; //30
	volatile uint32_t SQR2; //34
	volatile uint32_t SQR3; //38
	volatile uint32_t SQR4; //3C
	volatile uint32_t DR; //40
	uint32_t reserved4[2]; //44 48
	volatile uint32_t JSQR; //4C
	uint32_t reserved5[4]; //50 54 58 5C
	volatile uint32_t OFR1; //60
	volatile uint32_t OFR2; //64
	volatile uint32_t OFR3; //68
	volatile uint32_t OFR4; //6C
	uint32_t reserved6[4]; //70 74 78 7C
	volatile uint32_t JDR1; //80
	volatile uint32_t JDR2; //84
	volatile uint32_t JDR3; //88
	volatile uint32_t JDR4; //8C
	uint32_t reserved7[4]; //90 94 98 9C
	volatile uint32_t AWD2CR; //A0
	volatile uint32_t AWD3CR; //A4
	uint32_t reserved8[2]; //A8 AC
	volatile uint32_t DIFSEL; //B0
	volatile uint32_t CALFACT; //B0
} ADCType;

typedef struct {
    volatile uint32_t RTSR1;   // Rising Trigger Selection Register 1 (Offset: 0x00)
    volatile uint32_t FTSR1;   // Falling Trigger Selection Register 1 (Offset: 0x04)
    volatile uint32_t SWIER1;  // Software Interrupt Event Register 1 (Offset: 0x08)
    volatile uint32_t RPR1;    // Rising Pending Register 1 (Offset: 0x0C)
    volatile uint32_t FPR1;    // Falling Pending Register 1 (Offset: 0x10)
    volatile uint32_t SECCFGR1;    // EXTI security configuration register (Offset: 0x14)
    volatile uint32_t PRIVCFGR1;    // EXTI privilege configuration register (Offset: 0x18)
    uint32_t reserved1;     	// Reserved (Offset: 0x1C)
    volatile uint32_t RTSR2;   // Rising Trigger Selection Register 2 (Offset: 0x20)
    volatile uint32_t FTSR2;   // Falling Trigger Selection Register 2 (Offset: 0x24)
    volatile uint32_t SWIER2;  // Software Interrupt Event Register 2 (Offset: 0x28)
    volatile uint32_t RPR2;    // Rising Pending Register 2 (Offset: 0x2C)
    volatile uint32_t FPR2;    // Falling Pending Register 2 (Offset: 0x30)
    volatile uint32_t SECCFGR2;    // EXTI security enable register (Offset: 0x34)
    volatile uint32_t PRIVCFGR2;    // EXTI privilege enable register (Offset: 0x38)
    uint32_t reserved2[9];		// Reserved (Offset: 0x3C - 0x5C)
    volatile uint32_t EXTICR1; // External Interrupt Configuration Register 1 (0x60)
    volatile uint32_t EXTICR2; // External Interrupt Configuration Register 2 (0x64)
    volatile uint32_t EXTICR3; // External Interrupt Configuration Register 3 (0x68)
    volatile uint32_t EXTICR4; // External Interrupt Configuration Register 4 (0x6C)
    volatile uint32_t LOCKR;     // EXTI lock register (Offset: 0x70)
    uint32_t reserved3[3];		// Reserved (Offset: 0x74 - 0x7C)
    volatile uint32_t IMR1;     // Interrupt Mask Register 1 (Offset: 0x80)
    volatile uint32_t EMR1;     // Event Mask Register 1 (Offset: 0x84)
    uint32_t reserved4[2];      // Reserved (Offset: 0x88 - 0x8C)
    volatile uint32_t IMR2;     // Interrupt Mask Register 2 (Offset: 0x90)
    volatile uint32_t EMR2;     // Event Mask Register 2 (Offset: 0x94)
} EXTI_TypeDef;

typedef struct{
	volatile uint32_t CSR; //0
	uint32_t reserved1; //4
	volatile uint32_t CCR; //8
	volatile uint32_t CDR; //C
} ADCCommon;

#define ISER0 *((volatile uint32_t *) 0xE000E100) //read from cortex-m33 dgug (page300)
#define ISER1 *((volatile uint32_t *) 0xE000E104) //read from cortex-m33 dgug (page300)
#define ISER2 *((volatile uint32_t *) 0xE000E108) //read from cortex-m33 dgug (page300)

#define RCC_BASE		0x40021000 // RCC base address
#define RCC_AHB2ENR		*((volatile uint32_t *) (RCC_BASE + 0x04C)) // RCC AHB2 peripheral clock enable register
#define RCC_APB1ENR1	*((volatile uint32_t *) (RCC_BASE + 0x058)) // RCC APB1 peripheral clock enable register 1
#define RCC_APB2ENR 	*((volatile uint32_t *) (RCC_BASE + 0x060))
#define RCC_CCIPR1 		*((volatile uint32_t *) (RCC_BASE + 0x088))

#define GPIOA ((GPIO_TypeDef *) 0x42020000)
#define GPIOB ((GPIO_TypeDef *) 0x42020400)
#define GPIOC ((GPIO_TypeDef *) 0x42020800)
#define GPIOD ((GPIO_TypeDef *) 0x42020C00)
#define GPIOE ((GPIO_TypeDef *) 0x42021000)

#define TIM2 ((TIM_TypeDef *) 0x40000000)
#define TIM3 ((TIM_TypeDef *) 0x40000400)
#define TIM4 ((TIM_TypeDef *) 0x40000800)

#define TIM5 ((TIMxBasicType *)0x40000C00)
#define TIM6 ((TIMxBasicType *) 0x40001000)
#define TIM7 ((TIMxBasicType *) 0x40001400)

#define TIM15 ((TIM_General_Purpose_Type *) 0x40014000)
#define TIM16 ((TIM_General_Purpose_Type *) 0x40014400)

#define UART4 ((USARTType *) 0x40004C00)

#define EXTI ((EXTI_TypeDef *) 0x4002F400)

#define ADC1 ((ADCType *) 0x42028000)
#define ADC ((ADCCommon *) 0x42028300)

#define ON_DURATION 999 // Fixed ON duration
#define BUFFER_SIZE 4

void init(void);
void enable_bus_clocks(void);
void led_init(void);
void init_TIM7(void);
void servo_init(void);
void ultrasonic_trig_init(void);
void ultrasonic_echo_init(void);
void adc_init(void);
void uart4_init(void);
void button_init(void);
void init_TIM6(void);
void activate_7s(uint8_t);
void write_to_7s(uint8_t);
void init_7s(void);
void init_TIM5(void);
void init_TIM15(void);
void init_TIM16(void);


volatile uint32_t isFirstCaptured=0;
volatile uint32_t time=0;
volatile uint32_t distance=0;
volatile uint32_t thresholdDistance = 68;
volatile uint32_t pillDropped = 0;
volatile uint32_t checkCounter = 0;
volatile uint32_t off_duration = 0;
volatile uint8_t state = 0;
volatile uint16_t adc_value;
volatile uint8_t digit = 1;
volatile char input_buffer[BUFFER_SIZE];
volatile uint8_t buffer_index = 0;
volatile uint32_t cycles_left_to_drop = 0;
volatile uint32_t received_value = 0;
volatile uint16_t position = 0;
volatile uint32_t go_to_initial = 0;

#endif /* MAIN_H_ */
