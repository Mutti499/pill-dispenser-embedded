#include <stdint.h>
#include <string.h>
#include <stdlib.h>

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

typedef struct {
    volatile uint32_t CR1;  //0
    volatile uint32_t CR2;  //4
    volatile uint32_t CR3;  //8
    volatile uint32_t BRR;  //C
    uint32_t reserved1[2];  //10 14
    volatile uint32_t RQR;  //18
    volatile uint32_t ISR;  //1C
    volatile uint32_t ICR;  //20
    volatile uint32_t RDR;  //24
    volatile uint32_t TDR;  //28
    volatile uint32_t PRESC;//2C
} LPUARTType;

typedef struct {
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
} GPIOType;

#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)
#define LPUART1 ((LPUARTType *) 0x40008000)
#define UART4 ((USARTType *) 0x40004C00)
#define GPIOA ((GPIOType *) 0x42020000)
#define ISER2 *((volatile uint32_t *) 0xE000E108)
#define PWR_CR1 *((volatile uint32_t *) 0x40007000)
#define PWR_CR2 *((volatile uint32_t *) 0x40007004)
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define RCC_APB1ENR1 *((volatile uint32_t *) 0x40021058)
#define RCC_APB1ENR2 *((volatile uint32_t *) 0x4002105C)
#define GPIOG ((GPIOType *) 0x42021800)

// Global variables
volatile int tx_busy_lpuart1 = 0;
volatile int tx_busy_uart4 = 0;
volatile int current_index_lpuart1 = 0;
volatile int current_index_uart4 = 0;
volatile char *current_message_lpuart1 = NULL;
int message_length_lpuart1 = 0;
volatile int digit_count = 0;

void GPIO_Init_Master(void) {
    // Enable GPIOA and GPIOG
    RCC_AHB2ENR |= (1 << 0);
    RCC_AHB2ENR |= 1 << 6;

    // Configure PA0-1 for UART4
    GPIOA->MODER &= ~(0b1111);
    GPIOA->MODER |= 0b1010;
    GPIOA->AFRL &= ~(0b11111111);
    GPIOA->AFRL |= 0b10001000;  // AF8 for both pins

    // Configure PG7-8 for LPUART1
    GPIOG->MODER &= ~(0b0101 << (7 * 2));
    GPIOG->MODER |= 0b1010 << (7 * 2);
    GPIOG->AFRL &= ~(0b0111 << (7 * 4));
    GPIOG->AFRL |= 0b1000 << (7 * 4);
    GPIOG->AFRH &= ~0b0111;
    GPIOG->AFRH |= 0b1000;
}

void LPUART1_initialization(void) {
    // Power mode and clock setup
    RCC_APB1ENR1 |= 1 << 28;
    PWR_CR1 |= 1 << 14;
    PWR_CR2 |= 1 << 9;
    RCC_CCIPR1 &= ~(1 << 11);
    RCC_CCIPR1 |= 1 << 10;

    // LPUART1 setup
    RCC_APB1ENR2 |= 1;
    LPUART1->BRR = 8888;
    LPUART1->CR1 |= 1 << 29;    // FIFO enable
    LPUART1->CR1 |= 0b11 << 2;  // TX/RX enable
    LPUART1->CR1 |= 1 << 5;     // RXNE interrupt enable

    ISER2 |= 1 << 2;
    LPUART1->CR1 |= 1;          // UART enable
}

void UART4_Init(void) {
    RCC_APB1ENR1 |= (1 << 19);  // Enable UART4 clock

    UART4->BRR = 417;           // 9600 baud at 4MHz
    UART4->CR1 |= (1 << 29);    // FIFO enable
    UART4->CR1 |= (0b11 << 2);  // TX/RX enable
    UART4->CR1 |= (1 << 5);     // RXNE interrupt enable

    ISER2 |= (1 << 0);
    UART4->CR1 |= 1;            // UART enable
}

void send_try_again(void) {
    current_message_lpuart1 = "\r\nThe int value is bigger than 999. We put first three digit of your number.\r\n";
    message_length_lpuart1 = strlen(current_message_lpuart1);
    current_index_lpuart1 = 0;

    if (!tx_busy_lpuart1) {
        tx_busy_lpuart1 = 1;
        LPUART1->TDR = current_message_lpuart1[current_index_lpuart1++];
        LPUART1->CR1 |= (1 << 7); // Enable TC interrupt
    }
}

void send_welcome_message(void) {
    current_message_lpuart1 = "Welcome! Please enter a second value between 0 and 999 \r\n";
    message_length_lpuart1 = strlen(current_message_lpuart1);
    current_index_lpuart1 = 0;

    if (!tx_busy_lpuart1) {
        tx_busy_lpuart1 = 1;
        LPUART1->TDR = current_message_lpuart1[current_index_lpuart1++];
        LPUART1->CR1 |= (1 << 7); // Enable TC interrupt
    }
}

void UART4_IRQHandler(void) {
    // Handle Receive
    if ((UART4->ISR & (1 << 5)) != 0) {
        char UART4_received = UART4->RDR;
        UART4->ICR = (1 << 5);  // Clear RXNE flag

        switch(UART4_received) {
            case 'O':
                current_message_lpuart1 = "Your input is read and the value is set. You can change it when you need\r\n";
                break;
            case 'C':
            	current_message_lpuart1 = "The pill is taken!\r\nDispensing process completed waiting for new button press to start!\r\n";
            	break;
            case 'D':
                current_message_lpuart1 = "Time is up! Pill is being dispensed!\r\n";
                break;
            case 'T':
                current_message_lpuart1 = "The pill is taken!\r\n";
                break;
            case 'R':
                current_message_lpuart1 = "Waiting for new button press to start!\r\n";
                break;
            default:
                return;
        }

        message_length_lpuart1 = strlen(current_message_lpuart1);
        current_index_lpuart1 = 0;

        if (!tx_busy_lpuart1) {
            tx_busy_lpuart1 = 1;
            LPUART1->TDR = current_message_lpuart1[current_index_lpuart1++];
            LPUART1->CR1 |= (1 << 7); // Enable TC interrupt
        }
    }

    // Handle Transmit Complete
    if ((UART4->ISR & (1 << 6)) != 0) {
        UART4->ICR = (1 << 6);   // Clear TC flag
        tx_busy_uart4 = 0;
        UART4->CR1 &= ~(1 << 7); // Disable TC interrupt
    }
}

void LPUART1_IRQHandler(void) {
    // Handle Receive
    if ((LPUART1->ISR & (1 << 5)) != 0) {
        char received_char = LPUART1->RDR;
        LPUART1->ICR = (1 << 5);  // Clear RXNE flag

        if ((received_char >= '0' && received_char <= '9') || received_char == '\r' || received_char == '\n') {
            if (digit_count >= 3 && (received_char >= '0' && received_char <= '9')) {
                send_try_again();
                digit_count = 0;
                if (!tx_busy_uart4) {
                    tx_busy_uart4 = 1;
                    UART4->TDR = '\r';
                    UART4->CR1 |= (1 << 7);
                }
                return;
            } else if (!tx_busy_uart4) {
                tx_busy_uart4 = 1;
                UART4->TDR = received_char;
                UART4->CR1 |= (1 << 7);
            }
        }

        if (received_char >= '0' && received_char <= '9') {
            if (!tx_busy_lpuart1) {
                tx_busy_lpuart1 = 1;
                LPUART1->TDR = received_char;
                LPUART1->CR1 |= (1 << 7);
            }
            digit_count++;
        }

        if (received_char == '\r' || received_char == '\n') {
            if (!tx_busy_lpuart1) {
                tx_busy_lpuart1 = 1;
                LPUART1->TDR = '\r';
                current_message_lpuart1 = "\n";
                message_length_lpuart1 = 1;
                current_index_lpuart1 = 0;
                LPUART1->CR1 |= (1 << 7);
            }
            digit_count = 0;
        }
    }

    // Handle Transmit Complete
    if ((LPUART1->ISR & (1 << 6)) != 0) {
        LPUART1->ICR = (1 << 6);  // Clear TC flag

        if (current_index_lpuart1 < message_length_lpuart1) {
            LPUART1->TDR = current_message_lpuart1[current_index_lpuart1++];
        } else {
            tx_busy_lpuart1 = 0;
            LPUART1->CR1 &= ~(1 << 7); // Disable TC interrupt
        }
    }
}

void __enable_irq(void) {
    __asm volatile("mov r0, #0");
    __asm volatile("msr primask, r0");
}

int main(void) {
    GPIO_Init_Master();
    LPUART1_initialization();
    UART4_Init();
    __enable_irq();
    send_welcome_message();

    while(1) {
        __asm volatile("wfi");
    }
}
