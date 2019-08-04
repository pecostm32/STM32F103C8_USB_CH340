//Run openocd for debugging
//openocd -f ~/NetBeansProjects/STM_Drum_Box/STM32F103C8T6.cfg

#include "stm32f103_db.h"
#include "usb.h"

#define STACK_TOP 0x20005000

extern unsigned char  INIT_DATA_VALUES;
extern unsigned char  INIT_DATA_START;
extern unsigned char  INIT_DATA_END;
extern unsigned char  BSS_START;
extern unsigned char  BSS_END;

int main(void);
void resetHandler(void);

const void * intVectors[76] __attribute__((section(".vectors"))) = 
{
    (void*) STACK_TOP,
    resetHandler,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,
    usbIrqHandler,
    0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0, 0,
};

void resetHandler(void)
{
  unsigned char volatile *src;
  unsigned char volatile *dst;
  unsigned len;

  src= &INIT_DATA_VALUES;
  dst= &INIT_DATA_START;
  len= &INIT_DATA_END - &INIT_DATA_START;

  while(len--)
    *dst++ = *src++;

  dst = &BSS_START;

  len = &BSS_END - &BSS_START;

  while(len--)
    *dst++=0;

  main();
}

int main(void)
{
  //Setup flash to work with 72MHz clock
  //Enable the Prefetch Buffer and Set to 2 wait states
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
  
  //Configure system clock
  //External oscillator: 8MHz
  //PLL multiplicator: x9
  //SYSCLK: 64MHz
  //AHB: SYSCLK = 72MHz
  //APB1: SYSCLK/2 = 36MHz
  //APB2: SYSCLK/2 = 36MHz
  //ADC: SYSCLK/6 = 12MHz
  //USB: SYSCLK/1.5 = 48MHz
  RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2;
  
  //Enable external oscillator
  RCC->CR |= RCC_CR_HSEON;

  //Wait for the clock to become stable
  while((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY);

  //Enable the PLL
  RCC->CR |= RCC_CR_PLLON;

  //Wait for the PLL to become stable
  while((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY);

  //Switch to the PLL clock as system clock source. Since on reset these bits are set to 0 no need to clear first.
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  //Wait for the PLL to become the clock source
  while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

  //From this point on it is not possible to change the clock configuration without switching back to HSI
  
  //Enable the used peripherals. PORTA for USART1, PORTB for USART3, PORTC for the led, USART1 and Alternate Function IO
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;

  //Enable USART3 and USB
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  
  
  usbInit();

  while(1)
  {
    int16_t c = usbRead();
    
    if(c != -1)
      usbSend(c);
  }
}

