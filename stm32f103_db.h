#ifndef __STM32F030_H_
#define __STM32F030_H_

#include <stdint.h>

#define     __IO    volatile

#define     __IM     volatile const      //Defines 'read only' structure member permissions
#define     __OM     volatile            //Defines 'write only' structure member permissions
#define     __IOM    volatile            //Defines 'read / write' structure member permissions

//Power Control
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CSR;
} PWR_TypeDef;


//Reset and Clock Control
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;

//FLASH Registers
typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
} FLASH_TypeDef;

//Real-Time Clock
typedef struct
{
  __IO uint32_t CRH;
  __IO uint32_t CRL;
  __IO uint32_t PRLH;
  __IO uint32_t PRLL;
  __IO uint32_t DIVH;
  __IO uint32_t DIVL;
  __IO uint32_t CNTH;
  __IO uint32_t CNTL;
  __IO uint32_t ALRH;
  __IO uint32_t ALRL;
} RTC_TypeDef;

//External Interrupt/Event Controller
typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
} EXTI_TypeDef;

//Structure type to access the System Control Block (SCB).
typedef struct
{
  __IM  uint32_t CPUID;                  //Offset: 0x000 (R/ )  CPUID Base Register
  __IOM uint32_t ICSR;                   //Offset: 0x004 (R/W)  Interrupt Control and State Register
  __IOM uint32_t VTOR;                   //Offset: 0x008 (R/W)  Vector Table Offset Register
  __IOM uint32_t AIRCR;                  //Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
  __IOM uint32_t SCR;                    //Offset: 0x010 (R/W)  System Control Register
  __IOM uint32_t CCR;                    //Offset: 0x014 (R/W)  Configuration Control Register
  __IOM uint8_t  SHP[12U];               //Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15)
  __IOM uint32_t SHCSR;                  //Offset: 0x024 (R/W)  System Handler Control and State Register
  __IOM uint32_t CFSR;                   //Offset: 0x028 (R/W)  Configurable Fault Status Register
  __IOM uint32_t HFSR;                   //Offset: 0x02C (R/W)  HardFault Status Register
  __IOM uint32_t DFSR;                   //Offset: 0x030 (R/W)  Debug Fault Status Register
  __IOM uint32_t MMFAR;                  //Offset: 0x034 (R/W)  MemManage Fault Address Register
  __IOM uint32_t BFAR;                   //Offset: 0x038 (R/W)  BusFault Address Register
  __IOM uint32_t AFSR;                   //Offset: 0x03C (R/W)  Auxiliary Fault Status Register
  __IM  uint32_t PFR[2U];                //Offset: 0x040 (R/ )  Processor Feature Register
  __IM  uint32_t DFR;                    //Offset: 0x048 (R/ )  Debug Feature Register
  __IM  uint32_t ADR;                    //Offset: 0x04C (R/ )  Auxiliary Feature Register
  __IM  uint32_t MMFR[4U];               //Offset: 0x050 (R/ )  Memory Model Feature Register
  __IM  uint32_t ISAR[5U];               //Offset: 0x060 (R/ )  Instruction Set Attributes Register
        uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR;                  //Offset: 0x088 (R/W)  Coprocessor Access Control Register
} SCB_Type;

//Structure type to access the Nested Vectored Interrupt Controller (NVIC)
typedef struct
{
  __IOM uint32_t ISER[8U];               //Offset: 0x000 (R/W)  Interrupt Set Enable Register
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               //Offset: 0x080 (R/W)  Interrupt Clear Enable Register
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               //Offset: 0x100 (R/W)  Interrupt Set Pending Register
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               //Offset: 0x180 (R/W)  Interrupt Clear Pending Register
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               //Offset: 0x200 (R/W)  Interrupt Active bit Register
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               //Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide)
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   //Offset: 0xE00 ( /W)  Software Trigger Interrupt Register
}  NVIC_Type;

//Universal Serial Bus Full Speed Device
typedef struct
{
  __IO uint16_t EP0R;                 //USB Endpoint 0 register,         Address offset: 0x00
  __IO uint16_t RESERVED0;            //Reserved
  __IO uint16_t EP1R;                 //USB Endpoint 1 register,         Address offset: 0x04
  __IO uint16_t RESERVED1;            //Reserved
  __IO uint16_t EP2R;                 //USB Endpoint 2 register,         Address offset: 0x08
  __IO uint16_t RESERVED2;            //Reserved
  __IO uint16_t EP3R;                 //USB Endpoint 3 register,         Address offset: 0x0C
  __IO uint16_t RESERVED3;            //Reserved
  __IO uint16_t EP4R;                 //USB Endpoint 4 register,         Address offset: 0x10
  __IO uint16_t RESERVED4;            //Reserved
  __IO uint16_t EP5R;                 //USB Endpoint 5 register,         Address offset: 0x14
  __IO uint16_t RESERVED5;            //Reserved
  __IO uint16_t EP6R;                 //USB Endpoint 6 register,         Address offset: 0x18
  __IO uint16_t RESERVED6;            //Reserved
  __IO uint16_t EP7R;                 //USB Endpoint 7 register,         Address offset: 0x1C
  __IO uint16_t RESERVED7[17];        //Reserved
  __IO uint16_t CNTR;                 //Control register,                Address offset: 0x40
  __IO uint16_t RESERVED8;            //Reserved
  __IO uint16_t ISTR;                 //Interrupt status register,       Address offset: 0x44
  __IO uint16_t RESERVED9;            //Reserved
  __IO uint16_t FNR;                  //Frame number register,           Address offset: 0x48
  __IO uint16_t RESERVEDA;            //Reserved
  __IO uint16_t DADDR;                //Device address register,         Address offset: 0x4C
  __IO uint16_t RESERVEDB;            //Reserved
  __IO uint16_t BTABLE;               //Buffer Table address register,   Address offset: 0x50
  __IO uint16_t RESERVEDC;            //Reserved
} USB_TypeDef;

//Packet memory entry
typedef struct
{
  __IO uint16_t DATA;
       uint16_t RESERVED;
}PMA_ENTRY;

//Structure for PMA
typedef struct
{
  PMA_ENTRY MEMORY[256];
} PMA_TypeDef;

//Structure for Buffer Descriptor Table
typedef struct
{
  PMA_ENTRY TX_ADDRESS;
  PMA_ENTRY TX_COUNT;
  PMA_ENTRY RX_ADDRESS;
  PMA_ENTRY RX_COUNT;
} BTABLE_ENTRY;

//Structure for endpoint descriptors
typedef struct
{
  BTABLE_ENTRY EPD[8];
} BTABLE_TypeDef;

typedef union
{
  uint16_t word;
  struct BYTES
  {
    uint8_t low;
    uint8_t high;
  } bytes;
} wbcombi;


#define PMA_BASE    (0x40006000L)  //USB_IP Packet Memory Area base address

#define PMA         ((PMA_TypeDef *) PMA_BASE)
#define EPBTABLE    ((BTABLE_TypeDef *) PMA_BASE)



#define RCC_BASE   0x40021000
#define PWR_BASE   0x40007000
#define RTC_BASE   0x40002800
#define EXTI_BASE  0x40010400

#define USB_BASE   0x40005C00

#define AFIO_BASE  0x40010000
#define GPIOA_BASE 0x40010800
#define GPIOB_BASE 0x40010C00
#define GPIOC_BASE 0x40011000
#define GPIOD_BASE 0x40011400
#define GPIOE_BASE 0x40011800
#define GPIOF_BASE 0x40011C00
#define GPIOG_BASE 0x40012000

#define ADC1_BASE  0x40012400

#define USART_BASE 0x40013800

#define NVIC_BASE  0xE000E100
#define SCB_BASE   0xE000ED00                    //System Control Block Base Address

#define FLASH_BASE 0x40022000

#define FLASH_START 0x08000000


#define PWR                 ((PWR_TypeDef *)PWR_BASE)
#define RCC                 ((RCC_TypeDef *)RCC_BASE)
#define USB                 ((USB_TypeDef *)USB_BASE)
#define FLASH               ((FLASH_TypeDef *)FLASH_BASE)
#define RTC                 ((RTC_TypeDef *)RTC_BASE)
#define EXTI                ((EXTI_TypeDef *)EXTI_BASE)
#define SCB                 ((SCB_Type *)SCB_BASE)
#define NVIC                ((NVIC_Type *)NVIC_BASE)


#define USB_LP_CAN1_RX0_IRQn  20

#define FLASH_ACR_LATENCY_2   0x00000004
#define FLASH_ACR_PRFTBE      0x00000010               //Prefetch Buffer Enable

#define RCC_CFGR_PLLMULL9     0x001C0000               //PLL input clock*9
#define RCC_CFGR_PLLSRC       0x00010000               //PLL entry clock source
#define RCC_CFGR_ADCPRE_DIV6  0x00008000               //PCLK2 divided by 6
#define RCC_CFGR_PPRE1_DIV2   0x00000400               //HCLK divided by 2
#define RCC_CFGR_PPRE2_DIV2   0x00002000               //HCLK divided by 2

#define RCC_CR_HSEON          0x00010000               //External High Speed clock enable
#define RCC_CR_HSERDY         0x00020000               //External High Speed clock ready flag

#define RCC_CR_PLLON          0x01000000               //PLL enable
#define RCC_CR_PLLRDY         0x02000000               //PLL clock ready flag

#define RCC_CFGR_SW_PLL       0x00000002               //PLL selected as system clock
#define RCC_CFGR_SWS_PLL      0x00000008               //PLL used as system clock

// Bit definition for RCC_APB2ENR register
#define RCC_APB2ENR_AFIOEN                   0x00000001              //Alternate Function I/O clock enable
#define RCC_APB2ENR_IOPAEN                   0x00000004              //I/O port A clock enable
#define RCC_APB2ENR_IOPBEN                   0x00000008              //I/O port B clock enable
#define RCC_APB2ENR_IOPCEN                   0x00000010              //I/O port C clock enable
#define RCC_APB2ENR_IOPDEN                   0x00000020              //I/O port D clock enable
#define RCC_APB2ENR_IOPEEN                   0x00000040              //I/O port E clock enable
#define RCC_APB2ENR_ADC1EN                   0x00000200              //ADC 1 interface clock enable
#define RCC_APB2ENR_ADC2EN                   0x00000400              //ADC 2 interface clock enable
#define RCC_APB2ENR_TIM1EN                   0x00000800              //TIM1 Timer clock enable
#define RCC_APB2ENR_SPI1EN                   0x00001000              //SPI 1 clock enable
#define RCC_APB2ENR_USART1EN                 0x00004000              //USART1 clock enable

//Bit definition for RCC_APB1ENR register
#define RCC_APB1ENR_TIM2EN                   0x00000001              //Timer 2 clock enabled
#define RCC_APB1ENR_TIM3EN                   0x00000002              //Timer 3 clock enable
#define RCC_APB1ENR_TIM4EN                   0x00000004              //Timer 4 clock enable
#define RCC_APB1ENR_WWDGEN                   0x00000800              //Window Watchdog clock enable
#define RCC_APB1ENR_USART2EN                 0x00020000              //USART 2 clock enable
#define RCC_APB1ENR_USART3EN                 0x00040000              //USART 3 clock enable
#define RCC_APB1ENR_I2C1EN                   0x00200000              //I2C 1 clock enable
#define RCC_APB1ENR_I2C2EN                   0x00400000              //I2C 2 clock enable
#define RCC_APB1ENR_CAN1EN                   0x02000000              //CAN1 clock enable
#define RCC_APB1ENR_SPI2EN                   0x00004000              //SPI 2 clock enable
#define RCC_APB1ENR_USBEN                    0x00800000              //USB Device clock enable
#define RCC_APB1ENR_BKPEN                    0x08000000              //Backup interface clock enable
#define RCC_APB1ENR_PWREN                    0x10000000              //Power interface clock enable


#define PWR_CR_DBP                           0x00000100              //Disable Backup Domain write protection

//Bit definition for RCC_CSR register
#define RCC_CSR_LSION                        0x00000001              //Internal Low Speed oscillator enable
#define RCC_CSR_LSIRDY                       0x00000002              //Internal Low Speed oscillator Ready
#define RCC_CSR_RMVF                         0x01000000              //Remove reset flag
#define RCC_CSR_PINRSTF                      0x04000000              //PIN reset flag
#define RCC_CSR_PORRSTF                      0x08000000              //POR/PDR reset flag
#define RCC_CSR_SFTRSTF                      0x10000000              //Software Reset flag
#define RCC_CSR_IWDGRSTF                     0x20000000              //Independent Watchdog reset flag
#define RCC_CSR_WWDGRSTF                     0x40000000              //Window watchdog reset flag
#define RCC_CSR_LPWRRSTF                     0x80000000              //Low-Power reset flag

//Bit definition for RCC_BDCR register
#define RCC_BDCR_LSEON                       0x00000001              //External Low Speed oscillator enable
#define RCC_BDCR_LSERDY                      0x00000002              //External Low Speed oscillator Ready
#define RCC_BDCR_LSEBYP                      0x00000004              //External Low Speed oscillator Bypass

//RTC configuration
#define RCC_BDCR_RTCSEL                      0x00000300              //RTCSEL[1:0] bits (RTC clock source selection)
#define RCC_BDCR_RTCSEL_NOCLOCK              0x00000000              //No clock
#define RCC_BDCR_RTCSEL_LSE                  0x00000100              //LSE oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_LSI                  0x00000200              //LSI oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_HSE                  0x00000300              //HSE oscillator clock divided by 128 used as RTC clock

#define RCC_BDCR_RTCEN                       0x00008000              //RTC clock enable
#define RCC_BDCR_BDRST                       0x00010000              //Backup domain software reset


#define RTC_CRL_RSF                          0x00000008               //Registers Synchronized Flag
#define RTC_CRL_CNF                          0x00000010               //Configuration Flag
#define RTC_CRL_RTOFF                        0x00000020               //RTC operation OFF


#define AIRCR_VECTKEY_MASK      ((uint32_t)0x05FA0000)

#define EXTI_Line18             ((uint32_t)0x40000)  //External interrupt line 18 Connected to the USB Device/USB OTG FS

#define NVIC_PriorityGroup_2    ((uint32_t)0x500)    //2 bits for pre-emption priority, 2 bits for subpriority


#define USB_CNTR_SOFM                           0x00000200              //Start Of Frame Interrupt Mask
#define USB_CNTR_RESETM                         0x00000400              //RESET Interrupt Mask
#define USB_CNTR_SUSPM                          0x00000800              //Suspend mode Interrupt Mask
#define USB_CNTR_CTRM                           0x00008000              //Correct Transfer Interrupt Mask

#define USB_EP_TX_VALID                         0x00000030              //EndPoint TX VALID
#define USB_EP_TX_NAK                           0x00000020              //EndPoint TX NAKed

#define USB_EP_CTR_RX                           0x00008000              //EndPoint Correct TRansfer RX
#define USB_EPRX_STAT                           0x00003000              //EndPoint RX STATus bit field
#define USB_EP_SETUP                            0x00000800              //EndPoint SETUP
#define USB_EP_T_FIELD                          0x00000600              //EndPoint TYPE
#define USB_EP_KIND                             0x00000100              //EndPoint KIND
#define USB_EP_CTR_TX                           0x00000080              //EndPoint Correct TRansfer TX
#define USB_EPTX_STAT                           0x00000030              //EndPoint TX STATus bit field
#define USB_EPADDR_FIELD                        0x0000000F              //EndPoint ADDRess FIELD

#define  USB_EPREG_MASK                      (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | USB_EPADDR_FIELD)

#define USB_ISTR_EP_ID                          0x0000000F               //Endpoint Identifier
#define USB_ISTR_ESOF                           0x00000100               //Expected Start Of Frame
#define USB_ISTR_SOF                            0x00000200               //Start Of Frame
#define USB_ISTR_RESET                          0x00000400               //USB RESET request
#define USB_ISTR_SUSP                           0x00000800               //Suspend mode request
#define USB_ISTR_WKUP                           0x00001000               //Wake up
#define USB_ISTR_ERR                            0x00002000               //Error
#define USB_ISTR_PMAOVR                         0x00004000               //Packet Memory Area Over / Underrun
#define USB_ISTR_CTR                            0x00008000               //Correct Transfer

#define USB_EP_BULK                             0x00000000               //EndPoint BULK
#define USB_EP_CONTROL                          0x00000200               //EndPoint CONTROL
#define USB_EP_INTERRUPT                        0x00000600               //EndPoint INTERRUPT

#define USB_EP_RX_DIS                           0x00000000               //EndPoint RX DISabled
#define USB_EP_RX_VALID                         0x00003000               //EndPoint RX VALID

#define USB_DADDR_EF                            0x00000080               //Enable Function














#define RCC_CR 0x00
#define RCC_CFGR 0x04
#define RCC_AHBENR 0x14
#define RCC_APB2ENR 0x18
#define RCC_APB1ENR 0x1C
#define RCC_BDCR 0x20
#define RCC_CSR 0x24

#define PWR_CR 0x00

#define RTC_CRL 0x04
#define RTC_PRLH 0x08
#define RTC_PRLL 0x0C
#define RTC_CNTH 0x18
#define RTC_CNTL 0x1C

#define AFIO_MAPR 0x04

#define GPIO_CRL 0x00
#define GPIO_CRH 0x04
#define GPIO_IDR 0x08
#define GPIO_ODR 0x0C
#define GPIO_BSRR 0x10

#define ADC_SR 0x00
#define ADC_CR1 0x04
#define ADC_CR2 0x08
#define ADC_SMPR1 0x0C
#define ADC_SMPR2 0x10
#define ADC_SQR1 0x2C
#define ADC_SQR2 0x30
#define ADC_SQR3 0x34
#define ADC_DR 0x4C

#define USART_SR 0x00
#define USART_DR 0x04
#define USART_BRR 0x08
#define USART_CR1 0x0C
#define USART_CR2 0x10

#define NVIC_ISER 0x00

#define FLASH_SR 0x0C
#define FLASH_CR 0x10
#define FLASH_AR 0x14

#define IRQ_UART 37

#define PIN_MODE_IN 0
#define PIN_MODE_OUT 1
#define PIN_MODE_OUT_FAST 3
#define PIN_MODE_OUT_SLOW 2

#define PIN_CNF_I_ANA 0
#define PIN_CNF_I_FLT 1
#define PIN_CNF_I_PULL 2

#define PIN_CNF_O_PP 0
#define PIN_CNF_O_OD 1
#define PIN_CNF_O_APP 2
#define PIN_CNF_O_AOD 3

#endif


