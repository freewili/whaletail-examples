/*
 *---------------------------------------------------------------------------
 *
 * Copyright (c) 2020, 2021 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc.
 * and its licensors.By using this software you agree to the terms of the
 * associated Analog Devices Software License Agreement.
 *
 *---------------------------------------------------------------------------
 */
/*
 * This file has been ported from its original location to be included with
 * the SparkFun ADIN1110 Arduino driver, file contents have been changed to
 * give generic arduino board support
*/

#include "boardsupport.h"

#include <string.h>

#ifdef ORCA_WHALE_TAIL
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#define HIGH 1
#define LOW 0
#define digitalWrite(p,s) gpio_put(p, s)
#define digitalRead(p) gpio_get(p)

class SerialMock
{
    public:
        void print(const char* str)
        {
            printf("%s", str); 
        }     
        void print(int v)
        {
            printf("%d", v); 
        }        
        void println()
        {
            printf("\r\n");
        }
};

SerialMock Serial;

uint32_t millis()
{
    return to_ms_since_boot(get_absolute_time());
}


#define SPI_instance = SP_10BTL_SPI;
#else
#include "Arduino.h"
#include <SPI.h>

static SPIClass* SPI_instance = &SPI;
#endif



// Enable this define to print all spi messages, note this will severely impact performance
 #define DEBUG_SPI

#if defined(ARDUINO_ARCH_MBED) 
#include <mbed.h>
rtos::Thread thread;
rtos::Semaphore updates(0);
#endif

#define RESET_DELAY       (1)
#define AFTER_RESET_DELAY (100)

static          ADI_CB gpfSpiCallback = NULL;
static void     *gpSpiCBParam = NULL;

static          ADI_CB gpfGPIOIntCallback = NULL;
static void     *gpGPIOIntCBParam = NULL;

static uint8_t status_led_pin = LED_BUILTIN;
static uint8_t interrupt_pin = DEFAULT_ETH_INT_Pin;
static uint8_t reset_pin = DEFAULT_ETH_RESET_Pin;
static uint8_t chip_select_pin = DEFAULT_ETH_SPI_CS_Pin;


void SPI_TxRxCpltCallback(void);
void BSP_IRQCallback(void);

volatile bool bIRQEn = false;

/*
* Functions that are part of the driver, that do nothing in the arduino port
*/
void BSP_ErrorLed(bool on) { /*NO ERROR LED*/ }

void BSP_FuncLed1(bool on) { /* NO FuncLed1 LED */ }

void BSP_FuncLed1Toggle(void) { /* NO FuncLed1 LED */ }

void BSP_FuncLed2(bool on) { /* NO FuncLed2 LED */ }

void BSP_FuncLed2Toggle(void) { /* NO FuncLed2 LED */ }

void BSP_getConfigPins(uint16_t *value) { /* This board has no config pins, so odnt do anything */ }


void BSP_disableInterrupts(void)
{
    #if defined(ARDUINO_ARCH_APOLLO3) || defined(ARDUINO_SPARKFUN_THINGPLUS_RP2040) || defined(ARDUINO_SPARKFUN_MICROMOD_RP2040)
        //These architectures have problems with entering/exiting critical section so just don't do it
        bIRQEn = true;
    #elif defined(ORCA_WHALE_TAIL)

    #else
	noInterrupts();
    #endif //defined(ARDUINO_ARCH_APOLLO3) || defined(ARDUINO_SPARKFUN_THINGPLUS_RP2040) || defined(ARDUINO_SPARKFUN_MICROMOD_RP2040)

    bIRQEn = false;
}


void BSP_enableInterrupts(void)
{
    #if defined(ARDUINO_ARCH_APOLLO3) || defined(ARDUINO_SPARKFUN_THINGPLUS_RP2040) || defined(ARDUINO_SPARKFUN_MICROMOD_RP2040)
        //These architectures have problems with entering/exiting critical section so just don't do it
    #elif defined(ORCA_WHALE_TAIL)

    #else
	noInterrupts();
    #endif //defined(ARDUINO_ARCH_APOLLO3) || defined(ARDUINO_SPARKFUN_THINGPLUS_RP2040) || defined(ARDUINO_SPARKFUN_MICROMOD_RP2040)

    bIRQEn = true;
}

/*
 * Blocking delay function
 */
void BSP_delayMs(uint32_t delay)
{
    volatile uint32_t now;
    uint32_t checkTime  = BSP_SysNow();
    /* Read SysTick Timer every Ms*/
    while (1)
    {
      now  = BSP_SysNow();
       if (now - checkTime >= delay)
       {
          break;
       }
    }
}

/*
 * Hardware reset to DUT
 */
void BSP_HWReset(bool set)
{
    digitalWrite(reset_pin, LOW);
    BSP_delayMs(RESET_DELAY);
    digitalWrite(reset_pin, HIGH);
    BSP_delayMs(AFTER_RESET_DELAY);
}

/* LED functions */
static void bspLedSet(uint16_t pin, bool on)
{
    if (on)
    {
        digitalWrite(pin, HIGH);
    }
    else
    {
        digitalWrite(pin, LOW);
    }
}

static void bspLedToggle(uint16_t pin)
{
    digitalWrite(pin, !digitalRead(pin));
}

/*
 * Heartbeat LED, On arduino we just default this to LED_BUILTIN
 */
void BSP_HeartBeat(void)
{
    bspLedToggle(status_led_pin);
}

/*
 * HeartBeat LED, On arduino we just default this to LED_BUILTIN
 */
void BSP_HeartBeatLed(bool on)
{
    bspLedSet(status_led_pin, on);
}

/* All LEDs toggle, used to indicate hardware failure on the board */
void BSP_LedToggleAll(void)
{
    bspLedSet(status_led_pin, HIGH);
}

uint32_t BSP_spi2_write_and_read(uint8_t *pBufferTx, uint8_t *pBufferRx, uint32_t nbBytes, bool useDma)
{
    //Validate parameters
    if(!pBufferTx || !pBufferRx)
    {
        return 1;
    }
    if(useDma)
    { //no DMA support for arduino
        return 1;
    }

    #ifdef DEBUG_SPI
    Serial.print("writing numbytes = ");
    Serial.print(nbBytes);
    Serial.print(" : ");
    for(int i = 0; i < nbBytes; i++)
    {
        Serial.print(pBufferTx[i]);
        Serial.print(" ");
    }
    Serial.println();
    #endif

#ifdef ORCA_WHALE_TAIL
    digitalWrite(chip_select_pin, LOW);
    spi_write_read_blocking(spi0, pBufferTx, pBufferRx,nbBytes);
#else
    memcpy(pBufferRx, pBufferTx, nbBytes);
    SPI_instance->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    bspLedSet(chip_select_pin, LOW);
    SPI_instance->transfer(pBufferRx, nbBytes);
#endif

    //Driver expects there to be an interrupt that fires after completion
    //Since SPI is blocking for arduino this is overly complicated, just call the "callback" function now
    SPI_TxRxCpltCallback();

    #ifdef DEBUG_SPI
    Serial.print("read: ");
    for(int i = 0; i < nbBytes; i++)
    {
        Serial.print(pBufferRx[i]);
    }
    Serial.println();
    #endif

    return 0;
}

//Function called on SPI transaction completion
void SPI_TxRxCpltCallback(void)
{
    bspLedSet(chip_select_pin, HIGH);
#ifndef ORCA_WHALE_TAIL    
    SPI_instance->endTransaction();
#endif
    (*gpfSpiCallback)(gpSpiCBParam, 0, NULL);
}

// Register the SPI callback, in the driver the following macro is used:
// extern uint32_t HAL_SPI_Register_Callback(ADI_CB const *pfCallback, void *const pCBParam);
uint32_t BSP_spi2_register_callback(ADI_CB const *pfCallback, void *const pCBParam)
{
    gpfSpiCallback = (ADI_CB)pfCallback;
    gpSpiCBParam = pCBParam ;
    return 0;
}

/*
* Set the Chip select to desired level, true for HIGH or false for LOW
 */
void setSPI2Cs(bool set)
{
    if(set == true)
    {
        bspLedSet(chip_select_pin, HIGH);
    }
    else
    {
        bspLedSet(chip_select_pin, LOW);
    }
}

struct repeating_timer timer;

bool timer_callback(repeating_timer  *rt)
{
    if(bIRQEn)
    {
        BSP_IRQCallback();
    }
    return true;
}


// Register the callback for the interrupt pin, in the driver the following macro is used:
// extern uint32_t HAL_INT_N_Register_Callback(ADI_CB const *pfCallback, void *const pCBParam);
uint32_t BSP_RegisterIRQCallback(ADI_CB const *intCallback, void * hDevice)
{
    gpfGPIOIntCallback = (ADI_CB)intCallback;
    gpGPIOIntCBParam = hDevice ;

#ifndef ORCA_WHALE_TAIL
    attachInterrupt(interrupt_pin, BSP_IRQCallback, FALLING);
#endif
    return 0;
}

#if defined(ARDUINO_ARCH_MBED) 
// MBED will not allow SPI calls during ISR, which the callbacks may do
// So instead of directly calling in this function we start a thread that will call the callback after signalled to by this function
int num_int = 0;
void BSP_IRQCallback()
{
    //Signal to thread that this function was called
    updates.release();
}

void thread_fn( void ){  
    while(1)
    {
        //Once acquired, the IRQCallback function has run
        updates.acquire();

        if (gpfGPIOIntCallback)
        {
            (*gpfGPIOIntCallback)(gpGPIOIntCBParam, 0, NULL);
        }
    }
}
#else
//Outside of mbed cores, we just call the callback within the ISR
void BSP_IRQCallback()
{
    if (gpfGPIOIntCallback)
    {
        (*gpfGPIOIntCallback)(gpGPIOIntCBParam, 0, NULL);
    }
}
#endif

uint32_t BSP_SysNow(void)
{
    return millis();
}

uint32_t BSP_InitSystem(void)
{
#if defined(ARDUINO_ARCH_MBED)
    //Start a thread to handle the IRQ callback
    thread.start(thread_fn);
    thread.set_priority(osPriorityHigh);
#endif
  
#ifdef ORCA_WHALE_TAIL
    //Timer to check phy status
    add_repeating_timer_us(750, timer_callback, NULL, &timer);  
  
    //SPI TX
    gpio_set_function(WHALE_TAIL_SPI_TX, GPIO_FUNC_SPI);
    gpio_set_dir(WHALE_TAIL_SPI_TX,GPIO_OUT);  

    //SPI SCLK
    gpio_set_function(WHALE_TAIL_SPI_SCLK, GPIO_FUNC_SPI);
    gpio_set_dir(WHALE_TAIL_SPI_SCLK,GPIO_OUT);  

    //SPI RX
    gpio_set_function(WHALE_TAIL_SPI_RX, GPIO_FUNC_SPI);
    gpio_set_dir(WHALE_TAIL_SPI_RX,GPIO_IN);  

    //SPI Chipselect
    gpio_init(chip_select_pin);
    gpio_set_dir(chip_select_pin,GPIO_OUT);    
    gpio_put(chip_select_pin, true);       

    //INIT SPI driver
    spi_init(spi0, 10000000);

    //LED Status
    gpio_init(status_led_pin);
    gpio_set_dir(status_led_pin,GPIO_OUT);  

    //Phy Reset Pin
    gpio_init(reset_pin);
    gpio_set_dir(reset_pin,GPIO_OUT);  
#else
    SPI_instance->begin();
    pinMode(status_led_pin, OUTPUT);
    pinMode(interrupt_pin, INPUT);
    pinMode(reset_pin, OUTPUT);
    pinMode(chip_select_pin, OUTPUT);
    digitalWrite(chip_select_pin, HIGH);
    digitalWrite(reset_pin, HIGH);
#endif

    return 0;
}

//Set all the pins that are uysed throughout this module
uint32_t BSP_ConfigSystem(uint8_t status, uint8_t interrupt, uint8_t reset, uint8_t chip_select)
{
    status_led_pin = status;
    interrupt_pin = interrupt;
    reset_pin = reset;
    chip_select_pin = chip_select;
    return 0;
}

//Change just the chip select pin
uint32_t BSP_ConfigSystemCS(uint8_t chip_select)
{
    chip_select_pin = chip_select;
    return 0;
}

//User in the functions below, prints debug and error messages from within the driver
uint32_t msgWrite(char * ptr)
{
    Serial.print(ptr);
    return 0;
}

char aDebugString[150u];

void common_Fail(char *FailureReason)
{
    char fail[] = "Failed: ";
    char term[] = "\n\r";

    /* Ignore return codes since there's nothing we can do if it fails */
    msgWrite(fail);
    msgWrite(FailureReason);
    msgWrite(term);
 }

void common_Perf(char *InfoString)
{
    char term[] = "\n\r";

    /* Ignore return codes since there's nothing we can do if it fails */
    msgWrite(InfoString);
    msgWrite(term);
}

