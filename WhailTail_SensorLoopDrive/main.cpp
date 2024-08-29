/*
    Example code on how to interact with the dac161s997 sensor loop drive on the whaletail badge

    Built around https://github.com/Accelovant/dac161s997
*/

#include <stdio.h>
#include <stdint.h>
#include "dac161s997.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

//Handle SPI interfactions here
#include "spi_port.h"


#define SPI1_RX    8
#define SPI1_SCLK  10 
#define SPI1_TX    11
#define SPI1_CS    14 

#define LED_STATUS 25

#define LOOPDRIVE_SPI spi1
#define MODBUS_BUAD 115200

int main()
{
   //Enable USB printing    
    stdio_init_all();

    //LED 
    gpio_init(LED_STATUS);
    gpio_set_dir(LED_STATUS,GPIO_OUT);
    gpio_put(LED_STATUS, false);

    //SPI Receive Pin
    gpio_set_function(SPI1_RX, GPIO_FUNC_SPI);
    gpio_set_dir(SPI1_RX,GPIO_IN);  

    //SPI Clock Pin
    gpio_set_function(SPI1_SCLK, GPIO_FUNC_SPI);
    gpio_set_dir(SPI1_SCLK,GPIO_OUT);  

    //SPI Transmit Pin
    gpio_set_function(SPI1_TX, GPIO_FUNC_SPI);
    gpio_set_dir(SPI1_TX,GPIO_OUT);  

    //SPI Chipselect pin
    gpio_init(SPI1_CS);
    gpio_set_dir(SPI1_CS,GPIO_OUT);  
    gpio_put(SPI1_CS, true);  

    //Initalize SPI Driver
    spi_init(LOOPDRIVE_SPI, 4000000);

    //Setup parameters for SPI port
    dac161s997_dev_t dev0 = {
        .spi_index = (int)spi_get_index(LOOPDRIVE_SPI),
        .cs_num = SPI1_CS
    };

    int err = 0;

    //Init just like in https://github.com/Accelovant/dac161s997?tab=readme-ov-file
    printf("dac161s997_init(&dev0)\r\n");
    while (dac161s997_init(&dev0) != 0); //Loop forever until dac initalizes

    //Turn on led to inidicate the driver init the sensor
    gpio_put(LED_STATUS, true);

    //Set up current output
    printf("dac161s997_set_output(&dev0, 1000)\r\n");
    err = dac161s997_set_output(&dev0, 1000);
    if(err)
    {
        printf("dac161s997_set_output Failed with %d\r\n", err);
    }

    //Set up alarm to trigger on low cur
    printf("dac161s997_set_alarm(&dev0, DAC161S997_ALARM_LOW_FAIL)\r\n");
    err = dac161s997_set_alarm(&dev0, DAC161S997_ALARM_LOW_FAIL);
    if(err)
    {
        printf("dac161s997_set_alarm Failed with %d\r\n", err);
    }    

    //Set up alarm to trigger on high cur
    printf("dac161s997_set_alarm(&dev0, DAC161S997_ALARM_HIGH_FAIL)\r\n");
    err = dac161s997_set_alarm(&dev0, DAC161S997_ALARM_HIGH_FAIL);
    if(err)
    {
        printf("dac161s997_set_alarm Failed with %d\r\n", err);
    }       

    //Variable for status
    uint32_t status = 0;

    //Loop forever
    while(1)
    {
        //get and print status
        printf("dac161s997_get_status(&dev0, &status)\r\n");
        err = dac161s997_get_status(&dev0, &status);
        if(err)
        {
            printf("dac161s997_get_status Failed with %d\r\n", err);
        }
        else
        {
            printf("status: 0x%X\r\n", status);

            //Print out set status bit
            if(status & DAC161S997_STATUS_ABSENT)
            {
                printf("ABSET is set - Failed to communicate with the device!\r\n");
            }
            if(status & DAC161S997_STATUS_LOOP_ERR)
            {
                printf("LOOP_ERR is set - Error Detected in the 420 loop!\r\n");
            }
            if(status & DAC161S997_STATUS_COM_TIMEOUT)
            {
                printf("COM_TIMEOUT is set - SPI Communication Timeout detected!\r\n");                
            }
            if(status & DAC161S997_STATUS_FRAME_ERR)
            {
                printf("STATUS_FRAME_ERR is set - SPI error frame detected!\r\n");                     
            }
            if(status & DAC161S997_LO_ALARM_ERR)
            {
                printf("LO_ALARM_ERR is set - Output current is at low error value!\r\n");                     
            }                        
            if(status & DAC161S997_HI_ALARM_ERR)
            {
                printf("LI_ALARM_ERR is set - Output current is at high error value!\r\n");   
            }  

            //Printout space
            printf("\r\n\r\n\r\n", status);

        }

        sleep_ms(500);

        //Toggle LED
        gpio_put(LED_STATUS, !gpio_get(LED_STATUS));
    }


    return 0;
}