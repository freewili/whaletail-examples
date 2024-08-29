/*
    Example code on how to interact with the adin1110 10bt1L phy on the whaletail badge

    Built around https://github.com/sparkfun/SparkFun_ADIN1110_Arduino_Library with Example 1A
*/

#include <stdio.h>
#include "pico/stdlib.h"

//Sparkfun drivers
#include "SparkFun_ADIN1110_Arduino_Library/src/SparkFun_SinglePairEthernet.h"
#include "SparkFun_ADIN1110_Arduino_Library/src/sfe_spe_advanced.h"
#include "SparkFun_ADIN1110_Arduino_Library/src/adi_phy.h"

//Macs 
uint8_t deviceMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0x28, 0x51};
uint8_t destinationMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xC9};

SinglePairEthernet   adin1110;

int msg = 0;
const int NUM_MSGS = 8;
const int MAX_MSG_SIZE = 200;

char outputString[NUM_MSGS][MAX_MSG_SIZE] = {
    "Adin1110 Example 1a",
    "If this sketch is connected to another devices running example 1b, these messages will be echoed back",
    "User can define their own behavior for when data is recieved by defining their own rxCallback",
    "The conterpart to this example demonstrates how to access recieved data if using callback is not desired",
    "This example uses Serial.println in the callback, this is may not be best practice since it can happen in an interrupt",
    "Basic functionality of sending and recieving data is provided by the SinglePairEthernet class",
    "Messages are copied to memory internal to the created object",
    "If more performance is required, or you would like more control over memory, try the sfe_spe_advanced class"
};

volatile bool link = false;

void linkCallback(bool linkStatus)
{
    link = linkStatus;
}

#define FRAME_HEADER_SIZE 14

static void rxCallback(uint8_t * data, int dataLen, uint8_t * senderMac)
{
    gpio_put(LED_BUILTIN, !gpio_get(LED_BUILTIN));

    printf("Received Data: ");

    //Print out data
    for(int index = 0; index < dataLen; index++)
    {
        printf("%c", data[index]);
    }

    //Check for expected mac
    if(!adin1110.indenticalMacs(senderMac, destinationMAC))
    {
        printf("From an unknown source: ");
        for(int i = 0; i < 6; i++)
        {
          printf(" %02X", senderMac[i]);
          printf(" ");    
        }
        
    }   
    printf("\r\n"); 
}

int main()
{
   //Enable USB printing    
    stdio_init_all();

    //For com port hookup
    sleep_ms(5000); 

    //Start up phy
    if (!adin1110.begin(deviceMAC)) 
    {
        while(1)
        {
            printf("ADIN1110 Failed to Init!"); 
        }
    }
    else
    {
        printf("ADIN1110 INITED!");  
    }

    //Setup callbacks
    adin1110.setLinkCallback(linkCallback);
    adin1110.setRxCallback(rxCallback);

    //loop indefinitely 
    while(1)
    {
        //Check link status
        if(adin1110.getLinkStatus())
        {
            printf("Transmiting msg: %d!\r\n", msg);  
            adin1110.sendData((uint8_t *)outputString[msg], sizeof(outputString[msg]), destinationMAC);

            msg++;
            if(msg >= NUM_MSGS)
            {
                msg = 0;
            }   
            sleep_us(5000);
        }    
    }
}