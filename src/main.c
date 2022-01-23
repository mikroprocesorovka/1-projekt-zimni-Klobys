#include "stm8s.h"
#include "milis.h"
#include "stm8_hd44780.h"

#include "delay.h"

#include <stdio.h>
#define _ISOC99_SOURCE
#define _GNU_SOURCE

void setup(void)
{
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);      // taktovani MCU na 16MHz
    lcd_init(); //inicializace LCD

    lcd_gotoxy(0,0); //vypsání textu předem na displej
    lcd_puts("set");
    lcd_gotoxy(15,0);
    lcd_puts("s");
    lcd_gotoxy(0,1);
    lcd_puts("time left");
    lcd_gotoxy(15,1);
    lcd_puts("s");

    init_milis(); //inicializace mmilisu

    GPIO_Init(GPIOD, GPIO_PIN_6,GPIO_MODE_OUT_PP_LOW_SLOW); // nastavíme PD6 jako výstup pro bzučák
    GPIO_WriteHigh(GPIOD,GPIO_PIN_6); // zapíšeme na bzučák high aby ze začátku nebzučel

    GPIO_Init(GPIOE, GPIO_PIN_4,GPIO_MODE_IN_FL_NO_IT); // nastavíme PE4 jako vstup (tlačítko)

    GPIO_Init(GPIOC, GPIO_PIN_1,GPIO_MODE_IN_PU_NO_IT); // nastavení pinů pro nkoder
    GPIO_Init(GPIOC, GPIO_PIN_2,GPIO_MODE_IN_PU_NO_IT);     
    
    TIM1_DeInit();
    TIM1_TimeBaseInit(8, TIM1_COUNTERMODE_UP, 60, 8); //inicializace enkoderu
    TIM1_EncoderInterfaceConfig(TIM1_ENCODERMODE_TI12,
                                 TIM1_ICPOLARITY_FALLING,
                                 TIM1_ICPOLARITY_FALLING);
   TIM1_Cmd(ENABLE);

}

void delay_ms(uint16_t ms) {  //bzučák
    uint16_t  i;
    for (i=0; i<ms; i = i+1){
        _delay_us(250);
        _delay_us(248);
        _delay_us(250);
        _delay_us(250);
    }
}


void casovac(uint32_t cas){ //funkce casovac porovnává hodnoty milisu a milisu pri spusteni funkce jestli neprekrocil jejich rozdil uz cas zadany z enkoderu
    uint8_t temp = 1;
    uint32_t time2 = milis();
    uint8_t zbyvajic = 0;
    char text[32];
    while(temp){
        if(milis() - time2 > cas * 1000){ //pokud uz cas dojel tak se spusti na 1s bzucak
            GPIO_WriteLow(GPIOD,GPIO_PIN_6);
            delay_ms(1000);
            GPIO_WriteHigh(GPIOD,GPIO_PIN_6);
            temp = 0;
        }
        else{ //pokud stale nedojel cas tak se updatne zbyvajici cas
            zbyvajic = ((time2 + (cas * 1000)) - milis())/1000;
            lcd_gotoxy(12, 1);
            sprintf(text,"%1u ",zbyvajic);
            lcd_puts(text);
        }
    }

}

int main(void)
{

    unsigned int present_value = 0x0000; //proměnné pro enkoder
    unsigned int previous_value = 0x0001; //proměnné pro enkoder


    uint8_t timer = 0; // pokud se stiskne tlačítko spustí se loop s časovačem
    uint32_t time = 0; //inicializace proměnných pro výpočet časů

    char text1[32];
   
    setup();


    while (1) {
        if(GPIO_ReadInputPin(GPIOE,GPIO_PIN_4)==RESET){ //při stisknutí tlačítka se do time ulozi hodnota na enkoderu a spusti se timer
            timer = 1;
            time = present_value; //aktuální hodnota encoderu
        }

        if(timer == 1){ //pri spusteni se dá hodnota enkoderu do funkce casovac
            casovac(time);
            timer = 0;
        }

        present_value = TIM1_GetCounter();      
        if(present_value != previous_value)   // vypisování čísel při otáčení encoderem
        {
            lcd_gotoxy(12, 0);
            sprintf(text1,"%1u ",present_value);
            lcd_puts(text1);
        }
        previous_value = present_value;

    }
}

/*-------------------------------  Assert -----------------------------------*/
#include "__assert__.h"
