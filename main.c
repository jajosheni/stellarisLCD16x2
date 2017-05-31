#include "inc/lm4f120h5qr.h"
#include "inc/lm1602/pll.h"
#include <stdio.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

//! @ AUTHOR : SHENI HAMITAJ    

//================================================//
//==============systick==========================//

unsigned long systick_count = 0;

void systick_init(void)
{
  NVIC_ST_CTRL_R = 0;
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;
  NVIC_ST_CURRENT_R = 0;

  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE + NVIC_ST_CTRL_CLK_SRC;
}

void systick_delay(unsigned long delay)
{
  volatile unsigned long remaining;

  for(remaining=0; remaining<delay; remaining++)
  {
	  /* busy waiting */
  }

}


void systick_delay_ms(unsigned long delay)
{
  unsigned long i;

  for(i=0; i<delay; i++)
    systick_delay(5000);  // 1mS (on 50MHz)
}


unsigned long M;

void Random_Init(unsigned long seed){
  M = seed;
}


unsigned long Random(void){
  M = 1664525*M+1013904223;
  return(M);
}

//==============systick==========================//
//================================================//

//================================================//
//==============display===========================//

/*
 * RS	 PB0
 * EN	 PB1
 *
 * D4	 PD0
 * D5	 PD1
 * D6	 PD2
 * D7	 PD3
*/

#define delay_ms(x)     systick_delay_ms(x)

#define LCD_DATA        GPIO_PORTD_DATA_R
#define LCD_CONTROL     GPIO_PORTB_DATA_R
#define	EN_RESET()     LCD_EN(1),LCD_EN(0)

#define	LCD_RS(x)       ( (x) ? (LCD_CONTROL |= 0x01) : (LCD_CONTROL &= ~0x01) )
#define LCD_EN(x)       ( (x) ? (LCD_CONTROL |= 0x02) : (LCD_CONTROL &= ~0x02) )

void lcd_putch(char c)
{
  LCD_RS(1);

  delay_ms(2);

  LCD_DATA = ((c & 0xF0) >> 4);
  EN_RESET();
  LCD_DATA =  (c & 0x0F);
  EN_RESET();
}

void lcd_command(unsigned char c)
{
  LCD_RS(0);

  delay_ms(2);

  LCD_DATA = ((c & 0xF0) >> 4);
  EN_RESET();
  LCD_DATA =  (c & 0x0F);
  EN_RESET();
}

void lcd_reset(void)
{
  lcd_command(0x01);
  delay_ms(10);
}

void lcd_puts(const char* s)
{
  while(*s)
    lcd_putch(*s++);
}

void lcd_goto(char x, char y)
{
  if(x==1)
    lcd_command(0x80+((y-1)%16));
  else
    lcd_command(0xC0+((y-1)%16));
}

void lcd_init()
{
  SYSCTL_RCGC2_R     |=  SYSCTL_RCGC2_GPIOB;	// GPIOB Active 0-1
  GPIO_PORTB_DIR_R   |=  0x03;
  GPIO_PORTB_AFSEL_R &= ~0x03;
  GPIO_PORTB_DEN_R   |=  0x03;
  GPIO_PORTB_PCTL_R   =  0xFFFFFF00;
  GPIO_PORTB_AMSEL_R &=  0x03;

  SYSCTL_RCGC2_R     |=  SYSCTL_RCGC2_GPIOD;	// GPIOD Active 0-3
  GPIO_PORTD_DIR_R   |=  0x0F;
  GPIO_PORTD_AFSEL_R &= ~0x0F;
  GPIO_PORTD_DEN_R   |=  0x0F;
  GPIO_PORTD_PCTL_R  &=  0xFFFF0000;
  GPIO_PORTD_AMSEL_R &= ~0x0F;

  SYSCTL_RCGC2_R     |=  SYSCTL_RCGC2_GPIOC;	// GPIOC Active 4-7
  GPIO_PORTC_DIR_R   |=  0xF0;
  GPIO_PORTC_AFSEL_R &= ~0xF0;
  GPIO_PORTC_DEN_R   |=  0xF0;
  GPIO_PORTC_PCTL_R  &=  0x0000FFFF;
  GPIO_PORTC_AMSEL_R &= ~0xF0;

  LCD_RS(0);
  LCD_EN(0);

  lcd_command(0x28);  // 4 Bit , Double row LCD
  lcd_command(0x0C);  // Hide cursor
  lcd_command(0x06);  // Right Direction
  lcd_command(0x80);  // First LCD location
  lcd_command(0x28);  // 4 Bit , Double row LCD
  lcd_reset();    // Reset LCD
}

void lcd_opening(void)
{
	int flicker[25]={1,0,1,0,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,0};
	int f;

	for(f=0;f<32;f++)
	{
		if (flicker[f]==0)
		{
			GPIO_PORTC_DATA_R &= ~0xFF;
			lcd_reset();
			delay_ms(60);
		}
		if(flicker[f]==1)
		{
			lcd_reset();
			GPIO_PORTC_DATA_R |= 0xFF;
			lcd_puts("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
			lcd_goto(2,1);
			lcd_puts("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
			delay_ms(35);
		}
	}

		GPIO_PORTC_DATA_R &= ~0xFF;
		lcd_reset();
		lcd_puts("     System");

			GPIO_PORTC_DATA_R |= 0x10;
			delay_ms(500);
			GPIO_PORTC_DATA_R |= 0x20;
			delay_ms(500);
			GPIO_PORTC_DATA_R |= 0x40;
			delay_ms(500);
			GPIO_PORTC_DATA_R |= 0x80;
			lcd_goto(2,1);
			lcd_puts("   Programming");
}


//==============display==========================//
//================================================//

/*
 * Kırmızı yanarken	-	Ust satırda "kocaeli uni"
 * 2 saniye sonra	*	alt satırda "ad soyad"
 *
 * Beyaz yanarken	-	alt satırda "ad soyad"
 * 2 saniye sonra	*	ust satırda "kocaeli uni"
 *
 * Yesil yanarken	-	kırmızı gibi saga kaydırarak
 *
 * Mavi yanarken	-	beyaz gibi sola kaydırarak
 *
 * her 3 saniye rasgele degisecek
 */


//================================================//
//===================Programmed======================//

unsigned char rand_n;

unsigned long cpu_freq;
char kocaeli[12]="Kocaeli Uni";
char ad[14]="Sheme Hamitaj";
char kaydirma[33]="";

void red()
{
	GPIO_PORTC_DATA_R &= ~0xFF;
	GPIO_PORTC_DATA_R |= 0x80;
	lcd_reset();
	delay_ms(500);
	lcd_goto(1,1);
	lcd_puts(kocaeli);
	lcd_goto(2,1);
	delay_ms(2000);
	lcd_puts(ad);
}

void white()
{
	GPIO_PORTC_DATA_R &= ~0xFF;
	GPIO_PORTC_DATA_R |= 0x10;
	lcd_reset();
	delay_ms(500);
	lcd_goto(2,1);
	lcd_puts(ad);
	lcd_goto(1,1);
	delay_ms(2000);
	lcd_puts(kocaeli);
}

void green()
{
	GPIO_PORTC_DATA_R &= ~0xFF;
	GPIO_PORTC_DATA_R |= 0x20;
	lcd_reset();
	delay_ms(500);
	lcd_goto(1,1);

    int j=0,flag=0,i=0;

	for(i=0;i<33;i++)
		kaydirma[i]=' ';

    for(i=0;i<27;i++)
    {
        lcd_reset();
        if(kocaeli[i]=='\0')
            kaydirma[0]=' ', flag=1;
        else
        {
            if(flag==1)
            {
                /*do nothing as the text is over*/
            }else
                kaydirma[0]=kocaeli[10-i];
        }


        lcd_puts(kaydirma);
        for(j=32;j>-1;j--)
        {
            kaydirma[j+1]=kaydirma[j];
        }
        delay_ms(200);
    }

    j=0,flag=0;
    delay_ms(2000);
	lcd_goto(2,1);

	for(i=0;i<33;i++)
		kaydirma[i]=' ';

    for(i=0;i<32;i++)
    {
        lcd_reset();
        lcd_goto(2,1);
        if(ad[i]=='\0')
            kaydirma[0]=' ', flag=1;
        else
        {
            if(flag==1)
            {
                /*do nothing as the text is over*/
            }else
                kaydirma[0]=ad[12-i];
        }


        lcd_puts(kaydirma);
        for(j=32;j>-1;j--)
        {
            kaydirma[j+1]=kaydirma[j];
        }
        delay_ms(200);
    }
    j=0,flag=0;
}

void blue()
{
	GPIO_PORTC_DATA_R &= ~0xFF;
	GPIO_PORTC_DATA_R |= 0x40;
	lcd_reset();
	delay_ms(500);
	lcd_goto(2,1);

	int i=0,j=15,z=0,y=0,flag=0;

	for(i=0;i<33;i++)
		kaydirma[i]=' ';

	for(z=0;z<29;z++)
			    {
			         for(i=0;i<j;i++)
			        {
			            kaydirma[i]=' ';
			        }
			        if(ad[z]==0)
			            flag=1;
			        if(flag!=1)
			        kaydirma[15]=ad[z];
			        lcd_reset();
			        lcd_goto(2,1);
			        lcd_puts(kaydirma);
			        delay_ms(200);
			        j--;
			        for(y=0;y<31;y++)
			        {
			            kaydirma[y]=kaydirma[y+1];
			        }
			    }

	i=0,j=15,z=0,y=0,flag=0;
	lcd_goto(1,1);
	delay_ms(2000);

	for(i=0;i<33;i++)
		kaydirma[i]=' ';

    for(z=0;z<27;z++)
    {
         for(i=0;i<j;i++)
        {
            kaydirma[i]=' ';
        }
        if(kocaeli[z]==0)
            flag=1;
        if(flag!=1)
        kaydirma[15]=kocaeli[z];
        lcd_reset();
        lcd_puts(kaydirma);
        delay_ms(200);
        j--;
        for(y=0;y<31;y++)
        {
            kaydirma[y]=kaydirma[y+1];
        }
    }

	i=0,j=15,z=0,y=0,flag=0;
}


//===================Programmed======================//
//================================================//

int main(void)
{
	pll_init();
	systick_init();

	set_cpu_freq(3,1);
	cpu_freq = get_cpu_freq();

	int seed = 123468;
	int* pt = &seed;
	lcd_init();
	delay_ms(1000);
	lcd_opening();
	delay_ms(500);

	Random_Init(pt);

  while(1)
  {
	  rand_n = (Random()/10000000)%4;

	  if(rand_n==0){
		  delay_ms(3000);
		  white();
	  }

	  if(rand_n==1){
		  delay_ms(3000);
		  green();
	  }

	  if(rand_n==2){
		  delay_ms(3000);
		  blue();
	  }

	  if(rand_n==3){
		  delay_ms(3000);
		  red();
	  }

  }

}
