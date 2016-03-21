#include<avr/io.h>
#include<util/delay.h>
#include"lcd.h" 
#include<avr/interrupt.h>
#include"maze.h"
#include"func.h"
#include"solve.h"
//#include"uart.h"

unsigned char rec=0;
unsigned int flag=0;

final_show()
{
int tm=0;
reach();

while(1)
{
      if ((PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	   
	   rec=1;
	   flag=1;
	   break;
	  }
	  
	  else if((!(PINA&(1<<6))) && (PINA&(1<<7)))
	  {
	  
	   rec=2;
	   break;
	  }
	  
	  else if((PINA&(1<<6)) && (!(PINA&(1<<7))))
	  {
	   
       rec=2;
	   break;	
      }  

}
inch();

while(tm<4000)
       {
	   PORTB |= (1<<0);
       PORTB &=~(1<<1);// holdin object
	   _delay_us(50);
	   PORTB |= (1<<0);
       PORTB |= (1<<1);
	   _delay_us(50);
	   tm++;
	   PORTC|=(1<<3);

	   }
	   tm=0;
	   PORTC&=~(1<<3);
	   if ((PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	  rec=1;
	  flag=1;
	  }
	  else
	  {
	  rec=2;
	  }
inch();


		//turn('U');
		
		
	   
place_obj(rec);
turn('U');
inchb();
inchb();
inchb();

turn('N');

while(1)
{
      if ((PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	  if(flag==0) 
	   rec=1;
	  else
	   rec=3;
	   break;
	  }
	  
	  else if(!(PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	  
	   rec=2;
	   break;
	  }
	  
	  else if((PINA&(1<<6)) && !(PINA&(1<<7)))
	  {
	   
       rec=2;
	   break;	
      }  

}
inch();
 
while(tm<4000)
       {
	   PORTB |= (1<<0);
       PORTB &=~(1<<1);// holdin object
	   _delay_us(50);
	   PORTB |= (1<<0);
       PORTB |= (1<<1);
	   _delay_us(50);
	   tm++;
	   PORTC|=(1<<3);

	   }
	   tm=0;
	   PORTC&=~(1<<3);
	   if ((PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	  if(flag==1)
	  rec=3;
	  else
	  {
	  flag=1;
	  rec=1;
	  }
	  }
	  else
	  {
	  rec=2;
	  }

	   turn('N');
linefollower();
turn('N');
inch();

		//turn('U');


place_obj(rec);
turn('U');
inchb();
inchb();
inchb();

turn('N');


while(1)
{
      if ((PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	  if(flag==0) 
	   rec=1;
	  else
	   rec=3;
	   break;
	  }
	  
	  else if(!(PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	  
	   rec=2;
	   break;
	  }
	  
	  else if((PINA&(1<<6)) && !(PINA&(1<<7)))
	  {
	   
       rec=2;
	   break;	
      }  

}
inch();

while(tm<4000)
       {
	   PORTB |= (1<<0);
       PORTB &=~(1<<1);// holdin object
	   _delay_us(50);
	   PORTB |= (1<<0);
       PORTB |= (1<<1);
	   _delay_us(50);
	   tm++;
	   PORTC|=(1<<3);

	   }
	   tm=0;
	   PORTC&=~(1<<3);
	   if ((PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	  if(flag==1)
	  rec=3;
	  else
	  {
	  flag=1;
	  rec=1;
	  }
	  }
	  else
	  {
	  rec=2;
	  }

	   turn('N');
linefollower();
turn('N');
inch();

		//turn('U');


place_obj(rec);
turn('U');
inchb();
inchb();
inchb();

turn('N');
}

int main(void)
{
   DDRA = 0x00;
   PORTA = 0xFF;
   DDRD |=(1<<4)|(1<<5); //initialising PORTS of motors
   DDRD |=(1<<6)|(1<<7);
   PORTD |= (1<<4);
   PORTD &=~(1<<5);
   PORTD |= (1<<6);
   PORTD &=~(1<<7);
   DDRB |=(1<<0)|(1<<1); // lifting motor initialization
	 
   DDRC |=(1<<3); // initialising port of buzzer .

   PORTB |= (1<<5); // activating pullups for switch
	 
	    lcd_init();
		//initialize();
		//sei();
		//UCSRB|=(1<<RXCIE);
        PORTC|= (1<<1);  // initialising lcd
        lcd_gotoxy1('0');
	int ch;
	
  // PORTB |=(1<<5);
	initmatrix();
   while(1)
  {
 
   unsigned char c=0 ;
   
	timer_init();  //////
	/*lcd_gotoxy1(0);
	lcd_string("x:");
	lcd_showvalue1(currx);
	lcd_string("y:");
	lcd_showvalue1(curry);
	lcd_string("o:");
	lcd_showvalue1(ori);

	*/
   linefollower();
    
	lcd_gotoxy2(0);
    lcd_string("B:");
	ch=bridgeflag;
	lcd_showvalue1(ch);
	lcd_string("T:");
	ch=bridgetime;
	lcd_showvalue1(ch);
	lcd_string("f:");
	ch=f2;
	lcd_showvalue1(ch);
	
   PORTC &=~(1<<3);
   updatemaze();
   // lcd_string("D:");
   c = decidemaze();
   //lcd_char(c);
   mazestore();   //////
    lcd_gotoxy1(0);
	lcd_string("x:");
	lcd_showvalue1(currx);
	lcd_string("y:");
	lcd_showvalue1(curry);
  /* lcd_gotoxy1(0);
  
	lcd_string("x:");
	ch=a[currx][curry][1];
	lcd_showvalue1(ch);
	//lcd_char(' ');
	lcd_string("y:");
	ch=a[currx][curry][2];
	lcd_showvalue1(ch);*/
//lcd_gotoxy2(0);
	 lcd_string("n:");
	ch=a[currx][curry][0];
	lcd_showvalue(ch);
	//lcd_char(' ');
	
   
   inch();   
   turn(c);
	
	//if(!(PINA & (1<<1))||!(PINA & (1<<5)))  // flag = 0 as soon as it crosses the intersection
	//f2=0;
	
	if(c=='N')
	break;
   
   }
   lcd_cmd(0x01);

//conventionmatch();
simplify();

_delay_ms(15000);

while(1)
{
      if ((PINA&(1<<6)) && (PINA&(1<<7)))
	  {
	   
	   
	   break;
	  }
	  
	  if((!(PINA&(1<<6))) && (PINA&(1<<7)))
	  {
	  
	   
	   break;
	  }
	  
		if((PINA&(1<<6)) && (!(PINA&(1<<7))))
	  {
	   
       
	   break;	
      }  
}
while(1)
final_show();


}

reach()
{  
  int p=0;
  while(1)
   {
   
   linefollower();
   
   if(f2==0)
	{
		p++;
		f2=1;
    }
	 if(p==2)
	 break;
	}
	inch();
	turn('R');
	inchb();
	//inchb();
	inchb();
	turn('N');
	
  
}


/////////timer INTERUPT,,midpoint count 32

ISR(TIMER0_OVF_vect)
{
time++;
}