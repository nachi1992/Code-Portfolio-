#include<avr/io.h>
#include<util/delay.h>
//#include"lcd.h"
#include"LCD_mega16.h" 
#include"func1withoutadc.h"
//#include"adc.h"
void takecareofobject(void);
void objtransport(void);
void retreat(void);

unsigned int objx=0,objy=0;
char c,c1 ;

int main(void)
{
   DDRD |=(1<<4)|(1<<5); //initialising PORTS of motors
   DDRD |=(1<<6)|(1<<7);
   DDRB |=(1<<0)|(1<<1); // lifting motor initialization
	 
   DDRC |=(1<<3);// initialising port of buzzer .
   DDRD |=(1<<1); 

   PORTB |= (1<<5); // activating pullups for switch
	 
	    lcd_init();
        PORTC|= (1<<1);  // initialising lcd
        lcd_gotoxy1(0);
	    

	//init();             //adc initialization    
	
   while(1)
  {
    
   c1=0;  
	lcd_gotoxy1(0);
	lcd_string("x:");
	lcd_showvalue1(currx);
	lcd_string("y:");
	lcd_showvalue1(curry);
	lcd_string("o:");
	lcd_showvalue1(ori);
	
   linefollower();
   
   
   update();
   
   takecareofobject();
     
   c1 = decide();
       
	   inch();
	   
	   
    turn(c1);
	
	
	lcd_gotoxy2(0);
	
	lcd_char(c1);
	lcd_string("ox : ");
	lcd_showvalue1(objx);
	lcd_string("oy : ");
	lcd_showvalue1(objy);
	
	//if((PINA & (1<<1))||(PINA & (1<<5)))
	//f2=0;
	if(c1=='N')
	break;
   
   }

}



void takecareofobject()
{
  if ((blkiden==0)||(f1==0)) // OBJECT NOT THERE
  return;
  if (blkiden==1||blkiden==2) // OBJECT FOUND
  { 
  int i=0;
  turn('N');
  //_delay_ms(10);
  while(i<4000)
       {
	   PORTB |= (1<<0);
       PORTB &=~(1<<1);// holdin object
	   _delay_us(50);
	   PORTB |= (1<<0);
       PORTB |= (1<<1);
	   _delay_us(50);
	   i++;
	   PORTC|=(1<<3);

	   }
	   PORTC&=~(1<<3);
     linefollower();
	   
	   objx=currx;
       objy=curry; 
       
	   objtransport();
	   
	   turn('N');
	   
	  linefollower1();
	 
       turn('N');
      
       inchb();
	   inchb();
        i=0;
	  while(i<4000) 
	   {
	   PORTB &=~ (1<<0);
       PORTB |=(1<<1);// keepin object
	   _delay_us(85);
	   PORTB |= (1<<0);
       PORTB |= (1<<1);
	   _delay_us(15);
	   i++;
	   PORTC|=(1<<3);

	   }
	   PORTC&=~(1<<3);
	  inchb();
	  //inchb();
	   
	   turn('R');
	   if(ori==1)
		ori=3;
		else if(ori==2)
	    ori=4;  
	    else if(ori==3)
	    ori=1;
	    else if(ori==4)
	    ori=2;
	  
	   
	   
	  linefollower1();
	   
	   turn('N');
	   //inch();
       retreat();
	   blkiden=0;
	   f1=0;
	  
  
  }
  

}

void objtransport()
{
	inch();
    int tx,ty;
	
	tx= objx ;
	ty= objy ;
	
	if(tx==0 && ty == 1)
    return ; 
	
	
	if (ori==1&&ty==0)
	{ c ='U';
	  turn(c);
	}
	else if (ori==1)
	{ c ='L';
	  turn(c);
	}
	else if (ori==3)
    {  c ='R';
	  turn(c);
	}
    else if (ori==2)
	{ char c ='U';
	  turn(c);
	}

while(1)	
 {	
   linefollower();
   if(f2==0)
   {  
		if(ori==3)
		tx--;
		else if(ori==4)
		ty--;
		else if(ori==2)
		ty++;
		else if(ori==1)
		tx++;
		f2=1;
	}
	inch();
   if(tx==0 && ty == 1)
   break; 
   
   
   else if (ty==0&&tx>0&&ori==4)
   turn('L');
   
   else if(ty==1&&ori==4)
   turn('L');
   
   else if(tx==0&&ori==3)
   turn('L');  
   
   else if (tx>=0||ty>=1||ty<1)
   turn('S');
   
  }
 // inch();	
  if(ori==2)
  {
	turn('R');
	inch();
  }
  if(ori==4)
  {
	turn('L');
	inch();
  }

}


void retreat()
{
   unsigned int tx=0,ty=1;
   ori=1;
   
  
   if(objx==0&&objy==1)
   {
     turn('R');
	 return;
   }
   else if(objy==0||objy==1)
   turn('L');   
   else if(objx>0&&objy>1)
   turn('S');
   else if (objx==0&&objy>1)
   turn('R');
   
   while(1)
   {
      linefollower();
	  
	  if(f2==0)
	 { 
		if(ori==2)
		ty++;
		else if(ori==1)
		tx++; 
		else if(ori==3)
		tx--;
		else if(ori==4)
		ty--;
	    f2=1;
	  }
		inch();
	     if(tx==objx && ty == objy)
         break; 
   
         else if (ty==0&&tx==0)
         turn('R');
   
         else if((ty==0||ty==1)&&tx==objx)
         turn('R');
    
         else if (tx<objx||ty>objy||ty<objy)
         turn('S');

   }
     
	 switch (c) 
	   {
	     case 'L' : turn('L');
		          break; 
	     case 'R' : turn('R');
	              break;
	     case 'U' : turn('S');
	              break; 
		}         
	 
   
}

//ADC interrupt..............



/*ISR(ADC_vect)
{
	unsigned int data,b;
	b=ADCL;
	data=ADCH;
	data=((data<<8)|b);
	
	if((data < 512) && (blkiden==0) )
	blkiden=1;
	
	else if((data > 512) && (blkiden=1))
	blkiden=2;                            
	
	else
	blkiden=0;
	
	if(channel == 6)
	{
		ADMUX=0b11000111;
		channel=7;
	}
	else
	{
		ADMUX=0b11000110;
		channel=6;
	}	
	
	//ADMUX|=channel;
	ADCSRA|=(1<<ADIF);
}


/*if(channel == 6)
{
init(7);
channel = 7;
}
if(channel == 7)
{
init(6);
channel = 6;
}*/