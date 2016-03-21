void simplify(void);
int near(int,int);
void findpath(void);
void place_obj(int);
void simplify()
{
// pure c code attached here
int j,pilot=0;
ori_m=3,x=8,y=6;
for(i=1;i<9;i++)
{
	for(j=1;j<9;j++)
	{
	if(a[i-1][j-1][0]==100)
	{
		test[i][j]=100;
	}
	else
	{
		test[i][j]=maze[i-1][j-1];
	}
	
	}
}
	for(i=1;i<9;i++)
	{
		count++;
	 	for(j=1;j<9;j++)
	 	{	
	 		count++;
	 		update1=near(i,j);
	 		if(update1!=maze[i-1][j-1])
	 		{
	 			maze[i-1][j-1]=update1;
	 			i=0;
	 			break;
	 		}
	 	}
	 }
//printf("%d\n",count);
//display();
while(test[x][y]!=0)
{
findpath();
}
_delay_ms(10000);
lcd_gotoxy1(0);
for(i=0;i<16;i++)
{
	lcd_char(path[i]);
}

//place_obj(2);

for(i=0;i<25;i++)
path_centre[i]=path[i];

for(i=0;i<8;i++)
{
	for(j=0;j<8;j++)
	{
	
	maze[i][j]=pilot;
	pilot++;
	
	}
pilot=i+1;
}
 /*maze[8][8]=
		{
             {0,1,2,3,4,5,6,7},
			 {1,2,3,4,5,6,7,8},
			 {2,3,4,5,6,7,8,9},
			 {3,4,5,6,7,8,9,10},
			 {4,5,6,7,8,9,10,11},
			 {5,6,7,8,9,10,11,12},
			 {6,7,8,9,10,11,12,13},
			 {7,8,9,10,11,12,13,14},
        };*/
for(i=1;i<9;i++)
{
	for(j=1;j<9;j++)
	{
	if(a[i-1][j-1][0]==100)
	{
		test[i][j]=100;
	}
	else
	{
		test[i][j]=maze[i-1][j-1];
	}
	
	}
}
//for(step=0;step<pointer;step++)
//printf("%c\t",path[step]);
ori_m=3,x=8,y=6;


	for(i=1;i<9;i++)
	{
		count++;
	 	for(j=1;j<9;j++)
	 	{	
	 		count++;
	 		update1=near(i,j);
	 		if(update1!=maze[i-1][j-1])
	 		{
	 			maze[i-1][j-1]=update1;
	 			i=0;
	 			break;
	 		}
	 	}
	 }
	pointer=0;
while(test[x][y]!=0)
{
findpath();
}

_delay_ms(10000);
lcd_gotoxy1(0);
for(i=0;i<16;i++)
{
	lcd_char(path[i]);
}

for(i=0;i<25;i++)
path_end_left[i]=path[i];
pilot=0;

for(i=0;i<8;i++)
{
	for(j=7;j>=0;j--)
	{
	
	maze[i][j]=pilot;
	pilot++;
	
	}
pilot=i+1;
}

/*maze[8][8]={
             {7,6,5,4,3,2,1,0},
			 {8,7,6,5,4,3,2,1},
			 {9,8,7,6,5,4,3,2},
			 {10,9,8,7,6,5,4,3},
			 {11,10,9,8,7,6,5,4},
			 {12,11,10,9,8,7,6,5},
			 {13,12,11,10,9,8,7,6},
			 {14,13,12,11,10,9,8,7}
           };*/

ori_m=3,x=8,y=6;
for(i=1;i<9;i++)
{
	for(j=1;j<9;j++)
	{
	if(a[i-1][j-1][0]==100)
	{
		test[i][j]=100;
	}
	else
	{
		test[i][j]=maze[i-1][j-1];
	}
	
	}
}

	for(i=1;i<9;i++)
	{
		count++;
	 	for(j=1;j<9;j++)
	 	{	
	 		count++;
	 		update1=near(i,j);
	 		if(update1!=maze[i-1][j-1])
	 		{
	 			maze[i-1][j-1]=update1;
	 			i=0;
	 			break;
	 		}
	 	}
	 }
	 pointer=0;
while(test[x][y]!=0)
{
findpath();
}

_delay_ms(10000);
lcd_gotoxy1(0);
for(i=0;i<16;i++)
{
	lcd_char(path[i]);
}

for(i=0;i<25;i++)
path_end_right[i]=path[i];


//lcd_gotoxy1(0);
/*for(i=0;i<16;i++)
{
	lcd_char(path[i]);
}*/
//
//place_obj(2);
}

	

int near(int i,int j)
{
if(test[i][j]==100)
{	
	//lcd_char('Q');
	//_delay_ms(100);
	return 100;
}
else if(test[i][j]==30)
	{
	//lcd_char('R');
	//_delay_ms(100);
		test[i][j]=100;
		return 100;
	}
else if(test[i][j]==0)
	{
	//lcd_char('D');
	//_delay_ms(100);
	return 0;
	}
else 
{ 	
	//lcd_char('F');
	//_delay_ms(100);
	int a,b,c,d,e,min;
	a=test[i][j-1];
	b=test[i][j+1];
	c=test[i-1][j];
	d=test[i+1][j];
	e=test[bridge[i][j][0]][bridge[i][j][1]];
		min=a;
		if(b<a)
		min=b;
		if(c<min)
		min=c;
		if(d<min)
		min=d;
		if(e<min)
		min=e;
	if(min==100)
	{
		test[i][j]=100;
		return 100;
	}
	else
	{
		test[i][j]=min+1;
		return min+1;
	}
}
}
void findpath(void)
{
unsigned char cc;
/*lcd_gotoxy1(0);
lcd_string("X:");
lcd_showvalue1(x);
lcd_string("Y:");
lcd_showvalue1(y);
lcd_string("Y:");
cc=y;
lcd_showvalue(test[x][y]);
_delay_ms(900);*/
int a,b,c,d,e,min,bx,by;
	a=test[x][y-1];
	b=test[x][y+1];
	c=test[x-1][y];
	d=test[x+1][y];
	e=test[bridge[x][y][0]][bridge[x][y][1]];
		min=a;
			if(ori_m==1)
			path[pointer]='R';
			if(ori_m==2)
			path[pointer]='S';
			if(ori_m==3)
			path[pointer]='L';
			if(ori_m==4)
			path[pointer]='U';
		
		if(b<a)
		{
			min=b;
			if(ori_m==1)
			path[pointer]='L';
			if(ori_m==2)
			path[pointer]='U';
			if(ori_m==3)
			path[pointer]='R';
			if(ori_m==4)
			path[pointer]='S';
		}
		
		if(c<min)
		{
			min=c;
			if(ori_m==1)
			path[pointer]='U';
			if(ori_m==2)
			path[pointer]='R';
			if(ori_m==3)
			path[pointer]='S';
			if(ori_m==4)
			path[pointer]='L';
		}
		if(d<min)
		{
			min=d;
			if(ori_m==1)
			path[pointer]='S';
			if(ori_m==2)
			path[pointer]='L';
			if(ori_m==3)
			path[pointer]='U';
			if(ori_m==4)
			path[pointer]='R';
		}
		if(e<min)
		{
			min=e;
			bx=bridge[x][y][0]-x;
			by=bridge[x][y][1]-y;
			if(bx==1 && by==1)
			{
				if(bridge[x][y][2]==1)
				{
					if(ori_m==1)
					{
						path[pointer]='L';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==2)
					{
						path[pointer]='U';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==3)
					{
						path[pointer]='R';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==4)
					{
						path[pointer]='S';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					ori_m=1;
				}
				if(bridge[x][y][2]==2)
				{
					if(ori_m==1)
					{
						path[pointer]='S';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==2)
					{
						path[pointer]='L';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==3)
					{
						path[pointer]='U';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==4)
					{
						path[pointer]='R';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					ori_m=4;
				}
				x=x+1;
				y=y+1;
			}
			if(bx==1 && by==-1)
			{
				if(bridge[x][y][2]==1)
				{
					if(ori_m==1)
					{
						path[pointer]='R';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==2)
					{
						path[pointer]='S';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==3)
					{
						path[pointer]='L';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==4)
					{
						path[pointer]='U';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					ori_m=1;
				}
				if(bridge[x][y][2]==2)
				{
					if(ori_m==1)
					{
						path[pointer]='S';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==2)
					{
						path[pointer]='L';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==3)
					{
						path[pointer]='U';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==4)
					{
						path[pointer]='R';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					ori_m=2;
				}
				x=x+1;
				y=y-1;
			}
			if(bx==-1 && by==1)
			{
				if(bridge[x][y][2]==1)
				{
					if(ori_m==1)
					{
						path[pointer]='L';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==2)
					{
						path[pointer]='U';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==3)
					{
						path[pointer]='R';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==4)
					{
						path[pointer]='S';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					ori_m=3;
				}
				if(bridge[x][y][2]==2)
				{
					if(ori_m==1)
					{
						path[pointer]='U';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==2)
					{
						path[pointer]='R';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==3)
					{
						path[pointer]='S';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==4)
					{
						path[pointer]='L';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					ori_m=4;
				}
				x=x-1;
				y=y+1;
			}	
			if(bx==-1 && by==-1)
			{
				if(bridge[x][y][2]==1)
				{
					if(ori_m==1)
					{
						path[pointer]='R';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==2)
					{
						path[pointer]='S';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==3)
					{
						path[pointer]='L';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					if(ori_m==4)
					{
						path[pointer]='U';
						path[pointer+1]='R';
						path[pointer+2]='R';
					}
					ori_m=3;
				}
				if(bridge[x][y][2]==2)
				{
					if(ori_m==1)
					{
						path[pointer]='U';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==2)
					{
						path[pointer]='R';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==3)
					{
						path[pointer]='S';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					if(ori_m==4)
					{
						path[pointer]='L';
						path[pointer+1]='L';
						path[pointer+2]='L';
					}
					ori_m=2;
				}
				x=x-1;
				y=y-1;
			}
			pointer=pointer+3;
		}
		if(a==min)
		{
			ori_m=2;	
			pointer++;
			y=y-1;
		}
		else if(b==min)
		{
			ori_m=4;
			pointer++;
			y=y+1;
		}
		else if(c==min)
		{
			ori_m=3;
			pointer++;
			x=x-1;
		}
		else if(d==min)
		{
			ori_m=1;
			pointer++;
			x=x+1;
		}
	/*lcd_gotoxy2(0);
lcd_string("near");

lcd_showvalue1(min);*/	
}

void place_obj(int type)
{
signed int pt=0,end=0;
char ch;

if(type==2)
{
	for(pt=0;pt<25;pt++)
	{
		ch=path_centre[pt];
		inch();
		turn(ch);
		if(ch=='N')
		break;
		linefollower();
		PORTD ^=(1<<1);
	}
end=pt;
	for(pt=0;pt<25;pt++)
	{
		if(path_centre[pt]=='L')
		path_centre[pt]='R';
		else if(path_centre[pt]=='R')
		path_centre[pt]='L';
	}
turn('N');
inch();
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
turn('U');
	for(pt=end-1;pt>=0;pt--)
	{
		linefollower();
		ch=path_centre[pt];
		inch();
		turn(ch);
		PORTD ^=(1<<1);
	}
turn('N');
}
else if(type==3)
{
	for(pt=0;pt<25;pt++)
	{
		ch=path_end_right[pt];
		inch();
		turn(ch);
		if(ch=='N')
		break;
 		linefollower();
		PORTD ^=(1<<1);
	}
end=pt;
	for(pt=0;pt<25;pt++)
	{
		if(path_end_right[pt]=='L')
		path_end_right[pt]='R';
		else if(path_end_right[pt]=='R')
		path_end_right[pt]='L';
	}
turn('N');
inch();
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
turn('U');
	for(pt=end-1;pt>=0;pt--)
	{
		linefollower();
		ch=path_end_right[pt];
		inch();
		turn(ch);
		PORTD ^=(1<<1);
	}
turn('N');
}
else if(type==1)
{
	for(pt=0;pt<25;pt++)
	{
		ch=path_end_left[pt];
		inch();
		turn(ch);
		if(ch=='N')
		break;
		linefollower();
		PORTD ^=(1<<1);
	}
end=pt;
	for(pt=0;pt<25;pt++)
	{
		if(path_end_left[pt]=='L')
		path_end_left[pt]='R';
		else if(path_end_left[pt]=='R')
		path_end_left[pt]='L';
	}
turn('N');
inch();
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
turn('U');
	for(pt=end-1;pt>=0;pt--)
	{
		linefollower();
		ch=path_end_left[pt];
		inch();
		turn(ch);
		PORTD ^=(1<<1);
	}
turn('N');
}


}