//to store maze :)
void initmatrix(void);
void updatemaze(void);
char decidemaze(void);
void timer_init(void);
void conventionmatch(void);

unsigned int currx=0,curry=0,ori=1,f1=0,f2=0,bridge_detect=0,no=0;


unsigned int
	objx,
	objy,
	a[8][8][4],
	test[10][10],
	bridge[10][10][3],
	nodeflag=0,
	bridgeflag=0,
	pointer=0,
	time=0,
	bridgetime=0;

unsigned int maze[8][8]=
	{
		  {6,5,4,3,3,4,5,6},
		  {5,4,3,2,2,3,4,5},
		  {4,3,2,1,1,2,3,4},
		  {3,2,1,0,0,1,2,3},
		  {3,2,1,0,0,1,2,3},
		  {4,3,2,1,1,2,3,4},
		  {5,4,3,2,2,3,4,5},
		  {6,5,4,3,3,4,5,6}
	   };

unsigned char   path[25],
				path_centre[25],
				path_end_left[25],
				path_end_right[25];
unsigned int temp0,it,jt,kt;
int update1,i,count=0,step,ori_m=3,x=8,y=6;


// pls see bridge cases in these two functions

void initmatrix()
{
int i,j,k;

 for(i=0;i<8;i++)
 {
	for(j=0;j<8;j++)
	{
		for(k=1;k<4;k++)
		{
			a[i][j][k]=0;
		
		}
	
	}
 }


	for(i=0;i<8;i++)
	{
		for(j=0;j<8;j++)
		{
		
				a[i][j][0]=maze[i][j];
			
		
		}
	
	}


	for(i=0;i<10;i++)
	{
		for(j=0;j<10;j++)
		{
			for(k=0;k<3;k++)
			{
				bridge[i][j][k]=0;
			
			}
		
		}
	
	}
	for(i=0;i<10;i++)
	{
		for(j=0;j<10;j++)
		{
		
				test[i][j]=100;
			
		
		}
	
	}
	for(i=1;i<9;i++)
	{
		for(j=1;j<9;j++)
		{
		
				test[i][j]=maze[i-1][j-1];
			
		
		}
	
	}
	for(i=0;i<25;i++)
		{
			path[i]='N';
			path_centre[i]='N';
			path_end_left[i]='N';
			path_end_right[i]='N';
		}
		  
	
}

void updatemaze()
{  
   if((bridgeflag==0) && (f2==0))
	{
	PORTB^(1<<4);
		if(ori==1)
		currx++;
		else if(ori==2)
		curry--;
		else if(ori==3)
		currx--;
		else if(ori==4)
		curry++; 
		f2=1;
	}
	else
	return;
}



char decidemaze()
{  
  if(bridgeflag==0)
      {
		if(currx==0 && curry == 7)
		return 'N'; 
		else if(currx==0)
		return 'R';
		else if(currx==7)
		return 'L';  
		else if(currx<7&&currx>0)
		return 'S';
	  }


}



/*void inch(void)
{
	   PORTD |= (1<<4);
       PORTD &=~(1<<5);
       PORTD |= (1<<6);   // going  inch forward for better turn
       PORTD &=~(1<<7);
	   _delay_ms(200);
	   PORTD |= (1<<4);
       PORTD |=(1<<5);
       PORTD |= (1<<6);  
       PORTD |=(1<<7);
}*/

void timer_init(void)
{
	sei();
	time=0;
	TCCR0=0b00000101;
	TIMSK|=(1<<0);
	TCNT0=0x00;
}

void mazestore(void)
{
	if(nodeflag==1)
	{	
		a[currx][curry][0]=100;
		nodeflag=0;
		no++;
		
	}
	
	if(bridgeflag==1)
	{
		if(ori==1)
		{
			a[currx+1][curry][1]=currx;
			a[currx+1][curry][2]=curry+1;
			a[currx+1][curry][3]=1;
			a[currx][curry+1][1]=currx+1;
			a[currx][curry+1][2]=curry;
			a[currx][curry+1][3]=2;
		}
		else if(ori==2)
		{
			a[currx][curry+1][1]=currx-1;
			a[currx][curry+1][2]=curry;
			a[currx][curry+1][3]=2;
			a[currx-1][curry][1]=currx;
			a[currx-1][curry][2]=curry+1;
			a[currx-1][curry][3]=1;
			
		}
			
		else if(ori==3)
		{
			a[currx-1][curry][1]=currx;
			a[currx-1][curry][2]=curry-1;
			a[currx-1][curry][3]=1;
			a[currx][curry-1][1]=currx-1;
			a[currx][curry-1][2]=curry;
			a[currx][curry-1][3]=2;
		}
		
		else if(ori==4)
		{
			a[currx][curry-1][1]=currx+1;
			a[currx][curry-1][2]=curry;
			a[currx][curry-1][3]=2;
			a[currx+1][curry][1]=currx;
			a[currx+1][curry][2]=curry-1;
			a[currx+1][curry][3]=1;
		}
	}
	
	else if(bridgeflag==2)
	{
		if(ori==1)
		{
			a[currx][curry][1]=currx+1;
			a[currx][curry][2]=curry+1;
			a[currx][curry][3]=1;
			a[currx+1][curry+1][1]=currx;
			a[currx+1][curry+1][2]=curry;
			a[currx+1][curry+1][3]=2;
		}
		else if(ori==2)
		{
			
			a[currx][curry][1]=currx-1;
			a[currx][curry][2]=curry+1;
			a[currx][curry][3]=2;
			a[currx-1][curry+1][1]=currx;
			a[currx-1][curry+1][2]=curry;
			a[currx-1][curry+1][3]=1;
		}
			
		else if(ori==3)
		{
			a[currx][curry][1]=currx-1;
			a[currx][curry][2]=curry-1;
			a[currx][curry][3]=1;
			a[currx-1][curry-1][1]=currx;
			a[currx-1][curry-1][2]=curry;
			a[currx-1][curry-1][3]=2;
		}
		
		else if(ori==4)
		{
			a[currx][curry][1]=currx+1;
			a[currx][curry][2]=curry-1;
			a[currx][curry][1]=2;
			a[currx+1][curry-1][1]=currx;
			a[currx+1][curry-1][2]=curry;
			a[currx+1][curry-1][3]=1;
		}
	}
	
	else if(bridgeflag==3)
	{
		if(ori==1)
		{
			a[currx+1][curry][1]=currx;
			a[currx+1][curry][2]=curry-1;
			a[currx+1][curry][3]=1;
			a[currx][curry-1][1]=currx+1;
			a[currx][curry-1][2]=curry;
			a[currx][curry-1][3]=2;
		}
		else if(ori==2)
		{
			a[currx][curry+1][1]=currx+1;
			a[currx][curry+1][2]=curry;
			a[currx][curry+1][3]=2;
			a[currx+1][curry][1]=currx;
			a[currx+1][curry][2]=curry+1;
			a[currx+1][curry][1]=1;
		}
			
		else if(ori==3)
		{
			a[currx-1][curry][1]=currx;
			a[currx-1][curry][2]=curry+1;
			a[currx-1][curry][3]=1;
			a[currx][curry+1][1]=currx-1;
			a[currx][curry+1][2]=curry;
			a[currx][curry+1][3]=2;
		}
		
		else if(ori==4)
		{
			a[currx][curry-1][1]=currx-1;
			a[currx][curry-1][2]=curry;  //situation wont arise
			a[currx][curry-1][3]=2;
			a[currx-1][curry][1]=currx;
			a[currx-1][curry][2]=curry-1;
			a[currx-1][curry][3]=1;
			
		}
	}

	else if(bridgeflag==4)
	{
		if(ori==1)
		{
			a[currx][curry][1]=currx+1;
			a[currx][curry][2]=curry-1;
			a[currx][curry][3]=1;
			a[currx+1][curry-1][1]=currx;
			a[currx+1][curry-1][2]=curry;
			a[currx+1][curry-1][3]=2;
		}
		else if(ori==2)
		{
			a[currx][curry][1]=currx+1;
			a[currx][curry][2]=curry+1;
			a[currx][curry][3]=2;
			a[currx+1][curry+1][1]=currx;
			a[currx+1][curry+1][2]=curry;
			a[currx+1][curry+1][3]=1;
		}
			
		else if(ori==3)
		{
			a[currx][curry][1]=currx-1;
			a[currx][curry][2]=curry+1;
			a[currx][curry][3]=1;
			a[currx-1][curry+1][1]=currx;
			a[currx-1][curry+1][2]=curry;
			a[currx-1][curry+1][3]=2;
		}
		
		else if(ori==4)
		{
			a[currx][curry][1]=currx-1;
			a[currx][curry][2]=curry-1;
			a[currx][curry][3]=2;
			a[currx-1][curry-1][1]=currx;
			a[currx-1][curry-1][2]=curry;
			a[currx-1][curry-1][3]=1;
		}
	}
	bridgeflag=0;
}

void conventionmatch()
{
	/*for(it=0;it<=7;it++)
   {
		for(jt=0;jt<=7;jt++)
		{	
			for(kt=0;kt<=3;kt++)
			{
				temp0=a[it][jt][kt];
				a[it][jt][kt]=a[it][7-jt][kt];
				a[it][7-jt][kt]=temp0;
			}
		}
	}
	
	for(it=0;it<=7;it++)
   {
		for(jt=0;jt<=7;jt++)
		{	
			a[it][jt][2]=7-a[it][jt][2];
		}
	}*/

	for(it=1;it<=8;it++)
	{
		for(jt=1;jt<=8;jt++)
		{
			test[it][jt]=a[it-1][jt-1][0];
			bridge[it][jt][0]=a[it-1][jt-1][1];
			bridge[it][jt][1]=a[it-1][jt-1][2];
			bridge[it][jt][2]=a[it-1][jt-1][3];
		}
	}
}	
			
			
			
			
			
			
			
			
			
			
	
