#include "fuzzy.h"

const int E[5]= {0,2,4,8,12};
const float DE[5]={300,400,550,700,850};
const int KD[5]={1,1,2,3,3};//{2,4,9,13,15}//¸ü¸Ä57
const int KP[5]={40,45,50,55,65};//5->4,9->8
const int P_rule[5][5]=
{
  {0,1,2,2,3},//0d   0,1,2,3,4    //4

  {1,1,2,2,3},//1d   0,1,2,3,4    //5     {3,1,0,0,0,1,3,}

  {1,2,3,3,3},//2d   1,1,3,3,4    //6

  {1,2,3,3,4},//3d   1,2,3,4,4    //7

  {2,3,3,3,4} //4d   2,3,3,4,4    //8
};
const int D_rule[5][5]=
{  
//d0,1,2,3,4,5,6,7,8
  {0,1,1,2,2},//4p    0,1,1,2,2   //4
  
  {0,1,1,2,2},//5p    0,1,1,2,2   //5

  {1,1,2,3,3},//6p    1,1,2,3,3   //6

  {1,2,3,4,4},//7p    1,2,3,3,4   //7

  {2,2,3,4,4},//8p    2,2,3,4,4   //8
};
	
float Fuzzy_control(float Error,float DError,unsigned char P_D)//p:0 D:1
{
	
	unsigned char x1,x2,y1,y2;
	float result;
	int Number[4],UF[4],U_tmp[4];
	
	Error=(Error<0?-Error:Error);
	DError=(DError<0?-DError:DError);
	
	if(Error<=E[1])
    {
     x1=0;
     x2=1;
     Number[1]=100*(Error-E[0])/(E[1]-E[0]);
     Number[0]=100*(E[1]-Error)/(E[1]-E[0]);
    }
    else if(Error<=E[2])
    {
      x1=1;
      x2=2;
      Number[1]=100*(Error-E[1])/(E[2]-E[1]);
      Number[0]=100*(E[2]-Error)/(E[2]-E[1]);
    }
    else if(Error<=E[3])
    {
      x1=2;
      x2=3;
      Number[1]=100*(Error-E[2])/(E[3]-E[2]);
      Number[0]=100*(E[3]-Error)/(E[3]-E[2]);
    }
    else if(Error<=E[4])
   {
      x1=3;
      x2=4;
      Number[1]=100*(Error-E[3])/(E[4]-E[3]);
      Number[0]=100*(E[4]-Error)/(E[4]-E[3]);
   }
    else 
    {
      x1=4;
      x2=4;
      Number[1]=100;
      Number[0]=100;
    }

/*--------------------------------------------------------*/
	 
	 if(DError<=DE[1])                          
	{
		  y1=0;
		  y2=1;
		  Number[3]=100*(DError-DE[0])/(DE[1]-DE[0]);  //  /10
		  Number[2]=100*(DE[1]-DError)/(DE[1]-DE[0]);
	}
	else if(DError<=DE[2])
	{
		  y1=1;
		  y2=2;
		  Number[3]=100*(DError-DE[1])/(DE[2]-DE[1]);  //  /10
		  Number[2]=100*(DE[2]-DError)/(DE[2]-DE[1]);
	}
	else if(DError<=DE[3])
	{
		  y1=2;
		  y2=3;
		  Number[3]=100*(DError-DE[2])/(DE[3]-DE[2]);   //  /20
		  Number[2]=100*(DE[3]-DError)/(DE[3]-DE[2]);
	}
	else if(DError<=DE[4])
	{
		  y1=3;
		  y2=4;
		  Number[3]=100*(DError-DE[3])/(DE[4]-DE[3]);  //  /50
		  Number[2]=100*(DE[4]-DError)/(DE[4]-DE[3]);
	}
	else 
	{
		  y1=4;
		  y2=4;
		  Number[3]=100;
		  Number[2]=100;
	}
	
	if(Number[0]<=Number[2])UF[0]=Number[0]; else UF[0]=Number[2]; 
	if(Number[0]<=Number[3])UF[1]=Number[0]; else UF[1]=Number[3]; 
	if(Number[1]<=Number[2])UF[2]=Number[1]; else UF[2]=Number[2]; 
	if(Number[1]<=Number[3])UF[3]=Number[1]; else UF[3]=Number[3];
	 
	if(P_D==0)   //P
    {
    	U_tmp[0]=KP[P_rule[y1][x1]];
    	U_tmp[1]=KP[P_rule[y2][x1]];
    	U_tmp[2]=KP[P_rule[y1][x2]];
    	U_tmp[3]=KP[P_rule[y2][x2]];
    
    	result= (U_tmp[0]*UF[0]+U_tmp[1]*UF[1]+U_tmp[2]*UF[2]+U_tmp[3]*UF[3])/(UF[0]+UF[1]+UF[2]+UF[3]);
    }
    else        //D
    {
      	U_tmp[0]=KD[D_rule[x1][y1]];
      	U_tmp[1]=KD[D_rule[x2][y1]];
      	U_tmp[2]=KD[D_rule[x1][y2]];
      	U_tmp[3]=KD[D_rule[x2][y2]];
    
    	result= (U_tmp[0]*UF[0]+U_tmp[1]*UF[1]+U_tmp[2]*UF[2]+U_tmp[3]*UF[3])/(UF[0]+UF[1]+UF[2]+UF[3]);
    }
	 return result;
}
