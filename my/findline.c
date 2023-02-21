#include "findline.h"
#include "pid.h"
//绝对值函数
#define Abs(a) (a>0?a:-a)
float k3;//测试变量

byte angle_line=0;//打角行
byte all_state=0; //总变量 0无元素  1十字 2三岔 3环岛 4入库


byte my_Mid = (Max_Col + Min_Col) >> 1;
float midline_deviation; //中线方差
float midline_deviation1;
float midline_deviation2;
//
byte Lstart=0;
byte Rstart=0;
byte long_turn_flag_left=ROW; //左线连续行
byte long_turn_flag_right=ROW;//右线连续行
float rou_of_left=0;    //左线方差
float rou_of_right=0;   //右线方差

//
byte all_lose_end=0;
byte L_Blank=0;         //左丢线数
byte R_Blank=0;         //右丢线数
/****************************速度变量*******************************/
byte Protect_num=0;//速度保护
int16 v_set[]={};
int16 error=0;
byte now_state=0;//当前状态 0停，1长直道，2短直道,3弯道，4环，5正常三岔，6三岔转向,7入库
byte longstr=0;
byte shortstr=0;
byte cir=0;
byte jin_sancha=0;//进三岔辅助判断速度要素
float mid_k;//中线斜率
float mid_dev;//中线方差
/**************************最小二乘法变量***************************/
float parameterA;//截距
float parameterB;//斜率
/************************入库/出库变量******************************/
byte in_ruku=0;
byte direct=0; //低点补线列
byte start_ruku=0; //开始补线
byte ruku_count=0; //第几次经过库
//其中top_x与top_y  和三岔共用
/**************************环岛变量********************************/

//左环岛



//右环岛




/**************************三岔变量********************************/
byte top_x=0,top_y=0;
byte last_left = 0;
byte last_right = 0;
byte in_sancha = 0; //判断是否在三岔路口内
byte in_sancha2=0; //判断是否在三岔其中一条道上
byte hen_sancha=1;//判断这次在三岔内是横走还是直走  0直走   1横走。初始1，则第一圈直走第二圈横走，反之
byte out_sancha=0; //多次判断
byte sancha_dir=1;//左转还是右转，0左转1右转
/*************************十字变量********************************/
byte all_lose=0;    //左线右线都丢线的行数
byte line_directe=0;    //扫线方向，0默认，1左，2右
byte in_shizi=0;
byte left1_flag = 0;        //左下行
byte left2_flag = 0;        //左上行
byte right1_flag = 0;        //右下行
byte right2_flag = 0;        //右上行
byte shizi_flag = 0;      //是否进入十字区域
byte shizi_count = 0;    //多次计算防止误差
byte yanchi = 99;        //出十字延迟

byte last_left1=0;         //上一帧左下行
byte last_left2=0;         //上一帧左上行
byte last_right1=0;         //上一帧右下行
byte last_right2=0;         //上一帧右上行

//数值越界保护函数 (byte型)
byte Pro_value(int value)
{
    if (value >= Max_Col)
        value = Max_Col;
    else if (value <= Min_Col)
        value = Min_Col;
    return (byte)value;
}

//最小二乘法
float regression(int8 *p, int8 startline, int8 endline, int8 step, float *k,float *b)
{//传入对应的指针，计算出斜率和截距，截距和截距通过指针返回，斜率直接返回，用于单片机 步长最小为1 最大不超过endline-startline
    int8 i = 0;
    if(step<1) step=1;
    if((endline-startline)/step<1) step=endline-startline;
    int sumlines = (endline - startline)/step;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    for (i = startline; i < endline; i+=step)
    {
        sumX += i;
        sumY += p[i];
    }
    if (sumlines == 0) sumlines=1;

    averageX = sumX / sumlines;     //x的平均值
    averageY = sumY / sumlines;     //y的平均值

    for (i = startline; i < endline; i+=step)
    {
        sumUp += (p[i] - averageY) * (i - averageX);
        sumDown += (i - averageX) * (i - averageX);
    }
    if (sumDown == 0) (*k) = 0;
    else (*k) = sumUp / sumDown;
    (*b) = averageY - (*k) * averageX;
    return (*k);
}

//方差计算函数
float deviation(int8 *p, int startline, int endline,byte is_midline)  //传入左线 中线 右线指针 开始行 终止行 返回方差
{
    float sumX = 0;
    float sumX2 = 0;
    float dat = 45;
    for (int i = startline; i < endline; i++)
    {
        if(!is_midline) dat = Pro_value((i - endline) * (p[startline] - p[endline]) / (startline - endline) + p[endline]);
        sumX += (p[i]-dat);
        sumX2 += (p[i] - dat) * (p[i] - dat);
    }
    return sumX2 / (endline - startline) - (sumX / (endline - startline)) * (sumX / (endline - startline));
}

//角搜索
byte angle_scan(int8 *p,byte row)
{
    byte k1=p[row+5]-p[row];
    byte k2=p[row-5]-p[row];
    if(k1*k2>0) return 1;
    else return 0;
}


////向量夹角判断
//byte vector(byte x1,byte y1,byte x2,byte y2,byte x3,byte y3)//x1 y1 为两向量起始点,返回0为锐角，1为钝角
//{
//    int cos_i=0;
//    byte xa=x2-x1;
//    byte ya=y2-y1;
//    byte xb=x3-x1;
//    byte yb=y3-y1;
//    cos_i=xa*xb+ya*yb;
//    if (cos_i>=0) return 0;
//    else return 1;
//}

//补线函数（两点补线）
void Fill_Line(byte x1, byte y1, byte x2, byte y2, byte mx)  //mx=0补左线 mx=1补右线
{

    byte i;
    for (i = y1; i < y2; i++)
    {
        if (mx == 0)
        {
            L_black[i] = Pro_value((i - y2) * (x1 - x2) / (y1 - y2) + x2);
        }
        else R_black[i] = Pro_value((i - y2) * (x1 - x2) / (y1 - y2) + x2);
        LCenter[i] = (byte)((L_black[i] + R_black[i]) >> 1);
    }
}


/*****************扫线*******************/
void pill(byte row){ //画线
    byte i;
    for(i=2;i<88;i++){
        p_Pixels[row][i]=Black;
    }
}


void L_Start_Scan(byte i)
{
    byte j;
    for (j = L_black[i]; j >= Min_Col; j--)
    {
        if (p_Pixels[i][j] == White && p_Pixels[i][j - 1] == Black&& p_Pixels[i][j - 2] == Black)
        {
            R_black[i] = (byte)(j - 1);
            LoseR[i] = 0;
            if(R_black[i]>L_black[i-1]) finalend=i-1;
            break;
        }
        if (j <= Min_Col)
        {
            R_black[i] = Min_Col;
            LoseR[i] = 1;
            R_Blank+=1;
            break;
        }

    }
    LCenter[i] = (byte)((L_black[i] + R_black[i]) >> 1);
}
void R_Start_Scan(byte i)
{
    byte j;


    for (j = R_black[i]; j <= Max_Col; j++)
    {
        if (p_Pixels[i][j] == White && p_Pixels[i][j + 1] == Black && p_Pixels[i][j + 2] == Black)
        {
            L_black[i] = (byte)(j + 1);
            LoseL[i] = 0;
            if(R_black[i-1]>L_black[i]) finalend=i-1;
            break;
        }
        if (j >= Max_Col)
        {
            L_black[i] = Max_Col;
            LoseL[i] = 1;
            L_Blank+=1;
            break;
        }
    }
}


void R_Edge_Scan(byte Num)
{
    int j;

    uint8_t R_Scan_Row = 90;
    if (Num == 0)
    {

        R_Scan_Row = Mid;

    }
    else
    {

        R_Scan_Row = LCenter[Num - 1];

    }

    R_Scan_Row = Pro_value(R_Scan_Row);
    if (Num == 0)
    {

        for (j = R_Scan_Row; j >= Min_Col; j--)
        {
            if (p_Pixels[Num][ j] == White && p_Pixels[Num][ j - 1] == Black && p_Pixels[Num][ j - 2] == Black)
            {
                R_black[Num] = (uint8_t)(j - 1);
                LoseR[Num] = 0;
                break;
            }
            if (j <= Min_Col)
            {
                R_black[Num] = (uint8_t)(Min_Col);
                LoseR[Num] = 1;
                R_Blank+=1;
            }

        }

        LCenter[Num] = (byte)((L_black[Num] + R_black[Num]) >> 1);
    }//首行中线扫描



    //边沿扫描
    else
    {



        j = Pro_value(R_black[Num - 1]);
        if (p_Pixels[Num][ j] == Black) //黑色向左边扫
        {
            for (; j <= Max_Col; j++)
                if (p_Pixels[Num][ j] == Black && p_Pixels[Num][ j + 1] == White)//找到白色
                {

                    R_black[Num] = (uint8_t)(j);
                    break;
                }
            if (j >= Max_Col)
            {
                R_black[Num] = (uint8_t)(Max_Col);
                LoseR[Num] = 1;
                R_Blank+=1;

            }


        }
        else
        {
            for (; j >= Min_Col; j--)
                if (p_Pixels[Num][ j] == White && p_Pixels[Num][j - 1] == Black)//找到黑色
                {
                    R_black[Num] = (uint8_t)(j - 1);
                    break;
                }

            if (j <= Min_Col)
            {
                R_black[Num] = (uint8_t)(Min_Col);
                LoseR[Num] = 1;
                R_Blank+=1;

            }


        }


        LCenter[Num] = (byte)((L_black[Num] + R_black[Num]) >> 1);
    }


}
void L_Edge_Scan(byte Num)
{
    int j;

    uint8_t L_Scan_Row = 90;
    if (Num == 0)
    {

        L_Scan_Row = Mid;

    }
    else
    {

        L_Scan_Row = LCenter[Num - 1];

    }

    L_Scan_Row = Pro_value(L_Scan_Row);
    if (Num == 0)
    {

        for (j = L_Scan_Row; j <= Max_Col; j++)
        {
            if (p_Pixels[Num][ j] == White && p_Pixels[Num][ j + 1] == Black && p_Pixels[Num][ j + 2] == Black)
            {
                L_black[Num] = (uint8_t)(j + 1);
                LoseL[Num] = 0;
                break;
            }
            if (j >= Max_Col)
            {
                L_black[Num] = (uint8_t)(Max_Col);
                LoseL[Num] = 1;
                L_Blank+=1;
            }

        }

        LCenter[Num] = (byte)((L_black[Num] + R_black[Num]) >> 1);
    }//首行中线扫描



    //边沿扫描
    else
    {



        j = Pro_value(L_black[Num - 1]);
        if (p_Pixels[Num][ j] == White) //白色向左边扫
        {
            for (; j <= Max_Col; j++)
                if (p_Pixels[Num][ j] == White && p_Pixels[Num][ j + 1] == Black)//找到白色
                {

                    L_black[Num] = (uint8_t)(j + 1);
                    break;
                }
            if (j >= Max_Col)
            {
                L_black[Num] = (uint8_t)(Max_Col);
                LoseL[Num] = 1;
                L_Blank+=1;
            }


        }
        else
        {
            for (; j >= Min_Col; j--)
                if (p_Pixels[Num][j] == Black && p_Pixels[Num][ j - 1] == White)//找到黑色
                {
                    L_black[Num] = (uint8_t)(j);
                    break;
                }

            if (j <= Min_Col)
            {
                L_black[Num] = (uint8_t)(Min_Col);
                LoseL[Num] = 1;
                L_Blank+=1;

            }


        }


        LCenter[Num] = (byte)((L_black[Num] + R_black[Num]) >> 1);
    }

}

//中线扫描
void Center_Scan(byte i) //本行中心扫线
        {
            byte j ;
            if (i == 0)
                j = Mid;
            else
                j = LCenter[i - 1];
            for (; j <= Max_Col; j++)
            {
                if (p_Pixels[i][j] == White && p_Pixels[i][j + 1] == Black&&p_Pixels[i][j+2]==Black)
                {
                    L_black[i] = (byte)(j + 1);
                    break;
                }
                if (j >= Max_Col)
                {
                    L_black[i] = Max_Col;
                    LoseL[i] = 1;
                    L_Blank++;
                    //////Console.WriteLine(i + "hang");
                }

            }
            if (i == 0)
                j = Mid;
            else
                j = LCenter[i - 1];
            //////Console.WriteLine(j + "j");
            for (; j >= Min_Col; j--)
            {
                if (p_Pixels[i][j] == White && p_Pixels[i][j - 1] == Black&&p_Pixels[i][j-2]==Black)
                {
                    R_black[i] = (byte)(j - 1);
                    break;
                }
                if (j <= Min_Col)
                {
                    R_black[i] = Min_Col;
                    LoseR[i] = 1;
                    R_Blank++;

                }

            }
            //Track_Width[i] = (byte)(L_black[i] - R_black[i]);
            LCenter[i] = (byte)((L_black[i] + R_black[i]) >> 1);
            //if(L_black[i]-R_black[i]<10&&finalend==ROW) finalend=i;
        }

/****************************十字********************************/

byte k_pb(byte*p,byte start,byte end) //斜入十字斜率判别,符合斜入十字返回1
{
    return 1;
    float k1=(p[end]-p[start])/(end-start);
    float k2=regression(p,start-5,start,1,&parameterB,&parameterA);
    k3=k1-k2;
    if(k3> -0.5 && k3<0.5) return 1;
    else return 0;
}

void line_rescan(byte row,byte flag) // 0左 1右 2中
{
    L_Blank=R_Blank=0;
    byte i;
    if(flag==1)
    {
        for(i=row;i<finalend;i++)
        {
        R_Edge_Scan(i);
        R_Start_Scan(i);
        }
    }
    else if(flag==0)
    {
        for(i=row;i<finalend;i++)
        {
        L_Edge_Scan(i);
        L_Start_Scan(i);
        }
    }
    else
    {
        for(i=row;i<finalend;i++)
        {
            Center_Scan(i);
        }
    }
}
void conv1(byte row, byte mx)  //mx=0,先扫下，mx=1，先扫上
{
            byte y = 0, yy = 0;
            byte i=0;
            y = L_black[row];
            yy = R_black[row];

            if (mx==0 && row<40 && all_lose_end<10)
            {
                 if(left1_flag==0 && LoseL[row]==0 && p_Pixels[Pro_value(row+15)][L_black[row]-10]==White && p_Pixels[Pro_value(row+5)][Pro_value(y+3)]==White && p_Pixels[row-1][Pro_value(y+1)]==Black && p_Pixels[row-2][Pro_value(y+2)]==Black &&  L_black[row+3]>=L_black[row]+2 && L_black[row+4]>=L_black[row]+3  && L_black[row-3]>=L_black[row] && L_black[Pro_value(row-6)]>L_black[row-3])
                 {//左下
                     left1_flag=row;
                     while(left1_flag>=1 &&L_black[left1_flag-1]<L_black[left1_flag] && p_Pixels[left1_flag-1][L_black[left1_flag-1]+1]==Black) left1_flag-=1;
                     while(left1_flag>=1 &&L_black[left1_flag+1]<L_black[left1_flag] && p_Pixels[left1_flag+1][L_black[left1_flag+1]+1]==Black) left1_flag+=1;
                  }
                 if(right1_flag==0 && LoseR[row]==0 && p_Pixels[Pro_value(row+15)][R_black[row]+10]==White && p_Pixels[Pro_value(row+5)][Pro_value(yy-3)]==White && p_Pixels[row-1][Pro_value(yy-1)]==Black && p_Pixels[row-2][Pro_value(yy-2)]==Black && R_black[row+3]<=R_black[row]-2 && R_black[row+4]<=R_black[row]-3  && R_black[row-3]<=R_black[row] && R_black[Pro_value(row-6)] <R_black[row-3])
                 {//右下
                     right1_flag=row;
                     while(right1_flag>=1 &&R_black[right1_flag-1]>R_black[right1_flag] && p_Pixels[right1_flag-1][R_black[right1_flag-1]-1]==Black) right1_flag-=1;
                     while(right1_flag>=1 &&R_black[right1_flag+1]>R_black[right1_flag] && p_Pixels[right1_flag+1][R_black[right1_flag+1]-1]==Black) right1_flag+=1;
                 }
            }
            else if(mx==1)
            {
                if(L_black[row]<80 && L_black[row]>10)
                {
                    i=0;
                    while(left2_flag==0 && row>left1_flag && LoseL[row]==0 && (LoseL[row-5]==1 || L_black[row-5]>L_black[row]+5) && p_Pixels[row-3][Pro_value(L_black[row]+8)]==White&& p_Pixels[row][Pro_value(L_black[row]+1)]==Black && L_black[row-3]>L_black[row]+3 && L_black[row-4]>L_black[row]+4 && L_black[row]-L_black[Pro_value(row+3)]<6) //&& L_black[row+3]<=L_black[row] )//&& L_black[row+3]<L_black[row+1])
                    {//左上
                        row++;
                        i++;
                        if(i>3) break; //最多迭代三次
                    }//右上
                    if(i) left2_flag=row;
                }
                if(R_black[row]<80 && R_black[row]>10)
                {
                    i=0;
                    while(right2_flag==0 && row>right1_flag && LoseR[row]==0 && (LoseR[row-5]==1 || R_black[row-5]<R_black[row]-5)&& p_Pixels[row-3][Pro_value(R_black[row]-8)]==White && p_Pixels[row][Pro_value(R_black[row]-1)]==Black && R_black[row-3]<R_black[row]-3 && R_black[row-4]<R_black[row]-4 && R_black[Pro_value(row+3)]-R_black[row]<6)  //&& R_black[row+3]>=R_black[row] )//&& R_black[row+3] >R_black[row+1])
                    {
                        row++;
                        i++;
                        if(i>3) break;
                    }
                    if(i) right2_flag=row;
                }

            }
}

void cross_init()
{
    shizi_flag = 1;
    in_shizi = 1;
    yanchi = 0;

}
void scan_high()//上点扫描
{
    left2_flag=right2_flag=0;
    byte i;
    for (i = 4; i < 50; i++)
    {
        if (i<finalend) conv1(i, 1);
    }
}

byte pro_judge(byte left,byte right)//防止三岔误判成十字   返回1近似十字  0近似三岔   2 pass
{
    byte x=(left+right)/2;
    byte y=(L_black[left]+R_black[right])/2;
    if(in_shizi && left1_flag>30 && right1_flag>30 && left2_flag==0 && right2_flag==0)
    {
        if(p_Pixels[x+10][y]==White && p_Pixels[x+15][y]==White && p_Pixels[x+20][y]==White) return 1;
        else return 0;
    }
    else return 2;

}
byte cross2()
       {

           byte i;
           left1_flag = 0;
           left2_flag = 0;
           right1_flag = 0;
           right2_flag = 0;

           //扫线方向
           if(line_directe==1) line_rescan(0,0);
           else if(line_directe==2) line_rescan(0,1);


           for (i = 2; i < 45; i++)//扫下点
           {
               if (i<finalend) conv1(i, 0);
           }
           scan_high();//扫上点

           if(left1_flag!=0 && right1_flag!=0 && left2_flag!=0 && right2_flag!=0 && abs(left1_flag-right1_flag)<5) line_directe=0; //重置中线扫描
           else if(right1_flag==0 && right2_flag==0)
           {
               if((left1_flag==0 && left2_flag==0)|| (left1_flag!=0 && left2_flag !=0 )) line_directe=0; //重置中线扫描
           }

           if(left1_flag==0 && right1_flag==0)
           {
               if(in_shizi==0 && yanchi>=3) return 0;
           }




           if(R_Blank>10 && left1_flag<40)
           {
               if(left2_flag!=0 && right1_flag==0  && right2_flag==0 && line_directe!=2)
               {
                   line_directe=2; //看到左边两个点，则右扫线
                   line_rescan(left1_flag,1);
               }
               if(left1_flag!=0 && right1_flag==0 && left2_flag!=0 && right2_flag!=0 && line_directe!=0)
               {
                   line_directe=0; //看到三个点则中间扫线
                   line_rescan(left1_flag,2);
               }
               if(left1_flag!=0 && right1_flag==0 && line_directe!=2) //有左下点
               {                                   //只有一个左下点则向右扫线
                   line_directe=2;
                   line_rescan(left1_flag,1);
               }
               else if(left1_flag!=0 && right1_flag!=0 && (right1_flag-left1_flag)>=5 && line_directe!= 1) //右下点比左下点高，则向左扫线
               {
                   line_directe=1;
                   line_rescan(left1_flag,0);
               }
           }
           else if(L_Blank>10 && right1_flag<40) //有右下点
           {
               if(left1_flag==0 && left2_flag==0 &&  right2_flag!=0 && line_directe!=1)
               {
                   line_directe=1; //看到右边两个点，则左扫线
                   line_rescan(right1_flag,0);
               }
               if(left1_flag==0 && right1_flag!=0 && left2_flag!=0 && right2_flag!=0 && line_directe!=0)
               {
                   line_directe=0; //看到三个点则中间扫线
                   line_rescan(right1_flag,2);
               }
               if(left1_flag==0 && right1_flag!=0 && line_directe!= 1)//只有一个右下点则向左扫线
               {
                   line_directe=1;
                   line_rescan(right1_flag,0);
               }

               else if (left1_flag!=0 && right1_flag!=0 && (left1_flag-right1_flag)>=5 && line_directe!= 2)//左下点比右下点高，则向右扫线
               {
                   line_directe=2;
                   line_rescan(right1_flag,1);
               }

           }
           if(line_directe !=0) scan_high();//扫上点

           //防止上点在下点正上方
           if(left1_flag!=0 && left2_flag !=0 && (L_black[left1_flag]-L_black[left2_flag])<=1) left2_flag=0;
           if(right1_flag!=0 && right2_flag !=0 && (R_black[right2_flag]-R_black[right1_flag])<=1) right2_flag=0;

           //防止上点找错
           if(left2_flag!=0 && right2_flag!=0)
            {
               if(left2_flag-right2_flag>8) left2_flag=0;
               else if(right2_flag-left2_flag>8) right2_flag=0;
            }

           //防止错道扫线
           if(left1_flag!=0 || right1_flag!=0)
           {

               if(right1_flag !=0 && left2_flag!=0 && right2_flag==0 && L_black[left2_flag]<(L_black[left1_flag]+R_black[right1_flag])/2)
               {
                   line_directe=1;
                   line_rescan(right1_flag,0);
                   scan_high();
               }
               if(left1_flag!=0 && left2_flag==0 && right2_flag!=0 && R_black[right2_flag]>(L_black[left1_flag]+R_black[right1_flag])/2)
               {
                   line_directe=2;
                   line_rescan(left1_flag,1);
                   scan_high();
               }

               //防止只看到一个下点的情况下错误地进行边缘扫线
               if(left1_flag!=0 && right1_flag==0 && left2_flag!=0 && L_black[left1_flag]-L_black[left2_flag]>30)
               {
                   line_directe=0;
                   left2_flag=0;
               }
               if(left1_flag==0 && right1_flag!=0 && right2_flag!=0 && R_black[right2_flag]-R_black[right1_flag]>30)
               {
                   line_directe=0;
                   right2_flag=0;
               }
           }

           //防止上点找错
           if(left2_flag!=0 && right2_flag!=0)
           {
               if(L_black[left2_flag]<=R_black[right2_flag]) left2_flag=right2_flag=0;
           }

           if(left1_flag==0 && right1_flag ==0)
           {
               if(left2_flag==0 && R_black[right2_flag]>70) right2_flag=0;
               if(right2_flag==0 && L_black[left2_flag]<20) left2_flag=0;
           }


           //测试，防止下点超前扫
//           if(left1_flag!=0 && left2_flag==0 && right1_flag==0 && right2_flag!=0 && left1_flag>right2_flag+3)
//           {
//               left1_flag=0;
//               left2_flag=right2_flag+2;
//           }
//           else if(left1_flag==0 && left2_flag!=0 && right1_flag!=0 && right2_flag==0 && left2_flag+3<right1_flag)
//           {
//               right1_flag=0;
//               right2_flag=left2_flag+2;
//           }

           //防止三岔误判成十字
           //if(pro_judge(left1_flag,right1_flag)==0) in_shizi=0;

//           pill(left1_flag);
//           pill(left2_flag);
//           pill(right1_flag);
//           pill(right2_flag);

           if (in_shizi == 0 && shizi_flag == 0 &&(left1_flag > 0 && left2_flag > 0 && right1_flag > 0 && right2_flag > 0)&& L_Blank>10 && R_Blank>10 && p_Pixels[45][my_Mid]==White)//|| (L_Blank >= 15 && right1_flag != 0 && R_Blank<=15)||( R_Blank >= 15 &&  left1_flag != 0 && L_Blank <=15)
           {//直入
               cross_init();
           }
//           if(in_shizi == 0
//                   && shizi_flag == 0
//                   &&((left1_flag > 0 && left2_flag == 0 && right1_flag == 0 && right2_flag == 0 && L_Blank>10 && R_Blank>40 && p_Pixels[50][my_Mid-10]==White ) || (left1_flag == 0 && left2_flag == 0 && right1_flag > 0 && right2_flag == 0 && L_Blank>40 && R_Blank>10 && p_Pixels[50][my_Mid+10]==White )))//斜入
//           {//斜入(只看到一边下点）
//               if(left1_flag!=0 && p_Pixels[50][15]==White && p_Pixels[50][20]==White)
//               {
//                   //if(vector(L_black[left1_flag], left1_flag, L_black[left1_flag+3], left1_flag+3, L_black[Pro_value(left1_flag-3)], Pro_value(left1_flag-3))==1) return 0;
//               }
//               if(right1_flag!=0 && p_Pixels[50][75]==White && p_Pixels[50][70]==White)
//              {
//                  //if(vector(R_black[right1_flag], right1_flag, R_black[right1_flag+3], right1_flag+3, R_black[Pro_value(right1_flag-3)], Pro_value(right1_flag-3))==1) return 0;
//              }
//               cross_init();
//           }
           if(in_shizi == 0 && shizi_flag == 0 && ((right1_flag==0 && left1_flag > 0 && left2_flag > 0 &&  R_Blank>30 && p_Pixels[left2_flag+5][L_black[left2_flag]-10]==White && k_pb(L_black,left1_flag,left2_flag)) || (left1_flag==0 && right1_flag > 0 && right2_flag > 0 && L_Blank>30 && p_Pixels[right2_flag+5][R_black[right2_flag]+10]==White && k_pb(R_black,right1_flag,right2_flag))))//斜入
           {//斜入（看到一边上下点）
              //if(pro_judge(left1_flag,right1_flag)==0) return 0;
              //if((rou_of_right<=10 && L_Blank<15) || (rou_of_left<=10 && R_Blank<15)) return 0;
              cross_init();
           }
           if (in_shizi == 1 || yanchi <= 5)
           {
               //进出十字控制部分
               if( shizi_flag == 1)
               {
                   if (left1_flag == 0 && right1_flag == 0 && left2_flag<=10 && right2_flag<=10)
                   {
                       shizi_count++;
                   }
                   else if(shizi_count<10) shizi_count=0;
               }

               if(in_shizi && all_lose <7 && all_lose_end <5 && long_turn_flag_left==ROW && long_turn_flag_right==ROW && (L_Blank<10 || R_Blank<10))
               {
                   shizi_count = 0;
                   in_shizi = 0;
                   shizi_flag = 0;
                   line_directe=0;
               }
               if((shizi_count>=10 && shizi_flag == 1 && all_lose<13) )//|| (shizi_flag == 1 && (L_Blank<10 || R_Blank<10>>) && all_lose<13 && left1_flag==0 && left2_flag==0 &&(right1_flag)))    //离开十字
               {
                   if ((left1_flag == 0 && left2_flag <= 10 && right1_flag == 0 && right2_flag <= 10  ) || (left2_flag==0 && right2_flag==0 && L_Blank < 10 && R_Blank < 10) || (L_Blank < 15 && R_Blank < 15 && all_lose <5))//||( && all_lose<10))
                   {
                       shizi_count = 0;
                       in_shizi = 0;
                       shizi_flag = 0;
                       line_directe=0;
                   }
               }
               if (shizi_flag == 0)
               {
                   yanchi++;
               }

               if(1)
               {
//                      if(left1_flag==0 && left2_flag==0 && L_Blank<30)
//                      {
//                          left1_flag=last_left1;
//                          left2_flag=last_left2;
//                          while(L_black[left2_flag-1]-L_black[left2_flag]<=2) left2_flag--;
//                      }
//                      if(right1_flag==0 && right2_flag==0 && R_Blank<30)
//                      {
//                          right1_flag=last_right1;
//                          right2_flag=last_right2;
//                          while(R_black[right2_flag]-R_black[right2_flag-1]<=2) right2_flag--;
//                      }
//                      if(left1_flag<5 && left1_flag!=0 && left2_flag==0) left2_flag=last_left2;
//                      if(right1_flag<5 && right1_flag!=0 && right2_flag==0) right2_flag=last_right2;

                   //第一类补点/已知三点求第四点/
                       if(left1_flag!=0 && left2_flag!=0)
                       {
                           if(right1_flag==0 && right2_flag!=0 && abs(R_black[right2_flag+5]-R_black[right2_flag])>5)
                           {
                               right1_flag=Pro_value(right2_flag+left1_flag-left2_flag);
                               R_black[right1_flag]=Pro_value(R_black[right2_flag]+L_black[left2_flag]-L_black[left1_flag]);
                           }
                           else if(right1_flag!=0 && right2_flag==0 && LoseR[Pro_value(right1_flag-3)])
                           {
                               right2_flag=Pro_value(right1_flag+left2_flag-left1_flag+2);
                               //R_black[right2_flag]=Pro_value(R_black[right1_flag]+L_black[left1_flag]-L_black[left2_flag]);
                           }
                       }
                       else if(right1_flag!=0 && right2_flag!=0)
                       {
                           if(left1_flag==0 && left2_flag!=0 && abs(L_black[left2_flag+5]-L_black[left2_flag])>5)
                           {
                               left1_flag=Pro_value(left2_flag+right1_flag-right2_flag);
                               L_black[left1_flag]=Pro_value(L_black[left2_flag]+R_black[right2_flag]-R_black[right1_flag]);
                           }
                           else if(left1_flag!=0 && left2_flag==0 && LoseL[Pro_value(left1_flag-3)])
                           {
                               left2_flag=Pro_value(left1_flag+right2_flag-right1_flag+2);
                               //L_black[left2_flag]=Pro_value(L_black[left1_flag]+R_black[right1_flag]-R_black[right2_flag]);
                           }
                       }
                   /***********单边补点************/

                      if (left1_flag == 0 && left2_flag != 0)       //只有左上
                      {
                          regression(L_black,left2_flag,left2_flag+5,1,&parameterB,&parameterA);
                          left1_flag=1;
                          if(parameterA>L_black[left2_flag])  L_black[1]=Pro_value(parameterA);
                          else L_black[1]=Pro_value(L_black[left2_flag]*2-parameterA);
                          //if(parameterB>0);
                      }
                      if (right1_flag == 0 && right2_flag != 0)     //只有右上
                      {
                          regression(R_black,right2_flag,right2_flag+5,1,&parameterB,&parameterA);
                          right1_flag=1;
                          if(parameterA<R_black[right2_flag]) R_black[1]=Pro_value(parameterA);
                          else R_black[1]=Pro_value(R_black[right2_flag]*2-parameterA);
                          //if(parameterB<0);
                      }
                      if (left1_flag != 0 && left2_flag == 0)     //只有左下
                      {
                          for(i=left1_flag-1;i>=1;i--)
                          {
                              if(LoseL[i]==1 || L_black[i]>87)
                              {
                                  i++;
                                  break;
                              }
                          }
                          regression(L_black,i,left1_flag,1,&parameterB,&parameterA);
                          left2_flag=50;
                          L_black[left2_flag]=L_black[left1_flag]-abs((50-left1_flag)*parameterB);

                      }
                      if (right1_flag != 0 && right2_flag == 0)     //只有右下
                      {
                          for(i=right1_flag-1;i>=1;i--)
                          {
                              if(LoseR[i]==1 || R_black[i]<3)
                              {
                                 i++;
                                 break;
                              }
                            }
                          regression(R_black,i,right1_flag,1,&parameterB,&parameterA);
                          right2_flag=50;
                          R_black[right2_flag]=R_black[right1_flag]+abs((50-right1_flag)*parameterB);
                      }

              }

               //防误差
//               last_left1=left1_flag;
//               last_left2=left2_flag;
//               last_right1=right1_flag;
//               last_right2=right2_flag;
               //补线部分
               if (left1_flag != 0 && left2_flag != 0) Fill_Line(L_black[left1_flag], left1_flag, L_black[left2_flag], left2_flag, 0);          //左上左下都有，补左线
               if (right1_flag != 0 && right2_flag != 0) Fill_Line(R_black[right1_flag], right1_flag, R_black[right2_flag], right2_flag, 1);    //右上右下都有，补右线
           }
       }
/**************************三岔********************************/
byte angle_xr(byte*p,byte row)//斜入三岔与斜入十字区别
{
    byte i;
    for(i=row-1;i>0;i--)
    {
        if(p[i]<3 || p[i]>87) break;
    }
    float k=regression(p,i,row,2,&parameterB,&parameterA);
    float k2=(top_x-p[row])/(top_y-row);
    if(abs(k-k2)>0.2) return 1;
    return 0;
}
byte find_point(byte x,byte y,byte i,byte mx)// x,y为坐标，i为维度(奇数),mx=0向左搜索   mx=1向右搜索
        {

            byte a;
            i = (i-1) / 2;
            byte start = 0;
            if (mx == 0) start = 0;
            else start = -i;
            for (a = start; a <= start+i; a++)//底行
            {
                if (p_Pixels[Pro_value(y - i)][ Pro_value(x + a)] == Black && p_Pixels[Pro_value(y-i-1)][ Pro_value(x + i)] == White && Pro_value(y - i) != 0 )
                {
                    top_x = Pro_value(x + a);
                    top_y = Pro_value(y - i);
                    return 1;
                }
            }
            if(mx==1)
            {
                for (a =1; a <= i; a++)//右侧列
                {
                    if (p_Pixels[Pro_value(y + a - i)][ Pro_value(x - i)] == Black && p_Pixels[Pro_value(y -i-1)][ Pro_value(x - i)] == White && Pro_value(y + a - i)!=0  )
                    {
                        top_x = Pro_value(x - i);
                        top_y = Pro_value(y + a - i);
                        return 1;
                    }
                }
            }
            else
            {
                for (a =1; a <= i; a++)//左侧列
                {
                    if (p_Pixels[Pro_value(y + a - i)][ Pro_value(x + i)] == Black && p_Pixels[Pro_value(y -i-1)][ Pro_value(x + i)] == White && Pro_value(y + a - i)!=0 )
                    {
                        top_x = Pro_value(x + i);
                        top_y = Pro_value(y + a - i);
                        return 1;
                    }
                }
            }

            return 0;
        }

void sancha_init()
{
    in_sancha=0;
    //if(hen_sancha==1) PID_Speed.SpeedSet=speed; //调回正常速度
    in_sancha2=!in_sancha2;
    last_left=0;
    last_right=0;
    out_sancha=0;
    top_x=top_y=0;
    jin_sancha=0;
}

void change_dir()   //直走横走切换
{
    if(SteerTest.dir)   //横走转成直走
    {

    }
    else
    {

    }
}
byte sancha() //三岔主函数
        {
            byte left_point = 0,right_point=0;
            byte i=5,j=5;
            byte a = 0;
            byte top_mid=0;
            while (i < 55)
            {
                if(p_Pixels[i][Pro_value(my_Mid)] == Black
                        && p_Pixels[i][Pro_value(my_Mid+1)] == Black) break;
                i++;  //寻找顶点
            }
            if (i != 55)
            {
                top_x = my_Mid;
                top_y = i;
                for (i = 0; i < 50; i++)
                {
                    if (find_point(top_x, top_y, 3,0) == 0)
                    {
                        if (find_point(top_x, top_y, 5,0) == 0)
                        {
                            if (find_point(top_x, top_y, 7,0) == 0)
                            {
                                break;
                            }
                        }
                    }
                    if(top_y <=2) break;
                }
                for (i = 0; i < 50; i++)
                {
                    if (find_point(top_x, top_y, 3,1) == 0)
                    {
                        if (find_point(top_x, top_y, 5,1) == 0)
                        {
                            if (find_point(top_x, top_y, 7,1) == 0)
                            {
                                break;
                            }
                        }
                    }
                    if(top_y <=2) break;
                }
                //修改1
                if(p_Pixels[top_y-2][top_x-5]==Black || p_Pixels[top_y-2][top_x+5]==Black)
                {
                    top_x=top_y=0;
                }
                if(top_x!=0 && top_y!=0)
                {
                    byte top_left = 0, top_right = 0;
                    for (i=1;i<15;i++)
                    {
                        if(top_left==0 && p_Pixels[top_y][Pro_value(top_x+i)]==White)
                        {
                            top_left =(byte)(Pro_value(top_x + i) - 1);
                        }
                        if (top_right == 0 && p_Pixels[top_y][Pro_value(top_x - i)] == White)
                        {
                            top_right = (byte)(Pro_value(top_x - i) + 1);
                        }
                    }
                    top_mid =(byte)( (top_left + top_right) / 2);
                }
            }
            else return 0;

            //if(p_Pixels[top_y-3][Pro_value(top_x+10)-2]==Black || p_Pixels[top_y-3][Pro_value(top_x-10)]==Black) return 0;

            byte y = 0, yy = 0;
            for (a = 3; a < Pro_value(top_y-5); a++) //寻找左右角点
            {

                 y = L_black[a];
                 yy = R_black[a];
                 if(LoseL[a]==0 && left_point==0 &&  p_Pixels[a][Pro_value(y+3)]==Black && L_black[a+3]>L_black[a]  && L_black[a-2]>=L_black[a] )
                 {//左下
                     left_point=a;
                     while(left_point>=1 && LoseL[left_point-1]==0 &&L_black[left_point-1]<L_black[left_point] && p_Pixels[left_point-1][L_black[left_point-1]+2]==Black) left_point-=1;
                     //while(left_point>=1 && L_black[left_point+1]<L_black[left_point] && p_Pixels[left_point+1][L_black[left_point+1]+2]==Black) left_point+=1;
                     if(angle_scan(L_black, left_point)==0 || left_point>=top_y) left_point=0;
                  }
                 if(LoseR[a]==0 && right_point==0 &&  p_Pixels[a][Pro_value(yy-3)]==Black &&  R_black[a+3]<R_black[a] && R_black[a-2]<=R_black[a])
                 {//右下
                     right_point=a;
                     while(right_point>=1 && LoseR[right_point-1]==0 &&R_black[right_point-1]>R_black[right_point] && p_Pixels[right_point-1][R_black[right_point-1]-2]==Black) right_point-=1;
                     //while(right_point>=1 &&R_black[right_point+1]>R_black[right_point] && p_Pixels[right_point+1][R_black[right_point+1]-2]==Black) right_point+=1;
                     if(angle_scan(R_black, right_point)==0 || right_point>=top_y) right_point=0;
                 }

            }
            if (left_point != 0 && in_sancha==0)
            {
                for (a = left_point + 3; a < 10; a++)
                {
                    if (L_black[a + 1] < L_black[a]) return 0;
                }

            }
            if (right_point != 0 && in_sancha == 0)
            {
                for (a = right_point + 3; a < 10; a++)
                {
                    if (R_black[a + 1] > R_black[a]) return 0;
                }

            }
            while (j < 55 && (p_Pixels[j][my_Mid] == White || p_Pixels[Pro_value(j-2)][Pro_value(my_Mid-8)] == Black || p_Pixels[Pro_value(j-2)][ Pro_value(my_Mid + 8)] == Black)) j++;  //寻找顶点
//            pill(top_y);
//            pill(left_point);

            //左限右限计算
            byte midmid = 0;
            byte chazhi = 0;
            if (left_point != 0 && right_point != 0)
            {
                midmid = L_black[left_point] - R_black[right_point];
                chazhi = right_point - left_point;
            }


            if (in_sancha==0 && left_point!=0 && right_point!=0 && (abs(left_point-right_point)<12) && j!=55 && top_y>10) //判断进入三岔
            {

                //若顶点不在左右限内，return
                if (top_mid > midmid*0.6+ R_black[right_point]+ chazhi || top_mid< midmid*0.4+ R_black[right_point]+ chazhi) return 0;


                if(p_Pixels[50][45]==White && p_Pixels[45][45]==White) return 0;
                for (i = -10; i <= 10; i+=5)
                {
                    if (p_Pixels[Pro_value(top_y - 3)][Pro_value(Pro_value(top_x + i))] == Black) return 0;
                }

                if(p_Pixels[top_y-3][Pro_value(top_x+10)]==White && p_Pixels[top_y-3][Pro_value(top_x-10)]==White)
                {
                    top_x=top_y=0;
                    in_sancha = 1;
                    if(in_sancha2==0)
                    {
                        hen_sancha=!hen_sancha;
                        if(hen_sancha==1 ) //横入三岔前减速
                        {
                            //speed=PID_Speed.SpeedSet;
                            //set_speed(v_hen);
                        }
                    }
                }
            }
            //pill(top_y);
            if (in_sancha == 1)  //判断离开三岔
            {

                if(//top_x<10 ||
                   top_y<=5 ||
                   (L_Blank<10 && R_Blank<10)||
                   (top_x > 85 && p_Pixels[Pro_value(top_y-2)][Pro_value(top_x+5)]==Black && p_Pixels[Pro_value(top_y-3)][Pro_value(top_x+8)]==Black)|| (top_x < my_Mid && p_Pixels[Pro_value(top_y-2)][Pro_value(top_x-5)]==Black && p_Pixels[Pro_value(top_y-3)][Pro_value(top_x-8)]==Black)||
                   (long_turn_flag_left==ROW && long_turn_flag_right==ROW) ) //||(R_Loss==0 && L_Loss!=0)||(R_Loss!=0 && L_Loss==0))
                {
                    out_sancha++;
                    if(out_sancha>=2)
                    {
                        sancha_init();
                        return 0;
                    }

                }
                else out_sancha=0;
                byte t = my_Mid + 10;
                byte count=0;
                for (t = my_Mid + 10; t > my_Mid - 10; t--)
                {
                    if (p_Pixels[1][t] == Black)
                    {
                        count++;
                        if(count>=6)
                        {
                            sancha_init();
                            return 0;
                        }
                    }
                }
            }

            if (in_sancha == 1)
            {
                //左点
                if(left_point<=3 && right_point!=0)
                {
                    left_point=right_point;
                }
                if(left_point!=0) last_left=left_point;
                else
                {
                    left_point=last_left;
                    if(last_left>0) last_left--;
                }

                //右点
                if(right_point<=3 && left_point!=0)
                {
                    right_point=left_point;
                }
                if(right_point!=0) last_right=right_point;
                else
                {
                    right_point=last_right;
                    if(last_right>0) last_right--;
                }

                if(sancha_dir==1) //向右
                {
                    Fill_Line(L_black[left_point], left_point, top_x, top_y, 0);
                    line_rescan(top_y-2,1);
                }
                else    //向左
                {
                    Fill_Line(R_black[right_point], right_point, top_x, top_y, 0);
                    line_rescan(top_y-2,0);
                }

                if((top_y<30 || top_x>70 )&& hen_sancha==1)
                {
                    change_dir();
                    sancha_init();
                }
//                for(i=top_y-3;i<(Max_Col);i++)
//                {
//                    Center_Scan(i);
//                    if(LoseL[i]) return 0;
//                }
            }
            return 0;
        }

/*******************************入库/出库*******************************/
void find_p(byte direct,byte *p,byte *lose) //第一次左右找点
{
    byte row;
    for (row = 0; row < 50; row++)
                    {
                        if (p_Pixels[row][direct] == White && p_Pixels[row + 1][direct] == Black && p_Pixels[row + 2][ direct] == Black && p_Pixels[row + 4][ direct] == Black && p_Pixels[row + 8][ direct] == Black && p_Pixels[Pro_value(row + 16)][direct] == Black)
                        {
                            row += 1;
                            while (p[row + 1] - p[row] > 3) row++;
                            if (lose[row] == 0 )//&& p[row] < 50)
                            {
                                if (lose[row - 5] == 0 || (p_Pixels[row+2][Pro_value(p[row]-5)]==White && p_Pixels[row+2][Pro_value(p[row]+5)]==White))
                                {
                                    in_ruku = 3;
                                    return;
                                }
                                top_x = p[row];
                                top_y = row;
                                return;
                            }
                        }
                    }
}
void ruku()//入库
        {
            byte row;
            byte i;
            byte block_count=0;

            if(ruku_count==0 && L_Blank<10 && R_Blank<10 && in_ruku!=0) //强制退出第一次经过库的状态
            {
                in_ruku=0;
                ruku_count=1;
                return;
            }

            if(in_ruku==0 && all_lose<15)
            {
                if ((L_Blank<20 && R_Blank>20)||(L_Blank > 20 && R_Blank < 20))
                {
                    for(row=30;row<45;row+=3)
                    {
                        block_count = 0;
                        for(i=15;i<75;i++)
                        {
                            if(p_Pixels[row][i]==White && p_Pixels[row][i+1] == Black && p_Pixels[row][i+2] == Black)
                            {
                                block_count++;
                            }
                        }
                        if (block_count>5)
                        {
                            if (L_Blank > 20)
                            {
                                in_ruku = 1; //左库
                                direct=2;
                            }
                            else
                            {
                                in_ruku = 2;//右库
                                direct=88;
                            }
                            break;
                        }
                    }
                    return;

                }
                else return;
            }
            else if(in_ruku==0) return;

            if (in_ruku == 1) //左库第一次找点
            {
                find_p(88,L_black,LoseL);
            }
            else if (in_ruku == 2) //右库第一次找点
            {
                find_p(2,R_black,LoseR);
            }
            if (in_ruku == 3)//八邻域找点
            {
                for(row=0;row<50;row++)
                {
                    if(p_Pixels[row][45]==White && p_Pixels[row + 2][45] == Black && p_Pixels[row + 4][45] == Black && p_Pixels[row + 8][45] == Black && p_Pixels[row + 16][45] == Black)
                    {
                        break;
                    }
                }
                if(row!=50)
                {
                    row++;
                    top_x = 45;
                    top_y = row;
                    if (p_Pixels[row - 3][65] == White && p_Pixels[row - 3][25] == White && p_Pixels[row - 3][50] == White && p_Pixels[row - 3][ 40] == White && p_Pixels[row +3][ 65] == Black && p_Pixels[row + 3][ 25] == Black) in_ruku = 5;//停
                }
                for (i = 0; i < 50; i++)
                {
                    if (find_point(top_x, top_y, 3,0) == 0)
                    {
                        if (find_point(top_x, top_y, 5,0) == 0)
                        {
                            break;
                        }
                    }
                }
                for (i = 0; i < 50; i++)
                {
                    if (find_point(top_x, top_y, 3,1) == 0)
                    {
                        if (find_point(top_x, top_y, 5,1) == 0)
                        {
                            break;
                        }
                    }
                }
            }

            if (ruku_count == 0) //第一次经过库
            {
                byte start=top_y;
                if(top_y<20) start=20;
                if (top_y < 10)
                {
                    ruku_count = 1;
                    in_ruku = 0;
                    return;
                }
                regression(L_black, start +5 ,start + 10,1,&parameterB,&parameterA);
                Fill_Line((byte)parameterA, 0, L_black[start + 5], (byte)(start + 5),0);
                regression(R_black, start + 5, start + 10, 1,&parameterB,&parameterA);
                Fill_Line((byte)parameterA, 0, R_black[start + 5], (byte)(start + 5), 1);
                return;
            }

            if (start_ruku == 0 && top_y<42) //开始入库
            {
                start_ruku = 1;
                now_state=7;
            }

            if (start_ruku == 0) return;

            if(direct==2 && top_y>10 && top_x>10 && !(in_ruku==3 && L_Blank<10))
            {
                for (i = 0; i < top_y; i++) L_black[i] = 88;
                Fill_Line(direct,0,top_x,top_y,1);
            }
            else if(direct==88 && top_y>10 && top_x<80 && !(in_ruku==3 && R_Blank<10))
            {
                for (i = 0; i < top_y; i++) R_black[i] = 2;
                Fill_Line(direct,0,top_x,top_y,0);
            }
            if (in_ruku == 5)
            {
                Is_Run=0; //入库，停
                //CLOSE_MOTOR;
            }

        }
void chuku()
{

}

/*******************************环岛************************************/
/*  环岛 基本入环 应该保持每个状态图像之间连续性   */
         byte Track_State = 0;   //环岛状态  0 1 2 3 4 5 6 7
         byte Track_Dir = 0;    //1左环 2右环
         byte Track_Num = 1;    //环岛个数   跑前人工设定一下
         byte Track_type[] = { 0, 1, 2 };     //环岛类型 小 中 大   0  1  2
         byte Field_num = 0;     //检测场数
         byte Field_num1 = 0;
         uint8_t L_down_point[] = { 0, 0 };
         uint8_t L_down_point_flag = 0;    //本场找到标志位
         uint8_t L_down_point_last_flag = 0;//上场找到标志位
         uint8_t L_down_point_times = 0;  //左下拐点找到的次数

         uint8_t L_middle_point[] = { 0, 0 };
         uint8_t L_middle_point_flag = 0;     //本场找到标志位
         uint8_t L_middle_point_last_flag = 0;//上场找到标志位
         uint8_t L_middle_point_times = 0;  //左中拐点找到次数

         uint8_t L_up_point[]= { 0, 0 };
         uint8_t L_up_point_flag = 0;     //本场找到标志位
         uint8_t L_up_point_last_flag = 0;//上场找到标志位
         uint8_t L_up_point_times = 0;  //左上拐点找到次数

         uint8_t R_down_point[]= { 0, 0 };
         uint8_t R_down_point_flag = 0;
         uint8_t R_down_point_last_flag = 0;
         uint8_t R_down_point_times = 0;

         uint8_t R_middle_point[] = { 0, 0 };
         uint8_t R_middle_point_flag = 0;
         uint8_t R_middle_point_last_flag = 0;
         uint8_t R_middle_point_times = 0;

         uint8_t R_up_point[] = { 0, 0 };
         uint8_t R_up_point_flag = 0;
         uint8_t R_up_point_last_flag = 0;
         uint8_t R_up_point_times = 0;



        void find_leftdown_point(byte startline, byte endline)
        {
            byte j;

            for (j = startline; j < endline; j++)
            {
                if (j >= 2 && p_Pixels[j + 1][ Pro_value(L_black[j] + 2)] == White && abs(L_black[j - 1] - L_black[j - 2]) <= 3 && abs(L_black[j] - L_black[j - 1]) <= 3 && (L_black[j + 1] - L_black[j] >= 3)
                && p_Pixels[j][ Max_Col - 1] == White && LoseL[j - 2] == 0 && LoseL[j - 1] == 0 && LoseL[j] == 0)
                {
                    if (L_down_point_times >= 1)
                    {
                        if (abs(j - L_down_point[0]) > 10)
                        {
                            L_down_point_flag = 0;
                            break;
                        }
                    }
                    L_down_point[0] = j;//数组里面没有第0行
                    L_down_point[1] = L_black[j];
                    L_down_point_flag = 1;
                    L_down_point_times++;
                    break;
                }
                else
                {
                    L_down_point_flag = 0;
                }
            }
        }
        void find_rightdown_point(byte startline, byte endline)
        {
            byte j;

            for (j = startline; j < endline; j++)
            {
                if (j >= 2 && p_Pixels[j + 1][ Pro_value(R_black[j] - 2)] == White && abs(R_black[j - 1] - R_black[j - 2]) <= 3 && abs(R_black[j] - R_black[j - 1]) <= 3 && (R_black[j] - R_black[j + 1] >= 3)
                && p_Pixels[j][ Min_Col + 1] == White && LoseR[j - 2] == 0 && LoseR[j - 1] == 0 && LoseR[j] == 0)
                {
                    if (R_down_point_times >= 1)
                    {
                        if (abs(j - R_down_point[0]) > 10)
                        {
                            R_down_point_flag = 0;
                            break;
                        }
                    }
                    R_down_point[0] = j;//数组里面没有第0行
                    R_down_point[1] = R_black[j];
                    R_down_point_flag = 1;
                    R_down_point_times++;
                    break;
                }
                else
                {
                    R_down_point_flag = 0;
                }
            }
        }
        void find_leftmiddle_point(byte startline, byte endline)
        {
            byte j;

            for (j = startline; j <= endline; j++)
            {

                //找左拐点
                if (j >= 5 && L_black[j - 5] - L_black[j] > 0 && L_black[j - 4] - L_black[j] > 0 && L_black[j - 3] - L_black[j] > 0 && L_black[j - 5] - L_black[j - 4] >= 0
                && L_black[j - 4] - L_black[j - 3] >= 0 && L_black[j - 3] - L_black[j - 2] >= 0 &&
                L_black[j + 4] - L_black[j] >= 0 && L_black[j + 3] - L_black[j] >= 0 && L_black[j + 2] - L_black[j] >= 0

                )
                {
                    if (L_middle_point_times >= 1)
                    {
                        if (abs(j - L_middle_point[0]) > 10)
                        {
                            L_middle_point_flag = 0;
                            break;
                        }
                    }
                    L_middle_point[0] = j;
                    L_middle_point[1] = L_black[j];
                    L_middle_point_flag = 1;
                    L_middle_point_times++;
                    break;
                }
                else
                {
                    L_middle_point_flag = 0;
                }
            }

        }
        void find_leftup_point(byte startline, byte endline)
        {
            byte j;
            for (j = startline; j < endline; j++)
            {
                if (j >= 2 && j < finalend && p_Pixels[j + 1][ Pro_value(L_black[j + 1] - 3)] == White && LoseL[j + 2] == 0 && LoseL[j + 3] == 0 && L_black[j] - L_black[j + 1] > 5&& L_black[j - 1] - L_black[j] >= 0 && L_black[j - 2] - L_black[j - 1] >= 0 && L_black[j + 1] - L_black[j + 2] >= 0 && L_black[j + 2] - L_black[j + 3] >= 0)
                //if(j>=2&& L_black[j] - L_black[j + 1] > 5 && p_Pixels[j + 1][ Pro_value(L_black[j + 1] - 3)] == White && j < finalend && LoseL[j + 2] == 0 && LoseL[j + 3] == 0)
                {
                    L_up_point[0] = (byte)(j + 1);
                    L_up_point[1] = L_black[j + 1];
                    L_up_point_flag = 1;
                    L_up_point_times++;
                    break;

                }
                else
                {
                    L_up_point_flag = 0;
                }
            }
        }
        void find_rightup_point(byte startline, byte endline)
        {
            byte j;
            for (j = startline; j < endline; j++)
            {
                if (j >= 2 && j < finalend && abs(R_black[j + 2] - R_black[j + 3]) <= 3 && abs(R_black[j+1]-R_black[j+2])<=3&& p_Pixels[j + 1][ Pro_value(R_black[j + 1] + 3)] == White && LoseR[j + 2] == 0 && LoseR[j + 3] == 0 && R_black[j+1] - R_black[j ] > 5 && R_black[j] - R_black[j-1] >= 0 && R_black[j-1] - R_black[j - 2] >= 0 && R_black[j + 2] - R_black[j + 1] >= 0 && R_black[j + 3] - R_black[j + 2] >= 0)
                {
                    R_up_point[0] = (byte)(j + 1);
                    R_up_point[1] = R_black[j + 1];
                    R_up_point_flag = 1;
                    R_up_point_times++;
                    break;

                }
                else
                {
                    R_up_point_flag = 0;
                }
            }
        }
        //环岛状态3找右下拐点--左环岛
        void Track_3_find_rightdown_point(byte startline, byte endline)
        {
            byte j;
            for (j = startline; j < endline; j++)
            {
                if (j >= 2&& j+1<finalend &&LoseR[j+2]==0&&LoseR[j+3]==0&& p_Pixels[j -1][ R_black[j +1]] == White && p_Pixels[j+3][R_black[j+1]]==Black && R_black[j + 1] - R_black[j] > 4 && abs(R_black[j - 1] - R_black[j - 2]) < 3 && abs(R_black[j] - R_black[j - 1]) < 3)
                {
                    R_down_point[0] = (byte)(j + 1);
                    R_down_point[1] = R_black[j + 1];
                    R_down_point_flag = 1;
                    R_down_point_times++;
                    break;
                }
                else
                {
                    R_down_point_flag = 0;
                }
            }
        }
        //环岛状态3找左下拐点--右环岛
        void Track_3_find_leftdown_point(byte startline,byte endline)
        {
            byte j;
            for (j = startline; j < endline; j++)
            {
                if (j >= 2 && j + 1 < finalend && LoseL[j + 2] == 0 && LoseL[j + 3] == 0 && p_Pixels[j - 1][ L_black[j + 1]] == White && p_Pixels[j + 3][ L_black[j + 1]] == Black && L_black[j] - L_black[j+1] > 4 && abs(L_black[j - 1] - L_black[j - 2]) < 3 && abs(L_black[j] - L_black[j - 1]) < 3)
                {
                    L_down_point[0] = (byte)(j + 1);
                    L_down_point[1] = L_black[j + 1];
                    L_down_point_flag = 1;
                    L_down_point_times++;
                    break;
                }
                else
                {
                    L_down_point_flag = 0;
                }
            }
        }
        void find_rightmiddle_point(byte startline, byte endline)
        {
            byte j;

            for (j = startline; j <= endline; j++)
            {

                //找左拐点
                if (j >= 5 && R_black[j] - R_black[j-5] > 0 && R_black[j] - R_black[j-4] > 0 && R_black[j] - R_black[j-3] > 0 && R_black[j - 4] - R_black[j - 5] >= 0
                && R_black[j - 3] - R_black[j - 4] >= 0 && R_black[j - 2] - R_black[j - 3] >= 0 &&
                R_black[j] - R_black[j+4] >= 0 && R_black[j] - R_black[j+3] >= 0 && R_black[j] - R_black[j+2] >= 0

                )
                {
                    if (R_middle_point_times >= 1)
                    {
                        if (abs(j - R_middle_point[0]) > 10)
                        {
                            R_middle_point_flag = 0;
                            break;
                        }
                    }
                    R_middle_point[0] = j;
                    R_middle_point[1] = R_black[j];
                    R_middle_point_flag = 1;
                    R_middle_point_times++;
                    break;
                }
                else
                {
                    R_middle_point_flag = 0;
                }
            }

        }
        void Clear_Flag1()
        {
            Field_num = 0;
            Field_num1 = 0;
            L_down_point_times = 0;
            //L_middle_point_times = 0;
            L_up_point_times = 0;
            R_down_point_times = 0;
            R_middle_point_times = 0;
            R_up_point_times = 0;
        }
        void Clear_FlagAll()
        {
            Track_Dir = 0;
            Track_State = 0;
            Track_Num--;
            Field_num = 0;
            Clear_Flag1();
            L_down_point_flag = 0;
            L_down_point_flag = 0;
            L_middle_point_flag = 0;
            L_middle_point_last_flag = 0;
            L_up_point_flag = 0;
            L_up_point_last_flag = 0;
            R_down_point_flag = 0;
            R_down_point_last_flag = 0;
            R_middle_point_flag = 0;
            R_middle_point_last_flag = 0;
            R_up_point_flag = 0;
            R_up_point_last_flag = 0;
        }
        //状态0
        //环岛检测
        void Island_Check()
        {
            byte j = 0;
            int K1, K2;
            if (Track_Num >= 1)
            {
                //右线连续行>=50才检测有无左环岛
                if (long_turn_flag_right >= 50&&Rstart<=20 )
                {

                    regression(R_black, Rstart, 50, 2,&parameterB,&parameterA);
                    K1 = (int)(parameterB * 100);
                    regression(R_black, Rstart, 40, 2,&parameterB,&parameterA);
                    K2 = (int)(parameterB * 100);
                    if (abs(K1 - K2) < 8)    //认为右线是直线
                    {
                        for (j = 3; j < 50; j++)
                        {
                            if (j >= 2 && p_Pixels[j + 1][ Pro_value(L_black[j] + 2)] == White && abs(L_black[j - 1] - L_black[j - 2]) <= 3 && abs(L_black[j] - L_black[j - 1]) <= 3 && (L_black[j + 1] - L_black[j] >= 3)
                        && p_Pixels[j][ Max_Col - 1] == White && LoseL[j - 2] == 0 && LoseL[j - 1] == 0 && LoseL[j] == 0)
                            {
                                Field_num++;
                                if (Field_num >= 3)  //连续3场都检测到
                                {
                                    Field_num = 0;
                                    Track_Dir = 1;
                                    Track_State = 1;
                                    //Console.WriteLine("检测到左环岛" + j);
                                    break;
                                }

                            }

                        }
                    }
                }
                //左线连续行>=50并且左线方差<=1右线方差>=10 才检测有无右环岛
                else if (long_turn_flag_left >= 50 &&Lstart<=20)
                {
                    regression(L_black, Lstart, 50, 2,&parameterB,&parameterA);
                    K1 = (int)(parameterB * 100);
                    regression(L_black, Lstart, 40, 2,&parameterB,&parameterA);
                    K2 = (int)(parameterB * 100);

                    if (abs(K1 - K2) < 8)//认为左线是直线
                    {
                        //Console.WriteLine("zuo线差值" + abs(K1 - K2));
                        for (j = 3; j < 50; j++)
                        {
                            if (j >= 2 && p_Pixels[j + 1][ Pro_value(R_black[j] - 2)] == White && abs(R_black[j - 1] - R_black[j - 2]) <= 3 && abs(R_black[j] - R_black[j - 1]) <= 3 && (R_black[j] - R_black[j + 1] >= 3)
                        && p_Pixels[j][ Min_Col + 1] == White && LoseR[j - 2] == 0 && LoseR[j - 1] == 0 && LoseR[j] == 0)
                            {
                                Field_num++;
                                if (Field_num >= 3)  //连续3场都检测到
                                {
                                    Field_num = 0;
                                    Track_Dir = 2;
                                    Track_State = 1;
                                    ////Console.WriteLine("检测到右环岛" + j);
                                    break;
                                }

                            }

                        }
                    }
                }
            }


        }
        //状态1
        //左环岛--找左下拐点和左中拐点并拉线 找不到左下和左中拐点或左下未找到左中拐点行数过小 进入状态2
        void Island_Start()
        {
            byte jj;
            if (Track_Dir == 1)   //为左环岛
            {
                //寻找左下拐点
                find_leftdown_point(0, 50);
                if (L_down_point_times == 0)  //一场都没找着
                {
                    L_down_point_flag = 0;
                    L_down_point_last_flag = 0;
                    L_down_point[0] = 0;
                    L_down_point[1] = L_black[0];
                }
                else if (L_down_point_times >= 1 && L_down_point_flag == 0 && L_down_point_last_flag == 1)  //本场未找着但上场找着
                {

                    for (jj = 2; jj < 40; jj++)   //从下往上寻找
                    {
                        if (LoseL[jj] == 0 && LoseL[jj + 1] == 1 && LoseL[jj + 2] == 1)
                        {
                            L_down_point[0] = jj;
                            L_down_point[1] = L_black[jj];
                            L_down_point_flag = 1;
                            L_down_point_times++;
                            break;
                        }
                        else
                        {

                            L_down_point[0] = 0;
                            L_down_point[1] = L_black[0];
                            L_down_point_flag = 0;
                        }
                    }

                }
                else if (L_down_point_flag == 0 && L_down_point_last_flag == 0)
                {
                    L_down_point[0] = 0;
                    L_down_point[1] = L_black[0];
                }
                //Console.WriteLine("找到状态"+ L_down_point_flag+"zuo下" + L_down_point[0]);

                //寻找左中拐点
                if (L_down_point_flag == 1)
                    find_leftmiddle_point((byte)(L_down_point[0] + 5), 55);
                else
                {
                    find_leftmiddle_point(10, 55);
                }

                if (L_middle_point_times == 0)  //一次都未找着
                {
                    for (jj = 59; jj > 0; jj--)
                    {
                        if (LoseR[jj] == 0) break;
                    }
                    L_middle_point[0] = jj;
                    L_middle_point[1] = Pro_value(L_down_point[1] + R_black[L_down_point[0]] - R_black[L_middle_point[0]]);

                }
                else if (L_middle_point_flag == 0 && L_middle_point_last_flag == 1) //上一场找着本场未找着 沿用上一场
                {

                }
                else if (L_middle_point_flag == 0 && L_middle_point_last_flag == 0)  //连续两场都没找着
                {

                    L_middle_point[0] = 0;
                    L_middle_point[1] = L_black[0];

                }
                //Console.WriteLine("zuo中" + L_middle_point[0]);
                //Console.WriteLine("zuo下当前flag" + L_down_point_flag + "zuo下上场flag" + L_down_point_last_flag);
                //Console.WriteLine("zuo中当前flag" + L_middle_point_flag + "zuo中上场flag" + L_middle_point_last_flag);
                Fill_Line(L_down_point[1], L_down_point[0], L_middle_point[1], L_middle_point[0], 0);
                if (L_middle_point_flag == 1 && L_middle_point[0] <= 40 && L_down_point_flag == 0)
                {
                    Field_num++;
                    if (Field_num >= 2)  //连续检测两场图像
                    {

                        Clear_Flag1();
                        Track_State = 2;
                        //Console.WriteLine("只找到左中拐点 进入状态2");
                    }
                }
                if (L_middle_point_flag == 0 && L_middle_point_last_flag == 0 && L_down_point_flag == 0 && L_down_point_last_flag == 0 && L_down_point_times >= 1 && L_middle_point_times >= 1)
                {

                    Clear_Flag1();
                    Track_State = 2;
                    //Console.WriteLine("全部未找到 进入状态2");
                }

                //更新场标志位
                L_down_point_last_flag = L_down_point_flag;
                L_middle_point_last_flag = L_middle_point_flag;
            }


            else if(Track_Dir==2) //为右环岛
            {
                find_rightdown_point(0, 50);
                if (R_down_point_times == 0)  //一场都没找着
                {
                    R_down_point_flag = 0;
                    R_down_point_last_flag = 0;
                    R_down_point[0] = 0;
                    R_down_point[1] = R_black[0];
                }
                else if (R_down_point_times >= 1 && R_down_point_flag == 0 && R_down_point_last_flag == 1)  //本场未找着但上场找着
                {

                    for (jj = 2; jj < 40; jj++)   //从下往上寻找
                    {
                        if (LoseR[jj] == 0 && LoseR[jj + 1] == 1 && LoseR[jj + 2] == 1)
                        {
                            R_down_point[0] = jj;
                            R_down_point[1] = R_black[jj];
                            R_down_point_flag = 1;
                            R_down_point_times++;
                            break;
                        }
                        else
                        {

                            R_down_point[0] = 0;
                            R_down_point[1] = R_black[0];
                            R_down_point_flag = 0;
                        }
                    }

                }
                else if (R_down_point_flag == 0 && R_down_point_last_flag == 0)
                {
                    R_down_point[0] = 0;
                    R_down_point[1] = R_black[0];
                }
                //Console.WriteLine("you下" + R_down_point[0]);
                if (R_down_point_flag == 1)
                    find_rightmiddle_point((byte)(R_down_point[0] + 5), 55);
                else
                {
                    find_rightmiddle_point(10, 55);
                }

                if (R_middle_point_times == 0)  //一次都未找着
                {
                    for (jj = 59; jj > 0; jj--)
                    {
                        if (LoseL[jj] == 0) break;
                    }
                    R_middle_point[0] = jj;
                    R_middle_point[1] = Pro_value(R_down_point[1] + L_black[R_down_point[0]] - L_black[R_middle_point[0]]);

                }
                else if (R_middle_point_flag == 0 && R_middle_point_last_flag == 1) //上一场找着本场未找着 沿用上一场
                {

                }
                else if (R_middle_point_flag == 0 && R_middle_point_last_flag == 0)  //连续两场都没找着
                {

                    R_middle_point[0] = 0;
                    R_middle_point[1] = R_black[0];//Pro_value(L_down_point[1] + R_black[L_down_point[0]] - R_black[L_middle_point[0]]);

                }
                //Console.WriteLine("you中" + R_middle_point[0]);
                //Console.WriteLine("you下当前flag" + R_down_point_flag + "you下上场flag" + R_down_point_last_flag);
                //Console.WriteLine("you中当前flag" + R_middle_point_flag + "you中上场flag" + R_middle_point_last_flag);
                Fill_Line(R_down_point[1], R_down_point[0], R_middle_point[1], R_middle_point[0], 1);
                if (R_middle_point_flag == 1 && R_middle_point[0] <= 40 && R_down_point_flag == 0)
                {
                    Field_num++;
                    if (Field_num >= 2)  //连续检测两场图像
                    {

                        Clear_Flag1();
                        Track_State = 2;
                        //Console.WriteLine("只找到右中拐点 进入状态2");
                    }
                }
                if (R_middle_point_flag == 0 && R_middle_point_last_flag == 0 && R_down_point_flag == 0 && R_down_point_last_flag == 0 && R_down_point_times >= 1 && R_middle_point_times >= 1)
                {

                    Clear_Flag1();
                    Track_State = 2;
                    //Console.WriteLine("全部未找到 进入状态2");
                }

                //更新场标志位
                R_down_point_last_flag = R_down_point_flag;
                R_middle_point_last_flag = R_middle_point_flag;
            }
        }
        //状态2
        //左环岛--找左中拐点拉线 找到左上拐点并且找不到左中拐点 进入状态3
        void Island_Enter1()
        {

            if (Track_Dir == 1)
            {
                find_leftmiddle_point(L_down_point[0], 40);
                if (L_middle_point_flag == 1)
                    find_leftup_point((byte)(L_middle_point[0] + 2), 50);
                else if (L_middle_point_flag == 0)
                {
                    find_leftup_point(12, 50);
                }
                //Console.WriteLine("zuo中当场flag" + L_middle_point_flag + "zuo中上场flag" + L_middle_point_last_flag);
                //Console.WriteLine("zuo上当场flag" + L_up_point_flag + "zuo上上场flag" + L_up_point_last_flag);
                //Console.WriteLine("zuo中" + L_middle_point[0]);
                //Console.WriteLine("zuo上" + L_up_point[0]);


                Fill_Line(L_black[0], 0, L_middle_point[1], L_middle_point[0], 0);
                Fill_Line(R_black[0], 0, L_up_point[1], L_up_point[0], 1);
                if (L_up_point_flag == 1 && L_up_point[0] <= 50 && L_middle_point_flag == 0 && L_middle_point_last_flag == 0)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 3;
                        //Console.WriteLine("只找到左上拐点 进入状态3");
                    }
                }
                if (L_up_point_flag == 1 && L_up_point[0] <= 46)
                {
                    Field_num1++;
                    if (Field_num1 >= 2)
                    {
                        Clear_Flag1();
                        Track_State = 3;
                        //Console.WriteLine("左上拐点过小 进入状态3");
                    }
                }

                L_up_point_last_flag = L_up_point_flag;
                L_middle_point_last_flag = L_middle_point_flag;
            }
            else if(Track_Dir==2)
            {
                find_rightmiddle_point(R_down_point[0], 40);
                if (R_middle_point_flag == 1)
                    find_rightup_point((byte)(R_middle_point[0] + 2), 50);
                else if (R_middle_point_flag == 0)
                {
                    find_rightup_point(12, 50);
                }
                //Console.WriteLine("you中当场flag" + R_middle_point_flag + "you中上场flag" + R_middle_point_last_flag);
                //Console.WriteLine("you上当场flag" + R_up_point_flag + "you上上场flag" + R_up_point_last_flag);
                //Console.WriteLine("you中" + R_middle_point[0]);
                //Console.WriteLine("you上" + R_up_point[0]);


                Fill_Line(R_black[0], 0, R_middle_point[1], R_middle_point[0], 1);
                Fill_Line(L_black[0], 0, R_up_point[1], R_up_point[0], 0);
                if (R_up_point_flag == 1 && R_up_point[0] <= 50 && R_middle_point_flag == 0 && R_middle_point_last_flag == 0)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 3;
                        //Console.WriteLine("只找到左上拐点 进入状态3");
                    }
                }
                if (R_up_point_flag == 1 && R_up_point[0] <= 46)
                {
                    Field_num1++;
                    if (Field_num1 >= 2)
                    {
                        Clear_Flag1();
                        Track_State = 3;
                        //Console.WriteLine("右上拐点过小 进入状态3");
                    }
                }
                R_up_point_last_flag = R_up_point_flag;
                R_middle_point_last_flag = R_middle_point_flag;
            }

        }
        //状态3
        //左环岛--切换扫线方式保证右线连续 找到右下拐点拉线 当右下拐点行数过小或者几场都找不到后进入状态4
        void Island_Enter2()
        {
            int K = 0;
            if (Track_Dir == 1)
            {
                Track_3_find_rightdown_point(0, 50);
                //Console.WriteLine("you下" + R_down_point[0]);
                Fill_Line(R_black[0], 0, R_down_point[1], R_down_point[0], 1);//补右线
                //Console.WriteLine("you下当场flag" + R_down_point_flag + "you下上场flag" + R_down_point_last_flag);
                if (R_down_point_flag == 1 && R_down_point[0] <= 20)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 4;
                        //Console.WriteLine("右下点行过小 进入状态4");
                    }
                }
                if (R_down_point_flag == 0 && R_down_point_last_flag == 0 && R_down_point_times >= 1)
                {

                    Clear_Flag1();
                    Track_State = 4;
                    //Console.WriteLine("未找到右下点 进入状态4");
                }
                regression(L_black, 0, finalend-5, 2,&parameterB,&parameterA);
                K = (int)(parameterB * 100);
                ////Console.WriteLine("zuoK" + K);
                if(R_down_point_flag == 0 && R_down_point_last_flag == 0 && R_down_point_times==0 &&Lstart>=40&&Rstart<=20&&abs(K)<=1)
                {
                    Clear_Flag1();
                    Track_State = 4;
                    //Console.WriteLine("完全未找到右下点 进入状态4");
                }
            }
            else if(Track_Dir==2)
            {
                Track_3_find_leftdown_point(0, 50);
                //Console.WriteLine("zuo下" + L_down_point[0]);
                Fill_Line(L_black[0], 0, L_down_point[1], L_down_point[0], 0);//补左线
                //Console.WriteLine("zuo下当场flag" + L_down_point_flag + "zuo下上场flag" + L_down_point_last_flag);
                if (L_down_point_flag == 1 && L_down_point[0] <= 20)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 4;
                        //Console.WriteLine("左下点行过小 进入状态4");
                    }
                }
                if (L_down_point_flag == 0 && L_down_point_last_flag == 0 && L_down_point_times >= 1)
                {

                    Clear_Flag1();
                    Track_State = 4;
                    //Console.WriteLine("未找到左下点 进入状态4");
                }
                regression(R_black, 0, finalend - 5, 2,&parameterB,&parameterA);
                K = (int)(parameterB * 100);
                if (L_down_point_flag == 0 && L_down_point_last_flag == 0 && L_down_point_times == 0 && Rstart >= 40 && Lstart <= 20 && abs(K) <= 1)
                {
                    Clear_Flag1();
                    Track_State = 4;
                    //Console.WriteLine("完全未找到右下点 进入状态4");
                }
            }

        }
        //状态4 在环中
        //左环岛--找右中拐点类似三角形状 找到后并且行数过小 进入状态5
        void Island_In()
        {
            byte j;
            if (Track_Dir == 1)
            {
                for (j = 15; j < 50; j++)
                {


                    if ((R_black[j] > R_black[j - 1] || R_black[j] > R_black[j + 1]) && R_black[j - 3] < R_black[j] && R_black[j + 3] < R_black[j] && R_black[j - 2] <= R_black[j - 1] && R_black[j - 3] <= R_black[j - 2] && R_black[j + 1] >= R_black[j + 2] && R_black[j + 2] >= R_black[j + 3] && p_Pixels[j][ Pro_value(R_black[j] - 2)] == Black && p_Pixels[j + 3][ R_black[j]] == White && p_Pixels[j - 3][ R_black[j]] == White && p_Pixels[j][ Pro_value(R_black[j] + 3)] == White)

                    {

                        R_middle_point[0] = j;
                        R_middle_point[1] = R_black[j];
                        R_middle_point_times++;
                        R_middle_point_flag = 1;
                        break;
                    }
                    else
                    {
                        R_middle_point_flag = 0;
                    }
                }
                if (R_middle_point_flag == 1 && R_middle_point[0] <= 45)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 5;
                        //Console.WriteLine("找到右上拐点 进入状态5");
                    }
                }

                //Console.WriteLine("you中" + R_middle_point[0]);
            }
            else if(Track_Dir==2)
            {
                for (j = 15; j < 50; j++)
                {

                    if ((L_black[j-1] > L_black[j] || L_black[j+1] > L_black[j]) && L_black[j - 3] > L_black[j] && L_black[j + 3] > L_black[j] && L_black[j - 2] >= L_black[j - 1] && L_black[j - 3] >= L_black[j - 2] && L_black[j + 1] <= L_black[j + 2] && L_black[j + 2] <= L_black[j + 3] && p_Pixels[j][ Pro_value(L_black[j] + 2)] == Black && p_Pixels[j + 3][ L_black[j]] == White && p_Pixels[j - 3][ L_black[j]] == White && p_Pixels[j][ Pro_value(L_black[j] - 3)] == White)

                    {

                        L_middle_point[0] = j;
                        L_middle_point[1] = L_black[j];
                        L_middle_point_times++;
                        L_middle_point_flag = 1;
                        break;
                    }
                    else
                    {
                        L_middle_point_flag = 0;
                    }
                }
                if (L_middle_point_flag == 1 && L_middle_point[0] <= 45)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 5;
                        //Console.WriteLine("找到左上拐点 进入状态5");
                    }
                }
                //Console.WriteLine("zuo中" + L_middle_point[0]);
            }

        }
        //状态5
        //左环岛--找右中拐点再找左上拐点拉线 右中拐点未找到或行数过小 进入状态6
        void Island_Out1()
        {

            byte j = 0;
            if (Track_Dir == 1)
            {
                for (j = 4; j <= 45; j++)
                {


                    if ((R_black[j] > R_black[j - 1] || R_black[j] > R_black[j + 1]) && R_black[j - 3] < R_black[j] && R_black[j + 3] < R_black[j] && R_black[j - 2] <= R_black[j - 1] && R_black[j - 3] <= R_black[j - 2] && R_black[j + 1] >= R_black[j + 2] && R_black[j + 2] >= R_black[j + 3] && p_Pixels[j][ Pro_value(R_black[j] - 2)] == Black && p_Pixels[j + 3][ R_black[j]] == White && p_Pixels[j - 3][ R_black[j]] == White && p_Pixels[j][ Pro_value(R_black[j] + 3)] == White)

                    {

                        R_middle_point[0] = j;
                        R_middle_point[1] = R_black[j];
                        R_middle_point_times++;
                        R_middle_point_flag = 1;
                        break;
                    }
                    else
                    {
                        R_middle_point_flag = 0;
                    }
                }
                if (R_middle_point_times == 0)
                {
                    R_middle_point[0] = 0;
                    R_middle_point[1] = R_black[0];
                    R_middle_point_flag = 0;
                }
                else if (R_middle_point_flag == 0 && R_middle_point_last_flag == 1)  //本场未找着上场找着
                {

                    R_middle_point_flag = 0;

                }

                for (j = 30; j < 57; j++)
                {
                    if (p_Pixels[j - 1][ 87] == White && p_Pixels[j][ 87] == White && p_Pixels[j + 1][ 87] == Black && p_Pixels[j + 2][ 87] == Black)
                    {
                        L_up_point[0] = j;
                        L_up_point[1] = 87;
                        L_up_point_flag = 1;
                        break;
                    }
                    else
                    {
                        L_up_point_flag = 0;
                    }
                }
                //判断是否提前进入状态6
                for (j = 0; j < 50; j++)
                {
                    if (p_Pixels[j][ Min_Col] == White && p_Pixels[j + 1][ Min_Col] == White && p_Pixels[j + 2][ Min_Col] == Black && p_Pixels[j + 3][ Min_Col] == Black)
                    {
                        R_down_point[0] = j;
                        R_down_point[1] = Max_Col;
                        R_down_point_flag = 1;
                        R_down_point_times++;
                        break;
                    }
                    else R_down_point_flag = 0;
                }


                if (L_up_point_flag == 0)
                {
                    L_up_point[0] = 59;
                    L_up_point[1] = 89;
                }
                //Console.WriteLine("you中" + R_middle_point[0]);
                //Console.WriteLine("you中列" + R_middle_point[1]);
                ////Console.WriteLine("zuo上" + L_up_point[0]);
                ////Console.WriteLine("zuo上列" + L_up_point[1]);
                ////Console.WriteLine("you中当场flag" + R_middle_point_flag + "you中上场flag" + R_middle_point_last_flag);

                Fill_Line(R_middle_point[1], R_middle_point[0], L_up_point[1], L_up_point[0], 1);

                if (R_middle_point[0] <= 15 && R_middle_point_flag == 1)
                {

                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("右中点行过小 进入状态6");

                }
                if (R_middle_point_flag == 0 && R_middle_point_last_flag == 0 && R_middle_point_times >= 1)
                {
                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("找不到右中点 进入状态6");
                }
                if(R_middle_point_flag == 0 && R_middle_point_last_flag == 0&& R_down_point_flag==1&&R_down_point[0]<=40)
                {
                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("提前 进入状态6");

                }
                R_middle_point_last_flag = R_middle_point_flag;
                R_down_point_last_flag = R_down_point_flag;
            }
            else if(Track_Dir==2)
            {
                for (j = 4; j <=55; j++)
                {

                    if ((L_black[j - 1] > L_black[j] || L_black[j + 1] > L_black[j]) && L_black[j - 3] > L_black[j] && L_black[j + 3] > L_black[j] && L_black[j - 2] >= L_black[j - 1] && L_black[j - 3] >= L_black[j - 2] && L_black[j + 1] <= L_black[j + 2] && L_black[j + 2] <= L_black[j + 3] && p_Pixels[j][ Pro_value(L_black[j] + 2)] == Black && p_Pixels[j + 3][ L_black[j]] == White && p_Pixels[j - 3][ L_black[j]] == White && p_Pixels[j][ Pro_value(L_black[j] - 3)] == White)

                    {

                        L_middle_point[0] = j;
                        L_middle_point[1] = L_black[j];
                        L_middle_point_times++;
                        L_middle_point_flag = 1;
                        break;
                    }
                    else
                    {
                        L_middle_point_flag = 0;
                    }
                }
                if (L_middle_point_times == 0)
                {
                    L_middle_point[0] = 0;
                    L_middle_point[1] = L_black[0];
                    L_middle_point_flag = 0;
                }
                else if (L_middle_point_flag == 0 && L_middle_point_last_flag == 1)  //本场未找着上场找着
                {

                    L_middle_point_flag = 0;

                }

                for (j = 30; j < 57; j++)
                {
                    if (p_Pixels[j - 1][ Min_Col] == White && p_Pixels[j][ Min_Col] == White && p_Pixels[j + 1][ Min_Col] == Black && p_Pixels[j + 2][ Min_Col] == Black)
                    {
                        R_up_point[0] = j;
                        R_up_point[1] = Min_Col;
                        R_up_point_flag = 1;
                        break;
                    }
                    else
                    {
                        R_up_point_flag = 0;
                    }
                }
                if (R_up_point_flag == 0)
                {
                    R_up_point[0] = 59;
                    R_up_point[1] = Min_Col;
                }
                //判断是否提前进入状态6
                for (j = 0; j < 50; j++)
                {
                    if (p_Pixels[j][ Max_Col] == White && p_Pixels[j + 1][ Max_Col] == White && p_Pixels[j + 2][ Max_Col] == Black && p_Pixels[j + 3][ Max_Col] == Black)
                    {
                        L_down_point[0] = j;
                        L_down_point[1] = Max_Col;
                        L_down_point_flag = 1;
                        L_down_point_times++;
                        break;
                    }
                    else L_down_point_flag = 0;
                }
                Fill_Line(L_middle_point[1], L_middle_point[0], R_up_point[1], R_up_point[0], 0);

                if (L_middle_point[0] <= 10 && L_middle_point_flag == 1)
                {

                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("左中点行过小 进入状态6");

                }
                if (L_middle_point_flag == 0 && L_middle_point_last_flag == 0 && L_middle_point_times >= 2)
                {
                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("找不到左中点 进入状态6");
                }
                if (L_middle_point_flag == 0 && L_middle_point_last_flag == 0 && L_down_point_flag == 1 && L_down_point[0] <= 40)
                {
                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("提前 进入状态6");

                }
                L_middle_point_last_flag = L_middle_point_flag;
                L_down_point_last_flag = L_down_point_flag;


            }

        }
        //状态6
        //左环岛--找左上拐点 找到并且行数过小进入状态7
        void Island_Out2()
        {
            byte j;

            if (Track_Dir == 1)
            {
                for (j = 30; j < 57; j++)
                {
                    if (p_Pixels[j - 1][ 87] == White && p_Pixels[j][ 87] == White && p_Pixels[j + 1][ 87] == Black && p_Pixels[j + 2][ 87] == Black)
                    {
                        L_middle_point[0] = j;
                        L_middle_point[1] = 87;
                        L_middle_point_flag = 1;
                        break;
                    }
                    else
                    {
                        L_middle_point_flag = 0;
                    }
                }
                if (L_middle_point_flag == 0)
                {
                    L_middle_point[0] = 59;
                    L_middle_point[1] = 89;
                }
                ////Console.WriteLine("找到" + L_middle_point_flag + "zuo中" + L_middle_point[0]);
                for(j=0;j<40;j++)
                {
                    if (p_Pixels[j][ Min_Col] == White && p_Pixels[j + 1][ Min_Col] == White && p_Pixels[j + 2][ Min_Col] == Black && p_Pixels[j + 3][ Min_Col] == Black)
                    {
                        R_down_point[0] = j;
                        R_down_point[1] = Min_Col;
                        R_down_point_flag = 1;
                        R_down_point_times++;
                        break;
                    }
                    else R_down_point_flag = 0;
                }
                find_leftup_point(0, 55);
                //Console.WriteLine("找到" + L_up_point_flag + "zuo上" + L_up_point[0]);
                Fill_Line(R_black[0], 0, L_middle_point[1], L_middle_point[0], 1);

                if (L_up_point[0] <= 53 && L_up_point_flag == 1 && L_up_point_times >= 3)
                {
                    Clear_Flag1();
                    Track_State = 7;
                    //Console.WriteLine("找到左上拐点 进入状态7");

                }
                //Console.WriteLine("找到"+ R_down_point_flag+"you下" + R_down_point[0]);
                if (R_down_point_flag==1&&R_down_point[0]<=15&&R_down_point_times>=2)
                {
                    Clear_Flag1();
                    Track_State = 7;
                    //Console.WriteLine("绝大部分在左侧 进入状态7");
                }

                L_up_point_last_flag = L_up_point_flag;
            }
            else if(Track_Dir==2)
            {
                for (j = 30; j < 57; j++)
                {

                    if (p_Pixels[j - 1][ Min_Col] == White && p_Pixels[j][ Min_Col] == White && p_Pixels[j + 1][ Min_Col] == Black && p_Pixels[j + 2][ Min_Col] == Black)
                    {
                        R_middle_point[0] = j;
                        R_middle_point[1] = Min_Col;
                        R_middle_point_flag = 1;
                        break;
                    }
                    else
                    {
                        R_middle_point_flag = 0;
                    }
                }
                if (R_middle_point_flag == 0)
                {
                    R_middle_point[0] = 59;
                    R_middle_point[1] = Min_Col;
                }

                for (j =10; j < 40; j++)
                {
                    if (p_Pixels[j][ Max_Col] == White && p_Pixels[j + 1][ Max_Col] == White && p_Pixels[j + 2][ Max_Col] == Black && p_Pixels[j + 3][ Max_Col] == Black)
                    {
                        L_down_point[0] = j;
                        L_down_point[1] = Max_Col;
                        L_down_point_flag = 1;
                        L_down_point_times++;
                        break;
                    }
                    else L_down_point_flag = 0;
                }
                //Console.WriteLine("找到"+ L_down_point_flag+"左下行" + L_down_point[0]);
                find_rightup_point(0, 55);
                //Console.WriteLine("找到" + R_up_point_flag + "you上" + R_up_point[0]);
                Fill_Line(L_black[0], 0, R_middle_point[1], R_middle_point[0], 0);
                if (R_up_point[0] <= 53 && R_up_point_flag == 1 &&R_up_point_times>=4&& R_up_point_times >= 3)
                {
                    Clear_Flag1();
                    Track_State = 7;
                    //Console.WriteLine("进入状态7");

                }
                //if (L_down_point_flag == 1 && L_down_point[0] <= 15 && L_down_point_times >= 2)
                //{
                //    Clear_Flag1();
                //    Track_State = 7;
                //    //Console.WriteLine("绝大部分在右侧 进入状态7");
                //}

            }
        }
        //状态7
        //左环岛--找左上拐点和左线第0行拉线 行数<=20出环岛
        void Island_Out3()
        {
            if (Track_Dir == 1)
            {
                find_leftup_point(0, 55);
                //Console.WriteLine("找到" + L_up_point_flag + "zuo上" + L_up_point[0]);
                Fill_Line(L_black[0], 0, L_up_point[1], L_up_point[0], 0);

                if (L_up_point_flag == 1 && L_up_point[0] <= 20)
                {

                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_FlagAll();
                        //Console.WriteLine("出环岛");
                    }


                }
                if (L_up_point_flag == 0 && L_up_point_last_flag == 0 && Lstart<=25&&Rstart<=25 &&Lstart!=0&&Rstart!=0)
                {
                    Field_num1++;
                    if (Field_num1 >= 2)
                    {

                        Clear_FlagAll();
                        //Console.WriteLine("未找到左上 出环岛");
                    }
                }
                L_up_point_last_flag = L_up_point_flag;
            }
            else if(Track_Dir==2)
            {
                find_rightup_point(0, 55);
                //Console.WriteLine("找到" + R_up_point_flag + "zuo上" + R_up_point[0]);
                Fill_Line(R_black[0], 0, R_up_point[1], R_up_point[0], 1);

                if (R_up_point_flag == 1 && R_up_point[0] <= 20)
                {

                    Field_num++;
                    if (Field_num >= 2)
                    {
                        Field_num = 0;
                        Clear_FlagAll();
                        //Console.WriteLine("出环岛");
                    }


                }
                if (R_up_point_flag == 0 && R_up_point_last_flag == 0 && Rstart <= 25 && Lstart <= 25 && Lstart != 0 && Rstart != 0)
                {
                    Field_num = 0;
                    Clear_FlagAll();
                    //Console.WriteLine("出环岛");
                }
                R_up_point_last_flag = R_up_point_flag;
            }


        }

        void Island()
        {
            byte j=0;
            if(Track_State==2)
            {
                L_Blank = R_Blank = 0;
                if(Track_Dir==1)
                for (j = 0; j < 60; j++)
                {
                    R_Edge_Scan(j);
                    R_Start_Scan(j);
                }
                else if(Track_Dir==2)
                    for (j = 0; j < 60; j++)
                    {
                        L_Edge_Scan(j);
                        L_Start_Scan(j);
                    }
            }
            else if(Track_State==3)
            {
                L_Blank = R_Blank = 0;
                if (Track_Dir==1)
                for (j = 0; j < 60; j++)
                {
                    L_Edge_Scan(j);
                    L_Start_Scan(j);
                }
                else if(Track_Dir==2)
                    for (j = 0; j < 60; j++)
                    {
                        R_Edge_Scan(j);
                        R_Start_Scan(j);
                    }
            }


            switch (Track_State)
            {

                case 0: Island_Check(); break;
                case 1: Island_Start(); break;
                case 2: Island_Enter1(); break;
                case 3: Island_Enter2(); break;
                case 4: Island_In();break;
                case 5: Island_Out1();break;
                case 6: Island_Out2(); break;
                case 7: Island_Out3(); break;

            }

        }


/*******************************其他要素判别*******************************/
/*
void add_i(byte a,byte b,byte c)
{
    longstr=a;
    shortstr=b;
    cir=c;
}


void single_an()
{
    byte end = finalend < 50 ? finalend : 50;
    byte i;
    byte single_error;
    byte start=end;
    error=0;
    for(i=11;i<end-3;i+=5)
    {
        single_error=LCenter[i] - my_Mid;
        error+=single_error;
        if(deviation(LCenter,i-3,i+3)>1 && start==end) start=i;
    }
    //pill(start);
    if(start>35) add_i(3,0,0);
    else if(start>20) add_i(0,3,0);
    else add_i(0,0,3);
}
void pd()//now_state   0停，1长直道，2短直道,3弯道，4环，5正常三岔，6三岔转向
{
    single_an();
    if(in_sancha==1)
    {
        if((top_y<40 || top_x>60 )&&  hen_sancha==1 && jin_sancha==0)
        {
            now_state=6;
            jin_sancha=1;
        }
        else if((top_y<40 || top_x>60 )&& hen_sancha==0 && jin_sancha==0)
        {
            now_state=5;
            jin_sancha=1;
        }
        else now_state=2;
    }
    else
    {
        if(longstr>2) now_state=1;
        else if(shortstr>2) now_state=2;
        else if(cir>2) now_state=3;
    }
}
*/


/*******************************分段打角*******************************/
void set_angle_line(byte num)//设置打角行
{
    byte end=finalend>40?40:finalend;
    if(num>end) angle_line=end-1;
    else if(num<=0) angle_line=0;
    else angle_line=num;
}

void get_angle_line()
{
    byte min=10;
    if(act_speed_y<200) set_angle_line(0);
    else if(act_speed_y<400)
    {
        if(act_speed_y<250)  set_angle_line((act_speed_y-200)/5+min);//10-20
        else if(act_speed_y<300) set_angle_line((act_speed_y-150)/8+min);//22-28
        else if(act_speed_y<320) set_angle_line((act_speed_y-120)/10+min);//28-30
        else if(act_speed_y<340) set_angle_line((act_speed_y-110)/10+min);//31-33
        else if(act_speed_y<360) set_angle_line((act_speed_y-100)/10+min);//34-36
        else if(act_speed_y<380) set_angle_line((act_speed_y+45)/15+min);//37-38
        else set_angle_line((act_speed_y+180)/20+min);//38-39
    }
    else set_angle_line(40);//40
}


/*****************************速度保护**********************************/
void Protect()
{
    if(p_Pixels[0][LCenter[0]+1]==Black&&p_Pixels[1][LCenter[1]+1]==Black)
    {
        Protect_num++;
    }
    else Protect_num=0;
    if(Protect_num>=10)  //连续两场图像都检测到
    {
        Protect_num=0;
        Is_Run=0;
    }
}
/*******************************主显示函数*******************************/
void Row_Process(byte Num)
{
    Center_Scan(Num);
}

void FieldProcess()
{
    if(!in_sancha && !in_sancha2 && !in_ruku && !Track_State) cross2();
    if(!in_shizi && !in_ruku && !Track_State) sancha();
    if(!in_sancha && !in_sancha2 && !in_shizi && !Track_State) ruku();
    //if(!in_sancha && !in_sancha2 && !in_shizi && !in_ruku) Island();

    //总变量 0无元素  1十字 2三岔 3环岛 4入库
    if(in_shizi) all_state=1;
    else if(in_sancha || in_sancha2) all_state=2;
    else if(Track_State) all_state=3;
    else if(in_ruku) all_state=4;
    else all_state=0;

    get_angle_line();
    //Protect();//速度保护
}
//每帧图像标志位更新
void Data_init()
{
    byte j;
    all_lose_end=0;
    all_lose=0;
    long_turn_flag_left=ROW;
    long_turn_flag_right=ROW;
    Lstart=0;
    Rstart=0;
    for(j=0;j<ROW;j++)
    {
        LoseL[j]=0;
        LoseR[j]=0;
    }
    L_Blank=0;
    R_Blank=0;
    finalend=ROW;
}
//图像处理函数
void SignalProcess()
{
    byte RowNum=0;
    Data_init();    //每帧图像参数初始化
    for (RowNum = 0; RowNum < ROW; RowNum++)
    {
        Row_Process(RowNum); //图像行处理,进行基本扫线，获取图像信息
        if (RowNum>=20&& abs(L_black[RowNum] - L_black[RowNum - 1]) >=5&&long_turn_flag_left==ROW) long_turn_flag_left = RowNum;
        if (RowNum >= 20 && abs(R_black[RowNum] - R_black[RowNum - 1]) >= 5&&long_turn_flag_right==ROW) long_turn_flag_right = RowNum;
        if (RowNum>=1&&p_Pixels[RowNum][R_black[RowNum]]==Black&&p_Pixels[RowNum-1][R_black[RowNum-1]]==Black&&Rstart==0) Rstart = RowNum-1;
        if (RowNum>=1&&p_Pixels[RowNum][L_black[RowNum]] == Black && p_Pixels[RowNum-1][L_black[RowNum-1]] == Black && Lstart == 0) Lstart = RowNum-1;
        if(finalend==ROW&&RowNum>=30&&L_black[RowNum]-R_black[RowNum]<10)
         {
                            finalend = (byte)(RowNum-1);

         }
        if(RowNum<40 && LoseL[RowNum] && LoseR[RowNum]) all_lose++;
        else if(all_lose_end==0) all_lose_end=RowNum;
    }
    rou_of_left=deviation(L_black,Lstart, 30,0);
    rou_of_right =deviation(R_black, Rstart, 30,0);
    Mid = (byte)((LCenter[0] + LCenter[1]) >> 1);   //更新每帧图像中值
    FieldProcess();     //场处理函数
    if(finalend<45)
    {
        regression(LCenter,0,finalend,1,&parameterB,&parameterA);
    }
    else
    {
        regression(LCenter,0,30,1,&parameterB,&parameterA);
    }
    midline_deviation2=deviation(LCenter,30,45,1);
    midline_deviation1=deviation(LCenter,20,35,1);
    midline_deviation=deviation(LCenter,10,25,1);

}
