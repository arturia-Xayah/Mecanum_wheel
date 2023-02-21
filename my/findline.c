#include "findline.h"
#include "pid.h"
//����ֵ����
#define Abs(a) (a>0?a:-a)
float k3;//���Ա���

byte angle_line=0;//�����
byte all_state=0; //�ܱ��� 0��Ԫ��  1ʮ�� 2���� 3���� 4���


byte my_Mid = (Max_Col + Min_Col) >> 1;
float midline_deviation; //���߷���
float midline_deviation1;
float midline_deviation2;
//
byte Lstart=0;
byte Rstart=0;
byte long_turn_flag_left=ROW; //����������
byte long_turn_flag_right=ROW;//����������
float rou_of_left=0;    //���߷���
float rou_of_right=0;   //���߷���

//
byte all_lose_end=0;
byte L_Blank=0;         //������
byte R_Blank=0;         //�Ҷ�����
/****************************�ٶȱ���*******************************/
byte Protect_num=0;//�ٶȱ���
int16 v_set[]={};
int16 error=0;
byte now_state=0;//��ǰ״̬ 0ͣ��1��ֱ����2��ֱ��,3�����4����5��������6����ת��,7���
byte longstr=0;
byte shortstr=0;
byte cir=0;
byte jin_sancha=0;//���������ж��ٶ�Ҫ��
float mid_k;//����б��
float mid_dev;//���߷���
/**************************��С���˷�����***************************/
float parameterA;//�ؾ�
float parameterB;//б��
/************************���/�������******************************/
byte in_ruku=0;
byte direct=0; //�͵㲹����
byte start_ruku=0; //��ʼ����
byte ruku_count=0; //�ڼ��ξ�����
//����top_x��top_y  ��������
/**************************��������********************************/

//�󻷵�



//�һ���




/**************************�������********************************/
byte top_x=0,top_y=0;
byte last_left = 0;
byte last_right = 0;
byte in_sancha = 0; //�ж��Ƿ�������·����
byte in_sancha2=0; //�ж��Ƿ�����������һ������
byte hen_sancha=1;//�ж�������������Ǻ��߻���ֱ��  0ֱ��   1���ߡ���ʼ1�����һȦֱ�ߵڶ�Ȧ���ߣ���֮
byte out_sancha=0; //����ж�
byte sancha_dir=1;//��ת������ת��0��ת1��ת
/*************************ʮ�ֱ���********************************/
byte all_lose=0;    //�������߶����ߵ�����
byte line_directe=0;    //ɨ�߷���0Ĭ�ϣ�1��2��
byte in_shizi=0;
byte left1_flag = 0;        //������
byte left2_flag = 0;        //������
byte right1_flag = 0;        //������
byte right2_flag = 0;        //������
byte shizi_flag = 0;      //�Ƿ����ʮ������
byte shizi_count = 0;    //��μ����ֹ���
byte yanchi = 99;        //��ʮ���ӳ�

byte last_left1=0;         //��һ֡������
byte last_left2=0;         //��һ֡������
byte last_right1=0;         //��һ֡������
byte last_right2=0;         //��һ֡������

//��ֵԽ�籣������ (byte��)
byte Pro_value(int value)
{
    if (value >= Max_Col)
        value = Max_Col;
    else if (value <= Min_Col)
        value = Min_Col;
    return (byte)value;
}

//��С���˷�
float regression(int8 *p, int8 startline, int8 endline, int8 step, float *k,float *b)
{//�����Ӧ��ָ�룬�����б�ʺͽؾ࣬�ؾ�ͽؾ�ͨ��ָ�뷵�أ�б��ֱ�ӷ��أ����ڵ�Ƭ�� ������СΪ1 ��󲻳���endline-startline
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

    averageX = sumX / sumlines;     //x��ƽ��ֵ
    averageY = sumY / sumlines;     //y��ƽ��ֵ

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

//������㺯��
float deviation(int8 *p, int startline, int endline,byte is_midline)  //�������� ���� ����ָ�� ��ʼ�� ��ֹ�� ���ط���
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

//������
byte angle_scan(int8 *p,byte row)
{
    byte k1=p[row+5]-p[row];
    byte k2=p[row-5]-p[row];
    if(k1*k2>0) return 1;
    else return 0;
}


////�����н��ж�
//byte vector(byte x1,byte y1,byte x2,byte y2,byte x3,byte y3)//x1 y1 Ϊ��������ʼ��,����0Ϊ��ǣ�1Ϊ�۽�
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

//���ߺ��������㲹�ߣ�
void Fill_Line(byte x1, byte y1, byte x2, byte y2, byte mx)  //mx=0������ mx=1������
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


/*****************ɨ��*******************/
void pill(byte row){ //����
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
    }//��������ɨ��



    //����ɨ��
    else
    {



        j = Pro_value(R_black[Num - 1]);
        if (p_Pixels[Num][ j] == Black) //��ɫ�����ɨ
        {
            for (; j <= Max_Col; j++)
                if (p_Pixels[Num][ j] == Black && p_Pixels[Num][ j + 1] == White)//�ҵ���ɫ
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
                if (p_Pixels[Num][ j] == White && p_Pixels[Num][j - 1] == Black)//�ҵ���ɫ
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
    }//��������ɨ��



    //����ɨ��
    else
    {



        j = Pro_value(L_black[Num - 1]);
        if (p_Pixels[Num][ j] == White) //��ɫ�����ɨ
        {
            for (; j <= Max_Col; j++)
                if (p_Pixels[Num][ j] == White && p_Pixels[Num][ j + 1] == Black)//�ҵ���ɫ
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
                if (p_Pixels[Num][j] == Black && p_Pixels[Num][ j - 1] == White)//�ҵ���ɫ
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

//����ɨ��
void Center_Scan(byte i) //��������ɨ��
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

/****************************ʮ��********************************/

byte k_pb(byte*p,byte start,byte end) //б��ʮ��б���б�,����б��ʮ�ַ���1
{
    return 1;
    float k1=(p[end]-p[start])/(end-start);
    float k2=regression(p,start-5,start,1,&parameterB,&parameterA);
    k3=k1-k2;
    if(k3> -0.5 && k3<0.5) return 1;
    else return 0;
}

void line_rescan(byte row,byte flag) // 0�� 1�� 2��
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
void conv1(byte row, byte mx)  //mx=0,��ɨ�£�mx=1����ɨ��
{
            byte y = 0, yy = 0;
            byte i=0;
            y = L_black[row];
            yy = R_black[row];

            if (mx==0 && row<40 && all_lose_end<10)
            {
                 if(left1_flag==0 && LoseL[row]==0 && p_Pixels[Pro_value(row+15)][L_black[row]-10]==White && p_Pixels[Pro_value(row+5)][Pro_value(y+3)]==White && p_Pixels[row-1][Pro_value(y+1)]==Black && p_Pixels[row-2][Pro_value(y+2)]==Black &&  L_black[row+3]>=L_black[row]+2 && L_black[row+4]>=L_black[row]+3  && L_black[row-3]>=L_black[row] && L_black[Pro_value(row-6)]>L_black[row-3])
                 {//����
                     left1_flag=row;
                     while(left1_flag>=1 &&L_black[left1_flag-1]<L_black[left1_flag] && p_Pixels[left1_flag-1][L_black[left1_flag-1]+1]==Black) left1_flag-=1;
                     while(left1_flag>=1 &&L_black[left1_flag+1]<L_black[left1_flag] && p_Pixels[left1_flag+1][L_black[left1_flag+1]+1]==Black) left1_flag+=1;
                  }
                 if(right1_flag==0 && LoseR[row]==0 && p_Pixels[Pro_value(row+15)][R_black[row]+10]==White && p_Pixels[Pro_value(row+5)][Pro_value(yy-3)]==White && p_Pixels[row-1][Pro_value(yy-1)]==Black && p_Pixels[row-2][Pro_value(yy-2)]==Black && R_black[row+3]<=R_black[row]-2 && R_black[row+4]<=R_black[row]-3  && R_black[row-3]<=R_black[row] && R_black[Pro_value(row-6)] <R_black[row-3])
                 {//����
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
                    {//����
                        row++;
                        i++;
                        if(i>3) break; //����������
                    }//����
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
void scan_high()//�ϵ�ɨ��
{
    left2_flag=right2_flag=0;
    byte i;
    for (i = 4; i < 50; i++)
    {
        if (i<finalend) conv1(i, 1);
    }
}

byte pro_judge(byte left,byte right)//��ֹ�������г�ʮ��   ����1����ʮ��  0��������   2 pass
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

           //ɨ�߷���
           if(line_directe==1) line_rescan(0,0);
           else if(line_directe==2) line_rescan(0,1);


           for (i = 2; i < 45; i++)//ɨ�µ�
           {
               if (i<finalend) conv1(i, 0);
           }
           scan_high();//ɨ�ϵ�

           if(left1_flag!=0 && right1_flag!=0 && left2_flag!=0 && right2_flag!=0 && abs(left1_flag-right1_flag)<5) line_directe=0; //��������ɨ��
           else if(right1_flag==0 && right2_flag==0)
           {
               if((left1_flag==0 && left2_flag==0)|| (left1_flag!=0 && left2_flag !=0 )) line_directe=0; //��������ɨ��
           }

           if(left1_flag==0 && right1_flag==0)
           {
               if(in_shizi==0 && yanchi>=3) return 0;
           }




           if(R_Blank>10 && left1_flag<40)
           {
               if(left2_flag!=0 && right1_flag==0  && right2_flag==0 && line_directe!=2)
               {
                   line_directe=2; //������������㣬����ɨ��
                   line_rescan(left1_flag,1);
               }
               if(left1_flag!=0 && right1_flag==0 && left2_flag!=0 && right2_flag!=0 && line_directe!=0)
               {
                   line_directe=0; //�������������м�ɨ��
                   line_rescan(left1_flag,2);
               }
               if(left1_flag!=0 && right1_flag==0 && line_directe!=2) //�����µ�
               {                                   //ֻ��һ�����µ�������ɨ��
                   line_directe=2;
                   line_rescan(left1_flag,1);
               }
               else if(left1_flag!=0 && right1_flag!=0 && (right1_flag-left1_flag)>=5 && line_directe!= 1) //���µ�����µ�ߣ�������ɨ��
               {
                   line_directe=1;
                   line_rescan(left1_flag,0);
               }
           }
           else if(L_Blank>10 && right1_flag<40) //�����µ�
           {
               if(left1_flag==0 && left2_flag==0 &&  right2_flag!=0 && line_directe!=1)
               {
                   line_directe=1; //�����ұ������㣬����ɨ��
                   line_rescan(right1_flag,0);
               }
               if(left1_flag==0 && right1_flag!=0 && left2_flag!=0 && right2_flag!=0 && line_directe!=0)
               {
                   line_directe=0; //�������������м�ɨ��
                   line_rescan(right1_flag,2);
               }
               if(left1_flag==0 && right1_flag!=0 && line_directe!= 1)//ֻ��һ�����µ�������ɨ��
               {
                   line_directe=1;
                   line_rescan(right1_flag,0);
               }

               else if (left1_flag!=0 && right1_flag!=0 && (left1_flag-right1_flag)>=5 && line_directe!= 2)//���µ�����µ�ߣ�������ɨ��
               {
                   line_directe=2;
                   line_rescan(right1_flag,1);
               }

           }
           if(line_directe !=0) scan_high();//ɨ�ϵ�

           //��ֹ�ϵ����µ����Ϸ�
           if(left1_flag!=0 && left2_flag !=0 && (L_black[left1_flag]-L_black[left2_flag])<=1) left2_flag=0;
           if(right1_flag!=0 && right2_flag !=0 && (R_black[right2_flag]-R_black[right1_flag])<=1) right2_flag=0;

           //��ֹ�ϵ��Ҵ�
           if(left2_flag!=0 && right2_flag!=0)
            {
               if(left2_flag-right2_flag>8) left2_flag=0;
               else if(right2_flag-left2_flag>8) right2_flag=0;
            }

           //��ֹ���ɨ��
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

               //��ֹֻ����һ���µ������´���ؽ��б�Եɨ��
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

           //��ֹ�ϵ��Ҵ�
           if(left2_flag!=0 && right2_flag!=0)
           {
               if(L_black[left2_flag]<=R_black[right2_flag]) left2_flag=right2_flag=0;
           }

           if(left1_flag==0 && right1_flag ==0)
           {
               if(left2_flag==0 && R_black[right2_flag]>70) right2_flag=0;
               if(right2_flag==0 && L_black[left2_flag]<20) left2_flag=0;
           }


           //���ԣ���ֹ�µ㳬ǰɨ
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

           //��ֹ�������г�ʮ��
           //if(pro_judge(left1_flag,right1_flag)==0) in_shizi=0;

//           pill(left1_flag);
//           pill(left2_flag);
//           pill(right1_flag);
//           pill(right2_flag);

           if (in_shizi == 0 && shizi_flag == 0 &&(left1_flag > 0 && left2_flag > 0 && right1_flag > 0 && right2_flag > 0)&& L_Blank>10 && R_Blank>10 && p_Pixels[45][my_Mid]==White)//|| (L_Blank >= 15 && right1_flag != 0 && R_Blank<=15)||( R_Blank >= 15 &&  left1_flag != 0 && L_Blank <=15)
           {//ֱ��
               cross_init();
           }
//           if(in_shizi == 0
//                   && shizi_flag == 0
//                   &&((left1_flag > 0 && left2_flag == 0 && right1_flag == 0 && right2_flag == 0 && L_Blank>10 && R_Blank>40 && p_Pixels[50][my_Mid-10]==White ) || (left1_flag == 0 && left2_flag == 0 && right1_flag > 0 && right2_flag == 0 && L_Blank>40 && R_Blank>10 && p_Pixels[50][my_Mid+10]==White )))//б��
//           {//б��(ֻ����һ���µ㣩
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
           if(in_shizi == 0 && shizi_flag == 0 && ((right1_flag==0 && left1_flag > 0 && left2_flag > 0 &&  R_Blank>30 && p_Pixels[left2_flag+5][L_black[left2_flag]-10]==White && k_pb(L_black,left1_flag,left2_flag)) || (left1_flag==0 && right1_flag > 0 && right2_flag > 0 && L_Blank>30 && p_Pixels[right2_flag+5][R_black[right2_flag]+10]==White && k_pb(R_black,right1_flag,right2_flag))))//б��
           {//б�루����һ�����µ㣩
              //if(pro_judge(left1_flag,right1_flag)==0) return 0;
              //if((rou_of_right<=10 && L_Blank<15) || (rou_of_left<=10 && R_Blank<15)) return 0;
              cross_init();
           }
           if (in_shizi == 1 || yanchi <= 5)
           {
               //����ʮ�ֿ��Ʋ���
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
               if((shizi_count>=10 && shizi_flag == 1 && all_lose<13) )//|| (shizi_flag == 1 && (L_Blank<10 || R_Blank<10>>) && all_lose<13 && left1_flag==0 && left2_flag==0 &&(right1_flag)))    //�뿪ʮ��
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

                   //��һ�ಹ��/��֪��������ĵ�/
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
                   /***********���߲���************/

                      if (left1_flag == 0 && left2_flag != 0)       //ֻ������
                      {
                          regression(L_black,left2_flag,left2_flag+5,1,&parameterB,&parameterA);
                          left1_flag=1;
                          if(parameterA>L_black[left2_flag])  L_black[1]=Pro_value(parameterA);
                          else L_black[1]=Pro_value(L_black[left2_flag]*2-parameterA);
                          //if(parameterB>0);
                      }
                      if (right1_flag == 0 && right2_flag != 0)     //ֻ������
                      {
                          regression(R_black,right2_flag,right2_flag+5,1,&parameterB,&parameterA);
                          right1_flag=1;
                          if(parameterA<R_black[right2_flag]) R_black[1]=Pro_value(parameterA);
                          else R_black[1]=Pro_value(R_black[right2_flag]*2-parameterA);
                          //if(parameterB<0);
                      }
                      if (left1_flag != 0 && left2_flag == 0)     //ֻ������
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
                      if (right1_flag != 0 && right2_flag == 0)     //ֻ������
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

               //�����
//               last_left1=left1_flag;
//               last_left2=left2_flag;
//               last_right1=right1_flag;
//               last_right2=right2_flag;
               //���߲���
               if (left1_flag != 0 && left2_flag != 0) Fill_Line(L_black[left1_flag], left1_flag, L_black[left2_flag], left2_flag, 0);          //�������¶��У�������
               if (right1_flag != 0 && right2_flag != 0) Fill_Line(R_black[right1_flag], right1_flag, R_black[right2_flag], right2_flag, 1);    //�������¶��У�������
           }
       }
/**************************����********************************/
byte angle_xr(byte*p,byte row)//б��������б��ʮ������
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
byte find_point(byte x,byte y,byte i,byte mx)// x,yΪ���꣬iΪά��(����),mx=0��������   mx=1��������
        {

            byte a;
            i = (i-1) / 2;
            byte start = 0;
            if (mx == 0) start = 0;
            else start = -i;
            for (a = start; a <= start+i; a++)//����
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
                for (a =1; a <= i; a++)//�Ҳ���
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
                for (a =1; a <= i; a++)//�����
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
    //if(hen_sancha==1) PID_Speed.SpeedSet=speed; //���������ٶ�
    in_sancha2=!in_sancha2;
    last_left=0;
    last_right=0;
    out_sancha=0;
    top_x=top_y=0;
    jin_sancha=0;
}

void change_dir()   //ֱ�ߺ����л�
{
    if(SteerTest.dir)   //����ת��ֱ��
    {

    }
    else
    {

    }
}
byte sancha() //����������
        {
            byte left_point = 0,right_point=0;
            byte i=5,j=5;
            byte a = 0;
            byte top_mid=0;
            while (i < 55)
            {
                if(p_Pixels[i][Pro_value(my_Mid)] == Black
                        && p_Pixels[i][Pro_value(my_Mid+1)] == Black) break;
                i++;  //Ѱ�Ҷ���
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
                //�޸�1
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
            for (a = 3; a < Pro_value(top_y-5); a++) //Ѱ�����ҽǵ�
            {

                 y = L_black[a];
                 yy = R_black[a];
                 if(LoseL[a]==0 && left_point==0 &&  p_Pixels[a][Pro_value(y+3)]==Black && L_black[a+3]>L_black[a]  && L_black[a-2]>=L_black[a] )
                 {//����
                     left_point=a;
                     while(left_point>=1 && LoseL[left_point-1]==0 &&L_black[left_point-1]<L_black[left_point] && p_Pixels[left_point-1][L_black[left_point-1]+2]==Black) left_point-=1;
                     //while(left_point>=1 && L_black[left_point+1]<L_black[left_point] && p_Pixels[left_point+1][L_black[left_point+1]+2]==Black) left_point+=1;
                     if(angle_scan(L_black, left_point)==0 || left_point>=top_y) left_point=0;
                  }
                 if(LoseR[a]==0 && right_point==0 &&  p_Pixels[a][Pro_value(yy-3)]==Black &&  R_black[a+3]<R_black[a] && R_black[a-2]<=R_black[a])
                 {//����
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
            while (j < 55 && (p_Pixels[j][my_Mid] == White || p_Pixels[Pro_value(j-2)][Pro_value(my_Mid-8)] == Black || p_Pixels[Pro_value(j-2)][ Pro_value(my_Mid + 8)] == Black)) j++;  //Ѱ�Ҷ���
//            pill(top_y);
//            pill(left_point);

            //�������޼���
            byte midmid = 0;
            byte chazhi = 0;
            if (left_point != 0 && right_point != 0)
            {
                midmid = L_black[left_point] - R_black[right_point];
                chazhi = right_point - left_point;
            }


            if (in_sancha==0 && left_point!=0 && right_point!=0 && (abs(left_point-right_point)<12) && j!=55 && top_y>10) //�жϽ�������
            {

                //�����㲻���������ڣ�return
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
                        if(hen_sancha==1 ) //��������ǰ����
                        {
                            //speed=PID_Speed.SpeedSet;
                            //set_speed(v_hen);
                        }
                    }
                }
            }
            //pill(top_y);
            if (in_sancha == 1)  //�ж��뿪����
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
                //���
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

                //�ҵ�
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

                if(sancha_dir==1) //����
                {
                    Fill_Line(L_black[left_point], left_point, top_x, top_y, 0);
                    line_rescan(top_y-2,1);
                }
                else    //����
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

/*******************************���/����*******************************/
void find_p(byte direct,byte *p,byte *lose) //��һ�������ҵ�
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
void ruku()//���
        {
            byte row;
            byte i;
            byte block_count=0;

            if(ruku_count==0 && L_Blank<10 && R_Blank<10 && in_ruku!=0) //ǿ���˳���һ�ξ������״̬
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
                                in_ruku = 1; //���
                                direct=2;
                            }
                            else
                            {
                                in_ruku = 2;//�ҿ�
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

            if (in_ruku == 1) //����һ���ҵ�
            {
                find_p(88,L_black,LoseL);
            }
            else if (in_ruku == 2) //�ҿ��һ���ҵ�
            {
                find_p(2,R_black,LoseR);
            }
            if (in_ruku == 3)//�������ҵ�
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
                    if (p_Pixels[row - 3][65] == White && p_Pixels[row - 3][25] == White && p_Pixels[row - 3][50] == White && p_Pixels[row - 3][ 40] == White && p_Pixels[row +3][ 65] == Black && p_Pixels[row + 3][ 25] == Black) in_ruku = 5;//ͣ
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

            if (ruku_count == 0) //��һ�ξ�����
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

            if (start_ruku == 0 && top_y<42) //��ʼ���
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
                Is_Run=0; //��⣬ͣ
                //CLOSE_MOTOR;
            }

        }
void chuku()
{

}

/*******************************����************************************/
/*  ���� �����뻷 Ӧ�ñ���ÿ��״̬ͼ��֮��������   */
         byte Track_State = 0;   //����״̬  0 1 2 3 4 5 6 7
         byte Track_Dir = 0;    //1�� 2�һ�
         byte Track_Num = 1;    //��������   ��ǰ�˹��趨һ��
         byte Track_type[] = { 0, 1, 2 };     //�������� С �� ��   0  1  2
         byte Field_num = 0;     //��ⳡ��
         byte Field_num1 = 0;
         uint8_t L_down_point[] = { 0, 0 };
         uint8_t L_down_point_flag = 0;    //�����ҵ���־λ
         uint8_t L_down_point_last_flag = 0;//�ϳ��ҵ���־λ
         uint8_t L_down_point_times = 0;  //���¹յ��ҵ��Ĵ���

         uint8_t L_middle_point[] = { 0, 0 };
         uint8_t L_middle_point_flag = 0;     //�����ҵ���־λ
         uint8_t L_middle_point_last_flag = 0;//�ϳ��ҵ���־λ
         uint8_t L_middle_point_times = 0;  //���йյ��ҵ�����

         uint8_t L_up_point[]= { 0, 0 };
         uint8_t L_up_point_flag = 0;     //�����ҵ���־λ
         uint8_t L_up_point_last_flag = 0;//�ϳ��ҵ���־λ
         uint8_t L_up_point_times = 0;  //���Ϲյ��ҵ�����

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
                    L_down_point[0] = j;//��������û�е�0��
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
                    R_down_point[0] = j;//��������û�е�0��
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

                //����յ�
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
        //����״̬3�����¹յ�--�󻷵�
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
        //����״̬3�����¹յ�--�һ���
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

                //����յ�
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
        //״̬0
        //�������
        void Island_Check()
        {
            byte j = 0;
            int K1, K2;
            if (Track_Num >= 1)
            {
                //����������>=50�ż�������󻷵�
                if (long_turn_flag_right >= 50&&Rstart<=20 )
                {

                    regression(R_black, Rstart, 50, 2,&parameterB,&parameterA);
                    K1 = (int)(parameterB * 100);
                    regression(R_black, Rstart, 40, 2,&parameterB,&parameterA);
                    K2 = (int)(parameterB * 100);
                    if (abs(K1 - K2) < 8)    //��Ϊ������ֱ��
                    {
                        for (j = 3; j < 50; j++)
                        {
                            if (j >= 2 && p_Pixels[j + 1][ Pro_value(L_black[j] + 2)] == White && abs(L_black[j - 1] - L_black[j - 2]) <= 3 && abs(L_black[j] - L_black[j - 1]) <= 3 && (L_black[j + 1] - L_black[j] >= 3)
                        && p_Pixels[j][ Max_Col - 1] == White && LoseL[j - 2] == 0 && LoseL[j - 1] == 0 && LoseL[j] == 0)
                            {
                                Field_num++;
                                if (Field_num >= 3)  //����3������⵽
                                {
                                    Field_num = 0;
                                    Track_Dir = 1;
                                    Track_State = 1;
                                    //Console.WriteLine("��⵽�󻷵�" + j);
                                    break;
                                }

                            }

                        }
                    }
                }
                //����������>=50�������߷���<=1���߷���>=10 �ż�������һ���
                else if (long_turn_flag_left >= 50 &&Lstart<=20)
                {
                    regression(L_black, Lstart, 50, 2,&parameterB,&parameterA);
                    K1 = (int)(parameterB * 100);
                    regression(L_black, Lstart, 40, 2,&parameterB,&parameterA);
                    K2 = (int)(parameterB * 100);

                    if (abs(K1 - K2) < 8)//��Ϊ������ֱ��
                    {
                        //Console.WriteLine("zuo�߲�ֵ" + abs(K1 - K2));
                        for (j = 3; j < 50; j++)
                        {
                            if (j >= 2 && p_Pixels[j + 1][ Pro_value(R_black[j] - 2)] == White && abs(R_black[j - 1] - R_black[j - 2]) <= 3 && abs(R_black[j] - R_black[j - 1]) <= 3 && (R_black[j] - R_black[j + 1] >= 3)
                        && p_Pixels[j][ Min_Col + 1] == White && LoseR[j - 2] == 0 && LoseR[j - 1] == 0 && LoseR[j] == 0)
                            {
                                Field_num++;
                                if (Field_num >= 3)  //����3������⵽
                                {
                                    Field_num = 0;
                                    Track_Dir = 2;
                                    Track_State = 1;
                                    ////Console.WriteLine("��⵽�һ���" + j);
                                    break;
                                }

                            }

                        }
                    }
                }
            }


        }
        //״̬1
        //�󻷵�--�����¹յ�����йյ㲢���� �Ҳ������º����йյ������δ�ҵ����йյ�������С ����״̬2
        void Island_Start()
        {
            byte jj;
            if (Track_Dir == 1)   //Ϊ�󻷵�
            {
                //Ѱ�����¹յ�
                find_leftdown_point(0, 50);
                if (L_down_point_times == 0)  //һ����û����
                {
                    L_down_point_flag = 0;
                    L_down_point_last_flag = 0;
                    L_down_point[0] = 0;
                    L_down_point[1] = L_black[0];
                }
                else if (L_down_point_times >= 1 && L_down_point_flag == 0 && L_down_point_last_flag == 1)  //����δ���ŵ��ϳ�����
                {

                    for (jj = 2; jj < 40; jj++)   //��������Ѱ��
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
                //Console.WriteLine("�ҵ�״̬"+ L_down_point_flag+"zuo��" + L_down_point[0]);

                //Ѱ�����йյ�
                if (L_down_point_flag == 1)
                    find_leftmiddle_point((byte)(L_down_point[0] + 5), 55);
                else
                {
                    find_leftmiddle_point(10, 55);
                }

                if (L_middle_point_times == 0)  //һ�ζ�δ����
                {
                    for (jj = 59; jj > 0; jj--)
                    {
                        if (LoseR[jj] == 0) break;
                    }
                    L_middle_point[0] = jj;
                    L_middle_point[1] = Pro_value(L_down_point[1] + R_black[L_down_point[0]] - R_black[L_middle_point[0]]);

                }
                else if (L_middle_point_flag == 0 && L_middle_point_last_flag == 1) //��һ�����ű���δ���� ������һ��
                {

                }
                else if (L_middle_point_flag == 0 && L_middle_point_last_flag == 0)  //����������û����
                {

                    L_middle_point[0] = 0;
                    L_middle_point[1] = L_black[0];

                }
                //Console.WriteLine("zuo��" + L_middle_point[0]);
                //Console.WriteLine("zuo�µ�ǰflag" + L_down_point_flag + "zuo���ϳ�flag" + L_down_point_last_flag);
                //Console.WriteLine("zuo�е�ǰflag" + L_middle_point_flag + "zuo���ϳ�flag" + L_middle_point_last_flag);
                Fill_Line(L_down_point[1], L_down_point[0], L_middle_point[1], L_middle_point[0], 0);
                if (L_middle_point_flag == 1 && L_middle_point[0] <= 40 && L_down_point_flag == 0)
                {
                    Field_num++;
                    if (Field_num >= 2)  //�����������ͼ��
                    {

                        Clear_Flag1();
                        Track_State = 2;
                        //Console.WriteLine("ֻ�ҵ����йյ� ����״̬2");
                    }
                }
                if (L_middle_point_flag == 0 && L_middle_point_last_flag == 0 && L_down_point_flag == 0 && L_down_point_last_flag == 0 && L_down_point_times >= 1 && L_middle_point_times >= 1)
                {

                    Clear_Flag1();
                    Track_State = 2;
                    //Console.WriteLine("ȫ��δ�ҵ� ����״̬2");
                }

                //���³���־λ
                L_down_point_last_flag = L_down_point_flag;
                L_middle_point_last_flag = L_middle_point_flag;
            }


            else if(Track_Dir==2) //Ϊ�һ���
            {
                find_rightdown_point(0, 50);
                if (R_down_point_times == 0)  //һ����û����
                {
                    R_down_point_flag = 0;
                    R_down_point_last_flag = 0;
                    R_down_point[0] = 0;
                    R_down_point[1] = R_black[0];
                }
                else if (R_down_point_times >= 1 && R_down_point_flag == 0 && R_down_point_last_flag == 1)  //����δ���ŵ��ϳ�����
                {

                    for (jj = 2; jj < 40; jj++)   //��������Ѱ��
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
                //Console.WriteLine("you��" + R_down_point[0]);
                if (R_down_point_flag == 1)
                    find_rightmiddle_point((byte)(R_down_point[0] + 5), 55);
                else
                {
                    find_rightmiddle_point(10, 55);
                }

                if (R_middle_point_times == 0)  //һ�ζ�δ����
                {
                    for (jj = 59; jj > 0; jj--)
                    {
                        if (LoseL[jj] == 0) break;
                    }
                    R_middle_point[0] = jj;
                    R_middle_point[1] = Pro_value(R_down_point[1] + L_black[R_down_point[0]] - L_black[R_middle_point[0]]);

                }
                else if (R_middle_point_flag == 0 && R_middle_point_last_flag == 1) //��һ�����ű���δ���� ������һ��
                {

                }
                else if (R_middle_point_flag == 0 && R_middle_point_last_flag == 0)  //����������û����
                {

                    R_middle_point[0] = 0;
                    R_middle_point[1] = R_black[0];//Pro_value(L_down_point[1] + R_black[L_down_point[0]] - R_black[L_middle_point[0]]);

                }
                //Console.WriteLine("you��" + R_middle_point[0]);
                //Console.WriteLine("you�µ�ǰflag" + R_down_point_flag + "you���ϳ�flag" + R_down_point_last_flag);
                //Console.WriteLine("you�е�ǰflag" + R_middle_point_flag + "you���ϳ�flag" + R_middle_point_last_flag);
                Fill_Line(R_down_point[1], R_down_point[0], R_middle_point[1], R_middle_point[0], 1);
                if (R_middle_point_flag == 1 && R_middle_point[0] <= 40 && R_down_point_flag == 0)
                {
                    Field_num++;
                    if (Field_num >= 2)  //�����������ͼ��
                    {

                        Clear_Flag1();
                        Track_State = 2;
                        //Console.WriteLine("ֻ�ҵ����йյ� ����״̬2");
                    }
                }
                if (R_middle_point_flag == 0 && R_middle_point_last_flag == 0 && R_down_point_flag == 0 && R_down_point_last_flag == 0 && R_down_point_times >= 1 && R_middle_point_times >= 1)
                {

                    Clear_Flag1();
                    Track_State = 2;
                    //Console.WriteLine("ȫ��δ�ҵ� ����״̬2");
                }

                //���³���־λ
                R_down_point_last_flag = R_down_point_flag;
                R_middle_point_last_flag = R_middle_point_flag;
            }
        }
        //״̬2
        //�󻷵�--�����йյ����� �ҵ����Ϲյ㲢���Ҳ������йյ� ����״̬3
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
                //Console.WriteLine("zuo�е���flag" + L_middle_point_flag + "zuo���ϳ�flag" + L_middle_point_last_flag);
                //Console.WriteLine("zuo�ϵ���flag" + L_up_point_flag + "zuo���ϳ�flag" + L_up_point_last_flag);
                //Console.WriteLine("zuo��" + L_middle_point[0]);
                //Console.WriteLine("zuo��" + L_up_point[0]);


                Fill_Line(L_black[0], 0, L_middle_point[1], L_middle_point[0], 0);
                Fill_Line(R_black[0], 0, L_up_point[1], L_up_point[0], 1);
                if (L_up_point_flag == 1 && L_up_point[0] <= 50 && L_middle_point_flag == 0 && L_middle_point_last_flag == 0)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 3;
                        //Console.WriteLine("ֻ�ҵ����Ϲյ� ����״̬3");
                    }
                }
                if (L_up_point_flag == 1 && L_up_point[0] <= 46)
                {
                    Field_num1++;
                    if (Field_num1 >= 2)
                    {
                        Clear_Flag1();
                        Track_State = 3;
                        //Console.WriteLine("���Ϲյ��С ����״̬3");
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
                //Console.WriteLine("you�е���flag" + R_middle_point_flag + "you���ϳ�flag" + R_middle_point_last_flag);
                //Console.WriteLine("you�ϵ���flag" + R_up_point_flag + "you���ϳ�flag" + R_up_point_last_flag);
                //Console.WriteLine("you��" + R_middle_point[0]);
                //Console.WriteLine("you��" + R_up_point[0]);


                Fill_Line(R_black[0], 0, R_middle_point[1], R_middle_point[0], 1);
                Fill_Line(L_black[0], 0, R_up_point[1], R_up_point[0], 0);
                if (R_up_point_flag == 1 && R_up_point[0] <= 50 && R_middle_point_flag == 0 && R_middle_point_last_flag == 0)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 3;
                        //Console.WriteLine("ֻ�ҵ����Ϲյ� ����״̬3");
                    }
                }
                if (R_up_point_flag == 1 && R_up_point[0] <= 46)
                {
                    Field_num1++;
                    if (Field_num1 >= 2)
                    {
                        Clear_Flag1();
                        Track_State = 3;
                        //Console.WriteLine("���Ϲյ��С ����״̬3");
                    }
                }
                R_up_point_last_flag = R_up_point_flag;
                R_middle_point_last_flag = R_middle_point_flag;
            }

        }
        //״̬3
        //�󻷵�--�л�ɨ�߷�ʽ��֤�������� �ҵ����¹յ����� �����¹յ�������С���߼������Ҳ��������״̬4
        void Island_Enter2()
        {
            int K = 0;
            if (Track_Dir == 1)
            {
                Track_3_find_rightdown_point(0, 50);
                //Console.WriteLine("you��" + R_down_point[0]);
                Fill_Line(R_black[0], 0, R_down_point[1], R_down_point[0], 1);//������
                //Console.WriteLine("you�µ���flag" + R_down_point_flag + "you���ϳ�flag" + R_down_point_last_flag);
                if (R_down_point_flag == 1 && R_down_point[0] <= 20)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 4;
                        //Console.WriteLine("���µ��й�С ����״̬4");
                    }
                }
                if (R_down_point_flag == 0 && R_down_point_last_flag == 0 && R_down_point_times >= 1)
                {

                    Clear_Flag1();
                    Track_State = 4;
                    //Console.WriteLine("δ�ҵ����µ� ����״̬4");
                }
                regression(L_black, 0, finalend-5, 2,&parameterB,&parameterA);
                K = (int)(parameterB * 100);
                ////Console.WriteLine("zuoK" + K);
                if(R_down_point_flag == 0 && R_down_point_last_flag == 0 && R_down_point_times==0 &&Lstart>=40&&Rstart<=20&&abs(K)<=1)
                {
                    Clear_Flag1();
                    Track_State = 4;
                    //Console.WriteLine("��ȫδ�ҵ����µ� ����״̬4");
                }
            }
            else if(Track_Dir==2)
            {
                Track_3_find_leftdown_point(0, 50);
                //Console.WriteLine("zuo��" + L_down_point[0]);
                Fill_Line(L_black[0], 0, L_down_point[1], L_down_point[0], 0);//������
                //Console.WriteLine("zuo�µ���flag" + L_down_point_flag + "zuo���ϳ�flag" + L_down_point_last_flag);
                if (L_down_point_flag == 1 && L_down_point[0] <= 20)
                {
                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_Flag1();
                        Track_State = 4;
                        //Console.WriteLine("���µ��й�С ����״̬4");
                    }
                }
                if (L_down_point_flag == 0 && L_down_point_last_flag == 0 && L_down_point_times >= 1)
                {

                    Clear_Flag1();
                    Track_State = 4;
                    //Console.WriteLine("δ�ҵ����µ� ����״̬4");
                }
                regression(R_black, 0, finalend - 5, 2,&parameterB,&parameterA);
                K = (int)(parameterB * 100);
                if (L_down_point_flag == 0 && L_down_point_last_flag == 0 && L_down_point_times == 0 && Rstart >= 40 && Lstart <= 20 && abs(K) <= 1)
                {
                    Clear_Flag1();
                    Track_State = 4;
                    //Console.WriteLine("��ȫδ�ҵ����µ� ����״̬4");
                }
            }

        }
        //״̬4 �ڻ���
        //�󻷵�--�����йյ�����������״ �ҵ�����������С ����״̬5
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
                        //Console.WriteLine("�ҵ����Ϲյ� ����״̬5");
                    }
                }

                //Console.WriteLine("you��" + R_middle_point[0]);
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
                        //Console.WriteLine("�ҵ����Ϲյ� ����״̬5");
                    }
                }
                //Console.WriteLine("zuo��" + L_middle_point[0]);
            }

        }
        //״̬5
        //�󻷵�--�����йյ��������Ϲյ����� ���йյ�δ�ҵ���������С ����״̬6
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
                else if (R_middle_point_flag == 0 && R_middle_point_last_flag == 1)  //����δ�����ϳ�����
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
                //�ж��Ƿ���ǰ����״̬6
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
                //Console.WriteLine("you��" + R_middle_point[0]);
                //Console.WriteLine("you����" + R_middle_point[1]);
                ////Console.WriteLine("zuo��" + L_up_point[0]);
                ////Console.WriteLine("zuo����" + L_up_point[1]);
                ////Console.WriteLine("you�е���flag" + R_middle_point_flag + "you���ϳ�flag" + R_middle_point_last_flag);

                Fill_Line(R_middle_point[1], R_middle_point[0], L_up_point[1], L_up_point[0], 1);

                if (R_middle_point[0] <= 15 && R_middle_point_flag == 1)
                {

                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("���е��й�С ����״̬6");

                }
                if (R_middle_point_flag == 0 && R_middle_point_last_flag == 0 && R_middle_point_times >= 1)
                {
                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("�Ҳ������е� ����״̬6");
                }
                if(R_middle_point_flag == 0 && R_middle_point_last_flag == 0&& R_down_point_flag==1&&R_down_point[0]<=40)
                {
                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("��ǰ ����״̬6");

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
                else if (L_middle_point_flag == 0 && L_middle_point_last_flag == 1)  //����δ�����ϳ�����
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
                //�ж��Ƿ���ǰ����״̬6
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
                    //Console.WriteLine("���е��й�С ����״̬6");

                }
                if (L_middle_point_flag == 0 && L_middle_point_last_flag == 0 && L_middle_point_times >= 2)
                {
                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("�Ҳ������е� ����״̬6");
                }
                if (L_middle_point_flag == 0 && L_middle_point_last_flag == 0 && L_down_point_flag == 1 && L_down_point[0] <= 40)
                {
                    Clear_Flag1();
                    Track_State = 6;
                    //Console.WriteLine("��ǰ ����״̬6");

                }
                L_middle_point_last_flag = L_middle_point_flag;
                L_down_point_last_flag = L_down_point_flag;


            }

        }
        //״̬6
        //�󻷵�--�����Ϲյ� �ҵ�����������С����״̬7
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
                ////Console.WriteLine("�ҵ�" + L_middle_point_flag + "zuo��" + L_middle_point[0]);
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
                //Console.WriteLine("�ҵ�" + L_up_point_flag + "zuo��" + L_up_point[0]);
                Fill_Line(R_black[0], 0, L_middle_point[1], L_middle_point[0], 1);

                if (L_up_point[0] <= 53 && L_up_point_flag == 1 && L_up_point_times >= 3)
                {
                    Clear_Flag1();
                    Track_State = 7;
                    //Console.WriteLine("�ҵ����Ϲյ� ����״̬7");

                }
                //Console.WriteLine("�ҵ�"+ R_down_point_flag+"you��" + R_down_point[0]);
                if (R_down_point_flag==1&&R_down_point[0]<=15&&R_down_point_times>=2)
                {
                    Clear_Flag1();
                    Track_State = 7;
                    //Console.WriteLine("���󲿷������ ����״̬7");
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
                //Console.WriteLine("�ҵ�"+ L_down_point_flag+"������" + L_down_point[0]);
                find_rightup_point(0, 55);
                //Console.WriteLine("�ҵ�" + R_up_point_flag + "you��" + R_up_point[0]);
                Fill_Line(L_black[0], 0, R_middle_point[1], R_middle_point[0], 0);
                if (R_up_point[0] <= 53 && R_up_point_flag == 1 &&R_up_point_times>=4&& R_up_point_times >= 3)
                {
                    Clear_Flag1();
                    Track_State = 7;
                    //Console.WriteLine("����״̬7");

                }
                //if (L_down_point_flag == 1 && L_down_point[0] <= 15 && L_down_point_times >= 2)
                //{
                //    Clear_Flag1();
                //    Track_State = 7;
                //    //Console.WriteLine("���󲿷����Ҳ� ����״̬7");
                //}

            }
        }
        //״̬7
        //�󻷵�--�����Ϲյ�����ߵ�0������ ����<=20������
        void Island_Out3()
        {
            if (Track_Dir == 1)
            {
                find_leftup_point(0, 55);
                //Console.WriteLine("�ҵ�" + L_up_point_flag + "zuo��" + L_up_point[0]);
                Fill_Line(L_black[0], 0, L_up_point[1], L_up_point[0], 0);

                if (L_up_point_flag == 1 && L_up_point[0] <= 20)
                {

                    Field_num++;
                    if (Field_num >= 2)
                    {

                        Clear_FlagAll();
                        //Console.WriteLine("������");
                    }


                }
                if (L_up_point_flag == 0 && L_up_point_last_flag == 0 && Lstart<=25&&Rstart<=25 &&Lstart!=0&&Rstart!=0)
                {
                    Field_num1++;
                    if (Field_num1 >= 2)
                    {

                        Clear_FlagAll();
                        //Console.WriteLine("δ�ҵ����� ������");
                    }
                }
                L_up_point_last_flag = L_up_point_flag;
            }
            else if(Track_Dir==2)
            {
                find_rightup_point(0, 55);
                //Console.WriteLine("�ҵ�" + R_up_point_flag + "zuo��" + R_up_point[0]);
                Fill_Line(R_black[0], 0, R_up_point[1], R_up_point[0], 1);

                if (R_up_point_flag == 1 && R_up_point[0] <= 20)
                {

                    Field_num++;
                    if (Field_num >= 2)
                    {
                        Field_num = 0;
                        Clear_FlagAll();
                        //Console.WriteLine("������");
                    }


                }
                if (R_up_point_flag == 0 && R_up_point_last_flag == 0 && Rstart <= 25 && Lstart <= 25 && Lstart != 0 && Rstart != 0)
                {
                    Field_num = 0;
                    Clear_FlagAll();
                    //Console.WriteLine("������");
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


/*******************************����Ҫ���б�*******************************/
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
void pd()//now_state   0ͣ��1��ֱ����2��ֱ��,3�����4����5��������6����ת��
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


/*******************************�ֶδ��*******************************/
void set_angle_line(byte num)//���ô����
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


/*****************************�ٶȱ���**********************************/
void Protect()
{
    if(p_Pixels[0][LCenter[0]+1]==Black&&p_Pixels[1][LCenter[1]+1]==Black)
    {
        Protect_num++;
    }
    else Protect_num=0;
    if(Protect_num>=10)  //��������ͼ�񶼼�⵽
    {
        Protect_num=0;
        Is_Run=0;
    }
}
/*******************************����ʾ����*******************************/
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

    //�ܱ��� 0��Ԫ��  1ʮ�� 2���� 3���� 4���
    if(in_shizi) all_state=1;
    else if(in_sancha || in_sancha2) all_state=2;
    else if(Track_State) all_state=3;
    else if(in_ruku) all_state=4;
    else all_state=0;

    get_angle_line();
    //Protect();//�ٶȱ���
}
//ÿ֡ͼ���־λ����
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
//ͼ������
void SignalProcess()
{
    byte RowNum=0;
    Data_init();    //ÿ֡ͼ�������ʼ��
    for (RowNum = 0; RowNum < ROW; RowNum++)
    {
        Row_Process(RowNum); //ͼ���д���,���л���ɨ�ߣ���ȡͼ����Ϣ
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
    Mid = (byte)((LCenter[0] + LCenter[1]) >> 1);   //����ÿ֡ͼ����ֵ
    FieldProcess();     //��������
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
