#include "menu.h"

int8 is_showpicture=0;
int8 index_ptr=0;//菜单索引
int8 top=7,bottom=0;
int8 changeFlag=0;//左右加减标志位
uint8 dir=1;//1为上下翻页，0为左右加减
/*
 * p: int16/uint16/float 的指针，
 * k:按一次加减的数
 * index:为case后面的数，为系统默认，只能是那个数
 * examp: case 0:
            myputs(6, y++, "data:%d",change);
            changeint16(&change,1,0);
            break;
   （如果有其他类型的参数需要更改，可自行定义，格式不变）
 * */
void changeint16(int16 *p,int8 k,int index)
{
    if(index_ptr==index && changeFlag)
    {
        (*p)+=k*changeFlag;
        changeFlag=0;
    }
}
void changeuint8(uint8 *p,int8 k,int index)
{
    if(index_ptr==index && changeFlag)
    {
        (*p)+=k*changeFlag;
        changeFlag=0;
    }
}
void changeuint16(uint16 *p,int8 k,int index){
    if(index_ptr==index && changeFlag)
    {
        (*p)+=k*changeFlag;
        changeFlag=0;
    }
}
void changefloat(float *p,float k,int index){
    if(index_ptr==index && changeFlag)
    {
        (*p)+=k*changeFlag;
        changeFlag=0;
    }
}


#if 1
typedef enum{
    Up_L,
    Down_R,
    Mode,
    No_Key
}MENU_TYPE;
void change_ptr1(MENU_TYPE key);
void print_arry(uint8 y);



void menu()
{
    uint8 i;
    uint8 y=0;
    MENU_TYPE key_get;

        key_get=Key_Down();
        change_ptr1(key_get);
        print_arry(index_ptr-bottom);
        y=0;
        if(dir)
        {
            if(is_showpicture)
            {
                oled_fill(0x00);
                is_showpicture=0;
                oled_wrcmd(0xa1);//图像恢复正常
                oled_wrcmd(0xc8);
            }
        }else
        {
            if(index_ptr==0&&is_showpicture==0)
            {
                oled_fill(0x00);
                oled_wrcmd(0xc0);//上下反转
                oled_wrcmd(0xa0);//左右反转
                is_showpicture=1;
            }
        }

        if(is_showpicture){
            DisplayImage_WithOLED();
            return;
        }

        for(i=bottom;i<=top;i++)//扫描bottom到loop的显示
        {

#define DATAMAX 50      //i的最大值
            switch(i)
            {
                case 0:
                    myputs(6, y++, " err:");
                    oled_printf_float(30,y-1,all_state,5,2);
                    break;
                case 1:
                    changeuint8(&Is_Run,1,1);
                    myputs(6, y++, "Is_Run:%d",Is_Run);
                    oled_printf_float(60,y-1,all_lose,2,3);
                    break;
                case 2:
                    myputs(6, y++, "v_k:");
                    changefloat(&v_k,0.1,2);
                    oled_printf_float(60,y-1,v_k,3,1);
                    break;
               //单电机调速pid
                                case 3:
                                    //changeint16(&motor_kd,1,5);
                                    myputs(20, y++, "left1:%4d",left1_flag);
                                    break;
                                case 4:
                                    myputs(20, y++, "left2:%4d",left2_flag);
                                    break;
                                case 5:
                                    myputs(20, y++, "all_lose_end:%4d",all_lose_end);
                                    break;
                                case 6:
                                    myputs(20, y++, "in_ruku:%4d",in_ruku);
                                    break;
                                case 7:
                                    myputs(20, y++, "ruku_count:%4d",ruku_count);
                                    break;
                                case 8:
                                    myputs(20, y++, "all_lose:%4d",all_lose);
                                    break;
//                case 5:
//                    changeint16(&Vset,2,5);
//                    myputs(6, y++, "vset:%d",Vset);
//                    break;

                                    //方向打角pid
//                        case 6:
//                            myputs(6, y++, "diri:");
//                            changefloat(&pid_dir.I,1,6);
//                            oled_printf_float(60,y-1,pid_dir.I,5,2);
//                            break;
//                        case 7:
//                            myputs(6, y++, "dirp:");
//                            changefloat(&pid_dir.P,1,7);
//                            oled_printf_float(60,y-1,pid_dir.P,5,2);
//                            break;
//        //速度显示
//                        case 8:
//                            myputs(6, y++, "dird:");
//                            changefloat(&pid_dir.D,1,8);
//                            oled_printf_float(60,y-1,pid_dir.D,5,2);
//                            break;
//                case 9:
//                    myputs(6, y++, " change dir:%d",SteerTest.dir);
//                    changeint16(&SteerTest.dir,1,9);
//                    break;
//                //转弯补偿pid
//                case 10:
//                   myputs(6, y++, "z_p");
//                   changefloat(&pid_z.kp,1,10);
//                   oled_printf_float(60,y-1,pid_z.kp,5,2);
//                   break;
//                case 11:
//                   myputs(6, y++, "z_d:");
//                   changefloat(&pid_z.kd,1,11);
//                   oled_printf_float(60,y-1,pid_z.kd,5,2);
//                   break;
//                case 12:
//                   myputs(6, y++, "z_i:");
//                   changefloat(&pid_z.ki,1,12);
//                   oled_printf_float(60,y-1,pid_z.ki,5,2);
//                   break;
//
//                case 13:
//                    changeint16(&v_min,5,13);
//                    myputs(6, y++, "v_min:%d",v_min);
//                    break;
//                case 14:
//                    myputs(12, y++, "gyro:%4d",icm_gyro_z);
//                    break;
//                case 15:
//                    myputs(25, y++, "act1:%4d",encoder_speed[0]);
//                    break;
//                case 16:
//                    myputs(12, y++, " vy:%4d",act_speed_y);
//                    break;
////四轮设定速度
//                case 17:
//                    myputs(25, y++, "set1:%4d",wheel.v1);
//                    break;
//                case 18:
//                    myputs(25, y++, "set2:%4d",wheel.v2);
//                    break;
//                case 19:
//                    myputs(25, y++, "set3:%4d",wheel.v3);
//                    break;
//                case 20:
//                    myputs(25, y++, "set4:%4d",wheel.v4);
//                    break;
//                case 21:
//                    oled_printf_float(0, y++, midline_deviation, 10, 2);
//                                    break;
//                case 22:
//                                    oled_printf_float(0, y++, midline_deviation1, 10, 2);
//                                                    break;
//                case 23:
//                                    oled_printf_float(0, y++, midline_deviation2, 10, 2);
//                                                    break;
//                case 24:
//                    y++;
//                    break;
////                    myputs(25, y++, "actl:%4d",act_speed_l);
////                    break;
////                case 23:
////                    myputs(25, y++, "actr:%4d",act_speed_r);
////                    break;
////                case 24:
////                    myputs(25, y++, "gyro:%4d",icm_gyro_z);
////                    break;
//
//                case 25:
//                                    changeint16(&v_longstr,5,25);
//                                    myputs(20, y++, "longstr:%4d",v_longstr);
//                                    break;
//                case 26:
//                                    changeint16(&v_str,5,26);
//                                    myputs(20, y++, "str:%4d",v_str);
//                                    break;
//                case 27:
//                                    changeint16(&v_cir,5,27);
//                                    myputs(20, y++, "cir:%4d",v_cir);
//                                    break;
            }
        }
}
#if 1


void change_ptr1(MENU_TYPE key)
{
    if(key==Down_R && dir)
    {
        index_ptr++;
        if(index_ptr>DATAMAX)
        {
            index_ptr=DATAMAX;
        }
        if(index_ptr>top){
            top++;
            bottom++;
            oled_fill(0x00);
        }
    }else
    if(key==Up_L && dir){
        index_ptr--;
        if(index_ptr<0)
        {
            index_ptr=0;
        }
        if(index_ptr<bottom){
            top--;
            bottom--;
            oled_fill(0x00);
        }
    }else
    if(key==Down_R && !dir)
    {
        changeFlag=1;
    }else
    if(key==Up_L && !dir)
    {
        changeFlag=-1;
    }else
    if(key==Mode)
    {
        dir=1-dir;
    }
}
#endif

void print_arry(uint8 y)
{
    if(y>0 && y<7){
        oled_p6x8str(0,y-1," ");
        oled_p6x8str(0,y+1," ");
    }else {
        if(y==0){
            oled_p6x8str(0,y+1," ");
        }
        else
        if(y==7){
            oled_p6x8str(0,y-1," ");
        }

    }
    oled_p6x8str(0,y,">");
}


#endif
