//K70程序
#include  "Apps/SystemTask.h"
#define AD0 MFGetAD(0)  //正前方灰度
#define AD1 (MFGetAD(1)-8) //左边灰度
#define AD2 (MFGetAD(2)-28) //右边灰度
#define AD3 MFGetAD(3) //右边红外
#define AD4 MFGetAD(4) //后边红外
#define AD5 MFGetAD(5) //左边红外
#define AD6 MFGetAD(6) //头

uint8 SERVO_MAPPING[10] = {1,2,3,4,5,6,7,8,9,10};
int dangerInt=320;
void shangtai();
void gongji();
void move(int ,int );
void huanchong(int nowspeed,int level);
void shou_yubei();
int danger();
int RelativeMove(void);
int max(int a,int b,int c);
int min(int a,int b,int c);
void houyang();
void qianqin();
void shangtai();

void standup();
void standup_fwd();
void zhili();
int main()
{
  MFInit();
  MFInitServoMapping(&SERVO_MAPPING[0],10);
  MFSetPortDirect(0x00000E00);
  MFSetServoMode(1,0);
  MFSetServoMode(2,0);
  MFSetServoMode(3,0);
  MFSetServoMode(4,0);
  MFSetServoMode(5,0);
  MFSetServoMode(6,0);
  MFSetServoMode(7,0);
  MFSetServoMode(9,1);
  MFSetServoMode(10,1);
  while((AD3<300));//软启动
  shangtai();
  while (1)
  {
	if((AD0<110||AD1<100)&&AD6>135){
		DelayMS(50);
		if((AD0<110||AD1<100)&&AD6>135){
			standup_fwd();
		}
	}
    if(MFGetDigiInput(8)){//如果站立
      if(MFGetDigiInput(1)&&MFGetDigiInput(0)){
		  MFSetServoPos(3,199,512);
		  MFSetServoPos(6,846,512);
		  MFSetServoPos(1,780,1023);
		  MFSetServoPos(8,296,1023);
		  MFSetServoPos(2,782,1023);
		  MFSetServoPos(7,795,1023);
		  MFServoAction();
      }else{
//        gongji();
        gongji();
      }
      zhili();
      if(AD4>200){//后边检测到人
    	  if(AD0>dangerInt){//安全区域
    		  move(-400,300);
    		  DelayMS(500);
    		  move(0,0);
    		  DelayMS(500);
    	  }else{
    		  move(-400,300);
    		  DelayMS(500);
    	  }
      }else if(AD3>160){//右边检测到敌人
        move(300,-400);
        DelayMS(500);
//        if(!MFGetDigiInput(1)){//前方检测到人
//        	gongji();
//        	gongji();
//        }
      }else if(AD5>160){
        move(-400,300);
        DelayMS(500);
        if(!MFGetDigiInput(1)){
//        	gongji();
//        	gongji();
        }
      }
        if(AD0>dangerInt){//安全区域
          move(300,300);
        }else{    //危险区域
          if(max(AD0,AD1,AD2)==1){//车子向着台子中心
            move(300,300);
          }else{
        	  //move(-300,400);
        	 // DelayMS(100);
            switch(min(AD0,AD1,AD2)){
              case 1:   //车子向外
              if(AD1<AD2){//左边是墙
                 move(300,-400);//右转弯
                 DelayMS(100);
              }else{
                 move(-400,300);//左转弯
                 DelayMS(100);
              }
      				break;
              case 2:   //车子右边是台子中心
                if(AD1<=dangerInt){//test
                  move(400,-300);
                  DelayMS(100);
                }
                break;
              case 3:   //车子左边是台子中心
                if(AD1<=dangerInt){//test
                  move(-300,400);
                  DelayMS(100);
                }
                break;
            }
          }
        }
    }else if(!MFGetDigiInput(8)){//倒下
  	  move(0,0);
  	  if(MFGetDigiInput(1)&&MFGetDigiInput(0)){//前面光电检测到物体
  		  DelayMS(10);
  		  if(MFGetDigiInput(1)&&MFGetDigiInput(0)){
  			standup();
  		  }
  	  }else{
  		standup_fwd();
  	  }

    }
  }
}
void shangtai()
{
  MFSetServoPos(4,512,300);
  MFSetServoPos(5,552,300);

  move(0,0);
  DelayMS(1400);
  MFSetServoPos(1,762,300);
  MFSetServoPos(2,96,300);
  MFSetServoPos(3,73,300);

  MFSetServoPos(6,950,300);
  MFSetServoPos(7,75,300);
  MFSetServoPos(8,311,300);
  MFServoAction();
  DelayMS(1400);
  MFSetServoPos(4,174,300);
  MFSetServoPos(5,894,300);


  DelayMS(1000);

  move(500,500);//加速上台
  DelayMS(100);
  move(500,500);
  DelayMS(1500);

  for(int i=0;i<20;i++){
	  move(500-10*i,500-i*20);//拐向台中央
	  DelayMS(25);
  }
  DelayMS(660);
  MFSetServoPos(4,512,300);//直腰
  MFSetServoPos(5,552,300);

  MFSetServoPos(3,512,512);//肩部
  MFSetServoPos(6,512,512);


  move(300,300);
  DelayMS(1500);

  // flag2 = 1 + 0;
  // flag1 = 1 + 1;
}

void gongji()
{
	if(AD0>dangerInt){
		  move(300,300);
		  //qianqin();
		//  MFSetServoPos(3,190,190);

		//  MFSetServoPos(6,760,500);、
		  //戳

		  MFSetServoPos(1,601,512);
		  MFSetServoPos(8,499,512);
		  MFServoAction();
		  DelayMS(50);

		  MFSetServoPos(3,153,512);
		  MFSetServoPos(6,886,512);
		  MFSetServoPos(7,671,1023);
		  MFSetServoPos(2,664,1023);
		  qianqin();
		  MFServoAction();
		  DelayMS(300);

		  //准备
		  houyang();
		  MFSetServoPos(3,199,512);
		  MFSetServoPos(6,846,512);

		  MFSetServoPos(1,780,800);
		  MFSetServoPos(8,296,800);
		  MFServoAction();
		  DelayMS(50);

		  MFSetServoPos(2,782,1023);
		  MFSetServoPos(7,795,1023);
		  MFServoAction();
		  DelayMS(100);

	}

}
void move(int x,int y )
{
//  gd2 = MFGetDigiInput(2);
  MFSetServoRotaSpd(9,x);//x
  MFSetServoRotaSpd(10,-y);//y
  MFServoAction();
}

void huanchong(int nowspeed,int level){
	for(int i=0;i<level;i++){
		move(nowspeed-(nowspeed/level),nowspeed-(nowspeed/level));
		nowspeed-=(nowspeed/level);
		DelayMS(10);
	}
	move(0,0);
}
void shou_yubei(){
	MFSetServoPos(1,762,512);
	MFSetServoPos(2,96,512);

	MFSetServoPos(7,75,512);
	MFSetServoPos(8,311,512);
	MFServoAction();
	DelayMS(300);
	MFSetServoPos(6,512,512);
	MFSetServoPos(3,512,512);
	MFServoAction();
}
int danger(){
	if(AD0<dangerInt)
		return 1;
}
int RelativeMove(void) {//运动的趋势
	int pre,now;
	pre=AD0;
	DelayMS(100);
	now=AD0;
	return now-pre;
}
int max(int a,int b,int c){
  int max;
  max=a>b?a:b;
  max=max>c?max:c;

  if(max==a){
	  return 1;
  }else if(max==b){
	  return 2;
  }else{
	  return 3;
  }
}
int min(int a,int b,int c){
  int min;
  min=a<b?a:b;
  min=min<c?min:c;
  if(min==a){
	  return 1;
  }else if(min==b){
	  return 2;
  }else{
	  return 3;
  }
}
void standup(){
  MFSetServoPos(4,512,512);
  MFSetServoPos(5,512,512);

//  MFSetServoPos(1,581,512);
//  MFSetServoPos(2,427,512);
//  MFSetServoPos(3,488,512);
//  MFSetServoPos(6,458,512);
//  MFSetServoPos(7,397,512);
//  MFSetServoPos(8,470,512);
//  MFServoAction();
//  DelayMS(1000);

  MFSetServoPos(1,825,600);
  MFSetServoPos(2,233,600);
  MFSetServoPos(3,517,600);
  MFSetServoPos(6,409,600);
  MFSetServoPos(7,198,600);
  MFSetServoPos(8,243,600);
  MFServoAction();
  DelayMS(500);

  MFSetServoPos(3,874,800);
  MFSetServoPos(6,87,800);
  MFServoAction();
  DelayMS(400);

  MFSetServoPos(1,523,800);
  MFSetServoPos(2,485,800);
  MFSetServoPos(7,479,800);
  MFSetServoPos(8,513,800);
  MFServoAction();
  move(0,0);
  DelayMS(500);

  MFSetServoPos(4,742,700);//弯腰
  MFSetServoPos(5,320,700);
  MFServoAction();
  DelayMS(800);
  zhili();
  shou_yubei();
  DelayMS(1000);
//  MFSetServoPos(3,563,512);
//  MFSetServoPos(6,401,512);
}
void zhili(){
	  MFSetServoPos(4,538,512);
	  MFSetServoPos(5,533,512);
	  MFServoAction();
}
void houyang(){
	  MFSetServoPos(4,572,512);
	  MFSetServoPos(5,505,512);
	  MFServoAction();
}
void qianqin(){
	  MFSetServoPos(4,501,512);
	  MFSetServoPos(5,571,512);
	  MFServoAction();
}
//前起
void standup_fwd(){
	MFSetServoPos(4,538,512);
	MFSetServoPos(5,533,512);

  MFSetServoPos(1,825,600);
  MFSetServoPos(2,233,600);
  MFSetServoPos(3,517,600);
  MFSetServoPos(6,409,600);
  MFSetServoPos(7,198,600);
  MFSetServoPos(8,243,600);
  MFServoAction();
  DelayMS(500);

  MFSetServoPos(3,235,800);
  MFSetServoPos(6,830,800);
  MFServoAction();
  DelayMS(400);

  MFSetServoPos(1,536,800);
  MFSetServoPos(2,539,800);
  MFSetServoPos(7,578,800);
  MFSetServoPos(8,499,800);
  MFServoAction();
  move(0,0);
  DelayMS(500);

  MFSetServoPos(4,319,700);//弯腰
  MFSetServoPos(5,748,700);
  MFServoAction();
  DelayMS(800);
  zhili();
  shou_yubei();
  DelayMS(1000);
//  MFSetServoPos(3,563,512);
//  MFSetServoPos(6,401,512);
}
void delay (int ms){
	for(int i=0;i<ms;i++){
		DelayMS(1);
		if(AD0<=dangerInt){
			break;
		}
	}
}
