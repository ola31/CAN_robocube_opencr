//can receive
#include <ros.h>
#include <RTOS.h>
#include <time.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>

#include <CAN.h>
#include<stdio.h>                
#include<stdlib.h>   
     
#define USE_USBCON
#define SEND_INTERVAL_MS 1000

#define distance 0.4  //바퀴 사이 간격
#define wheel_radius 0.085  //바퀴 반지름
#define gear_ratio 26  //원래 기어비 : 26



static bool same_vel=false;

void VelCb(const geometry_msgs::Twist& cmd_msg);
void Switch_command(const std_msgs::UInt8& cmd_string);

ros::NodeHandle  nh;
std_msgs::Int32 msg;  //추가
std_msgs::Int32 msg2;  //추가
ros::Publisher pub("r_rpm", &msg); //추가
ros::Publisher pub2("l_rpm", &msg2); //추가
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", VelCb );
ros::Subscriber<std_msgs::UInt8> sub2("cmd_string", Switch_command );

void CAN_Until_Semi(uint8_t* Arr,int j,int k);
void cmd_vel2RPM(uint8_t* vel_array, float lin_vel, float ang_vel);

//=====================================================================================================================================//
//================================================    모터 제어기 CAN통신 명령어 모음   ====================================================//
uint8_t MotorType[7]={'S','T','F','F','F','F',';'}; //모터 타입을 선정
uint8_t initialize[7]={'E','D','A','5','5','A',';'}; //모터 타입에 따라 기본값을 초기화
uint8_t Pulse_and_PolePair[]=
{'S','E','A','5','5','A',',',
'4','E','2','0',',','4','E','2','0',',',
'0','0','0','2',',',
'0','0','0','2',';' };                               //모터1회전당 펄스 수와 POLE PAIR 갯수를 설정
uint8_t ratio_and_MaxRPM[]=
{'S','G','A','5','5','A',',',
'1','2',',',
'1','2',',',
'2','8','0','0',',',
'2','8','0','0',';'};                               //모터의 감속비, 최대회전속도 설정
uint8_t Param_Motor_type[]={'S','T','F','F','F','E',';'};  //파라미터를 저장할 모터 타입
uint8_t Save_Param_at_EEPROM[] = {'E','s','A','5','5','A',';'};  //파라미터를 EEPROM에 저장
uint8_t ParamLimit_and_3Ctrl_Param[] = 
{'S','Q','0','0','0','0','5','0','0','0',',',
'0','0','0','0','2','0','0','0',',',
'0','0','3','E','8','0','0','0',',',
'0','0','8','0',',',
'0','0','0','0','0','1','0','0',',',
'0','0','4','0',',',
'0','0','8','0',',',
'0','0','0','0','0','1','0','0',',',
'0','0','4','0',',',
'0','1','5','0',',',
'0','0','0','0','0','0','0','4',';'};                       //위치/속도제어모드에서 속도게인과 , RJM_VER7형의 3상/5상 STEP모터와 BLDC 모터의 전류제어 파라메터를 설정
uint8_t DIP_Switch_option[]=
{'S','X','0','0','0','0','0','0','7','0',',',
'F','F','F','F','F','F','F','F',';'};                      //통신과 제어의 option을 선택하는 DIP switch의 값을 설정
uint8_t Hall_A_and_U_volt_host[] =
{'W','E','2','6','5','0',',',
'2','6','5','0',';'};     
//Hall_A상 상승에지에 대한 U상 전압 positive zero crossing 포인트의 위상지연 값을 설정하며, 
//POLE_PAIR 수, MOTORx_POSITION_SCALE_FACTOR를 읽어서 host로 전송
uint8_t Current_Limit [] = {'S','w','9','0','0',',','9','0','0',';'};    //제어 시 사용하는 전류의 리밋을 설정
uint8_t LogicCurrLimit_TimeThreshold [] = 
{'S','I','0','0','7','0','0',',',
'0','0','7','0','0',',',
'0','1','0','0','0',',',
'0','1','0','0','0',',',
'0','0','0','0','5',',',
'0','0','0','0','5',',',
'0','0','5','0','0',',',
'0','0','5','0','0',';'};                                    //구동회로와 기구를 보호하기 위한 logic에서 사용하는 전류 리밋값과 time threshold를 설정
uint8_t vel_loca_Ctrl_accel_ratio [] =
{'S','a','0','0','0','1','0',',',
'0','0','0','1','0',',',
'0','0','0','1','0',',',
'0','0','0','1','0',';'};                                  //속도제어 및 위치제어 모드에서 가속 및 감속율을 설정
uint8_t loca_Ctrl_P2P_vel [] = {'S','S','1','0','0','0',',','1','0','0','0',';'};  //위치제어모드에서 Point-to-Point 이동 시 이동속도 값을 설정
uint8_t loca_Move_accel_period []= {'S','s','3','0','0',',','3','0','0',';'};             //PA, PB, Pa 명령에 의한 위치구동시의 가속/감속 기간을 설정
uint8_t SAVE_PARAM [] = {'E','s','A','5','5','A',';'};                             //파라미터 저장
uint8_t Set_ID[7] = {'S','A','F','E','0','1',';'};      //id를 1로 설정

uint8_t PWR_On[7] = {'P','E','0','0','0','1',';'};       //id가 1인 제어기의 모터 출력을 on
uint8_t PWR_Off[7] = {'P','D','0','0','0','1',';'};       //id가 1인 제어기의 모터 출력을 off
uint8_t Local_CTRL[7] = {'S','M','0','2','0','2',';'};        //위치 제어 모드
uint8_t Local_Move[10] = {'P','A','5','1','3','0','0','0','0',';'};   //+방향으로 130000만큼 이동(5000000이 원점)
uint8_t Vel_CTRL[7] = {'S','M','0','5','0','5',';'};       //속도 제어 모드
uint8_t Vel_Move[12] = {'S','V','1','0','0','0',',','1','0','0','0',';'};  //1000rpm으로 이동
uint8_t Curr_CTRL[7] = {'S','M','0','7','0','7',';'};         //전류 제어 모드
uint8_t Curr_Move[8] = {'S','C','4','1',',','4','1',';'};                 // 모터의 전류를 0.5A으로 설정

uint8_t Vel_GOGO[14]={'S','V','+','0','1','0','0',',','+','0','1','0','0',';'};   ///속도값(rpm) 배열
uint8_t re_set[7] = {'P','R','0','0','0','1',';'};
uint8_t nothing[1]={';'};
uint8_t Q2[3]={'Q', '2', ';'}; //추가
uint8_t QP[4]={'Q','P','?',';'}; //추가
//================================================    모터 제어기 CAN통신 명령어 모음   ====================================================//
//=====================================================================================================================================//

uint8_t*goArr=nothing;

//int count111=0;
/****************************************************************/
/********************(ROS TOPIC RETURN 함수)**********************/

void VelCb(const geometry_msgs::Twist& cmd_msg){   //cmd_vel값을 받아, Vel_GOGO배열에 속도(rpm)값 저장
   float linearX = cmd_msg.linear.x;
   float angularZ = cmd_msg.angular.z;
   cmd_vel2RPM(Vel_GOGO, linearX, angularZ);
}

void Switch_command(const std_msgs::UInt8& cmd_string){   //정수값(1~9)를 받아, 각 경우에 해당하는 명령배열을 goArr에 저장   
  uint8_t Mode = cmd_string.data;
  switch(Mode){
    case 1 : goArr=PWR_On; break;
    case 2 : goArr=PWR_Off; break;
    case 3 : goArr=Vel_CTRL; break;
    case 4 : goArr=Vel_GOGO; break;
    case 7 : goArr=re_set; break;
    case 8 : goArr=Q2; break; //추가
    case 9 : goArr=QP; break; //추가
    default : goArr=nothing;
  }  
}
/******************************************************************/
/******************************************************************/

uint32_t t_time, id, i;
can_message_t tx_msg, rx_msg;
/*
 *  typedef struct 
 *  {
 *    uint32_t id      : Identifier of received message
 *    uint32_t length  : Length of received message data
 *    uint8_t  data[8] : Data of received message
 *    uint8_t  format  : Type of ID
 *  } can_message_t;
 * 
 * BAUDRATE :
 *   CAN_BAUD_125K
 *   CAN_BAUD_250K
 *   CAN_BAUD_500K
 *   CAN_BAUD_1000K
 * 
 * FORMAT :
 *   CAN_STD_FORMAT
 *   CAN_EXT_FORMAT
*/

osThreadId thread_id_CAN;
osThreadId thread_id_COMMAND;
//osThreadId thread_id_printing;

////////////////////////////////////////////////////////////
//// 50ms 마다 goArr 에 저장된 배열을 CAN통신으로 날려보냄.. ////
static void Thread_CAN(void const *argument)  
{
  (void) argument;
  
  tx_msg.id = 0x1;                      //제어기 ID=1
  tx_msg.format = CAN_STD_FORMAT; //

  for(;;)  //무한 반복  
  {
    if(same_vel==false){CAN_Until_Semi(goArr,0,0);}
    delay(50);
  }
}
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

/**************************************************************/
/**********************  SET UP  ******************************/

void setup() 
{

  nh.initNode();
  nh.advertise(pub); //추가
  nh.advertise(pub2); //추가
  nh.subscribe(sub);
  nh.subscribe(sub2);
  
  Serial.begin(115200);
  Serial.println("============================");
  Serial.println("=== CAN Message Example! ===");
  if (CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT) == false)
  {Serial.println("CAN open fail!!");} 
  else
  {
    id = 0x123;
    CanBus.configFilter(id, 0, CAN_STD_FORMAT);
  }
/*
  CAN_Until_Semi(PWR_On,0,0);
  delay(700);
  CAN_Until_Semi(Vel_CTRL,0,0);
  delay(700);
  CAN_Until_Semi(Vel_GOGO,0,0);
 */
 
  // define thread
  osThreadDef(THREAD_NAME_CAN, Thread_CAN, osPriorityNormal, 0, 1024);
  osThreadDef(THREAD_NAME_COMMAND,  Thread_Command,  osPriorityNormal, 0, 1024);
  //osThreadDef(THREAD_NAME_PRINTING, Thread_printing,  osPriorityNormal, 0, 1024);

  // create thread
  thread_id_CAN = osThreadCreate(osThread(THREAD_NAME_CAN), NULL);
  thread_id_COMMAND = osThreadCreate(osThread(THREAD_NAME_COMMAND), NULL);
  //thread_id_printing = osThreadCreate(osThread(THREAD_NAME_PRINTING), NULL);

  // start kernel
  osKernelStart();
}

/*********************   SET UP   *****************************/
/**************************************************************/


void loop()   //loop함수는 사용하지 않음
{
}
int R_RPM=0,L_RPM=0;
static void Thread_Command(void const *argument)
{
  (void) argument;
  
  for(;;)
  {
    msg.data=(int)(R_RPM/26); //추가
    msg2.data=(int)(L_RPM/26); //추가
    pub.publish(&msg); //추가
    pub2.publish(&msg2);
    nh.spinOnce();
    delay(5);   //can통신을 보내주는 delay보다 짧아야 한다. 
  }
}


/*************배열인자를 8개씩 끊어서 CAN으로 보내주는 함수*********************/
void CAN_Until_Semi(uint8_t* Arr,int j,int k)  //j=0, k=0 으로 시작해야함
{
  int i;
    for(i = j; Arr[i]!=';' ; i++)
    { 
        if(i==(7+j)&&Arr[i]!=';'){tx_msg.data[i-k] = Arr[i]; break;}
        tx_msg.data[i-k] = Arr[i];
    }
    if(Arr[i]==';')
    {tx_msg.data[i-k]=';';
    tx_msg.length = (i-k)+1;
    CanBus.writeMessage(&tx_msg);
    }
    else
    {
      tx_msg.length = (i-k)+1;
      CanBus.writeMessage(&tx_msg);      
      k=k+8;
      j=j+8; 
      CAN_Until_Semi(Arr,j,k);
    }
}
/***************************************************************************/



/*****************************************************************************************************/
/************전진 선속도, 회전 각속도를 받아서 좌,우 바퀴의 회전속도(rpm)을 계산하고 속도 배열에 넣어주는 함수*****/


static int pre_R_RPM=-1;
static int pre_L_RPM=-1;




void cmd_vel2RPM(uint8_t* vel_array, float linear_vel, float angular_vel)
{
  R_RPM = (int)(gear_ratio*30.0*((2*linear_vel) + (distance*angular_vel))/(2*wheel_radius*3.141593));
  L_RPM = -1*(int)(gear_ratio*30.0*((2*linear_vel) - (distance*angular_vel))/(2*wheel_radius*3.141593));

  if(goArr==Vel_GOGO&&R_RPM==pre_R_RPM && L_RPM==pre_L_RPM){same_vel=true;} else {same_vel=false;pre_R_RPM=R_RPM;pre_L_RPM=L_RPM;     //추가2

  if(R_RPM >=  2800){R_RPM=  2800;}  
  if(R_RPM <= -2800){R_RPM= -2800;}
  if(L_RPM >=  2800){L_RPM=  2800;}
  if(L_RPM <= -2800){L_RPM= -2800;}

  if(R_RPM>=0){vel_array[2]='+';} else{vel_array[2]='-';R_RPM*=-1;}
  vel_array[6]=(uint8_t)(48+(R_RPM%10));
  vel_array[5]=(uint8_t)(48+((R_RPM/10)%10));
  vel_array[4]=(uint8_t)(48+(((R_RPM/10)/10)%10));
  vel_array[3]=(uint8_t)(48+((((R_RPM/10)/10)/10)%10));

  if(L_RPM>=0){vel_array[8]='+';} else{vel_array[8]='-';L_RPM*=-1;}
  vel_array[12]=(uint8_t)(48+(L_RPM%10));
  vel_array[11]=(uint8_t)(48+((L_RPM/10)%10));
  vel_array[10]=(uint8_t)(48+(((L_RPM/10)/10)%10));
  vel_array[9]=(uint8_t)(48+((((L_RPM/10)/10)/10)%10));
  
  }

}
/************************************************************************************************/
