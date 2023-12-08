#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <pthread.h>
#include <math.h>

#define ENCA 23
#define ENCB 24
#define SWITCH_1 16
#define SWITCH_2 20
#define SWITCH_3 21
#define MOTOR_1 3
#define MOTOR_2 5          
#define LED_R 13
#define LED_G 19
#define LED_Y 26

#define POS2ENC 236.35
#define LOOPTIME 1 // (ms)
#define MAX_INPUT 100

struct P {
   float one;
   float two;
   float thr;
};
struct P tr[15000] = { 0 };

//Setting PID Gain
int KP = 1100;
int KI = 0;
int KD = 0;


float reading[15000];

volatile int trajectory_num = 0;
volatile int terminateISR = 0;
volatile long encpulse = 0;
volatile float redgearPos = 0; 
volatile float prevgearPos = 0; 
volatile float error = 0;  
volatile float error_prev = 0; 
volatile float error_d = 0;  
volatile float error_i = 0;  
volatile float motor_input = 0;
int enc_count = 1;
int time_c = 0; 
int time_p = 0; 
volatile int cnt1 = 0;
volatile int cnt2 = 0; 
volatile int cnt3 = 0;

void write_file() {
   FILE* fp = fopen("read3_1.csv", "w");
   if (!fp) {
      printf("wrong output file\n");
      return;
   }
   for (int i = 0; i < 15000; i++) {
      fprintf(fp, "%f\n", reading[i]);
   }
   fclose(fp);
}

void read_file() {
   
   FILE* fp = fopen("1ms.csv", "r");
   if (!fp) {
      printf("wrong input file");
      return;
   }
   int index = 0;
   for (int i = 0; i < 15000; i++) {
         char line[50];
         fgets(line, 50, fp);
         sscanf(line, "%f,%f,%f\n", &tr[index].one, &tr[index].two, &tr[index].thr);
         index++;
   }
   fclose(fp);
}

void setup() {
   wiringPiSetupGpio();
   pinMode(ENCA, INPUT);
   pinMode(ENCB, INPUT);
   pinMode(SWITCH_1, INPUT);
   pinMode(SWITCH_2, INPUT);
   pinMode(SWITCH_3, INPUT);
   pinMode(LED_R, OUTPUT);
   pinMode(LED_G, OUTPUT);
   pinMode(LED_Y, OUTPUT);
   softPwmCreate(MOTOR_1, 0, MAX_INPUT);
   softPwmCreate(MOTOR_2, 0, MAX_INPUT);
   softPwmWrite(MOTOR_1, 0);
   softPwmWrite(MOTOR_2, 0);   
   read_file();
}

//read encoder A, B
void encAfunc() {
   int A = digitalRead(ENCA);
   int B = digitalRead(ENCB);

   if (A == HIGH) {
      if (B == HIGH) { encpulse++; }
      else { encpulse--; }
   }
   else {
      if (B == HIGH) { encpulse--; }
      else { encpulse++; }
   }
   redgearPos = (float)encpulse / POS2ENC;
}
void encBfunc() {
   int A = digitalRead(ENCA);
   int B = digitalRead(ENCB);

   if (B == HIGH) {
      if (A == HIGH) { encpulse--; }
      else { encpulse++; }
   }
   else {
      if (A == HIGH) { encpulse++; }
      else { encpulse--; }
   }
   redgearPos = (float)encpulse / POS2ENC;
}

//PID control
void PIDcontrol(int KP, int KI, int KD, float ref_pos) {  
   error = ref_pos - redgearPos;
   //error_d = - (redgearPos - prevgearPos) / (LOOPTIME * 0.001);    
   error_d = (error - error_prev) / (LOOPTIME * 0.001);
   error_i += (error * (LOOPTIME * 0.001));

   if (error > 0.1 && error < -0.1) {
      KP = 5000;
      KI = 5;
      KD = 10;
   }
   
   motor_input = KP * error + KD * error_d + KI * error_i;

   error_prev = error;
   prevgearPos = redgearPos;
   enc_count++;
   if (motor_input > 0) {
      if (motor_input > MAX_INPUT) {
         motor_input = MAX_INPUT;
      }
      softPwmWrite(MOTOR_1, 0);
      softPwmWrite(MOTOR_2, motor_input);
   }
   else {
      motor_input = motor_input * (-1);
      if (motor_input > MAX_INPUT) {
         motor_input = MAX_INPUT;
      }
      softPwmWrite(MOTOR_1, motor_input);
      softPwmWrite(MOTOR_2, 0);
   }
   if(enc_count%100==0) {printf("%d  %d  %d       %f\n", KP, KI, KD, motor_input);}
}


//+ trajectory_num + LED
void driveMotor_1() {
   
   while (cnt1 < 15000 && !terminateISR) {
      
      time_c = millis();
      if (time_c - time_p >= LOOPTIME) {
         time_p = time_c;
         PIDcontrol(1200, 0.001, 8, tr[cnt1].one);
         cnt1++;
         reading[cnt1] = redgearPos;
         if(cnt1%100 ==0)
         {printf("%f   %f\n",redgearPos,tr[cnt1].one);
            }
         if (trajectory_num != 1 || terminateISR) break;
      }
   }
   
   softPwmWrite(MOTOR_1, 0);
   softPwmWrite(MOTOR_2, 0);
   digitalWrite(LED_R,LOW);
   write_file();
   
}

void driveMotor_2() {  
   
   while (cnt2 < 15000 && !terminateISR) {
      time_c = millis();
      if (time_c - time_p >= LOOPTIME) {
         time_p = time_c;
         if (cnt2 < 4500) {
            PIDcontrol(1200, 0.001, 8, tr[cnt2].two);
         }
         else if(cnt2>4500 && cnt2<5176){
            softPwmWrite(MOTOR_1, 100);
            softPwmWrite(MOTOR_2, 0); //negative max
            }
         else if(cnt2>=5176 && cnt2<6910){
             PIDcontrol(3100, 1, 9, tr[cnt2].two); //positive max
            }
         else if(cnt2>=6910 && cnt2<7470){
            softPwmWrite(MOTOR_1, 100);
            softPwmWrite(MOTOR_2, 0);//negative max
            }
         else if(cnt2>=7470 && cnt2<8005){
            softPwmWrite(MOTOR_1, 0);
            softPwmWrite(MOTOR_2, 100);
            }
         else if (cnt2 < 10000 && cnt2 >= 8005){
            PIDcontrol(3100, 1, 9, tr[cnt2].two);
         }
         else if (cnt2 >= 10000) {
            PIDcontrol(1200, 0.001, 8, tr[cnt2].two);
         }
         cnt2++;
         reading[cnt2] = redgearPos;
         if(cnt2%100 ==0)
         {printf("%f   %f\n",redgearPos,tr[cnt2].two);
            }
         if (trajectory_num != 2 || terminateISR) break;
      }
   }
   softPwmWrite(MOTOR_1, 0);
   softPwmWrite(MOTOR_2, 0);
   digitalWrite(LED_G, LOW);
   write_file();
   digitalWrite(LED_R, LOW);
}

void driveMotor_3() {  
   
   while (cnt3 < 15000 && !terminateISR) {
      time_c = millis();
      if (time_c - time_p >= LOOPTIME) {
         time_p = time_c;
         
         if(cnt3<13503)
         {PIDcontrol(3100, 1, 9, tr[cnt3].thr);
            }
         else if(cnt3>=13503 && cnt3<14483){
            softPwmWrite(MOTOR_1, 100);
            softPwmWrite(MOTOR_2, 0); //negative max
            }
         else if(cnt3>=14483){
             softPwmWrite(MOTOR_1, 0);
            softPwmWrite(MOTOR_2, 100); //positive max
            }
         
         cnt3++;
         reading[cnt3] = redgearPos;
         if(cnt3%100 ==0)
         {printf("%f  %f\n",redgearPos,tr[cnt3].thr);
            }
         if (trajectory_num != 3 || terminateISR) break;
      }
   }
   softPwmWrite(MOTOR_1, 0);
   softPwmWrite(MOTOR_2, 0);
   digitalWrite(LED_Y, LOW);
   write_file();
}


void var_reset(){
   terminateISR = 1;
   softPwmWrite(MOTOR_1, 0);
   softPwmWrite(MOTOR_2, 0);
   encpulse = 0;
   motor_input = 0;
   error = 0;
   error_d = 0;
   error_i = 0;
   error_prev = 0;
   redgearPos = 0;
   prevgearPos = 0;
   time_c = 0;
   time_p = 0;
   terminateISR = 0; //important
   }

void reset1() {
   printf("reset1 started! %f\n",redgearPos);
   var_reset();
   cnt1=0;
   trajectory_num = 1;
   
   digitalWrite(LED_R, HIGH);
   digitalWrite(LED_G, LOW);
   digitalWrite(LED_Y, LOW);
   terminateISR = 0;
   printf("cnt1=%d", cnt1);
   printf("Motor stopped!\n");
   driveMotor_1();
}
void reset2() {
   printf("reset2 started! %f\n",redgearPos);
   var_reset();
   cnt2=0;
   trajectory_num = 2;
   
   digitalWrite(LED_R, LOW);
   digitalWrite(LED_G, HIGH);
   digitalWrite(LED_Y, LOW);
   terminateISR = 0;
   printf("cnt2=%d", cnt2);
   driveMotor_2();  
}
void reset3() {
   printf("reset3 started! %f\n",redgearPos);  
   var_reset();
   cnt3 = 0;
   trajectory_num=3;
  
   digitalWrite(LED_R, LOW);
   digitalWrite(LED_G, LOW);
   digitalWrite(LED_Y, HIGH);
   printf("cnt3=%d", cnt3);
   driveMotor_3();
}


int main() {

   setup();

   wiringPiISR(ENCA, INT_EDGE_BOTH, encAfunc);
   wiringPiISR(ENCB, INT_EDGE_BOTH, encBfunc);
   if (wiringPiISR(SWITCH_1, INT_EDGE_RISING, &reset1) < 0) {
      printf("error");
      return -1;
   }
   if (wiringPiISR(SWITCH_2, INT_EDGE_RISING, &reset2) < 0) {
      printf("error");
      return -1;
   }
   if (wiringPiISR(SWITCH_3, INT_EDGE_RISING, &reset3) < 0) {
      printf("error");
      return -1;
   }
  
   while (1) {
      
   }
}
