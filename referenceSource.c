#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <pthread.h>
#include <math.h>

#define ENCA 25
#define ENCB 18
#define SWITCH_1 20
#define SWITCH_2 21
#define MOTOR_1 19
#define MOTOR_2 26
#define LED_R 17
#define LED_G 27
#define LED_Y 22

#define POS2ENC 236 // pulse number
#define LOOPTIME 5 // (ms)
#define MAX_INPUT 100 // softpwmwrite maximum -> 퍼센트로 생각

#define PGAIN 500
#define IGAIN 0.001
#define DGAIN 1
#define pass 0.01

pthread_t thread1;
pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

struct P {
    float one; // read follow.csv
    float two; // write follow.csv
    float vel;
};

struct P tr[3000] = { 0 };

volatile int terminateISR = 0;

volatile long encpulse = 0;
volatile float redgearPos = 0;
volatile float prevgearPos = 0;

volatile float error = 0;
volatile float error_prev = 0;
volatile float error_d = 0;
volatile float error_i = 0;

volatile float error_vel = 0;
volatile float vel;
volatile float vel_prev = 0;
volatile float Stop_pos;

volatile int motor_input = 0;
int enc_count = 1;
int time_c = 0;
int time_p = 0;
int Flag = 1;
volatile int cnt1 = 0;
volatile int dcnt = 0;


void write_file() {
    FILE* fp = fopen("follow.csv", "w");
    if (!fp) {
        printf("wrong output file\n");
        return;
    }
    for (int i = 0; i < 15000 / LOOPTIME; i++) {
        fprintf(fp, "%f\n", tr[i].two);
    }
   
    fclose(fp);
     printf("write end!\n");
}

void read_file() {

    FILE* fp = fopen("follow.csv", "r");
    if (!fp) {
        printf("wrong input file");
        return;
    }
    int index = 0;
    for (int i = 0; i < 15000 / LOOPTIME; i++) {
        char line[50];
        fgets(line, 50, fp);
        sscanf(line, "%f\n", &tr[index].one);
        index++;
    }
    /*for (int j = 0; j < 15000 / LOOPTIME; j++) {
        if (j == 0) {
            tr[j].vel = tr[j].two / (LOOPTIME * 0.001);
        }
        else {
            tr[j].vel = (tr[j].two - tr[j - 1].two) / (LOOPTIME * 0.001);
        }
    }*/
    fclose(fp);
}

void setup() {
    wiringPiSetupGpio();
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    pinMode(SWITCH_1, INPUT);
    pinMode(SWITCH_2, INPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_Y, OUTPUT);
    softPwmCreate(MOTOR_1, 0, MAX_INPUT);
    softPwmCreate(MOTOR_2, 0, MAX_INPUT);
    softPwmWrite(MOTOR_1, 0);
    softPwmWrite(MOTOR_2, 0);
    read_file();
}

void var_reset() {
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
    terminateISR = 0;
    cnt1 = 0;
    dcnt = 0;
}

// ENCA,B
void encAfunc() {
    int A = digitalRead(ENCA);
    int B = digitalRead(ENCB);

    if (A == HIGH) {
        if (B == HIGH) { encpulse--; }
        else { encpulse++; }
    }
    else {
        if (B == HIGH) { encpulse++; }
        else { encpulse--; }
    }
    redgearPos = (float)encpulse / POS2ENC;
}
void encBfunc() {
    int A = digitalRead(ENCA);
    int B = digitalRead(ENCB);

    if (B == HIGH) {
        if (A == HIGH) { encpulse++; }
        else { encpulse--; }
    }
    else {
        if (A == HIGH) { encpulse--; }
        else { encpulse++; }
    }
    redgearPos = (float)encpulse / POS2ENC;
}

// low pass filter
void lowpass() { 
    vel = pass * vel + (1 - pass) * vel_prev;
    vel_prev = vel;
}

//PID
void PIDcontrol(int KP, int KI, int KD, float refpos) {
    error = refpos - redgearPos;
    error_d = (error - error_prev) / (LOOPTIME * 0.001);  //error_d = - (redgearPos - prevgearPos) / (LOOPTIME * 0.001); 
    error_i += (error * (LOOPTIME * 0.001));

    motor_input = KP * error + KD * error_d + KI * error_i;

    vel = (redgearPos - prevgearPos) / (LOOPTIME * 0.001); // velocity 계산 -> 외란 시 cnt 판단용
    
    lowpass();
    
    //error_vel = tr[cnt1].vel - vel;

    error_prev = error;
    prevgearPos = redgearPos;
    enc_count++;
    //printf("%d \n",motor_input);
    printf("motor input :%d Flag: %d Position: %f cnt1 : %d\n",motor_input,Flag, redgearPos,cnt1);
    
    //Flag => 1 : Normal drive
    if (Flag == 1) {

        if (motor_input > 0)
        {
            if (motor_input > MAX_INPUT)
            {
                motor_input = MAX_INPUT;
            }
            
            softPwmWrite(MOTOR_1, motor_input);
            softPwmWrite(MOTOR_2, 0); // Counterclockwise -> motor 1 only
        }
        else
        {
            motor_input = motor_input * (-1);
            
            if (motor_input > MAX_INPUT)
            {
                motor_input = MAX_INPUT;
            }
            
            softPwmWrite(MOTOR_1, 0);
            softPwmWrite(MOTOR_2, motor_input); // Clockwise
        }

        if (cnt1 > 500 && (error > 0.04|| error < -0.04) && (vel < 0.04 && vel > -0.04))
            //vel -> inside the range , error_vel -> over certain value
        {
            dcnt++; 
            printf("%d\n", dcnt);
            if (dcnt >= 10) Flag = 2;
        }
        else { dcnt = 0; }


    }
    // Flag == 2 : After collision
    else
    {   
        printf("Flag2 lunched\n");
        digitalWrite(LED_R, 1);
        digitalWrite(LED_G, 0);
        digitalWrite(LED_Y, 0);
        
        if (motor_input > 0) {
            Stop_pos = redgearPos;

            while (redgearPos - Stop_pos >= -0.5) 
            {
                softPwmWrite(MOTOR_1, 0);
                softPwmWrite(MOTOR_2, 20);
                //printf("%d      %f\n", Flag, Stop_pos);
            }

            softPwmWrite(MOTOR_1, 0);
            softPwmWrite(MOTOR_2, 0);
            delay(100);
                pthread_mutex_unlock(&dataMutex);
                pthread_exit(NULL);
            
        }
        else {
            Stop_pos = redgearPos;
            
            while (redgearPos - Stop_pos >= 0.5) 
            {
                softPwmWrite(MOTOR_1, 20);
                softPwmWrite(MOTOR_2, 0);
                //printf("%d      %f\n", Flag, Stop_pos);
            }
           
            softPwmWrite(MOTOR_1, 0);
            softPwmWrite(MOTOR_2, 0);
            
            delay(100);
                pthread_mutex_unlock(&dataMutex);
                pthread_exit(NULL);
            
        }

    }

}

// driveMotor
void* driveMotor_thread(void* arg) {

    pthread_mutex_lock(&dataMutex);

    while (cnt1 < 15000 / LOOPTIME && !terminateISR) 
    {
        time_c = millis();

        if (time_c - time_p >= LOOPTIME) 
        {
            time_p = time_c;
            PIDcontrol(PGAIN, IGAIN, DGAIN, tr[cnt1].two);
            cnt1++;
           // tr[cnt1].two = redgearPos;
            //if (cnt1 % 100 == 0) printf("%f   %f\n", redgearPos, tr[cnt1].one);
            if (terminateISR) break;
        }
    }

    softPwmWrite(MOTOR_1, 0);
    softPwmWrite(MOTOR_2, 0);

    pthread_mutex_unlock(&dataMutex);
    pthread_exit(NULL);
}


void driveMotor() {
    pthread_create(&thread1, NULL, driveMotor_thread, NULL);
}


void traject_memory() {

    /*pthread_mutex_lock(&dataMutex);*/
    printf("reset1 started! %f\n", redgearPos);

    var_reset();
    cnt1 = 0;

    digitalWrite(LED_R, 0);
    digitalWrite(LED_G, 0);
    digitalWrite(LED_Y, 1);
    terminateISR = 0;

    while (cnt1 < 15000 / LOOPTIME && !terminateISR) 
    {
        time_c = millis();

        if (time_c - time_p >= LOOPTIME) 
        {
            time_p = time_c;
            tr[cnt1].two = redgearPos;
            cnt1++;
            if (cnt1 % 100 == 0) printf("redgearPos: %f , encpulse: %ld , tr[cnt1].two: %f\n", redgearPos, encpulse, tr[cnt1].two);
            if (terminateISR) break;
        }
    }

    write_file();

    digitalWrite(LED_R, 0);
    digitalWrite(LED_G, 0);
    digitalWrite(LED_Y, 0);

  //  pthread_mutex_unlock(&dataMutex);
    //pthread_exit(NULL);

}
void traject_follow() {

    printf("reset2 started! %f\n", redgearPos);
    var_reset();

    digitalWrite(LED_R, 0);
    digitalWrite(LED_G, 1);
    digitalWrite(LED_Y, 0);
    terminateISR = 0;

    driveMotor();

}

int main() {

    setup();

    wiringPiISR(ENCA, INT_EDGE_BOTH, encAfunc);
    wiringPiISR(ENCB, INT_EDGE_BOTH, encBfunc);
    if (wiringPiISR(SWITCH_1, INT_EDGE_RISING, &traject_memory) < 0) {
        printf("error");
        return -1;
    }
    if (wiringPiISR(SWITCH_2, INT_EDGE_RISING, &traject_follow) < 0) {
        printf("error");
        return -1;
    }

    while (1) {

    }
}
