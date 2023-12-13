
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <pthread.h>
#include <math.h>

#define ENCA 18
#define ENCB 25
#define SWITCH_1 20
#define SWITCH_2 21
#define MOTOR_1 19
#define MOTOR_2 26
#define LED_R 17
#define LED_G 27
#define LED_Y 22

#define PGAIN 10
#define IGAIN 0
#define DGAIN 0

#define POS2ENC 236.35
#define LOOPTIME 5 // (ms)
#define MAX_INPUT 100

pthread_t thread1;
pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

struct P {
    float one;
    float two;
    float vel;
};

struct P tr[3000] = { 0 };

volatile int terminateISR = 0;

volatile long encpulse = 0;
volatile float redgearPos = 0;
volatile float prevgearPos = 0;

volatile long error = 0;
volatile long error_prev = 0;
volatile long error_d = 0;
volatile long error_i = 0;

volatile float error_vel = 0;
volatile float vel;

volatile int motor_input = 0;
int enc_count = 1;
int time_c = 0;
int time_p = 0;
volatile int cnt1 = 0;
volatile int dcnt = 0;
int Flag = 1;

volatile float Stop_pos;

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
}

void read_file() {

    FILE* fp = fopen("test.csv", "r");
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
    for (int j = 0; j < 15000 / LOOPTIME; j++) {
        if (j == 0) {
            tr[j].vel = tr[j].one / (LOOPTIME * 0.001);
        }
        else {
            tr[j].vel = (tr[j].one - tr[j - 1].one) / (LOOPTIME * 0.001);
        }
    }
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

//PID
void PIDcontrol(int KP, int KI, int KD, float refpos) {
    error = refpos - redgearPos;
    error_d = (error - error_prev) / (LOOPTIME * 0.001);  //error_d = - (redgearPos - prevgearPos) / (LOOPTIME * 0.001); 
    error_i += (error * (LOOPTIME * 0.001));

    motor_input = KP * error + KD * error_d + KI * error_i;
    vel = (redgearPos - prevgearPos) / (LOOPTIME * 0.001); // velocity 계산 -> 외란 시 cnt 판단용

    error_vel = tr[cnt1].vel - vel;


    error_prev = error;
    prevgearPos = redgearPos;
    enc_count++;

    //printf("%d \n",motor_input);
    if (Flag == 1) {

        if (motor_input > 0)
        {
            //softPwmWrite(MOTOR_1, motor_input);
            //softPwmWrite(MOTOR_2, 0);
            softPwmWrite(MOTOR_1, motor_input);
            softPwmWrite(MOTOR_2, 0);
            printf("%f   %f\n", redgearPos, refpos);
            //printf("%f    %f\n", vel, error_vel);

            if (motor_input > MAX_INPUT)
            {
                motor_input = MAX_INPUT;
            }

            if (vel < 0.00001 && vel>-0.00001 && error_vel > 0.001 && error_vel < -0.001)
                //vel -> inside the range , error_vel -> over certain value
            {
                dcnt++; // velocity가 충분히 낮으면 dcnt++
                printf("%d\n", dcnt);
                if (dcnt >= 50) Flag = 2;
            }

        }
        else if (motor_input < 0)
        {
            motor_input = motor_input * (-1);
            softPwmWrite(MOTOR_1, 0);
            softPwmWrite(MOTOR_2, motor_input);
            printf("%f   %f\n", redgearPos, refpos);
            //printf("%f   %f\n", vel, error_vel);

            if (motor_input > MAX_INPUT)
            {

                softPwmWrite(MOTOR_1, MAX_INPUT);
                softPwmWrite(MOTOR_2, 0);

            }

            if (vel < 0.00001 && vel>-0.00001 && error_vel > 0.001 && error_vel < -0.001)
            {
                dcnt++;
                printf("%d\n", dcnt);
                if (dcnt >= 50) Flag = 2;

            }

        }

    }
    else if (Flag == 2)
    {

        if (vel > 0) {
            Stop_pos = redgearPos;
            while (redgearPos - Stop_pos >= -0.5) {
                softPwmWrite(MOTOR_1, 0);
                softPwmWrite(MOTOR_2, 20);
                //printf("%d      %f\n", Flag, Stop_pos);
            }
            softPwmWrite(MOTOR_1, 0);
            softPwmWrite(MOTOR_2, 0);
        }


    }
    else {
        Stop_pos = redgearPos;
        while (redgearPos - Stop_pos >= 0.5) {

            softPwmWrite(MOTOR_1, 20);
            softPwmWrite(MOTOR_2, 0);
            //printf("%d      %f\n", Flag, Stop_pos);

        }
        softPwmWrite(MOTOR_1, 0);
        softPwmWrite(MOTOR_2, 0);
    }

}




//+ trajectory_num + LED
void* driveMotor_thread(void* arg) {

    pthread_mutex_lock(&dataMutex);

    while (cnt1 < 15000 / LOOPTIME && !terminateISR) {
        time_c = millis();
        if (time_c - time_p >= LOOPTIME) {
            time_p = time_c;
            PIDcontrol(PGAIN, IGAIN, DGAIN, tr[cnt1].one);
            cnt1++;
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
}

void traject_memory() {

    pthread_mutex_lock(&dataMutex);
    printf("reset1 started! %f\n", redgearPos);

    var_reset();
    cnt1 = 0;

    digitalWrite(LED_R, 0);
    digitalWrite(LED_G, 0);
    digitalWrite(LED_Y, 1);
    terminateISR = 0;
    while (cnt1 < 15000 / LOOPTIME && !terminateISR) {
        time_c = millis();
        if (time_c - time_p >= LOOPTIME) {
            time_p = time_c;
            tr[cnt1].two = redgearPos;
            cnt1++;
            if (cnt1 % 100 == 0) printf("%f  %ld %f\n", redgearPos, encpulse, tr[cnt1].one);
            if (terminateISR) break;
        }
    }
    write_file();
    digitalWrite(LED_R, 0);
    digitalWrite(LED_G, 0);
    digitalWrite(LED_Y, 0);

    pthread_mutex_unlock(&dataMutex);
    pthread_exit(NULL);

}
void traject_follow() {
    printf("reset2 started! %f\n", redgearPos);

    var_reset();
    digitalWrite(LED_R, 0);
    digitalWrite(LED_G, 1);
    digitalWrite(LED_Y, 0);
    terminateISR = 0;
    //delay(1000);
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
