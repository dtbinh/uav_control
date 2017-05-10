#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <termios.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
// 3 Threads: Keypad, Vicon data, Controller
#define NUM_THREADS 3
#define PI 3.1415926

// Adafruit PCA9685 Servo Information
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD
#define i2c_addr 0x40

// User header files
//#include "PCA9685.h"


// ESCs: BL v2

int mtr_addr[8]={41, 42, 43, 44, 45, 46, 47, 48};// motor addresses 1-6
int thr = 20;// i2c motor command

// Servos
int servo_addr[6]={0, 1, 2, 3, 4, 5};// servo addresses 1-6
uint16_t servopl[6];// i2c servo pulse length (duty_cycle[i] = servopl[i]/4095 @ ~325 Hz)

void test(vector<int> msg){
    printf(msg[0]);
}
int main()
{
    printf("testforking");
//    int i, ii, k;
//    int fhi2c;
//    // Open i2c:
//    fhi2c = open("/dev/i2c-1", O_RDWR);// Change
//    printf("Opening i2c port...\n");
//    if(fhi2c!=3)
//        printf("ERROR opening i2c port.\n");
//    else
//        printf("The i2c port is open.\n");
//    usleep(10);
//
//  
//   Reset Adafruit PCA9685
//  intf("Reseting Adafruit PCA9685 device...\n");
//  (resetDev(fhi2c)!=1)
//    printf("ERROR: Adafruit device not reset.\n");
//  se
//    printf("Adafruit 9685 device is reset and ready for use.\n");
//  
//   Set pwm frequency to 300 Hz -> ~325 Hz with Adafruit clock
//  oat freq = 300;
//  (pwmFreq(fhi2c, freq)!=1)
//    printf("ERROR: setting Adafruit pwm frequency.\n");
//  se
//    printf("Adafruit frequency set to %e Hz.\n", freq);
//  
//  intf("\n\nPress ENTER to begin spinning motors.\n");
//  anf("%c", &PressEnter);
//  intf("\n\n\n\n");
//  
//   Execute servo output commands
//  nt16_t pl_0[6] = {1300, 1285, 1230, 1280, 1215, 1275};
//  
//  r(i=0;i<6;i++)
//  
//    pwmPulse(fhi2c, servo_addr[i], 0, pl_0[i]);
//    tcflush(fhi2c, TCIFLUSH);
//    usleep(10);
//  
//  
//  
//      k = 6;
//  
//      // Execute motor output commands
//      for(ii=1;ii<2000;ii++)
//      {
//          {
//              if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[k])<0)
//                  printf("ERROR: ioctl\n");
//  
//              if(write(fhi2c, &thr, 1)!=1)
//                  printf("ERROR: write\n");
//              tcflush(fhi2c, TCIFLUSH);
//              usleep(50);
//          }
//      }
//  
//      close(fhi2c);

    return 0;
}
