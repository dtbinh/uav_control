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
#include <iostream>


using namespace std;
// User header files
class hw_interface{

    public:
        std::vector<int> motor_address;
        int fhi2c;
        bool MOTOR_ON;
        std::vector<int> status;

        hw_interface(std::vector<int> address){
            motor_address = address;
            open_i2c();
        };

        ~hw_interface(){};

        std::vector<int> get_motor_address(){
            return motor_address;
        }

        void single_motor_test(int motor, int throttle){
            tcflush(fhi2c, TCIOFLUSH);
            for(int ii=1;ii<2000;ii++)
            {
                if(ioctl(fhi2c, I2C_SLAVE, motor_address[motor])<0)
                    printf("ERROR: ioctl\n");

                if(write(fhi2c, &throttle, 1)!=1)
                    printf("ERROR: write\n");
                tcflush(fhi2c, TCIFLUSH);
                usleep(5000);
            }
        }

        std::vector<int> motor_command(std::vector<int> throttle, bool MotorWarmup, bool MOTOR_ON){
  // Execute motor output commands
//  int length = 6;
//  char buffer[6];
//  for(int i=0;i<4;i++){
//    tcflush(fhi2c, TCIOFLUSH);
//
//    usleep(500);
//
//    if(ioctl(fhi2c, I2C_SLAVE, motor_address[i])<0)
//    printf("ERROR: ioctl\n");
//    read(fhi2c,buffer,length);
//    //printf("Motor:%d ",i);
//    for(int k=0;k<length;k++){
//      status[i*6+k] = (int)((uint8_t)buffer[k]);
//      //printf("%d, ", buffer[k]);
//    }
//    //printf("\n");
//  }
//
            for(int i = 0; i < 4; i++){
    //printf("Motor %i I2C write command of %i to address %i (%e N).\n", i, thr[i], mtr_addr[i], f[i]);
            tcflush(fhi2c, TCIOFLUSH);
            usleep(500);
            if(ioctl(fhi2c, I2C_SLAVE, motor_address[i])<0)
                printf("ERROR: ioctl\n");
            if(MOTOR_ON == false)// set motor speed to zero
                throttle[i] = 0;
            else if(MotorWarmup == true)// warm up motors at 20 throttle command
                throttle[i] = 20;
            while(write(fhi2c, &throttle[i], 1)!=1)
                printf("ERROR: Motor %i I2C write command of %i to address %i not sent.\n", i, throttle[i], motor_address[i]);
            }
        return status;
        }

    void open_i2c(){
        this->fhi2c = open("/dev/i2c-1", O_RDWR);
        std::cout<<"opening i2c"<<std::endl;
        std::cout<<fhi2c<<std::endl;
          printf("Opening i2c port...\n");
        if(fhi2c<0)
            printf("ERROR opening i2c port.\n");
        else
            printf("The i2c port is open.\n");
        usleep(100000);
        tcflush(fhi2c, TCIFLUSH);
        usleep(100000);
          // Call and response from motors
  printf("Checking motors...\n");
  int motornum, motoraddr, motorworks;
  int thr0 = 0;// 0 speed test command
  char PressEnter;

  for(motornum = 1; motornum <= 4; motornum++){
    motoraddr = motornum+40;// 1, 2, 3, ... -> 41, 42, 43, ...
    while(1){
      motorworks = 1;// will remain 1 if all works properly
      if(ioctl(fhi2c, I2C_SLAVE, motoraddr)<0){
        printf("ERROR: motor %i ioctl\n", motornum);
        motorworks = 0;// i2c address not called
      }
      usleep(10);
      if(write(fhi2c, &thr0, 1)!=1){
        printf("ERROR: motor %i write\n", motornum);
        motorworks = 0;
      }
      usleep(10);
      tcflush(fhi2c, TCIOFLUSH);
      if(motorworks == 1){
        printf("Motor %i working...\n", motornum);
        break;
      }
      else{
        printf("Fix motor %i, then press ENTER.\n\n", motornum);
        printf("Note: another i2c device may interupt the signal if\n");
        printf("any i2c wires are attached to unpowered components.\n");
        scanf("%c",&PressEnter);
        break;
      }
    }
  }
  printf("All motors are working.\n");
    }
    std::vector<int> test(std::vector<int> msg){
        std::cout<<msg[0]<<std::endl;
        return msg;
    };
};
