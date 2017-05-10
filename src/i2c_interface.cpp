#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H
#include <linux/i2c-dev.h>
#include <sys/socket.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <Python.h>


static PyObject * send_command(PyObject *self, PyObject *args){
    
}
int main(){


    int fhi2c = open("/dev/i2c-1",O_RDWR);
    return 0;
}
#endif
