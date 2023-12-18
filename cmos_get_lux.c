#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>

#define BUF_SIZE 256

int main(int argc, char *argv[]){
    int fdin;
    char *path = "/sys/bus/iio/devices/iio:device0/in_voltage2_raw";
    char buf[BUF_SIZE];
    int raw;
    float lux;
    bool preState = true;
    bool state = true;

    while(1) {
        fdin = open(path, O_RDONLY);
        if(fdin == -1){
            printf("open() error \n");
            exit(1);
        }


        read(fdin, buf, sizeof(buf));

        //printf("%s\n", buf);
        close(fdin);

        raw = atoi(buf);
        lux = (-0.08f * (float)raw) + 81.96f;
        //printf("lux = %02f\n", lux);

        if (lux < 10) {
            state = false;
        }
        else {
            state = true;
        } 

        if (state != preState) {
            if (state == true) {
                system("v4l2-ctl -d /dev/v4l-subdev5 --set-ctrl 'band_stop_filter=1'");
                system("v4l2-ctl -d /dev/v4l-subdev6 --set-ctrl 'band_stop_filter=1'");
            }
            else {
                system("v4l2-ctl -d /dev/v4l-subdev5 --set-ctrl 'band_stop_filter=0'");
                system("v4l2-ctl -d /dev/v4l-subdev6 --set-ctrl 'band_stop_filter=0'");
            }
            preState = state;
        }
        sleep(1);
    }

        return 0;
}