#include "irobot.h"

int irobot_init(int* fd){

}
int irobot_setspeed(int* fd, double dvdt, double drdt){
	double tv = dvdt;
	double rv = drdt;
/*--------- START COPYnPASTE*/
  int16_t tv_mm, rad_mm;

  //printf("tv: %.3lf rv: %.3lf\n", tv, rv);

  tv_mm = (int16_t)rint(tv * 1e3);
  tv_mm = MAX(tv_mm, -CREATE_TVEL_MAX_MM_S);
  tv_mm = MIN(tv_mm, CREATE_TVEL_MAX_MM_S);

  if(rv == 0)
  {
    // Special case: drive straight
    rad_mm = 0x8000;
  }
  else if(tv == 0)
  {
    // Special cases: turn in place
    if(rv > 0)
      rad_mm = 1;
    else
      rad_mm = -1;

    tv_mm = (int16_t)rint(CREATE_AXLE_LENGTH * fabs(rv) * 1e3);
  }
  else
  {
    // General case: convert rv to turn radius
    rad_mm = (int16_t)rint(tv_mm / rv);
    // The robot seems to turn very slowly with the above
    //rad_mm /= 2;
    //printf("real rad_mm: %d\n", rad_mm);
    rad_mm = MAX(rad_mm, -CREATE_RADIUS_MAX_MM);
    rad_mm = MIN(rad_mm, CREATE_RADIUS_MAX_MM);
  }
		uint8_t vel_h = (tv_mm & 0xFF00) >> 8;
		uint8_t vel_l = (tv_mm & 0x00FF);
		uint8_t deg_h = (rad_mm & 0xFF00) >> 8;
		uint8_t deg_l = (rad_mm & 0x00FF);
/*------------------*/
        serialport_writebyte(*fd,137);
        serialport_writebyte(*fd,vel_h);
        serialport_writebyte(*fd,vel_l);
        serialport_writebyte(*fd,deg_h);
        serialport_writebyte(*fd,deg_l);




}

int irobot_responding(int* fd) {

}