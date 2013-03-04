#ifndef IROBOT_H
#define IROBOT_H

int irobot_init(int* fd);
int irobot_setspeed(double dvdt, double drdt);
int irobot_responding(int* fd);

#endif