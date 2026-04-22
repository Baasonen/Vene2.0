#include "init.h"

#include "magnetometer.h"

int modulesInit()
{
    int initFailed = 0;

    if (magInit()) {initFailed = 1;}

    return initFailed;
}