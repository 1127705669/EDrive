
#include <iostream>

#include "ros/ros.h"
#include <unistd.h>

int main(int argc, char *argv[])
{
    execlp("roslaunch", "roslaunch", "EROS","EROS.launch", NULL);
    std::cout << "EDrive" << std::endl;
    return 0;
}

