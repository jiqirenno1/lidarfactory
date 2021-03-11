//
// Created by ubuntu on 2021/2/25.
//

#ifndef LIDARFACTORY_MYSMSCL_H
#define LIDARFACTORY_MYSMSCL_H

#include <iostream>
#include "SMSCL.h"

class MySMSCL {
public:
    MySMSCL()=default;
    ~MySMSCL();
    void init(std::string& port, int baudRate);
    void start();
    int getPos();
    void stop();
private:
    SMSCL sm_;

};


#endif //LIDARFACTORY_MYSMSCL_H
