//
// Created by ubuntu on 2021/2/25.
//

#ifndef LIDARFACTORY_SM45_H
#define LIDARFACTORY_SM45_H

#include <iostream>
#include "SMSBL.h"

class SM45 {
public:
    SM45()=default;
    ~SM45();
    void init(std::string& port, int baudRate);
    void start();
    int getPos();
    void stop();
private:
    SMSBL sm_;

};


#endif //LIDARFACTORY_SM45_H
