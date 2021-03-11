//
// Created by ubuntu on 2021/2/25.
//

#include "MySMSCL.h"

void MySMSCL::init(std::string &port, int baudRate) {
    if(!sm_.begin(baudRate, port.c_str()))
    {
        std::cout<<"Failed to init smscl motor!"<<std::endl;
    }
    sm_.WritePosEx(1, 2048, 1000, 10);
    usleep(2270*1000);
    sm_.FeedBack(1);
    std::cout<< " init motor:"<<1<<" pos ="<<sm_.ReadPos(1)<<std::endl;
}

void MySMSCL::start() {
    s16 pos0 = 2048;
    s16 step = 200;
    std::cout<< " start motor! "<<std::endl;
    while(1) {

        s16 pos1 = pos0 + step;
        if (pos1 >= 2248 || pos1 <= 1848)
        {
            step = -step;
        }

        sm_.WritePosEx(1, pos1, 25, 5);
        usleep(8200*1000);//[(P1-P0)/V]*1000+(V/A)*10
        pos0=pos1;
    }

}

int MySMSCL::getPos() {
    int pos = 0;
    if(sm_.FeedBack(1)!=-1)
    {
        pos = sm_.ReadPos(1);
        usleep(10*1000);
    }
    return pos;
}

MySMSCL::~MySMSCL() {

}

void MySMSCL::stop() {
    sm_.WritePosEx(1, 2048, 1000, 10);
    usleep(2270*1000);
    sm_.end();
}
