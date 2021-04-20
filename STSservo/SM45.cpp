//
// Created by ubuntu on 2021/2/25.
//

#include "SM45.h"
#define DEBUG

void SM45::init(std::string &port, int baudRate) {
    if(!sm_.begin(baudRate, port.c_str()))
    {
        std::cout<<"Failed to init smscl motor!"<<std::endl;
    }
    //sm_.CalibrationOfs(1); //set middle pos
    sm_.WritePosEx(1, 2048, 100, 50);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
    usleep(2100*1000);
    if(sm_.FeedBack(1)!=-1){
        std::cout<< " init motor:"<<1<<" pos ="<<sm_.ReadPos(-1)<<std::endl;
    }

}

void SM45::start() {
    s16 pos0 = 2048;
    s16 step = 200;
    s16 max = 2248;
    s16 min = 1848;
    std::cout<< " start motor! "<<std::endl;
    while(1) {
        s16 pos1 = pos0 + step;
#ifdef DEBUG
        std::cout<<"motor give: "<<pos1<<std::endl;
#endif
        if (pos1 >= max || pos1 <= min)
        {
            step = -step;
        }

        sm_.WritePosEx(1, pos1, 100, 50);
        usleep(2100*1000);

#ifdef DEBUG
        if(sm_.FeedBack(1)!=-1){
            std::cout<< "motor:"<<1<<" pos ="<<sm_.ReadPos(-1)<<std::endl;
        }
        else
        {
            std::cout<<"error"<<std::endl;
        }
#endif
        pos0=pos1;
    }

}

int SM45::getPos() {
    int pos = 0;
    if(sm_.FeedBack(1)!=-1)
    {
        pos = sm_.ReadPos(1);
        usleep(10*1000);
    }
    return pos;
}

SM45::~SM45() {

}

void SM45::stop() {
    sm_.WritePosEx(1, 2048, 10, 10);
    usleep(1000*1000);
    sm_.end();
}
