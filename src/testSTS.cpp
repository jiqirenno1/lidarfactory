#include <iostream>
#include "SMSCL.h"

SMSCL sm;



int main(int argc, char **argv)
{
//    if(argc<2){
//        std::cout<< "argc error!"<<std::endl;
//        return 0;
//    }
    std::string path = "/dev/ttyUSB0";
    std::cout<< "serial:"<<path<<std::endl;
    if(!sm.begin(115200, path.c_str())){
        std::cout<< "Failed to init smscl motor!"<<std::endl;
        return 0;
    }
//    sm.CalibrationOfs(1);
//    sm.FeedBack(1);
//    while(1){
//        sm.WritePosEx(1, 4095, 2250, 50);//舵机(ID1)以最高速度V=2250步/秒，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
//        std::cout<< "pos ="<<4095<<std::endl;
//        usleep(2270*1000);//[(P1-P0)/V]*1000+(V/A)*10
//
//        sm.WritePosEx(1, 0, 2250, 50);//舵机(ID1)以最高速度V=2250步/秒，加速度A=50(50*100步/秒^2)，运行至P0=0位置
//        std::cout<< "pos ="<<0<<std::endl;
//        usleep(2270*1000);//[(P1-P0)/V]*1000+(V/A)*10
//    }

    sm.WritePosEx(1, 2048, 1000, 10);
    usleep(2270*1000);
    sm.FeedBack(1);
    std::cout<< " init pos ="<<sm.ReadPos(-1)<<std::endl;

    s16 pos0 = 2048;
    s16 step = 20;
    while(1) {

        s16 pos1 = pos0 + step;
        if (pos1 >= 2348 || pos1 <= 2048)
        {
            step = -step;
        }
        sm.WritePosEx(1, pos1, 200, 3);
        usleep(700*1000);
        if(sm.FeedBack(1)!=-1){
            std::cout<< "pos"<<pos1<<" ="<<sm.ReadPos(-1)<<std::endl;
        }
        pos0=pos1;
    }

    while(0){

//        sm.CarationOfs(1);
        sm.WritePosEx(1, 1800, 1000, 10);
        usleep(2270*1000);
        sm.FeedBack(1);
        std::cout<< "pos1800 ="<<sm.ReadPos(-1)<<std::endl;
        sm.WritePosEx(1, 2048, 1000, 10);
        usleep(2270*1000);
        sm.FeedBack(1);
        std::cout<< "pos2048 ="<<sm.ReadPos(-1)<<std::endl;
        sm.WritePosEx(1, 2500, 1000, 10);
        usleep(2270*1000);
        sm.FeedBack(1);
        std::cout<< "pos2500 ="<<sm.ReadPos(-1)<<std::endl;
//        sm.WriteSpe(1, 2400, 100);//舵机(ID1)以最高速度V=2400步/秒，加速度A=100(100*100步/秒^2)，旋转
//        std::cout<< "speed="<<2400<<std::endl;
//        sleep(2);
//        sm.WriteSpe(1, 0, 100);//舵机(ID1)以加速度A=100(100*100步/秒^2)，停止旋转(V=0)
//        std::cout<< "speed="<<0<<std::endl;
//        sleep(2);
//        sm.WriteSpe(1, -2400, 100);//舵机(ID1)以最高速度V=-2400步/秒，加速度A=100(100*100步/秒^2)，反向旋转
//        std::cout<< "speed="<<-2400<<std::endl;
//        sleep(2);
//        sm.WriteSpe(1, 0, 100);//舵机(ID1)以加速度A=100(100*100步/秒^2)，停止旋转(V=0)
//        std::cout<< "speed="<<0<<std::endl;
//        sleep(2);

        int Pos;
        int Speed;
        int Load;
        int Voltage;
        int Temper;
        int Move;
        int Current;
        if(sm.FeedBack(1)!=-1){
            Pos = sm.ReadPos(-1);
            Speed = sm.ReadSpeed(-1);
            Load = sm.ReadLoad(-1);
            Voltage = sm.ReadVoltage(-1);
            Temper = sm.ReadTemper(-1);
            Move = sm.ReadMove(-1);
            Current = sm.ReadCurrent(-1);
            std::cout<< "pos ="<<Pos<<std::endl;
            std::cout<< "Speed ="<<Speed<<std::endl;
            std::cout<< "Load ="<<Load<<std::endl;
            std::cout<< "Voltage ="<<Voltage<<std::endl;
            std::cout<< "Temper ="<<Temper<<std::endl;
            std::cout<< "Move ="<<Move<<std::endl;
            std::cout<< "Current ="<<Current<<std::endl;
            usleep(10*1000);
        }else{
            std::cout<< "read err ="<<std::endl;
            sleep(2);
        }
    }
    sm.end();
    return 1;
}
