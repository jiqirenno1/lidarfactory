## 基于 robosense lidar 的


#### what
- 基于飞特舵机SMBL45和robosense-lidar-16的厂区态势感知
```
lidarfactory
│   README.md
│
└───config  ---配置文件
│   
└───rs_driver ---雷达驱动
│   
└───STSservo ---舵机驱动
│
└───src
│   │   manager.h ---- 雷达，舵机建图的流程控制
│   │   ProcessPointClouds.h ---- 点云处理集合
│   │
│   └───apps---功能主函数
│   │    │   main1.cpp ----建图
│   │    │   calibrate.cpp ---- 标定
│   │    │   crush3D.cpp ---- 碰撞预警
│   │    │   test1.cpp ---- 功能测试
│   │    │   ...
│   │
│   └───sdf---建图
│   │    │   ...
│   │
│   └───utils
│       │   CodeBase64.h ---- base64编码
│       │   Httplib.h ------ http发送
│       │   ...
```
#### how