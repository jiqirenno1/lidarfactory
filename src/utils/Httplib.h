//
// Created by ubuntu on 2021/3/15.
//

#ifndef LIDARFACTORY_HTTPLIB_H
#define LIDARFACTORY_HTTPLIB_H
#include <opencv2/opencv.hpp>
#include "rapidjson/writer.h"
#include "CodeBase64.h"
#include <memory>
#include "httplib.h"

class Httplib {
public:
    Httplib() = default;
    ~Httplib() = default;
    void Send(std::string im, std::string host, int port, std::string path, float pitch_precision, float yaw_precision, float xoffset,
                      float yoffset);
private:

};


#endif //LIDARFACTORY_HTTPLIB_H
