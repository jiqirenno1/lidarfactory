//
// Created by ubuntu on 2021/3/12.
//

#ifndef LIDARFACTORY_PISTACHE_H
#define LIDARFACTORY_PISTACHE_H

#include <atomic>

#include <pistache/client.h>
#include <pistache/http.h>
#include <pistache/net.h>
#include <opencv2/opencv.hpp>
#include "rapidjson/writer.h"
#include "CodeBase64.h"

class pistache {
public:
    pistache()=default;
    ~pistache()=default;
    void pistacheSend(std::string im, std::string page, float pitch_precision, float yaw_precision, float xoffset,
                      float yoffset);
private:

};


#endif //LIDARFACTORY_PISTACHE_H
