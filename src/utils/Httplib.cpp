//
// Created by ubuntu on 2021/3/15.
//

#include "Httplib.h"
void Httplib::Send(std::string im, std::string host, int port, std::string path, float pitch_precision,
                   float yaw_precision, float xoffset, float yoffset) {

    auto start = std::chrono::system_clock::now();
    httplib::Client cli(host, port);


    //base64
    std::fstream f;
    f.open(im, std::ios::in|std::ios::binary);
    f.seekg(0, std::ios_base::end);
    std::streampos sp = f.tellg();
    int size = sp;
    std::cout << size << std::endl;
    char* buffer = (char*)malloc(sizeof(char)*size);
    f.seekg(0, std::ios_base::beg);//把文件指针移到到文件头位置
    f.read(buffer,size);
    std::cout << "file size:" << size << std::endl;
    std::shared_ptr<CodeBase64> cb64 = std::make_shared<CodeBase64>();
    std::string imgBase64 = cb64->base64_encode(buffer, size);

    //json
    rapidjson::StringBuffer strBuf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(strBuf);
    writer.StartObject();
    writer.Key("pitch_precision");
    writer.Double(pitch_precision);
    writer.Key("yaw_precision");
    writer.Double(yaw_precision);
    writer.Key("xoffset");
    writer.Double(xoffset);
    writer.Key("yoffset");
    writer.Double(yoffset);
    writer.Key("img");
    writer.String(imgBase64.c_str());
    writer.EndObject();
    std::string json = strBuf.GetString();

    auto res = cli.Post(path.c_str(), json, "application/json");

    auto end = std::chrono::system_clock::now();
    std::cout<<"Total time of send: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<"ms"<<std::endl;
}
