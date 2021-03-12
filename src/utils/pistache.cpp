//
// Created by ubuntu on 2021/3/12.
//

#include "pistache.h"
using namespace Pistache;
using namespace Pistache::Http;

void pistache::pistacheSend(std::string im, std::string page, float pitch_precision, float yaw_precision, float xoffset,
                            float yoffset) {
    Http::Client client;
    auto opts = Http::Client::options().threads(1).maxConnectionsPerHost(8);
    client.init(opts);
    std::vector<Async::Promise<Http::Response>> responses;
    auto start = std::chrono::system_clock::now();

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
    CodeBase64 *cb64 = new CodeBase64();
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

    auto resp = client.post(page).body(json).send();

    resp.then(
            [&](Http::Response response) {
                std::cout << "Response code = " << response.code() << std::endl;
                auto body = response.body();
                if (!body.empty())
                    std::cout << "Response body = " << body << std::endl;
            },
            [&](std::exception_ptr exc) {
                PrintException excPrinter;
                excPrinter(exc);
            });
    responses.push_back(std::move(resp));

    auto sync = Async::whenAll(responses.begin(), responses.end());
    Async::Barrier<std::vector<Http::Response>> barrier(sync);
    barrier.wait_for(std::chrono::seconds(3));

    auto end = std::chrono::system_clock::now();
    std::cout<<"Total time of send: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count()<<"ms"<<std::endl;
    client.shutdown();


}
