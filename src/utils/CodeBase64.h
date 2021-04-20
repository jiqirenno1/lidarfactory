//
// Created by ubuntu on 2021/3/12.
//

#ifndef LIDARFACTORY_CODEBASE64_H
#define LIDARFACTORY_CODEBASE64_H

#include <iostream>
class CodeBase64 {
public:
    CodeBase64()=default;
    ~CodeBase64()=default;
    const std::string base64_chars =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz"
            "0123456789+/";
    inline bool is_base64(const char c)
    {
        return (isalnum(c) || (c == '+') || (c == '/'));
    }
    std::string base64_encode(const char * bytes_to_encode, unsigned int in_len);
    std::string base64_decode(std::string const & encoded_string);

private:

};


#endif //LIDARFACTORY_CODEBASE64_H
