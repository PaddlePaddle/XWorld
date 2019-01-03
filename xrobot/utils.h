#ifndef UTILS_H_
#define UTILS_H_

#include <ctime>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "vendor/json.h"

inline bool exists_file(const char* fn) {
    struct stat buffer;
    return (stat(fn, &buffer) == 0);
}

std::string read_file(const std::string& fn);

void split(const std::string& s, char delim, std::vector<std::string>& v);

void cuda_visible_devices(std::vector<int>& device_ids);

void pad_to(std::string &str, const size_t num, const char paddingChar = '0');

bool json_parse_text(const std::string& filename, Json::Value& root);

std::string json_get_string(
        Json::Value* root,
        const std::string& name,
        const std::string& default_value = "");

int json_get_int(
        Json::Value* root,
        const std::string& name,
        const int default_value = 0);

float json_get_float(
        Json::Value* root,
        const std::string& name,
        const float default_value = 0.0f);

bool json_get_bool(
        Json::Value* root,
        const std::string& name,
        const bool default_value = false);

bool json_get_object(
        Json::Value*& result,
        Json::Value* object,
        const char* str,
        int expected_type = -1);

bool json_get_array(
        Json::Value*& result,
        Json::Value* array,
        unsigned int k,
        int expected_type = -1);


template<typename T>
std::shared_ptr<T> wptr_to_sptr(
        const std::weak_ptr<T>& wptr, bool can_be_null = false) {
    std::shared_ptr<T> sptr = wptr.lock();
    assert(can_be_null || sptr);
    return sptr;
}

#endif // UTILS_H_
