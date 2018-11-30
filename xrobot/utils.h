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

static std::clock_t benchmark_timer;

void startBenchmark();

void endBenchmark();

inline bool exists_file(const char* fn) {
    struct stat buffer;
    return (stat(fn, &buffer) == 0);
}

std::string read_file(const std::string& fn);

void split(const std::string& s, char delim, std::vector<std::string>& v);

void cuda_visible_devices(std::vector<int>& device_ids);

void padTo(std::string &str, const size_t num, const char paddingChar = '0');

int GetJsonArrayEntry(Json::Value *&result,
                      Json::Value *array,
                      unsigned int k,
                      int expected_type = -1);

int GetJsonObjectMember(Json::Value *&result,
                        Json::Value *object,
                        const char *str,
                        int expected_type = 0);

template<typename T>
std::shared_ptr<T> wptr_to_sptr(
        const std::weak_ptr<T>& wptr, bool can_be_null = false) {
    std::shared_ptr<T> sptr = wptr.lock();
    assert(can_be_null || sptr);
    return sptr;
}


#endif // UTILS_H_
