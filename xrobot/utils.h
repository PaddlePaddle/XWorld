#ifndef UTILS_H_
#define UTILS_H_

#include <stdlib.h>
#include <sys/stat.h>

#include <ctime>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

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

#endif // UTILS_H_