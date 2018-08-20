#include "utils.h"

void startBenchmark()
{
    benchmark_timer = std::clock();
}

void endBenchmark()
{
    double duration = (std::clock() - benchmark_timer) / (double) CLOCKS_PER_SEC;
    std::cout << "Duration: " << duration << std::endl;
}

std::string read_file(const std::string& fn) {
    FILE* f = fopen(fn.c_str(), "rt");
    if (!f) {
        throw std::runtime_error("cannot open '" + fn + "' with mode 'rt': "
                                 + std::strerror(errno)); }
    off_t ret = fseek(f, 0, SEEK_END);
    if (ret == (off_t) - 1) {
        fclose(f);
        throw std::runtime_error("cannot stat '" + fn + "': " + std::strerror(errno));
    }
    uint32_t file_size = (uint32_t) ftell(f); // ftell returns long int
    fseek(f, 0, SEEK_SET);
    std::string str;
    if (file_size == 0) {
        fclose(f);
        return str;
    }
    str.resize(file_size);
    try {
        int r = (int) fread(&str[0], file_size, 1, f);
        if (r==0) {
            throw std::runtime_error("cannot read from '" + fn + "', eof");
        }
        if (r!=1) {
            throw std::runtime_error("cannot read from '" + fn + "': "
                                     + std::strerror(errno));
        }
        fclose(f);
    } catch (...) {
        fclose(f);
        throw;
    }
    return str;
}

void split(const std::string& s, char delim, std::vector<std::string>& v) {
    size_t i = 0;
    auto pos = s.find(delim);
    while (pos != std::string::npos) {
        v.push_back(s.substr(i, pos-i));
        i = ++pos;
        pos = s.find(delim, pos);
    }

    if (i < s.length()) {
        v.push_back(s.substr(i, s.length()-i));
    }
}

void cuda_visible_devices(std::vector<int>& device_ids) {
    char* tmp = getenv("CUDA_VISIBLE_DEVICES");
    device_ids.clear();
    if (tmp != NULL) {
        std::string device_str(tmp);
        std::vector<std::string> tokens;
        split(device_str, ',', tokens);
        for (auto const& s : tokens) {
            device_ids.push_back(std::stoi(s));
        }
    }
}

void padTo(std::string &str, const size_t num, const char paddingChar)
{
    if(num > str.size()) str.insert(0, num - str.size(), paddingChar);
}
