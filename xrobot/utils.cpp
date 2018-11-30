#include "utils.h"

void startBenchmark() {
    benchmark_timer = std::clock();
}

void endBenchmark() {
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

void padTo(std::string &str, const size_t num, const char paddingChar) {
    if(num > str.size()) str.insert(0, num - str.size(), paddingChar);
}

int GetJsonArrayEntry(Json::Value *&result,
                      Json::Value *array,
                      unsigned int k,
                      int expected_type) {
    // Check array type
    if (array->type() != Json::arrayValue) {
        fprintf(stderr, "JSON: not an array\n");
        return 0;
    }
  
    // Check array size
    if (array->size() <= k) {
        fprintf(stderr, "JSON array has no member %d\n", k);
        return 0;
    }
  
    // Get entry
    result = &((*array)[k]);
    if (result->type() == Json::nullValue) {
        fprintf(stderr, "JSON array has null member %d\n", k);
        return 0;
    }
  
    // Check entry type
    if (expected_type > 0) {
        if (result->type() != expected_type) {
            fprintf(stderr,
                    "JSON array entry %d has unexpected type %d"
                    " (rather than %d)\n",
                    k,
                    result->type(),
                    expected_type);
            return 0;
        }
    }
    
    // Return success
    return 1;
}

int GetJsonObjectMember(Json::Value *&result,
                        Json::Value *object,
                        const char *str,
                        int expected_type) {
    // Check object type
    if (object->type() != Json::objectValue) {
        fprintf(stderr, "JSON: not an object\n");
        return 0;
    }
  
    // Check object member
    if (!object->isMember(str)) {
        fprintf(stderr, "JSON object has no member named %s\n", str);
        return 0;
    }
  
    // Get object member
    result = &((*object)[str]);
    if (result->type() == Json::nullValue) {
        fprintf(stderr, "JSON object has null member named %s\n", str);
        return 0;
    }
  
    // Check member type
    if (expected_type > 0) {
        if (result->type() != expected_type) {
            fprintf(stderr,
                    "JSON object member %s has unexpected type %d" 
                    " (rather than %d)\n",
                    str,
                    result->type(),
                    expected_type);
            return 0;
        }
    }
    
    // Check for empty strings
    if (result->type() == Json::stringValue) {
        if (result->asString().length() == 0) {
            fprintf(stderr,
                    "JSON object has zero length string named %s\n",
                    str);
            return 0;
        }
    }
  
    // Return success
    return 1;
}

