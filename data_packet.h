// Copyright (c) 2017 Baidu Inc. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <glog/logging.h>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <vector>

#include "memory_util.h"

namespace simulator {

// A struct for switching between double, float and uint8 buffers
struct Value {
    Value() : d_val_(nullptr), f_val_(nullptr), c_val_(nullptr) {}
    // we always return double regardless of the buffer type
    double get_value(int i) {
        if (d_val_) {
            CHECK(!f_val_);
            CHECK(!c_val_);
            return d_val_[i];
        } else if (f_val_) {
            CHECK(!c_val_);
            return f_val_[i];
        }
        CHECK(c_val_);
        return c_val_[i];
    }
    // we always accept double value regardless of the buffer type
    // there will be an implicit type conversion in the assignment
    void set_value(int i, double v) {
        if (d_val_) {
            CHECK(!c_val_);
            CHECK(!f_val_);
            d_val_[i] = v;
        } else if (f_val_) {
            CHECK(!c_val_);
            f_val_[i] = v;
        } else {
            CHECK(c_val_);
            c_val_[i] = v;
        }
    }

    bool is_uint8() { return c_val_; }
    double* d_val_;
    float* f_val_;
    uint8_t* c_val_;
};

typedef std::shared_ptr<Value> ValuePtr;

// Different databuffer might have different value types
class DataBuffer {
public:
    DataBuffer() : v_(std::make_shared<Value>()) {}
    virtual ~DataBuffer() {}

    virtual void copy_from(
        std::shared_ptr<DataBuffer> buffer) = 0;  // deep copy
    virtual size_t get_value_height() = 0;        // height of value
    virtual size_t get_value_width() = 0;         // width of value
    virtual size_t get_id_size() = 0;             // size of id
    virtual int* get_id() = 0;                    // pointer to id
    virtual std::string* get_str() = 0;           // pointer to string

    virtual void encode(util::BinaryBuffer& buf) const {
        LOG(FATAL) << "NOT IMPLEMENTED";
    }

    virtual void decode(util::BinaryBuffer& buf) {
        LOG(FATAL) << "NOT IMPLEMENTED";
    }

    ValuePtr get_value();  // pointer to value buffer

    size_t get_value_size() { return get_value_height() * get_value_width(); }

    // Get the value for a buffer
    // Because the buffer might have float, double, or uint8_t value,
    // the return type needs to be correctly specified when calling this
    // function
    template <typename T>
    T* get_value() {
        sync_value_ptr();  // set v_
        if (std::is_same<T, double>::value) {
            CHECK(v_->d_val_) << "wrong return type double";
            return (T*)v_->d_val_;
        } else if (std::is_same<T, float>::value) {
            CHECK(v_->f_val_) << "wrong return type float";
            return (T*)v_->f_val_;
        } else if (std::is_same<T, uint8_t>::value) {
            CHECK(v_->c_val_) << "wrong return type uint8_t";
            return (T*)v_->c_val_;
        }
        LOG(FATAL) << "incorrect return type";
        return nullptr;
    }

    virtual void sync_value_ptr() = 0;

    template <typename RandomIt>
    void copy_value(RandomIt first, RandomIt last) {
        CHECK_LE(last - first, get_value_width() * get_value_height());
        auto data = get_value();
        int i = 0;
        while (first != last) {
            *(first++) = data->get_value(i++);
        }
    }
    template <typename RandomIt>
    void copy_id(RandomIt first, RandomIt last) {
        CHECK_LE(last - first, get_id_size());
        int* data = get_id();
        while (first != last) {
            *(first++) = *(data++);
        }
    }

    // deep copy
    // when the iterator type is uint8_t*, we copy the values to pixels_;
    // otherwise we copy the values to reals_
    template <typename RandomIt>
    void set_value(RandomIt first, RandomIt last, int height = 1) {
        CHECK_EQ((last - first) % height, 0);
        init_value((last - first) / height,
                   height,
                   std::type_index(typeid(*first)) ==
                       std::type_index(typeid(uint8_t)));
        auto data = get_value();
        int i = 0;
        while (first != last) {
            data->set_value(i++, *(first++));
        }
    }
    // deep copy
    template <typename RandomIt>
    void set_id(RandomIt first, RandomIt last) {
        init_id(last - first);
        int* data = get_id();
        while (first != last) {
            *(data++) = *(first++);
        }
    }

    void set_str(const std::string& str) {
        init_str();
        auto str_ptr = get_str();
        *str_ptr = str;
    }

protected:
    // init value to 0
    // if pixels=true, the value buffer will be init as uint8_t type
    // otherwise it will be float type
    virtual void init_value(size_t w, size_t h, bool pixels) = 0;
    virtual void init_id(size_t sz) = 0;  // init id to 0
    virtual void init_str() = 0;
    ValuePtr v_;
};

typedef std::shared_ptr<DataBuffer> BufferPtr;

/**
 * The DataPacket is a generic class that stores a packet of data,
 * using (std::string->BufferPtr) as the underlying data structure.
 * Its content is a vector of buffers, where the buffer type is a template type
 * By specifying the buffer type, DataPacket can be turned into SimulatorState,
 * RobotData, etc
 */
template <class T>
class DataPacket {
public:
    DataPacket() {}

    // deep copy when T != Y
    template <class Y>
    DataPacket(const DataPacket<Y>& src) {
        copy_from(src);
    }

    // shallow copy for the same class
    DataPacket(const DataPacket& src) { data_ = src.data_; }

    /**
     * initialize by a list of data keys
     */
    DataPacket(const std::vector<std::string>& keys) {
        for (const auto& k : keys) {
            add_key(k);
        }
    }

    /**
     * Initialize by a list of data keys and data buffers
     */
    DataPacket(const std::vector<std::string>& keys,
               const std::vector<BufferPtr>& buffers) {
        CHECK_EQ(keys.size(), buffers.size());
        for (size_t i = 0; i < keys.size(); i++) {
            auto buffer = std::make_shared<T>();
            buffer->copy_from(buffers[i]);
            add_buffer(keys[i], buffer);
        }
    }

    /*
     * find out if the data contains a certain key
     */
    bool contain_key(std::string k) const {
        return data_.find(k) != data_.end();
    }

    /**
     * Get the buffer of key k
     */
    BufferPtr get_buffer(std::string k) const {
        CHECK(contain_key(k)) << k;
        return data_.at(k);
    }

    // Deep copy; k should exist
    void set_buffer(std::string k, BufferPtr buffer) {
        CHECK(contain_key(k)) << k;
        data_[k]->copy_from(buffer);
    }

    /**
     * Deep copy a buffer to key k; k does not exist
     **/
    void add_buffer(std::string k, BufferPtr buffer) {
        add_key(k);
        set_buffer(k, buffer);
    }

    template <typename Y>
    void add_buffer_value(std::string k,
                          const std::vector<Y>& buf,
                          int height = 1) {
        auto buffer = std::make_shared<T>();
        buffer->set_value(buf.begin(), buf.end(), height);
        add_buffer(k, buffer);
    }

    void add_buffer_id(std::string k, const std::vector<int>& buf) {
        auto buffer = std::make_shared<T>();
        buffer->set_id(buf.begin(), buf.end());
        add_buffer(k, buffer);
    }

    void add_buffer_str(std::string k, const std::string& str) {
        auto buffer = std::make_shared<T>();
        buffer->set_str(str);
        add_buffer(k, buffer);
    }

    /**
     * Add a key and init the default buffer
     **/
    void add_key(std::string k) {
        CHECK(!contain_key(k)) << k;
        data_[k] = std::make_shared<T>();
    }

    void remove_buffer(std::string k) {
        CHECK(contain_key(k)) << k;
        data_.erase(k);
    }

    size_t size() const { return data_.size(); }

    std::vector<std::string> get_keys() const {
        std::vector<std::string> keys;
        for (const auto& m : data_) {
            keys.push_back(m.first);
        }
        return keys;
    }

    /**
       Deep copy the src key k to the key k
    */
    template <class Y>
    void copy_from_by_key(DataPacket<Y>& src, std::string k) {
        CHECK(src.contain_key(k)) << k;
        CHECK(contain_key(k));
        data_[k]->copy_from(src.data_[k]);
    }

    /**
     * Copy from another data (deep)
     */
    template <class Y>
    void copy_from(const DataPacket<Y>& src) {
        data_.clear();
        for (const auto& k : src.get_keys()) {
            add_buffer(k, src.get_buffer(k));
        }
    }

    void encode(util::BinaryBuffer& buf) const {
        buf.append((std::size_t)data_.size());
        for (auto& kv : data_) {
            buf.append(kv.first);
            kv.second->encode(buf);
        }
    }

    void decode(util::BinaryBuffer& buf) {
        std::size_t sz;
        buf.read(sz);
        for (size_t i = 0; i < sz; ++i) {
            std::string key;
            BufferPtr t = std::make_shared<T>();
            buf.read(key);
            t->decode(buf);
            data_[key] = t;
        }
    }

private:
    std::unordered_map<std::string, BufferPtr> data_;
};

/**
 * A generic buffer class for representing simulator state at a time step
 * It contains three fields:
 * 1. reals_  -> storing floating numbers
 * 2. pixels_ -> storing unsigned chars
 * 2. id_     -> storing int numbers
 * 3. str_    -> storing string
 **/
class StateBuffer : public DataBuffer {
public:
    StateBuffer() :
            reals_(nullptr), pixels_(nullptr), id_(nullptr), str_(nullptr),
            BIT_REALS(1), BIT_PIXELS(2), BIT_ID(4), BIT_STR(8) {}
    void copy_from(std::shared_ptr<DataBuffer> buffer) override;  // deep copy
    void init_value(size_t w, size_t h, bool pixels) override;
    void init_id(size_t sz) override;
    void init_str() override;
    int* get_id() override;
    std::string* get_str() override;
    size_t get_value_width() override;
    size_t get_value_height() override;
    size_t get_id_size() override;
    bool operator==(const StateBuffer& other); // for ctest only
    void encode(util::BinaryBuffer& buf) const override;
    void decode(util::BinaryBuffer& buf) override;

private:
    void sync_value_ptr() override;
    template <typename T>
    bool pointer_compare(const std::shared_ptr<T>& a,
                         const std::shared_ptr<T>& b);
    // shallow copies by default
    std::shared_ptr<std::vector<float>> reals_;
    std::shared_ptr<std::vector<uint8_t>> pixels_;
    std::shared_ptr<std::vector<int>> id_;
    std::shared_ptr<std::string> str_;
    const uint8_t BIT_REALS;
    const uint8_t BIT_PIXELS;
    const uint8_t BIT_ID;
    const uint8_t BIT_STR;
};

typedef DataPacket<StateBuffer> StatePacket;

}  // namespace simulator
