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

#include <cstring>
#include <glog/logging.h>
#include <string>
#include <type_traits>
#include <vector>

namespace simulator {
namespace util {

typedef unsigned char uchar;

class BinaryBuffer {
public:
    ~BinaryBuffer();

    explicit BinaryBuffer(size_t size = 0);
    BinaryBuffer(size_t size, size_t capacity);
    BinaryBuffer(const void* data, size_t size);
    BinaryBuffer(const BinaryBuffer&);

    BinaryBuffer& operator=(const BinaryBuffer&);

    void assign(const void* data, size_t size);

    void take_over(void* data, size_t size);

    void append(const BinaryBuffer&);

    template <typename T>
    void append(const T t);

    void append(const std::string& str);

    template <typename T>
    void append(const std::vector<T>& vec);

    template <typename T>
    void append(const T* data, int num_elements);

    void start_reading();

    template<typename T>
    void read(T& t);

    void read(std::string& str);

    template <typename T>
    void read(std::vector<T>& vec);

    template<typename T>
    void read(T* t, int num_elements);

    size_t size () const;

    bool resize (size_t);

    size_t capacity () const;

    bool reserve (size_t);

    size_t offset() const;

    bool empty () const;

    bool eof() const;

    void clear ();

    uchar* data_mutable ();

    const uchar* data () const;

private:
    void delete_if_own() {
        if (own_) {
            delete [] data_;
        }
    }

    uchar* data_;
    size_t size_;
    size_t capacity_;
    bool own_;
    uchar* read_ptr_;
};

inline BinaryBuffer::~BinaryBuffer() {
    delete_if_own();
}

inline BinaryBuffer::BinaryBuffer(size_t size) {
    data_ = (size != 0 ? new uchar[size] : NULL);
    size_ = capacity_ = size;
    own_ = true;
}

inline BinaryBuffer::BinaryBuffer(size_t size, size_t capacity) {
    if (size > capacity) {
        size = capacity;
    }

    data_ = (capacity != 0 ? new uchar[capacity] : NULL);
    size_ = size;
    capacity_ = capacity;
    own_ = true;
}

inline BinaryBuffer::BinaryBuffer(const void* data, size_t size) {
    if (size != 0) {
        data_ = new uchar[size];
        std::memcpy(data_, data, size);
    } else {
        data_ = NULL;
    }
    size_ = capacity_ = size;
    own_ = true;
}

inline BinaryBuffer::BinaryBuffer(const BinaryBuffer& other) {
    if (other.capacity_) {
        data_ = new uchar[other.capacity_];
        if (other.size_) {
            std::memcpy(data_, other.data_, other.size_);
        }
    } else {
        data_ = NULL;
    }
    size_ = other.size_;
    capacity_ = other.capacity_;
    own_ = true;
}

inline BinaryBuffer& BinaryBuffer::operator=(const BinaryBuffer& other) {
    if (&other != this) {
        if (other.size_ > capacity_) {
            delete_if_own();
            data_ = new uchar[other.capacity_];
            capacity_ = other.capacity_;
            own_ = true;
        }
        std::memcpy(data_, other.data_, other.size_);
        size_ = other.size_;
    }
    return *this;
}

inline void BinaryBuffer::assign(const void* data, size_t size) {
    if (size > capacity_) {
        delete_if_own();
        data_ = new uchar[size];
        capacity_ = size;
        own_ = true;
    }
    std::memcpy(data_, data, size);
    size_ = size;
}

inline void BinaryBuffer::take_over(void* data, size_t size) {
    delete_if_own();
    data_ = static_cast<uchar*>(data);
    size_ = capacity_ = size;
    own_ = true;
}

inline void BinaryBuffer::append(const BinaryBuffer& other) {
    append(other.data_, other.size_);
}

template <typename T>
inline void BinaryBuffer::append(const T t) {
   append(&t, 1);
}

inline void BinaryBuffer::append(const std::string& str) {
    append((std::size_t)str.length());
    append(str.c_str(), str.length()+1);
}

template <typename T>
inline void BinaryBuffer::append(const std::vector<T>& vec) {
    append((std::size_t)vec.size());
    append(vec.data(), vec.size());
}

template <typename T>
inline void BinaryBuffer::append(const T* data, int num_elements) {
    if (num_elements == 0) {
        return;
    }

    size_t total_size = sizeof(T) * num_elements;
    reserve(total_size + size_);
    std::memcpy(data_ + size_, (uchar*)data, total_size);
    size_ = size_ + total_size;
}

inline void BinaryBuffer::start_reading() {
    read_ptr_ = data_;
}

template <typename T>
inline void BinaryBuffer::read(T& t) {
    read(&t, 1);
}

inline void BinaryBuffer::read(std::string& str) {
    std::size_t len;
    read(len);
    char tmp[len+1];
    read(tmp, len+1);
    str.assign(tmp);
}

template <typename T>
inline void BinaryBuffer::read(std::vector<T>& vec) {
    std::size_t size;
    read(size);
    vec.resize(size);
    read(vec.data(), size);
}

template <typename T>
inline void BinaryBuffer::read(T* t, int num_elements) {
    if (num_elements == 0) {
        return;
    }

    size_t total_size = sizeof(T) * num_elements;
    CHECK_LE(read_ptr_ - data_ + total_size, size_);
    std::memcpy(t, read_ptr_, total_size);
    read_ptr_ += total_size;
}

inline size_t BinaryBuffer::size() const {
    return size_;
}

inline bool BinaryBuffer::resize(size_t size) {
    bool capacity_changed = reserve(size);
    size_ = size;

    return capacity_changed;
}

inline size_t BinaryBuffer::capacity() const {
    return capacity_;
}

inline bool BinaryBuffer::reserve(size_t capacity) {
    if (capacity <= capacity_) {
        return false;
    }

    if (capacity_ == 0) {
        capacity_ = 1;
    }
    while (capacity_ < capacity) {
        capacity_ = capacity_ << 1;
    }

    uchar* d = new uchar[capacity_];
    std::memcpy(d, data_, size_);
    delete_if_own();
    data_ = d;
    own_ = true;

    return true;
}

inline bool BinaryBuffer::empty() const {
    return size_;
}

inline bool BinaryBuffer::eof() const {
    return (read_ptr_ - data_ == size_);
}

inline void BinaryBuffer::clear() {
    size_ = 0;
}

inline size_t BinaryBuffer::offset() const {
    return read_ptr_ - data_;
}

inline uchar* BinaryBuffer::data_mutable() {
    return data_;
}

inline const uchar* BinaryBuffer::data() const {
    return data_;
}

}} // simulator::util
