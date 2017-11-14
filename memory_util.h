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

/**
 * A simple archive interface that can perform serialization/deserialization on
 * basic c++ primitive types, string and vector.
 */
class BinaryBuffer {
public:
    //// Constructors
    /**
     * Create buffer by specifying size and capacity. BinaryBuffer owns the
     * buffer.
     */
    explicit BinaryBuffer(size_t size = 0, size_t capacity = 0);
    /**
     * Create buffer with an initial array by making a copy of it. BinaryBuffer
     * owns the buffer.
     *
     * @param[in]   data         the initial array
     * @param[in]   num_bytes    number of bytes to copy
     */
    explicit BinaryBuffer(const void* data, size_t num_bytes);
    /**
     * Copy constructor. BinaryBuffer owns the buffer.
     */
    explicit BinaryBuffer(const BinaryBuffer&);

    /**
     * Release the buffer if BinaryBuffer owns it.
     */
    ~BinaryBuffer();

    //// copy
    /**
     * Overload the assignment operation. The capacity will increase if
     * necessary, and in that case, BinaryBuffer owns the underlying data.
     */
    BinaryBuffer& operator=(const BinaryBuffer&);
    /**
     * Set buffer by copying specified number of bytes from an array.
     *
     * @param[in]  data         array to copy from
     * @param[in]  num_bytes    number of bytes to copy
     */
    void assign(const void* data, size_t num_bytes);
    /**
     * Release the previous owned buffer, and take over the ownership of the
     * data and use it as new buffer. After calling this function, it is
     * BinaryBuffer's responsibility to manage the data.
     *
     * @param[in]   data    data to take over
     * @param[in]   size    the size (in bytes) of the data
     */
    void take_over(void* data, size_t size);

    //// write/encode into buffer
    /**
     * Copy buffer from another BinaryBuffer to the end of buffer.
     */
    void append(const BinaryBuffer&);
    /**
     * Append data of type T to the end of buffer.
     */
    template <typename T>
    void append(const T t);
    /**
     * Append string to the end of buffer.
     */
    void append(const std::string& str);
    /**
     * Append vector to the end of buffer.
     */
    template <typename T>
    void append(const std::vector<T>& vec);
    /**
     * Append an array of specified number of elements to the end of buffer.
     * The total number of bytes added to buffer is:
     * num_elements * sizeof(T)
     *
     * @param[in]   data            array to append
     * @param[in]   num_elements    number of elements to append
     */
    template <typename T>
    void append(const T* data, int num_elements);
    /**
     * Insert data of type T to a specific location in buffer.
     *
     * @param[in]   p   insert location (in terms of bytes)
     * @param[in]   t   data to insert
     */
    template <typename T>
    inline void insert(size_t p, const T t);
    /**
     * Insert an array of specified number of elements to a specific location in
     * buffer.
     *
     * @param[in]   p               insert location (in terms of bytes)
     * @param[in]   data            array to insert
     * @param[in]   num_elements    number of elements to insert
     */
    template <typename T>
    inline void insert(size_t p, const T* data, int num_elements);

    //// read/decode from buffer
    /**
     * Move the read pointer to the beginning of buffer.
     */
    void rewind();
    /**
     * Get the offset of read pointer.
     */
    size_t offset() const;
    /**
     * Return true if read pointer reaches the end of buffer.
     */
    bool eof() const;
    /**
     * Get a value of data type T from buffer.
     *
     * @param[out]  t   the variable to store the value.
     */
    template<typename T>
    void read(T& t);
    /**
     * Get a string from buffer.
     *
     * @param[out]  str     the variable to store the string
     */
    void read(std::string& str);
    /**
     * Get a vector of type T from buffer.
     *
     * @param[out]  vec     the vector to store the data of type T
     */
    template <typename T>
    void read(std::vector<T>& vec);
    /**
     * Get specified number of data of type T from buffer.
     *
     * @param[out]  t               the vector to store the data of type T
     * @param[in]   num_elements    number of data to read
     */
    template<typename T>
    void read(T* t, int num_elements);

    //// capacity
    /**
     * Get the size (in terms of bytes) of buffer.
     */
    size_t size() const;
    /**
     * Resize the buffer. If the new size is larger than current capacity,
     * the function causes the buffer to reallocate its storage increasing its
     * capacity to at least the new size.
     */
    bool resize(size_t);
    /**
     * Get the capacity of buffer.
     */
    size_t capacity() const;
    /**
     * Request that the buffer be at least enough to contain a specified number
     * of bytes.
     */
    bool reserve(size_t);
    /**
     * Clear the buffer by setting size to zero. The data will not be removed.
     */
    void clear();
    /**
     * Return true if buffer is empty.
     */
    bool empty() const;

    //// buffer access
    /**
     * Return the pointer to buffer.
     */
    uchar* data_mutable();
    /**
     * Return the const pointer to buffer.
     */
    const uchar* data() const;

private:
    /**
     * Release the buffer if owns it
     */
    void delete_if_own() {
        if (own_) {
            delete [] data_;
        }
    }

    uchar* data_;       // the underlying buffer
    size_t size_;       // number of bytes in buffer
    size_t capacity_;   // buffer capacity
    bool own_;          // whether this object owns its buffer
    uchar* read_ptr_;   // read pointer. A new read operation starts from
                        // this pointer.
};

inline BinaryBuffer::~BinaryBuffer() {
    delete_if_own();
}

inline BinaryBuffer::BinaryBuffer(size_t size, size_t capacity) {
    // make sure the size is no larger than the capacity
    if (size > capacity) {
        size = capacity;
    }

    data_ = (capacity != 0 ? new uchar[capacity] : NULL);
    size_ = size;
    capacity_ = capacity;
    own_ = true;
}

inline BinaryBuffer::BinaryBuffer(const void* data, size_t num_bytes) {
    if (num_bytes != 0) {
        data_ = new uchar[num_bytes];
        std::memcpy(data_, data, num_bytes);
    } else {
        data_ = NULL;
    }
    size_ = capacity_ = num_bytes;
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
            // we re-allocate the buffer, so we owns the buffer.
            own_ = true;
        }
        std::memcpy(data_, other.data_, other.size_);
        size_ = other.size_;
    }
    return *this;
}

inline void BinaryBuffer::assign(const void* data, size_t num_bytes) {
    if (num_bytes > capacity_) {
        delete_if_own();
        data_ = new uchar[num_bytes];
        capacity_ = num_bytes;
        // we re-allocate the buffer, so we owns the buffer.
        own_ = true;
    }
    std::memcpy(data_, data, num_bytes);
    size_ = num_bytes;
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
    append((size_t)str.length());
    append(str.c_str(), str.length()+1);
}

template <typename T>
inline void BinaryBuffer::append(const std::vector<T>& vec) {
    append((size_t)vec.size());
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
    size_ += + total_size;
}

template <typename T>
inline void BinaryBuffer::insert(size_t p, const T t) {
    insert(p, &t, 1);
}

template <typename T>
inline void BinaryBuffer::insert(size_t p, const T* data, int num_elements) {
    if (num_elements == 0) {
        return;
    }

    size_t total_size = sizeof(T) * num_elements;
    reserve(size_ + total_size);
    std::memmove(data_ + p + total_size, data_ + p, size_ - p);
    std::memcpy(data_ + p, (uchar*)data, total_size);
    size_ += + total_size;
}

inline void BinaryBuffer::rewind() {
    read_ptr_ = data_;
}

template <typename T>
inline void BinaryBuffer::read(T& t) {
    read(&t, 1);
}

inline void BinaryBuffer::read(std::string& str) {
    size_t len;
    read(len);
    char tmp[len+1];
    read(tmp, len+1);
    str.assign(tmp);
}

template <typename T>
inline void BinaryBuffer::read(std::vector<T>& vec) {
    size_t size;
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
        capacity_ = capacity;
    }
    // enlarge the capacity by a factor of 2 until enough
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
    return offset() == size_;
}

inline void BinaryBuffer::clear() {
    read_ptr_ = data_;
    size_ = 0;
}

inline size_t BinaryBuffer::offset() const {
    return size_t(read_ptr_ - data_);
}

inline uchar* BinaryBuffer::data_mutable() {
    return data_;
}

inline const uchar* BinaryBuffer::data() const {
    return data_;
}

}} // simulator::util
