// MIT License

// Copyright (c) 2017 Baidu Inc. All rights reserved.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

#include "data_packet.h"

namespace simulator {

ValuePtr DataBuffer::get_value() {
    // update the value pointer before every read and write
    sync_value_ptr();
    return v_;
}

void StateBuffer::copy_from(BufferPtr buffer) {
    reals_ = nullptr;
    pixels_ = nullptr;
    id_ = nullptr;
    str_ = nullptr;
    // copy value
    if (buffer->get_value_size()) {
        auto data = buffer->get_value();
        init_value(buffer->get_value_size(), 1, data->is_uint8());
        auto dst_data = get_value();
        for (size_t i = 0; i < buffer->get_value_size(); i ++) {
            dst_data->set_value(i, data->get_value(i));
        }
    }
    // copy id
    if (buffer->get_id_size()) {
        int* id = buffer->get_id();
        set_id(id, id + buffer->get_id_size());
    }
    // copy string
    if (buffer->get_str()) {
        set_str(*buffer->get_str());
    }
}

void StateBuffer::init_value(size_t w, size_t h, bool pixels) {
    CHECK_EQ(h, 1UL) << "StateBuffer value always has unit height";
    if (pixels) {
        pixels_ = std::make_shared<std::vector<uint8_t>>(w, 0);
    } else {
        reals_ = std::make_shared<std::vector<float>>(w, 0);
    }
}

void StateBuffer::init_id(size_t sz) {
    id_ = std::make_shared<std::vector<int>>(sz, 0);
}

void StateBuffer::init_str() {
    str_ = std::make_shared<std::string>();
}

void StateBuffer::sync_value_ptr() {
    if (reals_) {
        CHECK_GT(reals_->size(), 0);
        // we always have floating values in state buffers
        CHECK(std::type_index(typeid(*(reals_->data())))
              == std::type_index(typeid(float)));
        v_->f_val_ = reals_->data();
    } else if (pixels_) {
        CHECK_GT(pixels_->size(), 0);
        v_->c_val_ = pixels_->data();
    }
    // else do nothing
}

int* StateBuffer::get_id() {
    if (id_) {
        return id_->data();
    } else {
        return 0;
    }
}

std::string* StateBuffer::get_str() {
    if (str_) {
        return str_.get();
    } else {
        return 0;
    }
}

size_t StateBuffer::get_value_width() {
    if (reals_) {
        return reals_->size();
    } else if (pixels_) {
        return pixels_->size();
    } else {
        return 0;
    }
}

size_t StateBuffer::get_value_height() {
    // state buffer always have unit value height
    if (reals_ || pixels_) {
        return 1;
    } else {
        return 0;
    }
}

size_t StateBuffer::get_id_size() {
    if (id_) {
        return id_->size();
    } else {
        return 0;
    }
}

}
