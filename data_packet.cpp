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
        for (size_t i = 0; i < buffer->get_value_size(); i++) {
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

void StateBuffer::init_str() { str_ = std::make_shared<std::string>(); }

void StateBuffer::sync_value_ptr() {
    if (reals_) {
        CHECK_GT(reals_->size(), 0);
        // we always have floating values in state buffers
        CHECK(std::type_index(typeid(*(reals_->data()))) ==
              std::type_index(typeid(float)));
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
