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

void StateBuffer::init_str() {
    str_ = std::make_shared<std::string>();
}

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

bool StateBuffer::operator==(const StateBuffer& other) {
    if (!pointer_compare(reals_, other.reals_) ||
        !pointer_compare(pixels_, other.pixels_) ||
        !pointer_compare(id_, other.id_) ||
        !pointer_compare(str_, other.str_)) {
        return false;
    }
    return true;
}

void StateBuffer::encode(util::BinaryBuffer& buf) const {
    uint8_t flags = (reals_ ? BIT_REALS : 0) | (pixels_ ? BIT_PIXELS : 0) |
                    (id_ ? BIT_ID : 0) | (str_ ? BIT_STR : 0);
    buf.append(flags);
    if (reals_) {
        buf.append(*reals_);
    }
    if (pixels_) {
        buf.append(*pixels_);
    }
    if (id_) {
        buf.append(*id_);
    }
    if (str_) {
        buf.append(*str_);
    }
}

void StateBuffer::decode(util::BinaryBuffer& buf) {
    uint8_t flags = 0;
    buf.read(flags);
    if (flags & BIT_REALS) {
        reals_ = std::make_shared<std::vector<float>>();
        buf.read(*reals_);
    }
    if (flags & BIT_PIXELS) {
        pixels_ = std::make_shared<std::vector<uint8_t>>();
        buf.read(*pixels_);
    }
    if (flags & BIT_ID) {
        id_ = std::make_shared<std::vector<int>>();
        buf.read(*id_);
    }
    if (flags & BIT_STR) {
        str_ = std::make_shared<std::string>();
        buf.read(*str_);
    }
}

template <typename T>
bool StateBuffer::pointer_compare(const std::shared_ptr<T>& a,
                                  const std::shared_ptr<T>& b) {
    if ((a && !b) || (!a && b) || (a && b && (*a != *b))) {
        return false;
    }
    return true;
}
} // namespace simulator
