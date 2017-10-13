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

#include "xitem.h"

namespace simulator {
namespace xwd {

std::unordered_map<std::string, cv::Mat> XItem::item_imgs_;

XItemPtr XItem::create_item(const Entity& e) {
    if (e.type == "agent") {
        return std::make_shared<XAgent>(e);
    } else {
        return std::make_shared<XItem>(e);
    }
}

cv::Mat XItem::get_item_image() {
    auto img_name = e_.asset_path;
    if (!img_name.empty()) {
        if (item_imgs_.count(img_name) == 0) {
            cv::Mat img;
            img = cv::imread(img_name, 1);
            CHECK(!img.empty()) << "could not open or find the image: " + img_name;
            cv::resize(img,
                       img,
                       cv::Size(XItem::item_size_, XItem::item_size_),
                       cv::INTER_LINEAR);
            item_imgs_[img_name] = img;
        }
        return item_imgs_[img_name];
    }
    return cv::Mat();
}

}
}  // namespace simulator::xwd
