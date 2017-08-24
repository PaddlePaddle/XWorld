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
#include <list>
#include <unordered_set>
#include <utility>
#include <vector>
#include "xitem.h"

namespace simulator {
namespace xwd {

class XMaze {
  public:
    XMaze(int height, int width) : height_(height), width_(width) {
        init();
        generate_maze();
    }

    void find_largest_component();

    // return the largest connected empty component
    void get_largest_comp(std::vector<Loc>& largest_comp) const;

    // return all the empty spaces
    void get_empty_space(std::vector<Loc>& empty) const;

    // return all the blocks
    void get_blocks(std::vector<Loc>& blocked) const;

    // set the number of blocks to n_blocks
    void modify_blocks(int n_blocks,
                       const std::unordered_set<Loc>& defined_block_locs,
                       const std::unordered_set<Loc>& defined_reachable_locs);

    void modify_reachable(int x,
                          int y,
                          const std::unordered_set<Loc>& defined_blocks);

    void print();

  private:
    static const std::vector<int> dx;
    static const std::vector<int> dy;

    static const int LARGEST_COMP = 0;
    static const int EMPTY = 1;
    static const int BLOCK = 2;

    void init();

    void generate_maze();

    void modify_block(int x, int y, bool add_block);

    void get_locations(std::vector<Loc>& locations, int type) const;

    int dfs(int id, int y, int x, std::vector<std::vector<int>>& visited) const;

    void bfs(int y,
             int x,
             std::vector<Loc>& trace,
             const std::unordered_set<Loc>& defined_blocks) const;

    const int height_;
    const int width_;
    std::vector<std::vector<bool>> maze_;
    std::vector<std::vector<bool>> largest_comp_;
};
}
}  // namespace simulator::xwd
