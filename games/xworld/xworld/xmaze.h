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

#pragma once
#include <vector>
#include <list>
#include <utility>
#include <unordered_set>
#include "xitem.h"

namespace simulator { namespace xwd {

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

    void modify_reachable(int x, int y, const std::unordered_set<Loc>& defined_blocks);

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

    void bfs(int y, int x, std::vector<Loc>& trace,
             const std::unordered_set<Loc>& defined_blocks) const;

    const int height_;
    const int width_;
    std::vector<std::vector<bool>> maze_;
    std::vector<std::vector<bool>> largest_comp_;
};

}} // namespace simulator::xwd
