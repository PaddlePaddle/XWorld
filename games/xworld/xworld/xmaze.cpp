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

#include "xmaze.h"
#include <queue>
#include <iostream>
#include "simulator_util.h"

namespace simulator { namespace xwd {

const std::vector<int> XMaze::dx = {0, 1, -1, 0};
const std::vector<int> XMaze::dy = {1, 0, 0, -1};

void XMaze::init() {
    maze_.resize(height_);
    for (int y = 0; y < height_; y ++) {
        maze_[y].resize(width_);
        for (int x = 0; x < width_; x ++) {
            maze_[y][x] = false;
        }
    }
}

void XMaze::generate_maze() {
    std::list<Loc> drillers;
    drillers.push_back({width_/2, height_/2});

    while (drillers.size() > 0) {
        std::list<Loc>::iterator m;
        std::list<Loc>::iterator _m;
        std::list<Loc>::iterator temp;
        m = drillers.begin();
        _m = drillers.end();
        while (m != _m) {
            bool remove_driller = false;
            switch (util::get_rand_ind(4)) {
                case 0:
                    (*m).y -= 2;
                    if ((*m).y < 0 || maze_[(*m).y][(*m).x]) {
                        remove_driller = true;
                        break;
                    }
                    maze_[(*m).y+1][(*m).x] = true;
                    break;
                case 1:
                    (*m).y += 2;
                    if ((*m).y >= height_ || maze_[(*m).y][(*m).x]) {
                        remove_driller = true;
                        break;
                    }
                    maze_[(*m).y-1][(*m).x] = true;
                    break;
                case 2:
                    (*m).x -= 2;
                    if ((*m).x < 0 || maze_[(*m).y][(*m).x]) {
                        remove_driller = true;
                        break;
                    }
                    maze_[(*m).y][(*m).x+1] = true;
                    break;
                case 3:
                    (*m).x += 2;
                    if ((*m).x >= width_ || maze_[(*m).y][(*m).x]) {
                        remove_driller = true;
                        break;
                    }
                    maze_[(*m).y][(*m).x-1] = true;
                    break;
            }
            if (remove_driller) {
                m = drillers.erase(m);
            } else {
                drillers.push_back({(*m).x,(*m).y});
                drillers.push_back({(*m).x,(*m).y});
                maze_[(*m).y][(*m).x] = true;
                m ++;
            }
        }
    }
}

void XMaze::modify_block(int x, int y, bool add_block) {
    CHECK(x >= 0 && x < width_);
    CHECK(y >= 0 && y < height_);
    maze_[y][x] = !add_block;
}

void XMaze::modify_reachable(int x, int y, const std::unordered_set<Loc>& defined_blocks) {
    CHECK(x >= 0 && x < width_);
    CHECK(y >= 0 && y < height_);
    // if [x,y] is alreay connected to the largest component, do nothing
    if (largest_comp_[y][x])
        return;
    std::vector<Loc> trace;  // [x,y]
    bfs(y, x, trace, defined_blocks);

    for (const auto& t : trace) {
        largest_comp_[t.y][t.x] = true;
        maze_[t.y][t.x] = true;
    }
}

void XMaze::modify_blocks(int n_blocks,
                          const std::unordered_set<Loc>& defined_block_locs,
                          const std::unordered_set<Loc>& defined_reachable_locs) {

    for (const auto& l : defined_block_locs) {
        modify_block(l.x, l.y, true);
    }

    std::vector<Loc> blocked;
    std::vector<Loc> empty;

    // add/remove blocks to empty spaces if necessary
    get_blocks(blocked);
    get_empty_space(empty);
    CHECK_EQ(blocked.size()+empty.size(), height_*width_);
    util::random_shuffle(empty);
    util::random_shuffle(blocked);

    ///////////////////// make adjustment to the generated maze /////////////////
    // add more blocks
    int cnt = 0;
    for (int i = blocked.size(); i < n_blocks; i ++) {
        Loc empty_lot;
        while (true) {
            CHECK_LT(cnt, empty.size());
            empty_lot = empty[cnt];
            cnt ++;
            // skip predefined reachable slots
            if (defined_reachable_locs.find(empty_lot) == defined_reachable_locs.end())
                break;
        }
        modify_block(empty_lot.x, empty_lot.y, true);
    }
    cnt = 0;
    // remove redundant blocks
    for (int i = n_blocks; i < int(blocked.size()); i ++) {
        Loc block_lot;
        while (true) {
            CHECK_LT(cnt, blocked.size());
            block_lot = blocked[cnt];
            cnt ++;
            // skip predefined blocks
            if (defined_block_locs.find(block_lot) == defined_block_locs.end())
                break;
        }
        modify_block(block_lot.x, block_lot.y, false);
    }
}

int XMaze::dfs(int id, int y, int x, std::vector<std::vector<int>>& visited) const {
    visited[y][x] = id;

    int total = 0;
    int yy = 0;
    int xx = 0;
    for (int i = 0; i < 4; i ++) {
        yy = y + dy[i];
        xx = x + dx[i];
        if (yy >= 0 && xx >= 0
            && yy < height_ && xx < width_
            && visited[yy][xx] == -1
            && maze_[yy][xx]) {
            total += dfs(id, yy, xx, visited);
        }
    }

    return total + 1;
}

void XMaze::bfs(int y, int x, std::vector<Loc>& trace,
                const std::unordered_set<Loc>& defined_blocks) const {
    auto merge = [this] (int i, int j) {
        return i * width_ + j;
    };
    auto split = [this] (int q, int& i, int& j) {
        i = q / width_;
        j = q % width_;
    };

    std::vector<int> prev(height_*width_, -1);
    std::vector<bool> visited(height_*width_, false);
    std::queue<int> que;
    que.push(merge(y, x));
    visited[merge(y, x)] = true;

    // don't go thourgh predefined blocks
    for (const auto& b : defined_blocks) {
        visited[merge(b.y, b.x)] = true;
    }

    int i = 0;
    int j = 0;
    int ii = 0;
    int jj = 0;
    int cnt = -1;
    int next = 0;
    bool success = false;
    while (!que.empty()) {
        cnt = que.front();
        que.pop();
        split(cnt, i, j);
        // the nearest largest-comp element is found
        if (largest_comp_[i][j]) {
            success = true;
            break;
        }
        for (int k = 0; k < 4; k ++) {
            ii = i + dy[k];
            jj = j + dx[k];
            next = merge(ii, jj);
            if (ii>=0 && ii<height_
                && jj>=0 && jj<width_
                && !visited[next]) {
                visited[next] = true;
                que.push(next);
                prev[next] = cnt;
            }
        }
    }
    if (success) {
        // trace back
        while (cnt != -1) {
            split(cnt, i, j);
            trace.push_back({j, i});
            cnt = prev[cnt];
        }
    }
    // in case where there is no path to the largest component
    // we only modify the starting point
    else
        trace.push_back({x, y});
}

void XMaze::find_largest_component() {
    std::vector<std::vector<int>> visited;
    visited.resize(height_);
    for (int y = 0; y < height_; y ++) {
        visited[y].resize(width_);
        for (int x = 0; x < width_; x ++) {
            visited[y][x] = -1;
        }
    }

    int id = 0;
    int n = 0;
    int max_n = 0;
    int max_id = 0;
    for (int i = 0; i < height_; i ++) {
        for (int j = 0; j < width_; j ++) {
            if (visited[i][j]==-1 && maze_[i][j]) {
                n = dfs(id, i, j, visited);
                if (n > max_n) {
                    max_id = id;
                    max_n = n;
                }
                id ++;
            }
        }
    }
    // get the mask for the largest component
    largest_comp_ = maze_;
    for (int i = 0; i < height_; i ++) {
        for (int j = 0; j < width_; j ++) {
            largest_comp_[i][j] = (visited[i][j] == max_id);
        }
    }
}

void XMaze::get_locations(std::vector<Loc>& locations, int type) const {
    std::function<bool (int i, int j)> qualifier;
    switch (type) {
        case LARGEST_COMP:
            qualifier = [this] (int i, int j) { return largest_comp_[i][j]; }; break;
        case EMPTY:
            qualifier = [this] (int i, int j) { return maze_[i][j]; }; break;
        case BLOCK:
            qualifier = [this] (int i, int j) { return !maze_[i][j]; }; break;
        default:
            LOG(FATAL) << "unrecognized location type";
    }
    locations.clear();
    for (int i = 0; i < height_; i ++) {
        for (int j = 0; j < width_; j ++) {
            if (qualifier(i, j))
                locations.push_back({j, i});
        }
    }
}

void XMaze::get_largest_comp(std::vector<Loc>& largest_comp) const {
    CHECK_GT(largest_comp_.size(), 0);
    get_locations(largest_comp, LARGEST_COMP);
}

void XMaze::get_empty_space(std::vector<Loc>& empty) const {
    CHECK_GT(maze_.size(), 0);
    get_locations(empty, EMPTY);
}

void XMaze::get_blocks(std::vector<Loc>& blocked) const {
    CHECK_GT(maze_.size(), 0);
    get_locations(blocked, BLOCK);
}

void XMaze::print() {
    std::cout << std::endl;
    for (int i = 0; i < height_; i ++) {
        for (int j = 0; j < width_; j ++) {
            if (maze_[i][j])
                std::cout << ".";
            else
                std::cout << "#";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

}} // namespace simulator::xwd
