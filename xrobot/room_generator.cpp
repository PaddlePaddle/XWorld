#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

struct Loc {
    int r;
    int c;

    Loc() : r(0), c(0) {}
    
    Loc(int rr, int cc) : r(rr), c(cc) {}

    Loc(const Loc &l) {
        r = l.r;
        c = l.c;
    }

    Loc& operator=(const Loc &l) {
        r = l.r;
        c = l.c;
        return *this;
    }
    
    bool operator==(const Loc &l) {
        return (r == l.r) && (c == l.c);
    }
};

class MapGenerator {
public: MapGenerator(int H,
                     int W,
                     int num_rooms,
                     int num_doors,
                     int max_size,
                     float rho) : H_(H),
                                  W_(W),
                                  num_rooms_(num_rooms),
                                  num_doors_(num_doors),
                                  max_size_(max_size),
                                  rho_(rho) { 
        M_ = _create_2d_matrix<int>(H_, W_);
        std::srand(std::time(NULL));
    }

    ~MapGenerator() {
        _release_2d_matrix(M_, H_);
    }

    void generate() {
        int r = rand() % H_;
        int c = rand() % W_;
        M_[r][c] = -1;
        for (int i = 0; i < num_rooms_; ++i) {
            _generate_a_room(i+1);
        }

        _generate_locked_doors();

        //print_map();

        _merge_rooms();
    }

    void print_map() {
        for (int r = 0; r < H_; ++r) {
            for (int c = 0; c < W_; ++c) {
                printf("%d ", std::max(0, M_[r][c]));
            }
            printf("\n");
        }
        for (auto& e : edges_) {
            printf("%d %d\n", e.first, e.second);
        }
    }

private:
    void _generate_a_room(const int room_id) {
        std::vector<Loc> candidates;
        for (int r = 0; r < H_; ++r) {
            for (int c = 0; c < W_; ++c) {
                if (M_[r][c] == -1) {
                    candidates.emplace_back(r, c);
                }
            }
        }

        int p = rand() % candidates.size();
        std::queue<Loc> Q;
        Q.push(candidates[p]);

        bool **visited = _create_2d_matrix<bool>(H_, W_);
        visited[candidates[p].r][candidates[p].c] = true;

        int room_size = 0;
        while (!Q.empty() && room_size < max_size_) {
            ++room_size;
            Loc l = Q.front();
            Q.pop();
            M_[l.r][l.c] = room_id;
            for (int i = 0; i < 4; ++i) {
                int r = l.r + dr[i];
                int c = l.c + dc[i];
                bool proceed = ((double)rand()/(RAND_MAX)) <= rho_;
                if (proceed && _available(r, c) && !visited[r][c]) {
                    Q.emplace(r, c);
                    visited[r][c] = true;
                }
            }
        }

        for (int r = 0; r < H_; ++r) {
            for (int c = 0; c < W_; ++c) {
                if (M_[r][c] > 0) {
                    for (int i = 0; i < 4; ++i) {
                        if (_available(r+dr[i], c+dc[i])) {
                            M_[r+dr[i]][c+dc[i]] = -1;
                        }
                    }
                }
            }
        }
        _release_2d_matrix(visited, H_);       
    }

    void _generate_locked_doors() {
        for (int r = 0; r < H_; ++r) {
            for (int c = 0; c < W_; ++c) {
                if (M_[r][c] <= 0) {
                    continue;
                }
                for (int k = 0; k < 4; ++k) {
                    int rr = r + dr[k];
                    int cc = c + dc[k];
                    if (rr >= 0 && rr < H_ && cc >= 0 && cc < W_ && 
                        M_[rr][cc] > 0 && M_[rr][cc] != M_[r][c]) {
                        adj_list_[M_[r][c]].insert(M_[rr][cc]);
                        adj_list_[M_[rr][cc]].insert(M_[r][c]);
                    }
                }
            }
        }
        std::unordered_set<int> candidates;
        int prev = 1;
        selected_.insert(prev);
        room_orders_.push_back(prev);
        for (int i = 0; i < num_doors_; ++i) {
            for (auto q : adj_list_[prev]) {
                if (selected_.find(q) == selected_.end()) {
                    candidates.insert(q);
                }
            }
            int t = rand() % candidates.size();
            std::vector<int> pool;
            auto it = candidates.begin();
            while (t > 0) {
                ++it;
                t--;
            }
            prev = *it;
            candidates.erase(prev);
            for (auto p : selected_) {
                if (adj_list_[p].find(prev) != adj_list_[p].end()) {
                    pool.push_back(p);
                }
            }
            std::random_shuffle(pool.begin(), pool.end());  
            edges_.emplace_back(pool[0], prev);
            selected_.insert(prev);
            room_orders_.push_back(prev);
        }

    }

    void _merge_rooms() {
        std::vector<int> counts;
        for (auto r : selected_) {
            counts.push_back(1);
            pa_[r] = counts.size();
        }
        while (selected_.size() < num_rooms_) {
            for (int i = 1; i <= num_rooms_; ++i) {
                if (selected_.find(i) == selected_.end()) {
                    int p = -1;
                    for (auto v : adj_list_[i]) {
                        if (selected_.find(v) != selected_.end() && 
                            (p < 0 || counts[pa_[v]-1] < counts[p-1])) {
                            p = pa_[v];
                        }
                    }
                    if (p > 0) {
                        selected_.insert(i);
                        pa_[i] = p;
                        counts[p-1]++;
                    } 
                }
            }
        }
        // std::cout << std::endl;
        // for (auto& p : pa_) {
        //     std::cout << p.first << " " << p.second << std::endl;
        // }
    }

    bool _available(int r, int c) {
        return (r >= 0 && r < H_ && c >= 0 && c < W_ && M_[r][c] <= 0);

    }

    template<typename T>
    T** _create_2d_matrix(int H, int W) {
        T** t = new T* [H];
        for (int i = 0; i < H; ++i) {
            t[i] = new T [W];
            memset(t[i], 0, sizeof(T)*W);
        }
        return t;
    }

    template<typename T>
    void _release_2d_matrix(T** t, int H) {
        for (int i = 0; i < H; ++i) {
            delete [] t[i];
        }
        delete [] t;
    }

public:
    int H_;
    int W_;
    int num_rooms_;
    int num_doors_;
    int max_size_;
    float rho_;
    int **M_;
    std::unordered_set<int> selected_;
    std::vector<int> room_orders_;
    std::vector<std::pair<int,int>> edges_;
    std::unordered_map<int, std::unordered_set<int>> adj_list_;
    std::unordered_map<int,int> pa_;

    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};
};

// int main() {
//     MapGenerator mg(10, 10, 9, 3, 10, 0.8);
//     mg.generate();

//     return 0;
// }
