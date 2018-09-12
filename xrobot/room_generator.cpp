#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <queue>
#include <random>

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
public: 
    MapGenerator(std::mt19937 mt, int H, int W, int num_rooms, int max_size, float rho) :
            mt_(mt), H_(H), W_(W), num_rooms_(num_rooms), max_size_(max_size), rho_(rho) {
        map_ = _create_2d_matrix<int>(H_, W_);
    }

    ~MapGenerator() {
        _release_2d_matrix(map_, H_);
    }

    void generate() {
        int r = _generate_random(H_);
        int c = _generate_random(W_);
        map_[r][c] = -1;
        for (int i = 0; i < num_rooms_; ++i) {
            _generate_a_room(i+1);
        }
    }

    void print_map() {
        for (int r = 0; r < H_; ++r) {
            for (int c = 0; c < W_; ++c) {
                printf("%d ", std::max(0, map_[r][c]));
            }
            printf("\n");
        }
    }

    int** get_map() const
    {
        return map_;
    }

private:
    void _generate_a_room(const int room_id) {
        std::vector<Loc> candidates;
        for (int r = 0; r < H_; ++r) {
            for (int c = 0; c < W_; ++c) {
                if (map_[r][c] == -1) {
                    candidates.emplace_back(r, c);
                }
            }
        }

        int p = _generate_random(candidates.size());
        std::queue<Loc> Q;
        Q.push(candidates[p]);

        bool **visited = _create_2d_matrix<bool>(H_, W_);
        visited[candidates[p].r][candidates[p].c] = true;

        int room_size = 0;
        //std::cout << room_id << std::endl;
        while (!Q.empty() && room_size < max_size_) {
            ++room_size;
            Loc l = Q.front();
            Q.pop();
            map_[l.r][l.c] = room_id;
            //std::cout << l.r << " " << l.c << std::endl;
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
                if (map_[r][c] > 0) {
                    for (int i = 0; i < 4; ++i) {
                        if (_available(r+dr[i], c+dc[i])) {
                            map_[r+dr[i]][c+dc[i]] = -1;
                        }
                    }
                }
            }
        }
        _release_2d_matrix(visited, H_);       
    }

    int _generate_random(const int high) {
        std::uniform_real_distribution<float> dist(0, high);
        return (int) dist(mt_);
    }

    bool _available(int r, int c) {
        return (r >= 0 && r < H_ && c >= 0 && c < W_ && map_[r][c] <= 0);

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

    std::mt19937 mt_;
    int H_;
    int W_;
    int num_rooms_;
    int max_size_;
    float rho_;
    int **map_;

    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};
};

// int main() {
//     MapGenerator mg(10, 10, 5, 8, 0.6);
//     mg.generate();
//     mg.print_map();

//     return 0;
// }
