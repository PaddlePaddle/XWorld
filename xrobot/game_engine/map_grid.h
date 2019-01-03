#ifndef MAP_GRID_H_
#define MAP_GRID_H_

#include "map.h"
#include "room_generator.cpp"

namespace xrobot {

constexpr float kUniformScale = 2.0f;
constexpr int   kNSubTile = 5;
constexpr float kTileSize = 4.0f;
constexpr float kPadding = 0.002f;

struct Tile;

struct BBox {
    BBox() : min_aabb(glm::vec2(0)), max_aabb(glm::vec2(0)) {}
    BBox(const glm::vec2 bbox_min, const glm::vec2 bbox_max) :
            min_aabb(bbox_min), max_aabb(bbox_max) {}

    BBox Offset(const glm::vec2& offset) const {
        BBox ret;
        ret.min_aabb = min_aabb + offset;
        ret.max_aabb = max_aabb + offset;
        return ret;
    }

    BBox Union(const BBox& other) const {
        BBox ret;
        ret.min_aabb = glm::min(min_aabb, other.min_aabb);
        ret.max_aabb = glm::max(max_aabb, other.max_aabb); 
        return ret;
    }

    bool Intersect(const BBox& other) const {
        return (min_aabb.x <= other.max_aabb.x && max_aabb.x >= other.min_aabb.x) &&
               (min_aabb.y <= other.max_aabb.y && max_aabb.y >= other.min_aabb.y);
    }

    bool Contains(const BBox& other) const {
        return (min_aabb.x <= other.min_aabb.x && max_aabb.x >= other.max_aabb.x) &&
               (min_aabb.y <= other.min_aabb.y && max_aabb.y >= other.max_aabb.y);
    }

    glm::vec2 Center() const {
        return (min_aabb + max_aabb) * 0.5f;
    }

    float BoundingSphereRadius() const {
        return glm::length(max_aabb - Center());
    }

    glm::vec2 min_aabb;
    glm::vec2 max_aabb;
};

struct SubTile {
    SubTile() : occupied(false), visited(false) {}

    glm::vec2 center;
    bool visited;
    bool occupied;
    std::weak_ptr<Tile> parent;
};

typedef std::shared_ptr<SubTile> SubTileSPtr;

struct Tile : public std::enable_shared_from_this<Tile> {		
    Tile(const std::string& urdf_path_str, const glm::vec2& center_vec2) :
            name("tile"),
            urdf_path(urdf_path_str),
            center(center_vec2), 
            has_wall{false},
            occupied(false), 
            subtiles(kNSubTile * kNSubTile),
            room_id(-1),
            roomgroup_id(-1),
            position(std::make_pair(0,0)) {}

    void ClearVisited() {
        for (auto& subtile : subtiles) {
            if(subtile) subtile->visited = false;
        }
    }

    void UpdateTile() {}

    void UpdateSubTiles() {
        for (int i = 0; i < (kNSubTile * kNSubTile); ++i) {
            subtiles[i] = std::make_shared<SubTile>();
            subtiles[i]->center = GetSubTileCenter(i);
            subtiles[i]->parent = shared_from_this();
        }
    }

    BBox GetTileBBox() {
        glm::vec2 bbox_min = center - glm::vec2(0.5f * kTileSize);
        glm::vec2 bbox_max = center + glm::vec2(0.5f * kTileSize);
        return BBox(bbox_min, bbox_max);
    }

    BBox GetSubTileBBox(const unsigned int id) {
        float subtile_size = kTileSize / kNSubTile;
        glm::vec2 sub_center = GetSubTileCenter(id);
        glm::vec2 bbox_min = sub_center - glm::vec2(0.5f * subtile_size);
        glm::vec2 bbox_max = sub_center + glm::vec2(0.5f * subtile_size);
        return BBox(bbox_min, bbox_max);
    }

    SubTileSPtr GetSubTile(const int x, const int y) {
        int id = x * kNSubTile + y;
        return subtiles[id];
    }

    glm::vec2 GetSubTileRelativePosition(const unsigned int id) {
        return glm::vec2(id / kNSubTile,
                    id % kNSubTile);
    }

    glm::vec2 GetSubTileCenter(const unsigned int id) {
        float subtile_size = kTileSize / kNSubTile;
        glm::vec2 rel_position = GetSubTileRelativePosition(id);
        return center - glm::vec2(0.5f * kTileSize) + glm::vec2(subtile_size * 0.5f) +
            rel_position * subtile_size;
    }

    int FindSubTile(const SubTileSPtr& subtile) {
        int res = -1;
        auto it = std::find(subtiles.begin(), subtiles.end(), subtile);
        if(it != subtiles.end()) {
            res = (int) (it - subtiles.begin());
        }
        return res;
    }

    int room_id;
    int roomgroup_id;
    glm::vec2 center;
    bool occupied;
    bool has_wall[4]; // +x -x +y -y
    std::pair<int,int> position;
    std::string name;
    std::string urdf_path;
    std::vector<SubTileSPtr> subtiles;
};

typedef std::shared_ptr<Tile> TileSPtr;

struct FileTag {
    std::string file_path;
    std::string tag;
};

struct Pile {
    std::vector<int> object_id_list;
    std::vector<glm::vec3> positions;
    std::vector<BBox> upper_bbox_list;
    BBox bbox;
};

struct Room {
    int visit_sequence;
    std::vector<TileSPtr> tiles;
};

struct RoomGroup {
    std::unordered_set<int> rooms;
    std::vector<SubTileSPtr> waypoints;
};

struct Door {
    std::pair<int,int> edge;
    glm::vec2 center;
    int door_id;
};

class MapGrid : public Map {
public:
    MapGrid();
    ~MapGrid();

    // return the room which agent located
    glm::vec3 GenerateLayout(const int w, const int l, const int n, const int d);
    void GenerateArena(const int w, const int l);
    void GenerateObjects();

    void SpawnSingleObjectAt(const std::string& path, const glm::vec3& position);
    bool SpawnSingleObject(const std::string& path, const int roomgroup_id = -1);
    bool SpawnStackOfObjects(
            const std::string& top_path, 
            const std::string& bottom_path, 
            const int num = 1, 
            const int roomgroup_id = -1);
    bool SpawnPairOfObjects(
            const std::string& left_path, 
            const std::string& right_path, 
            const int roomgroup_id = -1);
    void SpawnSingleObjectAt(const int obj_id, const glm::vec3& position);
    bool SpawnSingleObject(const int obj_id, const int roomgroup_id = -1);
    bool SpawnStackOfObjects(
            const int top_id,
            const int bottom_id, 
            const int num = 1,
            const int roomgroup_id = -1);
    bool SpawnPairOfObjects(
            const int left_id,
            const int right_id, 
            const int roomgroup_id = -1);

    void CreateAndLoadLockedDoorJSON(const std::string& json_path);
    void CreateAndLoadTileURDF(const std::string& urdf_path);
    void LoadWallURDF(const std::string& urdf_path);
    void LoadUnlockedDoorJSON(const std::string& json_path);
    void CreateAndLoadObjectFILE(
            const std::string& file_path, const std::string& tag);
    void CreateAndLoadKeyURDF(
            const std::string& urdf_path, const std::string& tag);

    void GenerateWall(const glm::vec3& position, const int d);
    void GenerateKey(const glm::vec3& position, const int key_id);
    void GenerateTile(const glm::vec3& position, const int tile_id);
    void GenerateLockedDoor(const glm::vec3& position, const int d, const int door_id);
    void GenerateUnlockedDoor(const glm::vec3& position, const int d);
    void GenerateEmpty(const BBox& box);
    
    void UpdateSceneBBox();
    bool IntersectWithBBox(const BBox& box);

    void ClearVisited();
    void ResolvePath();
    std::vector<TileSPtr> GetTileNeighbors(const TileSPtr& tile);
    std::vector<SubTileSPtr> GetSubTileNeighbors(const SubTileSPtr& subtile);
    SubTileSPtr GetSubTileFromWorldPosition(const glm::vec2& position);

    glm::vec2 GetASpaceNearPosition(const glm::vec2& position, const float radius);

    void ResetMap();

    std::string wall_urdf_path_;
    std::string unlocked_door_path_;
    std::vector<std::string> locked_door_path_list_;
    std::vector<std::string> tile_urdf_list_;
    std::vector<FileTag> key_path_list_;
    std::vector<FileTag> object_path_list_;
    std::vector<BBox> object_bbox_list_;
    std::vector<glm::vec2> object_height_list_;

    std::vector<Pile> pile_list_;
    std::vector<BBox> bbox_list_;
    BBox map_bounding_;

    std::vector<Room> rooms_;
    std::vector<RoomGroup> roomgroups_;
    std::map<std::pair<int,int>, TileSPtr> tiles_;

    bool agent_spawn_;
    bool resolve_path_;
    glm::vec2 agent_spawn_position_;

    std::shared_ptr<MapGenerator> map_;
    std::vector<Door> doors_list_;
    std::map<std::pair<int,int>, glm::vec3> doors_map_;

    std::random_device rand_device_;
    std::mt19937 mt_;

    int FindObject(const std::string& path);
    void UpdateDebugVisualization();
    float GetRandom(const float low, const float high);
    inline float VanDerCorpus(unsigned int bits);
    inline glm::vec2 Hammersley(const unsigned int i, const unsigned int N);
};
}

#endif 
