#include "../include/Planning.hpp"
#include "mpc_rbt_simulator/RobotConfig.hpp"

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "plan_path",
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        RCLCPP_INFO(get_logger(), "Služba plan_path pripravená.");
        
        
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);



        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Čakám na službu 'map'...");
        }

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

        auto future = map_client_->async_send_request(request,
            std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Odoslal som požiadavku na mapu.");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {

    auto response = future.get();
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Mapa úspešne načítaná: %d x %d",
            map_.info.width, map_.info.height);
    } else {
        RCLCPP_ERROR(get_logger(), "Nepodarilo sa načítať mapu.");
    }

    // ********
    // * Help *
    // ********
    /*
    auto response = future.get();
    if (response) {
        ...
    }
    */
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {

    RCLCPP_INFO(get_logger(), "Volanie služby plan_path prijaté.");

    // Naplníme dummy cestu medzi start a goal
    geometry_msgs::msg::PoseStamped start = request->start;
    geometry_msgs::msg::PoseStamped goal = request->goal;

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->get_clock()->now();

    // A* algoritmus a smoothing
    RCLCPP_WARN(get_logger(), "Dilate Map");
    dilateMap();
    RCLCPP_WARN(get_logger(), "AStar");
    aStar(request->start, request->goal);
    smoothPath();  // Ak je prázdna, nevadí

    // Skontroluj, či je cesta naplnená
    if (path_.poses.empty()) {
        RCLCPP_WARN(get_logger(), "Cesta je prázdna.");
    } else {
        RCLCPP_INFO(get_logger(), "Cesta obsahuje %ld bodov.", path_.poses.size());
    }

    // Uloženie naplánovanej cesty do odpovede
    response->plan = path_;
    
    // Publikovanie cesty
    path_pub_->publish(path_);

    // Uložíme aj do členskej pre publisher (do budúcna)
    path_ = path;
}

void PlanningNode::dilateMap() {
    // add code here

    // ********
    // * Help *
    // ********
    int width = map_.info.width;
    int height = map_.info.height;
    float resolution = map_.info.resolution;

    nav_msgs::msg::OccupancyGrid dilatedMap = map_;

    int r = (int) ((robot_config::HALF_DISTANCE_BETWEEN_WHEELS*1.5)/resolution);
    RCLCPP_WARN(get_logger(), "R: %d, W: %d, H: %d, res: %f, d=%f", r,width,height,resolution,(robot_config::HALF_DISTANCE_BETWEEN_WHEELS*1.5));


    for(auto i=r;i<width-r;++i){
        for(auto j=r;j<height-r;++j){
            //RCLCPP_WARN(get_logger(), "x: %d, y: %d", i,j);
            if(map_.data[i+j*width]){
                for(auto k=i-r;k<(i+r);++k){
                    for(auto l=j-r;l<(j+r);++l){
                    //RCLCPP_WARN(get_logger(), "x: %d, y: %d (dilated) r: %d", k,l,r);
                    dilatedMap.data[k+l*width] = 100;
                    }
                }
            }
        }
    }
    

    map_ = dilatedMap;
    
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    int width = map_.info.width;
    int height = map_.info.height;
    float resolution = map_.info.resolution;
    float origin_x = map_.info.origin.position.x;
    float origin_y = map_.info.origin.position.y;

    auto worldToMap = [&](float wx, float wy, int &mx, int &my) {
        mx = static_cast<int>((wx - origin_x) / resolution);
        my = static_cast<int>((wy - origin_y) / resolution);
    };

    int sx, sy, gx, gy;
    worldToMap(start.pose.position.x, start.pose.position.y, sx, sy);
    worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy);

    const int8_t OBSTACLE_THRESHOLD = 50;

    auto isFree = [&](int x, int y) {
        // Skontrolujeme, či je pozícia v platnom rozsahu
        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        int index = y * width + x;
        int8_t val = map_.data[index];
        return val >= 0 && val < OBSTACLE_THRESHOLD;
    };

    auto heuristic = [&](int x1, int y1, int x2, int y2) {
        return std::hypot(x1 - x2, y1 - y2);
    };

    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(width * height, false);

    auto startCell = std::make_shared<Cell>(sx, sy);
    startCell->g = 0;
    startCell->h = heuristic(sx, sy, gx, gy);
    startCell->f = startCell->g + startCell->h;
    openList.push_back(startCell);

    std::shared_ptr<Cell> goalCell = nullptr;

    std::array<std::pair<int, int>, 8> directions = {{
        {0, 3}, {3, 0}, {0, -3}, {-3, 0},
        {2, 2}, {-2, 2}, {2, -2}, {-2, -2}
    }};

    while (!openList.empty() && rclcpp::ok()) {
        auto currentIt = std::min_element(openList.begin(), openList.end(), [](auto a, auto b) {
            return a->f < b->f;
        });

        std::shared_ptr<Cell> current = *currentIt;
        openList.erase(currentIt);
        closedList[current->y * width + current->x] = true;

        if (current->x == gx && current->y == gy) {
            goalCell = current;
            break;
        }

        for (auto [dx, dy] : directions) {
            int nx = current->x + dx;
            int ny = current->y + dy;

            if (!isFree(nx, ny) || closedList[ny * width + nx]) continue;

            bool inOpen = false;
            for (auto &cell : openList) {
                if (cell->x == nx && cell->y == ny) {
                    inOpen = true;
                    break;
                }
            }

            if (!inOpen) {
                auto neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->g = current->g + heuristic(current->x, current->y, nx, ny);
                neighbor->h = heuristic(nx, ny, gx, gy);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                openList.push_back(neighbor);
            }
        }
    }

    path_.poses.clear();

    if (goalCell) {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        auto cell = goalCell;

        while (cell) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = cell->x * resolution + origin_x + resolution / 2.0;
            pose.pose.position.y = cell->y * resolution + origin_y + resolution / 2.0;
            pose.pose.orientation.w = 1.0;
            waypoints.push_back(pose);
            cell = cell->parent;
        }

        std::reverse(waypoints.begin(), waypoints.end());
        path_.header.frame_id = "map";
        path_.header.stamp = this->get_clock()->now();
        path_.poses = waypoints;

        RCLCPP_INFO(get_logger(), "Cesta naplánovaná cez %ld bodov.", path_.poses.size());
        // Publikujeme path_ po jej naplnení
        path_pub_->publish(path_);
    } else {
        RCLCPP_WARN(get_logger(), "Cesta sa nepodarila nájsť.");
    }
}


void PlanningNode::smoothPath() {
    if (path_.poses.size() < 3) {
        RCLCPP_WARN(get_logger(), "Path too short to smooth (needs at least 3 points)");
        return;
    }

    // Number of smoothing iterations (more iterations = smoother path)
    const int iterations = 10;
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path = path_.poses;

    for (int i =  0; i < iterations; ++i) {
        std::vector<geometry_msgs::msg::PoseStamped> temp_path;
        
        // Always keep the first point
        temp_path.push_back(smoothed_path.front());
        
        // Process intermediate points
        for (size_t j = 1; j < smoothed_path.size(); ++j) {
            const auto& p0 = smoothed_path[j-1];
            const auto& p1 = smoothed_path[j];
            
            // Create two new points between each pair of original points
            geometry_msgs::msg::PoseStamped q, r;
            
            // q = 3/4*p0 + 1/4*p1
            q.header = p0.header;
            q.pose.position.x = 0.75 * p0.pose.position.x + 0.25 * p1.pose.position.x;
            q.pose.position.y = 0.75 * p0.pose.position.y + 0.25 * p1.pose.position.y;
            q.pose.orientation.w = 1.0;  // Neutral orientation
            
            // r = 1/4*p0 + 3/4*p1
            r.header = p0.header;
            r.pose.position.x = 0.25 * p0.pose.position.x + 0.75 * p1.pose.position.x;
            r.pose.position.y = 0.25 * p0.pose.position.y + 0.75 * p1.pose.position.y;
            r.pose.orientation.w = 1.0;  // Neutral orientation
            
            temp_path.push_back(q);
            temp_path.push_back(r);
        }
        
        // Always keep the last point
        temp_path.push_back(smoothed_path.back());
        smoothed_path = temp_path;
    }

    // Optional: Reduce point density while maintaining smoothness
    if (smoothed_path.size() > 200) {  // If path is too long
        std::vector<geometry_msgs::msg::PoseStamped> simplified_path;
        const size_t step = smoothed_path.size() / 100;  // Target ~50 points
        for (size_t i = 0; i < smoothed_path.size(); i += step) {
            simplified_path.push_back(smoothed_path[i]);
        }
        // Keep the last point if not already included
        if (simplified_path.back().pose.position.x != smoothed_path.back().pose.position.x ||
            simplified_path.back().pose.position.y != smoothed_path.back().pose.position.y) {
            simplified_path.push_back(smoothed_path.back());
        }
        smoothed_path = simplified_path;
    }

    // Update the path
    path_.poses = smoothed_path;
    RCLCPP_INFO(get_logger(), "Path smoothed from %ld to %ld points", path_.poses.size(), smoothed_path.size());
}


Cell::Cell(int c, int r) : x(c), y(r), g(0), h(0), f(0), parent(nullptr) {
    // Initialize all member variables
}