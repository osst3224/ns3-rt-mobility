#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <sstream>
#include <iomanip>
#include <random>
#include <cmath>

// --- Data Structures ---
struct Point3D {
    double x, y, z;

    bool operator<(const Point3D& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }

    bool operator==(const Point3D& other) const {
        return std::abs(x - other.x) < 1e-6 &&
               std::abs(y - other.y) < 1e-6 &&
               std::abs(z - other.z) < 1e-6;
    }
};

using Graph = std::map<std::string, std::set<Point3D>>;

// --- Utilities ---
std::string pointToKey(const Point3D& pt) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << pt.x << "," << pt.y << "," << pt.z;
    return oss.str();
}

double distance(const Point3D& a, const Point3D& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) +
                     (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

// --- Graph Construction ---
Graph build_connection_graph_xyz(const std::vector<Point3D>& start, const std::vector<Point3D>& end) {
    Graph connection_map;
    for (size_t i = 0; i < start.size(); ++i) {
        std::string s_key = pointToKey(start[i]);
        std::string e_key = pointToKey(end[i]);

        connection_map[s_key].insert(end[i]);
        connection_map[e_key].insert(start[i]);
    }
    return connection_map;
}

void print_json_graph(const Graph& graph) {
    std::cout << "{" << std::endl;
    for (auto it = graph.begin(); it != graph.end(); ++it) {
        std::cout << "  \"" << it->first << "\": [";
        size_t j = 0;
        for (const auto& pt : it->second) {
            std::cout << "[" << pt.x << ", " << pt.y << ", " << pt.z << "]";
            if (++j < it->second.size()) std::cout << ", ";
        }
        std::cout << "]";
        if (std::next(it) != graph.end()) std::cout << ",";
        std::cout << std::endl;
    }
    std::cout << "}" << std::endl;
}

// --- Simulation ---
std::string getRandomKey(const Graph& graph, std::mt19937& rng) {
    size_t index = std::uniform_int_distribution<size_t>(0, graph.size() - 1)(rng);
    auto it = graph.begin();
    std::advance(it, index);
    return it->first;
}

Point3D getNextPoint(const Graph& graph, const Point3D& current, const Point3D* previous, std::mt19937& rng) {
    std::string key = pointToKey(current);
    auto it = graph.find(key);
    if (it == graph.end() || it->second.empty()) return {0, 0, 0};

    std::vector<Point3D> candidates;
    for (const auto& neighbor : it->second) {
        if (previous && distance(*previous, neighbor) < 1e-6) continue;
        candidates.push_back(neighbor);
    }

    if (candidates.empty()) return {0, 0, 0};
    size_t index = std::uniform_int_distribution<size_t>(0, candidates.size() - 1)(rng);
    return candidates[index];
}

// Interpolates a random point between two 3D points
Point3D interpolateRandomBetween(const Point3D& current, const Point3D& next, std::mt19937& rng) {
    std::uniform_real_distribution<> dist(0.0, 1.0);
    double alpha = dist(rng);  // Random weight between 0 and 1

    Point3D interpolated;
    interpolated.x = current.x + alpha * (next.x - current.x);
    interpolated.y = current.y + alpha * (next.y - current.y);
    interpolated.z = current.z + alpha * (next.z - current.z);

    return interpolated;
}

void simulateSinglePath(const Graph& graph,
                        std::vector<Point3D>& out_coords,
                        std::vector<double>& out_times,
                        double velocity_kmh,
                        double duration_seconds,
                        std::mt19937& rng) {
    std::string start_key = getRandomKey(graph, rng);
    std::istringstream ss(start_key);
    double x, y, z;
    char comma;
    ss >> x >> comma >> y >> comma >> z;
    Point3D current = {x, y, z};
    Point3D previous;

    double total_time = 0.0;
    double speed_mps = (velocity_kmh * 1000.0) / 3600.0;
    double dist;
    double travel_time;

    Point3D next = getNextPoint(graph, current, out_coords.size() > 1 ? &previous : nullptr, rng);

    Point3D intermediate = interpolateRandomBetween(current, next, rng);

    out_coords.push_back(intermediate); // Change this to intermediate coord
    out_times.push_back(0.0);

    dist = distance(intermediate, next);
    travel_time = dist / speed_mps;
    total_time += travel_time;
    out_coords.push_back(next);
    out_times.push_back(total_time);
    previous = current;
    current = next;

    while (total_time < duration_seconds) {
        next = getNextPoint(graph, current, out_coords.size() > 1 ? &previous : nullptr, rng);
        if (distance(next, {0, 0, 0}) < 1e-6) break;

        dist = distance(current, next);
        travel_time = dist / speed_mps;
        total_time += travel_time;

        out_coords.push_back(next);
        out_times.push_back(total_time);
        previous = current;
        current = next;
    }
}

void simulateUEPaths(const Graph& graph,
                     size_t num_trajectories,
                     std::vector<std::vector<Point3D>>& sampled_coords,
                     std::vector<double>& sampled_times,
                     int t_ms = 100,
                     double velocity_kmh = 30.0,
                     double duration_seconds = 3600.0,
                     int seed = 42) {
    std::mt19937 rng(seed);
    double t_interval = t_ms / 1000.0;

    // Generate the common sampled times
    for (double t = 0.0; t <= duration_seconds; t += t_interval)
        sampled_times.push_back(t);

    for (size_t i = 0; i < num_trajectories; ++i) {
        std::vector<Point3D> path_coords;
        std::vector<double> path_times;

        // Simulate one raw trajectory
        simulateSinglePath(graph, path_coords, path_times, velocity_kmh, duration_seconds, rng);

        // Interpolate at uniform time steps
        std::vector<Point3D> interpolated;
        size_t j = 0;
        for (double t : sampled_times) {
            while (j + 1 < path_times.size() && path_times[j + 1] < t)
                ++j;

            if (j + 1 >= path_times.size()) {
                interpolated.push_back(path_coords.back());
            } else {
                double t0 = path_times[j];
                double t1 = path_times[j + 1];
                const Point3D& p0 = path_coords[j];
                const Point3D& p1 = path_coords[j + 1];
                double alpha = (t - t0) / (t1 - t0);

                Point3D interp{
                    p0.x + alpha * (p1.x - p0.x),
                    p0.y + alpha * (p1.y - p0.y),
                    p0.z + alpha * (p1.z - p0.z)
                };
                interpolated.push_back(interp);
            }
        }

        sampled_coords.push_back(interpolated);
    }
}

std::vector<Point3D> interpolatePath(const std::vector<Point3D>& coords,
                                     const std::vector<double>& times,
                                     const std::vector<double>& sampled_times) {
    std::vector<Point3D> interpolated;

    size_t j = 0;
    for (double t : sampled_times) {
        while (j + 1 < times.size() && times[j + 1] < t)
            ++j;

        if (j + 1 >= times.size()) {
            // Hold the last point
            interpolated.push_back(coords.back());
        } else {
            double t0 = times[j];
            double t1 = times[j + 1];
            const Point3D& p0 = coords[j];
            const Point3D& p1 = coords[j + 1];
            double alpha = (t - t0) / (t1 - t0);

            Point3D interp{
                p0.x + alpha * (p1.x - p0.x),
                p0.y + alpha * (p1.y - p0.y),
                p0.z + alpha * (p1.z - p0.z)
            };
            interpolated.push_back(interp);
        }
    }

    return interpolated;
}



std::vector<Point3D> getStartPoints() {
    std::vector<Point3D> start = {{ -72. ,  -82. ,    1.5},
                                  {-388. ,   50. ,    1.5},
                                  {-488. ,   55. ,    1.5},
                                  {  33. , -110. ,    1.5},
                                  {  83. , -155. ,    1.5},
                                  { 180. , -205. ,    1.5},
                                  { 253. , -213. ,    1.5},
                                  {  -6. ,  156. ,    1.5},
                                  {-156. ,  212. ,    1.5},
                                  {-298. ,  270. ,    1.5},
                                  {  15. ,  205. ,    1.5},
                                  { 130. ,  170. ,    1.5},
                                  { 145. ,  490. ,    1.5},
                                  { 120. ,  280. ,    1.5},
                                  { 130. ,  170. ,    1.5},
                                  {  88. ,    0. ,    1.5},
                                  {  33. , -110. ,    1.5},
                                  {   5. , -100. ,    1.5},
                                  { -25. , -160. ,    1.5},
                                  { -80. , -200. ,    1.5},
                                  {-230. , -290. ,    1.5},
                                  {-295. , -315. ,    1.5},
                                  {-400. , -390. ,    1.5},
                                  {  80. ,  490. ,    1.5},
                                  {  78. ,  390. ,    1.5},
                                  {  15. ,  205. ,    1.5},
                                  {  -6. ,  156. ,    1.5},
                                  { -72. ,  -82. ,    1.5},
                                  {-171. , -173. ,    1.5},
                                  {-190. , -173. ,    1.5},
                                  {-493. , -352. ,    1.5},
                                  {-156. ,  212. ,    1.5},
                                  {-115. ,  272. ,    1.5},
                                  {-388. ,   50. ,    1.5},
                                  {-355. , -665. ,    1.5},
                                  {-175. , -510. ,    1.5},
                                  { -10. , -510. ,    1.5},
                                  { 430. , -490. ,    1.5},
                                  { 430. , -380. ,    1.5},
                                  { 470. , -315. ,    1.5},
                                  { 560. , -300. ,    1.5},
                                  { 580. , -230. ,    1.5},
                                  { 660. ,   20. ,    1.5},
                                  { 660. ,  160. ,    1.5},
                                  { 560. ,  300. ,    1.5},
                                  { 520. ,  360. ,    1.5},
                                  { 420. ,  435. ,    1.5},
                                  { 145. ,  490. ,    1.5},
                                  {  80. ,  490. ,    1.5},
                                  {   0. ,  480. ,    1.5},
                                  {-220. ,  445. ,    1.5},
                                  {-530. ,  335. ,    1.5},
                                  {-780. ,  130. ,    1.5},
                                  {-790. ,  -40. ,    1.5},
                                  {-603. , -445. ,    1.5},
                                  {-553. , -515. ,    1.5}};
                    
    return start;
}

std::vector<Point3D> getEndPoints() {
    std::vector<Point3D> end = {{-388. ,   50. ,    1.5},
                                {-488. ,   55. ,    1.5},
                                {-780. ,  130. ,    1.5},
                                {  83. , -155. ,    1.5},
                                { 180. , -205. ,    1.5},
                                { 253. , -213. ,    1.5},
                                { 470. , -315. ,    1.5},
                                {-156. ,  212. ,    1.5},
                                {-298. ,  270. ,    1.5},
                                {-530. ,  335. ,    1.5},
                                { 130. ,  170. ,    1.5},
                                { 660. ,   20. ,    1.5},
                                { 120. ,  280. ,    1.5},
                                { 130. ,  170. ,    1.5},
                                {  88. ,    0. ,    1.5},
                                {  33. , -110. ,    1.5},
                                {   5. , -100. ,    1.5},
                                { -25. , -160. ,    1.5},
                                { -80. , -200. ,    1.5},
                                {-230. , -290. ,    1.5},
                                {-295. , -315. ,    1.5},
                                {-400. , -390. ,    1.5},
                                {-553. , -515. ,    1.5},
                                {  78. ,  390. ,    1.5},
                                {  15. ,  205. ,    1.5},
                                {  -6. ,  156. ,    1.5},
                                { -72. ,  -82. ,    1.5},
                                {-171. , -173. ,    1.5},
                                {-190. , -173. ,    1.5},
                                {-493. , -352. ,    1.5},
                                {-603. , -445. ,    1.5},
                                {-115. ,  272. ,    1.5},
                                {   0. ,  480. ,    1.5},
                                {-298. ,  270. ,    1.5},
                                {-175. , -510. ,    1.5},
                                { -10. , -510. ,    1.5},
                                { 430. , -490. ,    1.5},
                                { 430. , -380. ,    1.5},
                                { 470. , -315. ,    1.5},
                                { 560. , -300. ,    1.5},
                                { 580. , -230. ,    1.5},
                                { 660. ,   20. ,    1.5},
                                { 660. ,  160. ,    1.5},
                                { 560. ,  300. ,    1.5},
                                { 520. ,  360. ,    1.5},
                                { 420. ,  435. ,    1.5},
                                { 145. ,  490. ,    1.5},
                                {  80. ,  490. ,    1.5},
                                {   0. ,  480. ,    1.5},
                                {-220. ,  445. ,    1.5},
                                {-530. ,  335. ,    1.5},
                                {-780. ,  130. ,    1.5},
                                {-790. ,  -40. ,    1.5},
                                {-603. , -445. ,    1.5},
                                {-553. , -515. ,    1.5},
                                {-355. , -665. ,    1.5}};

    return end;
}

// --- Main ---
//int main() {
//    // Example usage with a small subset
//    std::vector<Point3D> start = getStartPoints();
//
//    std::vector<Point3D> end = getEndPoints();
//
//    Graph graph = build_connection_graph_xyz(start, end);
//    //std::cout << "Constructed Graph:\n";
//    //print_json_graph(graph);
//
//    std::vector<std::vector<Point3D>> sampled_coords;
//    std::vector<double> sampled_times;
//
//    simulateUEPaths(graph, 3, sampled_coords, sampled_times, 100, 30.0, 120.0);
//
//    for (size_t i = 0; i < sampled_times.size(); ++i) {
//        const Point3D& pt = sampled_coords[2][i];
//        std::cout << "t = " << sampled_times[i] << "s: [" << pt.x << ", " << pt.y << ", " << pt.z << "]\n";
//    }
//
//    return 0;
//}
//