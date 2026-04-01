// ue-trajectories.h

#ifndef UE_TRAJECTORIES_H
#define UE_TRAJECTORIES_H

#include <vector>
#include <map>
#include <set>
#include <string>
#include <random>

// --- Data Structures ---
struct Point3D {
    double x, y, z;
    bool operator<(const Point3D& other) const;
    bool operator==(const Point3D& other) const;
};

using Graph = std::map<std::string, std::set<Point3D>>;

// --- Utilities ---
std::string pointToKey(const Point3D& pt);
double distance(const Point3D& a, const Point3D& b);

// --- Graph Construction ---
Graph build_connection_graph_xyz(const std::vector<Point3D>& start, const std::vector<Point3D>& end);
void print_json_graph(const Graph& graph);

// --- Simulation ---
std::string getRandomKey(const Graph& graph, std::mt19937& rng);
Point3D getNextPoint(const Graph& graph, const Point3D& current, const Point3D* previous, std::mt19937& rng);
void simulateSinglePath(const Graph& graph,
                        std::vector<Point3D>& out_coords,
                        std::vector<double>& out_times,
                        double velocity_kmh,
                        double duration_seconds,
                        std::mt19937& rng);
void simulateUEPaths(const Graph& graph,
                     size_t num_trajectories,
                     std::vector<std::vector<Point3D>>& sampled_coords,
                     std::vector<double>& sampled_times,
                     int t_ms = 100,
                     double velocity_kmh = 30.0,
                     double duration_seconds = 3600.0,
                     int seed = 42);

std::vector<Point3D> interpolatePath(const std::vector<Point3D>& coords,
                                     const std::vector<double>& times,
                                     const std::vector<double>& sampled_times);

std::vector<Point3D> getStartPoints();
std::vector<Point3D> getEndPoints();

#endif // UE_TRAJECTORIES_H
