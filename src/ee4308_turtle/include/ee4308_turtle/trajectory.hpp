#include "common.hpp"
#include "grid.hpp"

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
std::vector<Position> post_process(std::vector<Position> path, Grid& grid);  // returns the turning points
std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, double average_speed, double target_dt,
                                          Grid& grid);
std::vector<Position> generate_trajectory(const std::vector<Position>& raw_path, const double average_speed,
                                          const double target_dt, const Grid& grid);
bool is_safe_trajectory(std::vector<Position> trajectory, Grid& grid);
#endif