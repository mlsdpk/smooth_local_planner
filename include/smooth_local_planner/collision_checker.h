#pragma once

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <smooth_local_planner/line_iterator.h>
#include <smooth_local_planner/planner_utils.h>
#include <tf2/utils.h>

namespace smooth_local_planner {

static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;

class GridCollisionChecker {
 public:
  /**
   * @brief A constructor for smooth_local_planner::GridCollisionChecker
   * for static footprint collision checking
   * @param costmap The costmap to collision check against
   * @param footprint Static footprint
   */
  GridCollisionChecker(costmap_2d::Costmap2D* costmap,
                       const std::vector<geometry_msgs::Point>& footprint)
      : costmap_{costmap}, footprint_{footprint} {}

  /**
   * @brief Check the spiral paths are in collision or not within the local
   * costmap using the robot footprint with the technique of swath-based
   * collision checking.
   * Note: This function assumes that input spiral paths are in the map frame.
   * @param idxes Path indexes that are in collision
   * @param paths Spiral paths
   */
  void checkCollision(std::vector<bool>& status,
                      const std::vector<nav_msgs::Path>& paths) {
    // here we check all the points in the path
    // this might be computationally expensive
    // in future, may be we set one parameter for collision checking resolution
    status = std::vector<bool>(paths.size(), false);
    for (std::size_t i = 0; i < paths.size(); ++i) {
      for (const auto& pose : paths[i].poses) {
        if (isCollision(pose.pose.position.x, pose.pose.position.y,
                        tf2::getYaw(pose.pose.orientation))) {
          status[i] = true;
          break;
        }
      }
    }
  }

  /**
   * @brief Check if the footprint is in collision with the current shared
   * costmap (footprint here is not transformed to pose yet)
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle of pose to check against
   * @return boolean if in collision or not.
   */
  bool isCollision(const double x, const double y, const double theta) {
    // always check the cell corrdinate of the center of the robot
    unsigned int cell_x, cell_y;
    if (!costmap_->worldToMap(x, y, cell_x, cell_y)) {
      return true;
    }

    unsigned char cost = costmap_->getCost(cell_x, cell_y);
    if (cost == NO_INFORMATION || cost == LETHAL_OBSTACLE ||
        cost == INSCRIBED_INFLATED_OBSTACLE)
      return true;

    // now we need to check the full polygon footprint of the robot

    // create a new footprint by transforming the current one into desired
    // pose
    std::vector<geometry_msgs::Point> transformed_footprint;
    costmap_2d::transformFootprint(x, y, theta, footprint_,
                                   transformed_footprint);

    // now use this transformed footprint to check collision in the costmap
    unsigned int x0, x1, y0, y1;
    double footprint_cost = 0.0;

    // rasterize each line in the footprint
    for (unsigned int i = 0; i < transformed_footprint.size() - 1; ++i) {
      // get the cell coord of the first point
      if (!costmap_->worldToMap(transformed_footprint[i].x,
                                transformed_footprint[i].y, x0, y0)) {
        return true;
      }

      // get the cell coord of the second point
      if (!costmap_->worldToMap(transformed_footprint[i + 1].x,
                                transformed_footprint[i + 1].y, x1, y1)) {
        return true;
      }

      footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

      // if in collision, no need to continue
      if (footprint_cost == static_cast<double>(NO_INFORMATION) ||
          footprint_cost == static_cast<double>(LETHAL_OBSTACLE) ||
          footprint_cost == static_cast<double>(INSCRIBED_INFLATED_OBSTACLE)) {
        return true;
      }
    }

    // connect the first point in the footprint to the last point
    // get the cell coord of the last point
    if (!costmap_->worldToMap(transformed_footprint.back().x,
                              transformed_footprint.back().y, x0, y0))
      return true;

    // get the cell coord of the first point
    if (!costmap_->worldToMap(transformed_footprint.front().x,
                              transformed_footprint.front().y, x1, y1))
      return true;

    footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);
    if (footprint_cost == static_cast<double>(NO_INFORMATION) ||
        footprint_cost == static_cast<double>(LETHAL_OBSTACLE) ||
        footprint_cost == static_cast<double>(INSCRIBED_INFLATED_OBSTACLE)) {
      return true;
    }

    return false;
  }

  double lineCost(int x0, int x1, int y0, int y1) const {
    double line_cost = 0.0;
    double point_cost = -1.0;

    for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
      point_cost =
          pointCost(line.getX(), line.getY());  // Score the current point

      if (point_cost < 0) return point_cost;

      if (line_cost < point_cost) line_cost = point_cost;
    }

    return line_cost;
  }

  double pointCost(int x, int y) const {
    return static_cast<double>(costmap_->getCost(x, y));
  }

 private:
  costmap_2d::Costmap2D* costmap_;
  std::vector<geometry_msgs::Point> footprint_;
};
}  // namespace smooth_local_planner