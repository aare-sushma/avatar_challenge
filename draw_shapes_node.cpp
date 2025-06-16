#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h> 
#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/parameter_client.hpp>
#include <visualization_msgs/msg/marker.hpp>


using json = nlohmann::json;
using namespace std::chrono_literals;

std::vector<Eigen::Vector2d> generateBSpline(const std::vector<Eigen::Vector2d>& controlPoints,int degree, int numPoints) 
{
  std::vector<Eigen::Vector2d> curve;
  int n = controlPoints.size() - 1;
  int k = degree;

  // Generate knot vector (clamped)
  std::vector<double> knots(n + k + 2);
  for (int i = 0; i <= n + k + 1; ++i) {
    if (i <= k)
      knots[i] = 0;
    else if (i >= n + 1)
      knots[i] = 1;
    else
      knots[i] = (double)(i - k) / (n - k + 1);
  }

  // Evaluate basis spline at t
  std::function<double(int, int, double)> N = [&](int i, int k, double t) -> double {
    if (k == 0)
        return (knots[i] <= t && t < knots[i + 1]) ? 1.0 : 0.0;
    double a = (knots[i + k] - knots[i]) != 0 ? (t - knots[i]) / (knots[i + k] - knots[i]) * N(i, k - 1, t) : 0.0;
    double b = (knots[i + k + 1] - knots[i + 1]) != 0 ? (knots[i + k + 1] - t) / (knots[i + k + 1] - knots[i + 1]) * N(i + 1, k - 1, t) : 0.0;
    return a + b;
};

  // Sample points on the curve
  for (int step = 0; step < numPoints; ++step) {
    double t = ((double)step) / (numPoints - 1);
    Eigen::Vector2d point(0, 0);
    for (int i = 0; i <= n; ++i)
      point += controlPoints[i] * N(i, k, t);
    curve.push_back(point);
  }

  return curve;
}


geometry_msgs::msg::Pose transform2DPointTo3D(const std::pair<double, double>& pt_2d,
                                              const geometry_msgs::msg::Pose& start_pose)
{
    geometry_msgs::msg::Pose pose;

    // Convert orientation to tf2 quaternion
    tf2::Quaternion q;
    tf2::fromMsg(start_pose.orientation, q);

    // 2D point in plane's local frame
    tf2::Vector3 local(pt_2d.first, pt_2d.second, 0.0);

    // Rotate it using full quaternion (roll, pitch, yaw)
    tf2::Vector3 rotated = tf2::quatRotate(q, local);

    pose.position.x = start_pose.position.x + rotated.x();
    pose.position.y = start_pose.position.y + rotated.y();
    pose.position.z = start_pose.position.z + rotated.z();

    pose.orientation = start_pose.orientation;

    return pose;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("draw_shapes_node");


    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    auto marker_array_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", qos); 


    // Fetch robot_description and robot_description_semantic from /move_group if not declared
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "move_group");

    while (!param_client->wait_for_service(1s)) {
    RCLCPP_WARN(node->get_logger(), "Waiting for move_group parameter service...");
    }

    try {
        auto robot_description = param_client->get_parameter<std::string>("robot_description");
        auto semantic_description = param_client->get_parameter<std::string>("robot_description_semantic");

        node->declare_parameter("robot_description", robot_description);
        node->declare_parameter("robot_description_semantic", semantic_description);

        RCLCPP_INFO(node->get_logger(), "Declared robot_description and robot_description_semantic locally.");
    
    } 
    
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get parameters from move_group: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Starting draw_shapes_node...");

    // Load JSON
    std::ifstream ifs("/home/dev/dev_ws/src/avatar_challenge/config/shapes.json");
    json j;
    ifs >> j;
    RCLCPP_INFO(node->get_logger(), "Loaded JSON successfully");

    // MoveIt setup
    static const std::string PLANNING_GROUP = "xarm7";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    RCLCPP_INFO(node->get_logger(), "Move group initialized. Planning frame: %s", move_group.getPlanningFrame().c_str());
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    int marker_id = 0;
    for (const auto& shape : j["shapes"])
    {
    // Read starting pose
        auto pos = shape["start_pose"]["position"];
        auto rot = shape["start_pose"]["orientation_rpy"];

        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = pos[0];
        start_pose.position.y = pos[1];
        start_pose.position.z = pos[2];

        tf2::Quaternion q;
        q.setRPY(rot[0], rot[1], rot[2]);
        start_pose.orientation = tf2::toMsg(q);

        // Convert vertices to 3D poses
        std::vector<geometry_msgs::msg::Pose> waypoints;

        std::string shape_type = shape.contains("type") ? static_cast<std::string>(shape["type"]) : "polygon";

        if (shape_type == "arc") {
            auto center = shape["center"];
            double cx = center[0];
            double cy = center[1];
            double radius = shape["radius"];
            double start_angle = shape["start_angle"];
            double end_angle = shape["end_angle"];
            int segments = shape["segments"];

            for (int i = 0; i <= segments; ++i) {
                double theta = start_angle + (end_angle - start_angle) * i / segments;
                double x = cx + radius * cos(theta);
                double y = cy + radius * sin(theta);
                waypoints.push_back(transform2DPointTo3D({x, y}, start_pose));
            }
        } 
        else if (shape.contains("vertices")) {
            for (const auto& pt : shape["vertices"]) {
                std::pair<double, double> pt_2d = { pt[0], pt[1] };
                geometry_msgs::msg::Pose wp = transform2DPointTo3D(pt_2d, start_pose);
                if (!waypoints.empty()) {
                    geometry_msgs::msg::Pose prev = waypoints.back();
                    int blend_steps = 3;
                    for (int i = 1; i <= blend_steps; ++i) {
                        geometry_msgs::msg::Pose blend;
                        double alpha = (double)i / (blend_steps + 1);
                        blend.position.x = (1 - alpha) * prev.position.x + alpha * wp.position.x;
                        blend.position.y = (1 - alpha) * prev.position.y + alpha * wp.position.y;
                        blend.position.z = (1 - alpha) * prev.position.z + alpha * wp.position.z;
                        blend.orientation = prev.orientation;  // Keep same orientation for simplicity
                        waypoints.push_back(blend);
                    }
                }
                waypoints.push_back(wp);
            }
        } 
        else if (shape_type == "bspline") {
            auto control_points = shape["control_points"];
            int degree = shape.value("degree", 3);
            int num_points = shape.value("num_points", 50);

            std::vector<Eigen::Vector2d> ctrl_pts;
            for (const auto& pt : control_points) {
                ctrl_pts.emplace_back(pt[0], pt[1]);
            }

            std::vector<Eigen::Vector2d> bspline_points = generateBSpline(ctrl_pts, degree, num_points);

            for (const auto& pt : bspline_points) {
                geometry_msgs::msg::Pose wp = transform2DPointTo3D({pt.x(), pt.y()}, start_pose);
                waypoints.push_back(wp);
            }
        }
        else {
            RCLCPP_WARN(node->get_logger(), "Unknown shape type or missing vertices.");
            continue;
        }
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node->get_clock()->now();
        marker.ns = "shapes";
        marker.id = marker_id++;  // Increment for each shape
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;  // Line width
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        for (const auto& p : waypoints)
        {
            geometry_msgs::msg::Point point;
            point.x = p.position.x;
            point.y = p.position.y;
            point.z = p.position.z;
            marker.points.push_back(point);
        }
        rclcpp::sleep_for(1s);

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_array_pub->publish(marker_array);

        move_group.setPoseTarget(start_pose);
        moveit::planning_interface::MoveGroupInterface::Plan move_to_start_plan;
        bool success = (move_group.plan(move_to_start_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group.execute(move_to_start_plan);
            rclcpp::sleep_for(1s); 
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to move to start pose. Skipping this shape.");
            continue;
        }

        moveit::planning_interface::MoveGroupInterface::Plan start_plan;
        rclcpp::sleep_for(1s);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0, trajectory);

        if (fraction > 0.9)
        {
            RCLCPP_INFO(node->get_logger(), "Cartesian path planning success: %.2f%%", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group.execute(plan);
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Cartesian path planning incomplete: %.2f%%", fraction * 100.0);
            RCLCPP_WARN(node->get_logger(), "Skipping execution of this shape.");
        }
        move_group.clearPoseTargets(); 
        rclcpp::sleep_for(1s); // brief pause between shapes
    }

  RCLCPP_INFO(node->get_logger(), "All shapes executed.");
  rclcpp::sleep_for(5s);  
  rclcpp::shutdown();
  return 0;
}
