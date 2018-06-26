
// rviz表示用
// velocityの配列を受け取って軌跡をpublish
//

#include <geometry_msgs/Point.h>
#include "path_prediction/path_prediction.h"

namespace path_prediction{

	int PathPredictor::id;

	PathPredictor::PathPredictor()
	{
		std::string frame_id;
		std::string topic_pub;

		ros::param::param<double>
			("/path_prediction/step_size", step_size, 0.05);

		ros::param::param<std::string>
			("/path_prediction/topic_name", topic_pub, "/trajectory_predicted");

		ros::param::param<std::string>
			("/path_prediction/frame_id", frame_id, "/velodyne");

		trajectory_publisher = n.advertise<visualization_msgs::MarkerArray>(topic_pub, 1);

		line.header.frame_id = frame_id;
		line.ns = "predicted_trajectory";
		line.action = visualization_msgs::Marker::ADD;
		line.pose.orientation.w = 1.0;
		line.type = visualization_msgs::Marker::LINE_STRIP;
		line.scale.x = 0.02;
		line.color.a = 1.0;
		line.color.g = 0.8;
		line.color.b = 0.9;
	}

	PathPredictor::~PathPredictor() {}

	void PathPredictor::publish()
	{
		trajectory_publisher.publish(lines);
		id = 0;
		lines.markers.clear();
	}

	void PathPredictor::predict(const Eigen::Vector2d& init_position,
								const std::vector<Eigen::Vector2d>& velocities)
	{
		line.id = id++;
		line.header.stamp = ros::Time::now();

		geometry_msgs::Point p;

		p.x = init_position.x();
		p.y = init_position.y();
		p.z = 0.0;

		for(auto it = velocities.begin(); it != velocities.end(); ++it){
			p.x += it->x() * step_size;
			p.y += it->y() * step_size;
			line.points.push_back(p);
		}
		lines.markers.push_back(line);
		line.points.clear();
	}

	// private

} // namespace path_prediction
