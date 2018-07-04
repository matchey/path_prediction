
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
			("/path_prediction/frame_id", frame_id, "/map");

		trajectory_publisher = n.advertise<visualization_msgs::MarkerArray>(topic_pub, 10);

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
		if(!line.points.empty()){
			lines.markers.push_back(line);
		}
		trajectory_publisher.publish(lines);
		id = 0;
		line.points.clear();
		lines.markers.clear();
	}

	Eigen::Vector2d PathPredictor::predict(const Eigen::Vector2d& init_position,
										   const Eigen::Vector2d& velocity)
	{
		if(!line.points.empty()){
			lines.markers.push_back(line);
			line.points.clear();
		}

		line.id = id++;
		line.header.stamp = ros::Time::now();

		geometry_msgs::Point p;

		position = init_position;
		p.x = position.x();
		p.y = position.y();
		p.z = 0.0;
		line.points.push_back(p);

		predict(velocity);

		return position;
	}

	Eigen::Vector2d PathPredictor::predict(const Eigen::Vector2d& velocity)
	{
		geometry_msgs::Point p;

		position.x() += velocity.x() * step_size;
		position.y() += velocity.y() * step_size;

		p.x = position.x();
		p.y = position.y();

		line.points.push_back(p);

		return position;
	}

	// private

} // namespace path_prediction
