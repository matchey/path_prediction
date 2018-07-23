
// rviz表示用
// velocityの配列を受け取って軌跡をpublish
//

#include <geometry_msgs/Point.h>
#include "path_prediction/direct_paths.h"

namespace path_prediction{

	PathsDirector::PathsDirector()
	{
		std::string frame_id;
		std::string topic_pub;

		ros::param::param<std::string>
			("/path_prediction/topic_name", topic_pub, "/trajectory_predicted");

		ros::param::param<std::string>
			("/path_prediction/frame_id", frame_id, "/velodyne");

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

	PathsDirector::~PathsDirector() {}

	void PathsDirector::publish()
	{
		trajectory_publisher.publish(lines);
		lines.markers.clear();
	}

	void PathsDirector::predict(const visualization_msgs::MarkerArray::ConstPtr& humans)
	{
		geometry_msgs::Point p;
		Eigen::Vector2d velocity;
		PathPredictor path;

		for(auto it = humans->markers.begin(); it != humans->markers.end(); ++it){
			paths[it->id] = path;
			line.points.clear();
			double yaw = atan2(2*(it->pose.orientation.w * it->pose.orientation.z
								+ it->pose.orientation.x * it->pose.orientation.y),
					1 - 2*(pow(it->pose.orientation.y, 2) + pow(it->pose.orientation.z, 2)));

			normal_reaction_force::State4d own = {{it->pose.position.x, it->pose.position.y},
												  {it->scale.x * cos(yaw), it->scale.x * sin(yaw)}};
			vf.velocityConversion(own, velocity);
			own.position = path.predict(own.position, velocity);
			own.velocity = velocity;

			line.id = it->id;
			line.header.stamp = ros::Time::now();

			p.x = own.position.x();
			p.y = own.position.y();
			p.z = 0.0;
			line.points.push_back(p);

			for(int step = 0; step < 40; ++step){
				vf.velocityConversion(own, velocity);
				own.position = path.predict(velocity);
				own.velocity = velocity;
				p.x = own.position.x();
				p.y = own.position.y();
				line.points.push_back(p);
			}

			lines.markers.push_back(line);
		}
	}

	// private

} // namespace path_prediction

