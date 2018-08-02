
// rviz表示用
// velocityの配列を受け取って軌跡をpublish
//

#include <geometry_msgs/Point.h>
#include "path_prediction/path_prediction.h"
#include "path_prediction/direct_paths.h"

namespace path_prediction{

	PathsDirector::PathsDirector()
		// : obstacles(new nrf::pcNormal)
	{
		std::string frame_id;
		std::string topic_pub;

		ros::param::param<std::string>
			("/path_prediction/topic_name", topic_pub, "/trajectory_predicted");

		ros::param::param<std::string>
			("/path_prediction/frame_id", frame_id, "/velodyne");

		ros::param::param<double>
			("/path_prediction/step_num", step_num, 40);

		trajectory_publisher = n.advertise<vmsgs::MarkerArray>(topic_pub, 10);

		line.header.frame_id = frame_id;
		line.ns = "predicted_trajectory";
		line.action = vmsgs::Marker::ADD;
		line.pose.orientation.w = 1.0;
		line.type = vmsgs::Marker::LINE_STRIP;
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

	void PathsDirector::predict(const vmsgs::MarkerArray::ConstPtr& humans,
								const nrf::pcNormalPtr& pc)
	{
		geometry_msgs::Point p;
		Eigen::Vector2d velocity;
		// PathPredictor path;

		// vf.setHumans(humans);
		vf.setObstacles(pc);

		for(auto it = humans->markers.begin(); it != humans->markers.end(); ++it){
			// paths[it->id] = path;
			line.points.clear();
			double yaw = atan2(2*(it->pose.orientation.w * it->pose.orientation.z
								+ it->pose.orientation.x * it->pose.orientation.y),
					1 - 2*(pow(it->pose.orientation.y, 2) + pow(it->pose.orientation.z, 2)));

			nrf::State4d own = {{it->pose.position.x, it->pose.position.y},
								{it->scale.x * cos(yaw), it->scale.x * sin(yaw)}};
			// vf.velocityConversion(own, velocity);
			velocity = own.velocity;
			own.position = paths[it->id].predict(own.position, velocity);
			paths[it->id].getGoal(velocity);
			own.velocity = velocity;

			line.id = it->id;

			line.color.r = 0.0 + 0.5 * (it->id % 3);
			line.color.g = 0.4 + 0.1 * (it->id % 7);
			line.color.b = 0.2 + 0.2 * (it->id % 5);

			line.header.stamp = ros::Time::now();

			p.x = own.position.x();
			p.y = own.position.y();
			p.z = 0.0;
			line.points.push_back(p);

			for(int step = 0; step < step_num; ++step){
				// if(step > 30){
				// 	vf.velocityConversion(own, velocity);
				// }else{
				// 	velocity = own.velocity;
				// }
				vf.velocityConversion(own, velocity);
				own.position = paths[it->id].predict(velocity);
				paths[it->id].getGoal(velocity);
				own.velocity = velocity;
				p.x = own.position.x();
				p.y = own.position.y();
				line.points.push_back(p);
			}

			lines.markers.push_back(line);
		}

		auto it = paths.begin();
		while(it != paths.end()){
			if(!it->second.observed){
				paths.erase(it++);
			}else{
				it->second.observed = false;
				++it;
			}
		}
	}

	// private
	// void PathsDirector::createObstacles(const vmsgs::MarkerArray::ConstPtr& humans)
	// {
	// 	const double radius = 0.15;
	// 	nrf::pcNormalPtr human (new nrf::pcNormal);
    //
	// 	for(auto arrow = humans->markers.begin(); arrow != humans->markers.end(); ++arrow){
	// 	}
	// }

} // namespace path_prediction

