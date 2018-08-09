
// rviz表示用
// velocityの配列を受け取って軌跡をpublish
//

#include <geometry_msgs/Point.h>
#include "normal_reaction_force/normal_reaction_force.h"
#include "path_prediction/path_prediction.h"
#include "path_prediction/direct_paths.h"

namespace path_prediction{

	namespace nrf = normal_reaction_force;

	PathsDirector::PathsDirector()
	{
		std::string frame_id;
		std::string topic_pub;

		ros::param::param<std::string>
			("/path_prediction/topic_name", topic_pub, "/trajectory_predicted");

		ros::param::param<std::string>
			("/path_prediction/frame_id", frame_id, "/velodyne");

		ros::param::param<double>
			("/path_prediction/step_size", step_size, 40);

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

	void PathsDirector::createPaths(const pcNormalPtr& pc,
								    const vmsgs::MarkerArray::ConstPtr& humans)
	{
		nrf::VectorField vf;

		vf.setObstacles(pc);
		vf.setHumans(humans);

		geometry_msgs::Point p;
		p.z = 0.0;

		for(auto it = humans->markers.begin(); it != humans->markers.end(); ++it){
			line.points.clear();

			line.id = it->id;

			line.color.r = 0.0 + 0.5 * (it->id % 3);
			line.color.g = 0.4 + 0.1 * (it->id % 7);
			line.color.b = 0.2 + 0.2 * (it->id % 5);

			line.header.stamp = ros::Time::now();

			p.x = own.position.x();
			p.y = own.position.y();
			line.points.push_back(p);

			for(int step = 0; step < step_size; ++step){
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

} // namespace path_prediction

