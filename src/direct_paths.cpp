
// rviz表示用
// velocityの配列を受け取って軌跡をpublish
//

#include <geometry_msgs/Point.h>
#include "normal_reaction_force/normal_reaction_force.h"
#include "path_prediction/path_prediction.h"
#include "path_prediction/direct_paths.h"

namespace path_prediction{

	PathsDirector::PathsDirector()
		: is_linear(false)
	{
		std::string frame_id;
		std::string topic_pub;

		ros::param::param<std::string>
			("/path_prediction/topic_name", topic_pub, "/trajectory_predicted");

		ros::param::param<std::string>
			("/path_prediction/frame_id", frame_id, "/map");

		ros::param::param<double>
			("/path_prediction/step_size", step_size, 40);

		n.getParam("/path_prediction/is_linear", is_linear);

		trajectory_publisher = n.advertise<vmsgs::MarkerArray>(topic_pub, 1);

		line.header.frame_id = frame_id;
		line.ns = "predicted_trajectory";
		line.action = vmsgs::Marker::ADD;
		line.pose.orientation.w = 1.0;
		line.type = vmsgs::Marker::LINE_STRIP;
		line.scale.x = 0.02;
		line.color.a = 1.0;
		// line.color.r = 0.0;
		// line.color.g = 0.8;
		// line.color.b = 0.9;
		line.lifetime = ros::Duration(0.4);
	}

	PathsDirector::~PathsDirector() {}

	void PathsDirector::publish()
	{
		if(lines.markers.size()){
			trajectory_publisher.publish(lines);
			lines.markers.clear();
		}
	}

	void PathsDirector::createPaths(const pcNormalPtr& pc,
								    const vmsgs::MarkerArray::ConstPtr& arrays)
	{
		nrf::VectorField vf;

		setHumans(arrays);
		vf.setObstacles(pc);
		vf.setDistances(humans, nhumans);

		Eigen::Vector2d velocity_curved;
		std::vector<bool> is_curved(false, nhumans);

		geometry_msgs::Point p;
		p.z = 0.0;

		for(unsigned i = 0; i != nhumans; ++i){
			paths[arrays->markers[i].id].predict(humans[i].position, humans[i].velocity);
		}

		for(unsigned step = 0; step != step_size; ++step){
			if(!is_linear) vf.velocityConversion(humans); // velocity 更新
			for(unsigned i = 0; i != nhumans; ++i){
				humans[i].position = paths[arrays->markers[i].id].predict(humans[i].velocity);
				paths[arrays->markers[i].id].getGoal(humans[i].velocity); // velocity 更新

				p.x = humans[i].position.x();
				p.y = humans[i].position.y();
				lines.markers[i].points.push_back(p);
				// std::cout << "lines.size : " << lines.markers.size() << std::endl;
				// lines.markers[3*i].points.push_back(p);
				// lines.markers[3*i + 1].points.push_back(p);
				// lines.markers[3*i + 2].points.push_back(p);
			}

		}

		auto it = paths.begin();
		while(it != paths.end()){ // 全走査せずにやりたいけど...
			if(!it->second.observed){
				paths.erase(it++);
			}else{
				it->second.observed = false;
				++it;
			}
		}
	}

	// private
	void PathsDirector::setHumans(const vmsgs::MarkerArray::ConstPtr& arrays)
	{
		geometry_msgs::Point p;
		p.z = 0.0;

		humans.clear();

		nhumans = 0;
		for(auto it = arrays->markers.begin(); it != arrays->markers.end(); ++it){
			double yaw = atan2(2*(it->pose.orientation.w * it->pose.orientation.z
								+ it->pose.orientation.x * it->pose.orientation.y),
					1 - 2*(pow(it->pose.orientation.y, 2) + pow(it->pose.orientation.z, 2)));

			nrf::State4d human = {{it->pose.position.x, it->pose.position.y},
								  {it->scale.x * cos(yaw), it->scale.x * sin(yaw)}};

			humans.push_back(human);
			++nhumans;

			line.points.clear();

			line.id = it->id;

			line.color.r = 0.0 + 0.5 * (it->id % 3);
			line.color.g = 0.4 + 0.1 * (it->id % 7);
			line.color.b = 0.2 + 0.2 * (it->id % 5);

			p.x = human.position.x();
			p.y = human.position.y();
			line.points.push_back(p);

			line.header = it->header;

			lines.markers.push_back(line);
			// lines.markers.push_back(line); // velo -5 deg
			// lines.markers.push_back(line); // velo +5 deg
		}

	}

} // namespace path_prediction

