
// rviz表示用
// velocityの配列を受け取って軌跡をpublish
//

#include <geometry_msgs/Point.h>
#include "normal_reaction_force/normal_reaction_force.h"
#include "path_prediction/path_prediction.h"
#include "path_prediction/direct_paths.h"

namespace path_prediction{

	PathsDirector::PathsDirector()
	{
		std::string frame_id;
		std::string topic_pub;

		ros::param::param<std::string>
			("/path_prediction/topic_name", topic_pub, "/trajectory_predicted");

		ros::param::param<std::string>
			("/path_prediction/frame_id", frame_id, "/map");

		ros::param::param<double>
			("/path_prediction/step_size", step_size, 40);

		trajectory_publisher = n.advertise<vmsgs::MarkerArray>(topic_pub, 10);
		// markers_del_publisher = n.advertise<vmsgs::MarkerArray>("delold", 1);

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
		
		// del_id.header.frame_id = frame_id;
		// del_id.ns = "delall_trajectory";
		// del_id.action = vmsgs::Marker::DELETE;
		// del_id.pose.orientation.w = 1.0;
		// del_id.type = vmsgs::Marker::LINE_STRIP;
		// del_id.scale.x = 0.02;
		// del_id.color.a = 1.0;
	}

	PathsDirector::~PathsDirector() {}

	void PathsDirector::publish()
	{
		static int cnt = 0;
		ros::Publisher deleteall = n.advertise<vmsgs::Marker>("deleteall", 1);
		vmsgs::Marker delall;
		delall.header.frame_id = "/map";
		delall.ns = "predicted_trajectory";
		delall.pose.orientation.w = 1.0;
		delall.type = vmsgs::Marker::LINE_STRIP;
		// if(cnt){
		// 	delall.action = vmsgs::Marker::ADD;
		// }else{
		// 	delall.action = vmsgs::Marker::DELETEALL;
		// }
		delall.action = vmsgs::Marker::DELETEALL;
		deleteall.publish(delall);

		trajectory_publisher.publish(lines);
		lines.markers.clear();
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

		for(unsigned i = 0; i < nhumans; ++i){
			paths[arrays->markers[i].id].predict(humans[i].position, humans[i].velocity);
		}

		for(unsigned step = 0; step < step_size; ++step){
			for(unsigned i = 0; i < nhumans; ++i){
				vf.velocityConversion(humans);
				humans[i].position = paths[arrays->markers[i].id].predict(humans[i].velocity);
				paths[arrays->markers[i].id].getGoal(humans[i].velocity);

				p.x = humans[i].position.x();
				p.y = humans[i].position.y();
				lines.markers[i].points.push_back(p);
				// std::cout << "lines.size : " << lines.markers.size() << std::endl;
			}

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
	void PathsDirector::setHumans(const vmsgs::MarkerArray::ConstPtr& arrays)
	{
		geometry_msgs::Point p;
		p.z = 0.0;

		humans.clear();
		// lines.markers.clear();

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

			line.header.stamp = ros::Time::now();

			lines.markers.push_back(line);
		}

	}

} // namespace path_prediction

