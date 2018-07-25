
#ifndef DIRECT_PATH_H
#define DIRECT_PATH_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include "normal_reaction_force/normal_reaction_force.h"
#include "path_prediction/path_prediction.h"

namespace path_prediction{

	namespace nrf = normal_reaction_force;

	class PathsDirector{
		public:
		PathsDirector();
		~PathsDirector();
		void predict(const visualization_msgs::MarkerArray::ConstPtr&, const nrf::pcNormalPtr&);
		void publish();

		private:

		ros::NodeHandle n;
		ros::Publisher trajectory_publisher;
		std::map<int, PathPredictor> paths; // 走査遅いからmap使うのよくない?
		nrf::VectorField vf;
		visualization_msgs::MarkerArray lines;
		visualization_msgs::Marker line;
	};

} // namespace path_prediction

#endif // DIRECT_PATH_H

