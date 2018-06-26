
#ifndef PATH_PREDICTION_H
#define PATH_PREDICTION_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>

namespace path_prediction{
	class PathPredictor{
		public:
		PathPredictor();
		~PathPredictor();
		void predict(const Eigen::Vector2d&, const std::vector<Eigen::Vector2d>&);
		void publish();

		private:

		ros::NodeHandle n;
		ros::Publisher trajectory_publisher;
		visualization_msgs::MarkerArray lines;
		visualization_msgs::Marker line;
		double step_size; // [s]
		static int id;
	};
} // namespace path_prediction

#endif

