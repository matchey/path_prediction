
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
		Eigen::Vector2d predict(const Eigen::Vector2d&, const Eigen::Vector2d&);
		Eigen::Vector2d predict(const Eigen::Vector2d&);
		void publish();

		private:
		ros::NodeHandle n;
		ros::Publisher trajectory_publisher;
		Eigen::Vector2d position;
		visualization_msgs::MarkerArray lines;
		visualization_msgs::Marker line;
		double step_size; // [s]
		static int id;
		Eigen::Vector2d goal;
	};
} // namespace path_prediction

#endif

