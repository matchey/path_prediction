
#ifndef DIRECT_PATH_H
#define DIRECT_PATH_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <pcl/point_cloud.h>

namespace path_prediction{

	typedef pcl::PointNormal PointN;
	typedef pcl::PointCloud<PointN> pcNormal;
	typedef pcNormal::Ptr pcNormalPtr;
	
	namespace vmsgs = visualization_msgs;

	class PathPredictor;

	class PathsDirector{
		public:
		PathsDirector();
		~PathsDirector();
		void createPaths(const pcNormalPtr&, const vmsgs::MarkerArray::ConstPtr&);
		void publish();

		private:

		ros::NodeHandle n;
		ros::Publisher trajectory_publisher;
		vmsgs::MarkerArray lines;
		vmsgs::Marker line;

		double step_size; // 何ステップ先まで計算するか [回]
		std::map<int, PathPredictor> paths; // 走査遅いからmap使うのよくない?
	};

} // namespace path_prediction

#endif // DIRECT_PATH_H

