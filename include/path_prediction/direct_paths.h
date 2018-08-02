
#ifndef DIRECT_PATH_H
#define DIRECT_PATH_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
// #include <pcl/point_cloud.h>
#include "normal_reaction_force/normal_reaction_force.h"

// namespace normal_reaction_force{
// 	typedef pcl::PointNormal PointN;
// 	typedef pcl::PointCloud<PointN> pcNormal;
// 	typedef pcNormal::Ptr pcNormalPtr; // インナークラスは前方宣言できない
// 	class VectorField;
// }

namespace path_prediction{

	namespace nrf = normal_reaction_force;
	namespace vmsgs = visualization_msgs;

	class PathPredictor;

	class PathsDirector{
		public:
		PathsDirector();
		~PathsDirector();
		void setObstacles(const vmsgs::MarkerArray::ConstPtr&, const nrf::pcNormalPtr&);
		void predict(const vmsgs::MarkerArray::ConstPtr&, const nrf::pcNormalPtr&);
		void publish();

		private:
		void createObstacles(const vmsgs::MarkerArray::ConstPtr&);

		double step_num; // 何ステップ先まで計算するか [回]
		ros::NodeHandle n;
		ros::Publisher trajectory_publisher;
		std::map<int, PathPredictor> paths; // 走査遅いからmap使うのよくない?
		nrf::VectorField vf; // 実体は前方宣言では呼べない
		vmsgs::MarkerArray lines;
		vmsgs::Marker line;
		// nrf::pcNormalPtr obstacles;
	};

} // namespace path_prediction

#endif // DIRECT_PATH_H

