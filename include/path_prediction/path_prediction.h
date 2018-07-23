
#ifndef PATH_PREDICTION_H
#define PATH_PREDICTION_H

#include <ros/ros.h>
#include <Eigen/Core>

namespace path_prediction{

	struct NormalDistribution{
		Eigen::Vector2d mu;
		Eigen::Matrix4d sigma;
	};

	class PathPredictor{
		public:
		PathPredictor();
		~PathPredictor();
		Eigen::Vector2d predict(const Eigen::Vector2d&, const Eigen::Vector2d&);
		Eigen::Vector2d predict(const Eigen::Vector2d&);

		private:
		void goalEstimator(const Eigen::Vector2d&);

		Eigen::Vector2d position;
		double step_size; // [s]
		double sigma_init; // [s]
		NormalDistribution goal;
		bool emerged;
		bool observed;
	};
} // namespace path_prediction

#endif // PATH_PREDICTION_H

