
#ifndef PATH_PREDICTION_H
#define PATH_PREDICTION_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>

namespace path_prediction{

	struct NormalDistribution{
		Eigen::Vector2d mu;
		Eigen::Matrix2d sigma;
	};

	class PathPredictor{
		public:
		PathPredictor();
		~PathPredictor();
		Eigen::Vector2d predict(const Eigen::Vector2d&, const Eigen::Vector2d&);
		Eigen::Vector2d predict(const Eigen::Vector2d&);
		Eigen::Vector2d predict();
		void getGoal(Eigen::Vector2d&);

		bool observed;

		private:
		void goalEstimator(const Eigen::Vector2d&);

		Eigen::Vector2d position;
		double step_size; // [s]
		double sigma_init; // [s]
		NormalDistribution goal;
		bool emerged;
	};
} // namespace path_prediction

#endif // PATH_PREDICTION_H

