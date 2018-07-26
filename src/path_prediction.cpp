
//
// goal方位の推定とpathの生成
//

#include "path_prediction/path_prediction.h"

namespace path_prediction{

	PathPredictor::PathPredictor()
		: observed(false), emerged(false)
	{
		ros::param::param<double>
			("/path_prediction/step_size", step_size, 0.1); // 積分時のΔx
			// ("/path_prediction/step_size", step_size, 0.05); // 積分時のΔx

		ros::param::param<double>
			("/path_prediction/sigma_init", sigma_init, 1.0);// 3次元配列??
	}

	PathPredictor::~PathPredictor() {}

	Eigen::Vector2d PathPredictor::predict(const Eigen::Vector2d& init_position,
										   const Eigen::Vector2d& velocity)
	{
		observed = true;
		// goalEstimator(velocity);

		position = init_position;
		predict(velocity);

		return position;
	}

	Eigen::Vector2d PathPredictor::predict(const Eigen::Vector2d& velocity)
	{
		position.x() += velocity.x() * step_size;
		position.y() += velocity.y() * step_size;

		return position;
	}

	// private
	void PathPredictor::goalEstimator(const Eigen::Vector2d& velocity)
	{
		if(!emerged){// 最初の観測なら
			goal.mu << velocity.x(), velocity.y();
			goal.sigma << 1.0, 1.0,
						  1.0, 1.0; // paramから与える
			emerged = true;

			return;
		}

		// 事後確率の計算
		goal.mu << velocity.x(), velocity.y();
		goal.sigma << 1.0, 1.0,
					  1.0, 1.0;

	}

} // namespace path_prediction

