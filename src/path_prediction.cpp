
//
// goal方位の推定とpathの生成
//

#include <ros/ros.h>
#include "path_prediction/path_prediction.h"

namespace path_prediction{

	PathPredictor::PathPredictor()
		: observed(false), emerged(0)
	{
		ros::param::param<double>
			("/path_prediction/dt", dt, 0.2); // 積分時のΔx[s]
			// ("/path_prediction/dt", dt, 0.05); // 積分時のΔx

		ros::param::param<double>
			("/path_prediction/sigma_init", sigma_init, 1.0);// 3次元配列??
	}

	PathPredictor::~PathPredictor() {}

	Eigen::Vector2d PathPredictor::predict(const Eigen::Vector2d& init_position,
										   const Eigen::Vector2d& velocity)
	{
		observed = true;
		goalEstimator(velocity);

		position = init_position;
		predict(velocity);

		return position;
	}

	Eigen::Vector2d PathPredictor::predict(const Eigen::Vector2d& velocity)
	{
		// position.x() += velocity.x() * dt;
		// position.y() += velocity.y() * dt;

		// position += dt * goal.mu;
		position += velocity * dt;

		return position;
	}

	void PathPredictor::getGoal(Eigen::Vector2d& velocity)
	{
		if(emerged > 10){
			velocity = goal.mu;
		}
	}

	// private
	void PathPredictor::goalEstimator(const Eigen::Vector2d& velocity)
	{
		if(!emerged){// 最初の観測なら
			goal.mu = velocity;
			goal.sigma <<  0.1, -0.5,
						  -0.5,  0.1; // paramから与える
			++emerged;

			return;
		}

		Eigen::Matrix2d tau;
		tau << 0.5, 0.0,
		       0.0, 0.5;

		Eigen::Matrix2d k = (goal.sigma.inverse() + tau.inverse()).inverse();

		// goal.mu = velocity;
		// 事後確率の計算
		goal.mu = k * (goal.sigma.inverse() * velocity + tau.inverse() * goal.mu);
		goal.sigma = k;
		// goal.sigma << 0.0, 1.0,
		// 			  1.0, 0.0; // x, yを独立として計算 (x^2 + y^2 = 1 なのに..??)
		++emerged;
	}

} // namespace path_prediction

