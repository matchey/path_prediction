
#ifndef PATH_PREDICTION_H
#define PATH_PREDICTION_H

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
		Eigen::Vector2d predict();
		void getGoal(Eigen::Vector2d&); // 推定された目標方位を返す

		bool observed; // predictされたか

		private:
		void goalEstimator(const Eigen::Vector2d&); // 目標方位の更新

		Eigen::Vector2d position; // 現在位置
		double dt; // 積分時のΔt[s]
		double sigma_init; // 目標方位分布の初期分散
		NormalDistribution goal; // 目標方位
		int emerged; // 何回観測されたか
	};
} // namespace path_prediction

#endif // PATH_PREDICTION_H

