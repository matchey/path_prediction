
#include "path_prediction/path_prediction.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_path_prediction");

	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	path_prediction::PathPredictor predictor;

	Eigen::Vector2d init;
	Eigen::Vector2d v;
	std::vector<Eigen::Vector2d> vec;

	init.x() = 0.2;
	init.y() = 0.2;

	v = init;

	while(ros::ok()){
		for(int i = 0; i < 50; ++i){
			vec.push_back(v);
			v.x() += 0.05;
		}
		for(int i = 0; i < 5; ++i){
			predictor.predict(v, vec);
			v.x() -= 0.51;
			v.y() -= 0.51;
		}
		predictor.publish();
		vec.clear();

		// ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

