
#include "path_prediction/path_prediction.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_path_prediction");

	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	path_prediction::PathPredictor predictor;

	Eigen::Vector2d init_position;
	Eigen::Vector2d init_velocity;
	Eigen::Vector2d position;
	Eigen::Vector2d velocity;

	init_position.x() = 0.0;
	init_position.y() = 0.0;

	init_velocity.x() = 1.2;
	init_velocity.y() = 1.2;

	position = init_position;

	while(ros::ok()){
		// position = init_position;
		position.x() = init_position.x();
		position.y() += 0.001;
		velocity = init_velocity;
		for(int cluster = 0; cluster < 5; ++cluster){
			predictor.predict(position, velocity);
			for(int i = 0; i < 5; ++i){
				position = predictor.predict(velocity);
				velocity.x() += 0.05;
				velocity.y() -= 0.02;
			}
			position.y() -= 0.28;
		}
		predictor.publish();

		// ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

