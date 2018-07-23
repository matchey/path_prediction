
#include <pcl/io/pcd_io.h>
#include "normal_reaction_force/normal_reaction_force.h"
#include "path_prediction/path_prediction.h"

using namespace std;
using std::cout;
using std::endl;

class TestPredictor{
	public:
	TestPredictor();
	void process();

	private:
	void load_pcd();

	// pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc;
	pcl::PointCloud<pcl::PointNormal>::Ptr pc;
	normal_reaction_force::VectorField vf;
	path_prediction::PathPredictor predictor;
	normal_reaction_force::State4d own;
	Eigen::Vector2d init_position;
	Eigen::Vector2d init_velocity;
};

TestPredictor::TestPredictor()
	// : pc(new pcl::PointCloud<pcl::PointXYZINormal>)
	: pc(new pcl::PointCloud<pcl::PointNormal>)
{
	// init_position.x() = 3.0;
	// init_position.y() = 0.0;
	init_position.x() = -4.0;
	init_position.y() = -2.0;

	init_velocity.x() = 0.5;
	init_velocity.y() = 1.5;

	load_pcd();
}

void TestPredictor::load_pcd()
{
	// string filename = "cloud_49.pcd";
	string filename = "obs.pcd";

	if( pcl::io::loadPCDFile<pcl::PointNormal>(filename, *pc) == -1 ){
		cout << "load error" << endl;
		exit(1);
	}

	vf.setObstacles(pc);
}

void TestPredictor::process()
{
	Eigen::Vector2d position = init_position;
	Eigen::Vector2d velocity = init_velocity;
	
	velocity *= -1;
	own.position = position;
	own.velocity = velocity;

	vf.velocityConversion(own, velocity);
	own.position = predictor.predict(position, velocity);
	own.velocity = velocity;

	for(int step = 0; step < 60; ++step){
		vf.velocityConversion(own, velocity);
		own.position = predictor.predict(velocity);
		own.velocity = velocity;
	}
	predictor.publish();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_path_prediction");

	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	TestPredictor tp;

	while(ros::ok()){
		tp.process();
		loop_rate.sleep();
	}
	
	return 0;
}

