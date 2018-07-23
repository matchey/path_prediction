
// path predictionを試す
// 開発途中のデバッグ用

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "normal_reaction_force/normal_reaction_force.h"
#include "path_prediction/path_prediction.h"

using namespace std;
// using std::cout;
// using std::endl;
// using std::string;
// using std::vector;

class TestPredictor{
	public:
	TestPredictor();
	void process();

	private:
	void humanCallback(const visualization_msgs::MarkerArray::ConstPtr&);
	void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr&);

	ros::NodeHandle n;
	ros::Subscriber sub_human;
	ros::Subscriber sub_obs;
	// pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc;
	pcl::PointCloud<pcl::PointNormal>::Ptr pc;
	normal_reaction_force::VectorField vf;
	path_prediction::PathPredictor predictor;
	vector<normal_reaction_force::State4d> state_humans;
};

TestPredictor::TestPredictor()
	: pc(new pcl::PointCloud<pcl::PointNormal>)
{
	sub_human = n.subscribe<visualization_msgs::MarkerArray>
		                   ("/velocity_arrows", 1, &TestPredictor::humanCallback, this);

	sub_obs = n.subscribe<sensor_msgs::PointCloud2>
		                   ("/rm_ground", 1, &TestPredictor::obstacleCallback, this);
}

void TestPredictor::process()
{
	// cout << "in process" << endl;
	Eigen::Vector2d velocity;
	for(auto it = state_humans.begin(); it != state_humans.end(); ++it){
		vf.velocityConversion(*it, velocity);
		it->position = predictor.predict(it->position, velocity);
		it->velocity = velocity;

		// cout << "\nvelocity before : " << velocity << endl;
		for(int step = 0; step < 40; ++step){
			vf.velocityConversion(*it, velocity);
			it->position = predictor.predict(velocity);
			it->velocity = velocity;
		}
		// cout << "velocity after : " << velocity << endl;
	}
	state_humans.clear();
	predictor.publish();
}

void TestPredictor::humanCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	// cout << "in human cb" << endl;
	state_humans.clear();
	for(auto it = msg->markers.begin(); it != msg->markers.end(); ++it){

		double yaw = atan2(2*(it->pose.orientation.w * it->pose.orientation.z
					        + it->pose.orientation.x * it->pose.orientation.y),
				       1 - 2*(pow(it->pose.orientation.y, 2) + pow(it->pose.orientation.z, 2)));

		normal_reaction_force::State4d own = {{it->pose.position.x, it->pose.position.y},
										      {it->scale.x * cos(yaw), it->scale.x * sin(yaw)}};
		state_humans.push_back(own);
	}
}

void TestPredictor::obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	// cout << "in obs cb" << endl;
	pcl::fromROSMsg(*msg, *pc);
	vf.setObstacles(pc);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_human_path_prediction");

	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	TestPredictor tp;

	while(ros::ok()){
		tp.process();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

