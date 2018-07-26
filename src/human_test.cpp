
// path predictionを試す
// 開発途中のデバッグ用

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "path_prediction/direct_paths.h"

// using namespace std;
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
	pcl::PointCloud<pcl::PointNormal>::Ptr pc;

	path_prediction::PathsDirector paths;
	bool isObstacle;
	bool isHuman;
};

TestPredictor::TestPredictor()
	: pc(new pcl::PointCloud<pcl::PointNormal>), isObstacle(false), isHuman(false)
{
	sub_human = n.subscribe<visualization_msgs::MarkerArray>
		                   ("/velocity_arrows", 1, &TestPredictor::humanCallback, this);

	sub_obs = n.subscribe<sensor_msgs::PointCloud2>
		                   ("/rm_ground", 1, &TestPredictor::obstacleCallback, this);
}

void TestPredictor::process()
{
	if(isHuman){
		paths.publish();
		// cout << "in process" << endl;
	}
	isObstacle = isHuman = false;
}

void TestPredictor::humanCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	// paths.predict(msg);
	if(isObstacle){
		paths.predict(msg, pc);
		isHuman = true;
		// cout << "in human" << endl;
	}
}

void TestPredictor::obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	// cout << "in obs cb" << endl;
	pcl::fromROSMsg(*msg, *pc);
	// vf.setObstacles(pc);
	isObstacle = true;
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

