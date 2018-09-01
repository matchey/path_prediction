
// pathの予測結果と対応する時刻のpathをcsvに書き出す
// 評価実験用

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>

// using std::cout;
using std::endl;
using std::string;
// using std::vector;

class PathsWriter{
	typedef std::vector<visualization_msgs::MarkerArray> Path_t;

	public:
	PathsWriter();
	void process();

	private:
	void humanCallback(const visualization_msgs::MarkerArray::ConstPtr&);
	void predictedPathCallback(const visualization_msgs::MarkerArray::ConstPtr&);
	void write_csv();

	ros::NodeHandle n;
	ros::Subscriber sub_human;
	ros::Subscriber sub_paths;
	std::map<int, Path_t> paths;
};

PathsWriter::PathsWriter()
{
	sub_human = n.subscribe<visualization_msgs::MarkerArray>
		                   ("/velocity_arrows", 1, &PathsWriter::humanCallback, this);

	sub_paths = n.subscribe<visualization_msgs::MarkerArray>
		                   ("trajectory_predicted", 1, &PathsWriter::predictedPathCallback, this);
}

void PathsWriter::process()
{
}

void PathsWriter::humanCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
}

void PathsWriter::predictedPathCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	// 10Hzで40step (4秒間)
}

void write_csv()
{
	const string filename = "predected_paths.csv";
	std::ofstream ofs(filename);

	// for(){
	// 	ofs << 
	// }
	ofs << endl;

	ofs.close();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "predicted_path_writer");

	ros::NodeHandle n;
	// ros::Rate loop_rate(10);

	PathsWriter pw;
	ros::spin();

	// while(ros::ok()){
	// 	pw.process();
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
	
	return 0;
}

