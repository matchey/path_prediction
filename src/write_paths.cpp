
// pathの予測結果と対応する時刻のpathをcsvに書き出す
// 評価実験用

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>

using std::cout;
using std::endl;
using std::string;
// using std::vector;

class PathsWriter{
	// typedef std::vector<visualization_msgs::MarkerArray> Path_t;

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
	// std::map<int, Path_t> trajectory_gt; // ground_truth
	visualization_msgs::MarkerArray paths;
	ros::Time start_time;
	bool is_begin;
};

PathsWriter::PathsWriter()
	: is_begin(false)
{
	// start_time = ros::Time::now();

	sub_human = n.subscribe<visualization_msgs::MarkerArray>
		                   ("/velocity_arrows", 1, &PathsWriter::humanCallback, this);

	sub_paths = n.subscribe<visualization_msgs::MarkerArray>
		                   ("trajectory_predicted", 1, &PathsWriter::predictedPathCallback, this);
}

void PathsWriter::process()
{
}

void PathsWriter::humanCallback(const visualization_msgs::MarkerArray::ConstPtr& msgs)
{
}

void PathsWriter::predictedPathCallback(const visualization_msgs::MarkerArray::ConstPtr& msgs)
{
	// 10Hzで40step (4秒間)
	// for(auto it = msg->markers.begin(); it != msg->markers.end(); ++it){
	// }
	paths = *msgs;
	write_csv();
}

void PathsWriter::write_csv()
{
	// const string filename = "predected_paths.csv";
	// string filename;
	// std::ofstream ofs(filename); // 追記にしないと最後のしか残らない->closeしなければいい
	if(!is_begin){
		start_time = ros::Time::now();
		is_begin = true;
	}

	double d;
	int i;
	std::stringstream ss;
	ss << (ros::Time::now() - start_time).toSec();
	ss >> i >> d;
	// cout << "int:    " << i << endl;
	// cout << "double: " << d << endl;
	// cout << "ss:     " << ss.str() << endl;
	string filename = std::to_string(i) + "_" + std::to_string(d).substr(2, 4);
	
	for(auto it = paths.markers.begin(); it != paths.markers.end(); ++it){
		string dirname = std::to_string(it->id);
		string sys_cmd = "mkdir -p pr/" + dirname;
		system(sys_cmd.c_str());
		filename = "pr/" + dirname + "/" + filename + ".csv";
		std::ofstream ofs(filename);
		// ofs << "id," << it->id << endl;
		// for(auto points = it->points.begin(); points != it->points.end(); ++points){
		// 	ofs << points->x << ", ";
		// }
		// ofs << endl;
		// for(auto points = it->points.begin(); points != it->points.end(); ++points){
		// 	ofs << points->y << ", ";
		// }
		for(auto points = it->points.begin(); points != it->points.end(); ++points){
			ofs << points->x << "," << points->y << endl;
		}
		ofs.close();
	}
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

