
// pathの予測結果と対応する時刻のpathをcsvに書き出す
// 評価実験用

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <Eigen/Core>

using std::cout;
using std::endl;
using std::string;
// using std::vector;

class PathsWriter{
	// typedef std::vector<visualization_msgs::MarkerArray> Path_t;
	// typedef std::map< int, std::vector<Eigen::Vector2d> > Paths;

	public:
	PathsWriter();

	private:
	void humanCallback(const visualization_msgs::MarkerArray::ConstPtr&);
	void predictedPathCallback(const visualization_msgs::MarkerArray::ConstPtr&);
	void write_csv_pr();
	void write_csv_gt(const int&);
	void set_trajectory_gt();

	ros::NodeHandle n;
	ros::Subscriber sub_human;
	ros::Subscriber sub_paths;
	// std::map<int, Path_t> trajectory_gt; // ground_truth
	visualization_msgs::MarkerArray paths;
	ros::Time start_time;
	bool is_begin;

	double step_size; // 何ステップ先まで計算するか [回]
	visualization_msgs::MarkerArray trajectories;
	std::map< int, std::vector<Eigen::Vector2d> > trajectory_gt; // <id, trajectory>
	std::map< int, std::vector<string> > times_gt;
	std::map< int, bool > observed_gt;
};

PathsWriter::PathsWriter()
	: is_begin(false)
{
	// start_time = ros::Time::now();
	ros::param::param<double>
		("/path_prediction/step_size", step_size, 40);

	sub_human = n.subscribe<visualization_msgs::MarkerArray>
		                   ("/velocity_arrows", 1, &PathsWriter::humanCallback, this);

	sub_paths = n.subscribe<visualization_msgs::MarkerArray>
		                   ("trajectory_predicted", 1, &PathsWriter::predictedPathCallback, this);
}

void PathsWriter::humanCallback(const visualization_msgs::MarkerArray::ConstPtr& msgs)
{
	trajectories = *msgs;
}

void PathsWriter::predictedPathCallback(const visualization_msgs::MarkerArray::ConstPtr& msgs)
{
	// 10Hzで40step (4秒間)
	// for(auto it = msg->markers.begin(); it != msg->markers.end(); ++it){
	// }
	paths = *msgs;
	write_csv_pr();
	set_trajectory_gt();
}

void PathsWriter::write_csv_pr()
{
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

		if(system(sys_cmd.c_str())){/*戻り値何かに使える??*/}

		filename = "pr/" + dirname + "/" + filename + ".csv";
		std::ofstream ofs(filename);

		for(auto points = it->points.begin(); points != it->points.end(); ++points){
			ofs << points->x << "," << points->y << endl;
		}
		ofs.close();
	}
}

void PathsWriter::write_csv_gt(const int& id)
{
	int loop_count = times_gt[id].size() - (step_size + 1);

	string dirname = std::to_string(id);
	string sys_cmd = "mkdir -p gt/" + dirname;
	if(system(sys_cmd.c_str())){/*戻り値何かに使える??*/}

	for(int i = 0; i < loop_count; ++i){
		string filename = times_gt[id][i];
		filename = "gt/" + dirname + "/" + filename + ".csv";
		std::ofstream ofs(filename);
		for(int step = 0; step < step_size + 1; ++step){
			ofs << trajectory_gt[id][i + step].x() << "," << trajectory_gt[id][i + step].y() << endl;
		}
		ofs.close();
	}
}

void PathsWriter::set_trajectory_gt()
{
	if(!is_begin){
		start_time = ros::Time::now();
		is_begin = true;
	}

	double d;
	int i;
	std::stringstream ss;
	ss << (ros::Time::now() - start_time).toSec();
	ss >> i >> d;
	string filename = std::to_string(i) + "_" + std::to_string(d).substr(2, 4);

	for(auto it = trajectories.markers.begin(); it != trajectories.markers.end(); ++it){
		observed_gt[it->id] = true;
		times_gt[it->id].push_back(filename);
		Eigen::Vector2d position(it->pose.position.x, it->pose.position.y);
		trajectory_gt[it->id].push_back(position);
	}

	auto it_tr = trajectory_gt.begin();
	auto it_ti = times_gt.begin();
	auto it_ob = observed_gt.begin();
	while(it_ob != observed_gt.end()){
		if(!it_ob->second){
			write_csv_gt(it_ob->first);
			trajectory_gt.erase(it_tr++);
			times_gt.erase(it_ti++);
			observed_gt.erase(it_ob++);
		}else{
			it_ob->second = false;
			++it_tr;
			++it_ti;
			++it_ob;
		}
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

