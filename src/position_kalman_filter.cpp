#include "math.h"
#include "matrix/Matrix.hpp"
#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>

#define PI 3.14159265

struct robot_pose
{
	float x;
	float y;
	float theta;
};

static int n_x = 5;
bool _pose_valid;

robot_pose pose_last2;
robot_pose pose_last;
robot_pose pose;

matrix::Matrix<float, 5, 5> _P;



class position_estimator
{
public:
	position_estimator();

//	bool is_valid() {return _pos_valid;}
	void get_theta(const robot_pose pose_last2, const robot_pose pose_last, robot_pose pose);
	void kalman_filter(float dt);
private:
	ros::NodeHandle n;
    ros::Publisher pose_pub;
    ros::Subscriber pose_sub;
    std::string frame_id;

    geometry_msgs::Pose2D pose_filtered;

    void poseCallBack(const geometry_msgs::Pose2D::ConstPtr& pos);

	float time;
	float time_last;
};

position_estimator::position_estimator()
{
    n.param<std::string>("frame_id", frame_id, "position_estimator");
    pose_pub = n.advertise<geometry_msgs::Pose2D>("/pose_filtered", 5);
	pose_sub = n.subscribe<geometry_msgs::Pose2D>("/robot_position", 5, &position_estimator::poseCallBack, this);
}

//存入摄像头获得的机器人位置，保留最近三次位置数据，计算时间间隔
void position_estimator::poseCallBack(const geometry_msgs::Pose2D::ConstPtr& pos)
{
	pose_last2.x = pose_last.x;
	pose_last2.y = pose_last.y;
	pose_last2.theta = pose_last.theta;
	pose_last.x = pose.x;
	pose_last.y = pose.y;
	pose_last.theta = pose.theta;
	pose.x = pos->x;
	pose.y = pos->y;
	position_estimator::get_theta(pose_last2, pose_last, pose);
	time_last = time;
	time = ros::Time::now().toSec();
	float dt = time - time_last;
	position_estimator::kalman_filter(dt);
}

//计算4×4矩阵的逆，我用的matrix包里面没有求逆的。。。。eigen里如果有就把它换掉吧
void inverse(matrix::Matrix<float, 4, 4> I)
{
	float det = I(0, 0) * I(1, 1) + I(2, 2) + I(3, 3) + I(0, 0) * I(1, 2) * I(2, 3) * I(3, 1) + I(0, 0) * I(1, 3) * I(2, 1) * I(3, 2) + I(0, 1) * I(1, 0) * I(2, 3) * I(3, 2) + I(0, 1) * I(1, 2) * I(2, 0) * I(3, 3) + I(0, 1) * I(1, 3) * I(2, 2) * I(3, 0) + I(0, 2) * I(1, 0) * I(2, 1) * I(3, 3) + I(0, 2) * I(1, 1) * I(2, 3) * I(3, 0) + I(0, 2) * I(1, 3) * I(2, 0) * I(3, 1) + I(0, 3) * I(1, 0) * I(2, 2) * I(3, 1) + I(0, 3) * I(1, 1) * I(2, 0) * I(3, 2) + I(0, 3) * I(1, 2) * I(2, 1) * I(3, 0) - I(0, 0) * I(1, 1) * I(2, 3) * I(3, 2) - I(0, 0) * I(1, 2) * I(2, 1) * I(3, 3) - I(0, 0) * I(1, 3) * I(2, 2) * I(3, 1) - I(0, 1) * I(1, 0) * I(2, 2) * I(3, 3) - I(0, 1) * I(1, 2) * I(2, 3) * I(3, 0) - I(0, 1) * I(1, 3) * I(2, 0) * I(3, 2) - I(0, 2) * I(1, 0) * I(2, 3) * I(3, 1) - I(0, 2) * I(1, 1) * I(2, 0) * I(3, 3) - I(0, 2) * I(1, 1) * I(2, 0) * I(3, 3) - I(0, 2) * I(1, 3) * I(2, 1) * I(3, 0) - I(0, 3) * I(1, 0) * I(2, 1) * I(3, 2) - I(0, 3) * I(1, 1) * I(2, 2) * I(3, 0) - I(0, 3) * I(1, 2) * I(2, 0) * I(3, 1);
	
	float b11 = I(1, 1) * I(2, 2) * I(3, 3) + I(1, 2) * I(2, 3) * I(3, 1) + I(1, 3) * I(2, 1) * I(3, 2) - I(1, 1) * I(2, 3) * I(3, 2) - I(1, 2) * I(2, 1) * I(3, 3) - I(1, 3) * I(2, 2) * I(3, 1);
	float b12 = I(0, 1) * I(2, 3) * I(3, 2) + I(0, 2) * I(2, 1) * I(3, 3) + I(0, 3) * I(2, 2) * I(3, 1) - I(0, 1) * I(2, 2) * I(3, 3) - I(0, 2) * I(2, 3) * I(3, 1) - I(0, 3) * I(1, 2) * I(3, 2);
	float b13 = I(0, 1) * I(1, 2) * I(3, 3) + I(0, 2) * I(1, 3) * I(3, 1) + I(0, 3) * I(1, 1) * I(3, 2) - I(0, 1) * I(1, 3) * I(3, 2) - I(0, 2) * I(1, 1) * I(3, 3) - I(0, 3) * I(1, 2) * I(3, 1);
	float b14 = I(1, 1) * I(2, 2) * I(3, 3) + I(1, 2) * I(2, 3) * I(3, 1) + I(1, 3) * I(2, 1) * I(3, 2) - I(1, 1) * I(2, 3) * I(3, 2) - I(1, 2) * I(2, 1) * I(3, 3) - I(1, 3) * I(2, 2) * I(3, 1);
	
	float b21 = I(1, 0) * I(2, 3) * I(3, 2) + I(1, 2) * I(2, 0) * I(3, 3) + I(1, 3) * I(2, 2) * I(3, 0) - I(1, 0) * I(2, 2) * I(3, 3) - I(1, 2) * I(2, 3) * I(3, 0) - I(1, 3) * I(2, 0) * I(3, 2);
	float b22 = I(0, 0) * I(2, 2) * I(3, 3) + I(0, 2) * I(2, 3) * I(3, 0) + I(0, 3) * I(2, 0) * I(3, 2) - I(0, 0) * I(2, 3) * I(3, 2) - I(0, 2) * I(2, 0) * I(3, 3) - I(0, 3) * I(2, 2) * I(3, 0);
	float b23 = I(0, 0) * I(1, 3) * I(3, 2) + I(0, 2) * I(1, 0) * I(3, 3) + I(0, 3) * I(1, 2) * I(3, 0) - I(0, 0) * I(1, 2) * I(3, 3) - I(0, 2) * I(1, 3) * I(3, 0) - I(0, 3) * I(1, 0) * I(3, 2);
	float b24 = I(0, 0) * I(1, 2) * I(2, 3) + I(0, 2) * I(1, 3) * I(2, 0) + I(0, 3) * I(1, 0) * I(2, 2) - I(0, 0) * I(1, 3) * I(2, 2) - I(0, 2) * I(1, 0) * I(2, 3) - I(0, 3) * I(1, 2) * I(2, 0);
	
	float b31 = I(1, 0) * I(2, 1) * I(3, 3) + I(1, 1) * I(2, 3) * I(3, 0) + I(1, 3) * I(2, 1) * I(3, 0) - I(1, 0) * I(2, 3) * I(3, 1) - I(1, 1) * I(2, 0) * I(3, 3) - I(1, 3) * I(2, 1) * I(3, 0);
	float b32 = I(0, 0) * I(2, 3) * I(3, 1) + I(0, 1) * I(2, 0) * I(3, 3) + I(0, 3) * I(2, 1) * I(3, 0) - I(0, 0) * I(2, 1) * I(3, 3) - I(0, 1) * I(2, 3) * I(3, 0) - I(0, 3) * I(2, 0) * I(3, 1);
	float b33 = I(0, 0) * I(1, 1) * I(3, 3) + I(0, 1) * I(1, 3) * I(3, 1) + I(0, 3) * I(1, 0) * I(3, 1) - I(0, 0) * I(1, 3) * I(3, 1) - I(0, 1) * I(1, 0) * I(3, 3) - I(0, 3) * I(1, 1) * I(3, 0);
	float b34 = I(0, 0) * I(1, 3) * I(2, 1) + I(0, 1) * I(1, 0) * I(2, 3) + I(0, 3) * I(1, 1) * I(2, 1) - I(0, 0) * I(1, 1) * I(2, 3) - I(0, 1) * I(1, 3) * I(2, 0) - I(0, 3) * I(1, 0) * I(2, 1);
	
	float b41 = I(1, 0) * I(2, 2) * I(3, 1) + I(1, 1) * I(2, 0) * I(3, 2) + I(1, 2) * I(2, 1) * I(3, 0) - I(1, 0) * I(2, 1) * I(3, 2) - I(1, 1) * I(2, 2) * I(3, 0) - I(1, 2) * I(2, 1) * I(3, 1);
	float b42 = I(0, 0) * I(2, 1) * I(3, 2) + I(0, 1) * I(2, 2) * I(3, 0) + I(0, 2) * I(2, 0) * I(3, 1) - I(0, 0) * I(2, 2) * I(3, 1) - I(0, 1) * I(2, 0) * I(3, 2) - I(0, 2) * I(2, 1) * I(3, 0);
	float b43 = I(0, 0) * I(1, 2) * I(3, 1) + I(0, 1) * I(1, 0) * I(3, 2) + I(0, 2) * I(1, 1) * I(3, 1) - I(0, 0) * I(1, 1) * I(3, 2) - I(0, 1) * I(1, 2) * I(3, 0) - I(0, 2) * I(1, 0) * I(3, 1);
	float b44 = I(0, 0) * I(1, 1) * I(2, 2) + I(0, 1) * I(1, 2) * I(2, 0) + I(0, 2) * I(1, 0) * I(2, 1) - I(0, 0) * I(1, 2) * I(2, 1) - I(0, 1) * I(1, 0) * I(2, 2) - I(0, 2) * I(1, 1) * I(2, 0);
	
	I(0, 0) = b11 / det;
	I(0, 1) = b12 / det;
	I(0, 2) = b13 / det;
	I(0, 3) = b14 / det;
	
	I(1, 0) = b21 / det;
	I(1, 1) = b22 / det;
	I(1, 2) = b23 / det;
	I(1, 3) = b24 / det;
	
	I(2, 0) = b31 / det;
	I(2, 1) = b32 / det;
	I(2, 2) = b33 / det;
	I(2, 3) = b34 / det;
	
	I(3, 0) = b41 / det;
	I(3, 1) = b42 / det;
	I(3, 2) = b43 / det;
	I(3, 3) = b44 / det;
}

//用最近三次的机器人位置计算前进方向（0~2π），如果两次位置变化太小则认为机器人在原地旋转，角度设定为-1
void position_estimator::get_theta(const robot_pose pose_last2, const robot_pose pose_last, robot_pose pose)
{
	float dist = sqrt((pose.x - pose_last.x) * (pose.x - pose_last.x) + (pose.y - pose_last.y) * (pose.y - pose_last.y));
	if (dist < 0.02f)
	{
		pose.theta = -1.0f;
		pose.x = pose_last.x;
		pose.y = pose_last.y;
		return;
	}

	float x_mean = (pose_last2.x + pose_last.x + pose.x) / 3;
	float y_mean = (pose_last2.y + pose_last.y + pose.y) / 3;
	float k = ((pose_last2.x - x_mean)*(pose_last2.y - y_mean) + (pose_last.x - x_mean)*(pose_last.y - y_mean) + (pose.x - x_mean)*(pose.y - y_mean)) / ((pose_last2.x - x_mean)*(pose_last2.x - x_mean) + (pose_last.x - x_mean)*(pose_last.x - x_mean) + (pose.x - x_mean)*(pose.x - x_mean));
	if (pose.x > pose_last.x)
	{
		pose.theta = atan(k);
		if (pose.theta < 0)
		{
			pose.theta += 3 * PI / 2;
		}
		return;
	}
	else
	{
		pose.theta = atan(k) + PI;
		return;
	}
}

//机器人位置的卡尔曼滤波，假定速度为0.4m/s始终不变
void position_estimator::kalman_filter(float dt)
{
	matrix::Vector<float, 5> state;
	state(0) = pose.x;
	state(1) = 0.4 * cos(pose.theta);
	state(2) = pose.y;
	state(3) = 0.4 * sin(pose.theta);
	state(4) = pose.theta;

	matrix::Matrix<float, 5, 5> A;
	A.setIdentity();
	A(0, 1) = dt;
	A(2, 3) = dt;
	A(4, 5) = dt;

	matrix::Matrix<float, 4, 5> C;
	C(0, 0) = 1;
	C(1, 1) = 1;
	C(2, 2) = 1;
	C(3, 3) = 1;

	//这两个是噪声协方差矩阵，参数需要调
	matrix::Matrix<float, 5, 5> Sm;
	Sm.setZero();
	matrix::Matrix<float, 4, 4> So;
	So.setZero();

	matrix::Vector<float, 4> z;
	z = C * state;
	matrix::Matrix<float, 5, 4> K;
	matrix::Matrix<float, 4, 4> I;
	I = So + C * _P * C.transpose();
	inverse(I);
	K = _P * C.transpose() * I;

	state = A * state + K * (z - C * A * state);

	_P = A * _P * A.transpose() + Sm;


	pose.x = state(0);
	pose_filtered.x = state(0);
	pose.y = state(2);
	pose_filtered.y = state(2);
	pose.theta = state(4);
	pose_filtered.theta = state(4);

	//发布的位置信息是Pose2D消息，包括x,y,theta
	pose_pub.publish(pose_filtered);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Kalman filter for position esitmator");
	_P.setIdentity();
	position_estimator position_estimator;
	ros::spin();
}
