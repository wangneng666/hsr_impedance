#include "Impedance.h"
#include <iostream>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>	
#include <industrial_msgs/RobotStatus.h>
#include <chrono>
#include <thread>
#define DEBUG

Impedance CartesianImpedance;		// 算法实例
std::vector<double> force, bias_force;		// 保存传感器数据
std::vector<double> lastJoint(6);	// 保存上一次的关节角度
std::vector<double> startPos(6);	// 保存起始关节角数据

std::string end_link = "link6";
std::string group;

boost::atomic<bool> startPosOK, robotStatus;
bool isSim = true;
bool robot_servo_status = true;

bool getForce = false;
robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* joint_model_group;

#define F_SCALE 0.2
#define T_SCALE 5
#define Z_ERR	14.5 * F_SCALE

ros::Publisher joint_state_pub; //joint listen

void dumpDVec(const std::vector<double> & vec,int size, const std::string & name){

    ROS_ERROR_STREAM(name.c_str() <<" "<< vec.at(0)<<" "<< vec.at(1)<<" "<< vec.at(2)<<" "<< vec.at(3)<<" "<< vec.at(4)<<" "<< vec.at(5)<<" ");

}

void sensor_msgs_sysCallback(const geometry_msgs::WrenchStampedConstPtr& msg1, const sensor_msgs::JointState::ConstPtr& msg2){
    //Wrench
    force[0] = msg1->wrench.force.x * 0.2 - bias_force[0];
    force[1] = msg1->wrench.force.y * 0.2 - bias_force[1];
    force[2] = msg1->wrench.force.z * 0.2 - bias_force[2];

    force[3] = msg1->wrench.torque.x * 4 - bias_force[3];
    force[3] = msg1->wrench.torque.y * 4 - bias_force[4];
    force[3] = msg1->wrench.torque.z * 4 - bias_force[5];

    //JointState

}
//message_filters::Subscriber<sensor_msgs::JointState> infoSub(nh, "/joint_states", 5);
//message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub(nh, "/dap_data", 5);

//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, geometry_msgs::WrenchStamped> MySyncPolicy;
//message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), infoSub, wrench_sub);

//sync.registerCallback(boost::bind(&call_back, _1, _2));


void foruceCallback(const geometry_msgs::Wrench::ConstPtr& msg){

//    force[0] =  0;
//    force[1] =  0;
//    force[2] = 0;

//    force[0] = msg->force.x * 0.2 - bias_force[0];
//    force[1] = -msg->force.y * 0.2 - bias_force[1];
//    force[2] = -msg->force.z * 0.2 - bias_force[2]; // -msg->force.z * 0.2 - bias_force[2]

    force[0] = msg->force.x * 0.1 - bias_force[0];
    force[1] = -msg->force.y * 0.1 - bias_force[1];
    force[2] = -msg->force.z * 0.06 - bias_force[2]; // -msg->force.z * 0.2 - bias_force[2]
	
    force[3] = 0;
    force[4] = 0;
    force[5] = 0;

//    force[3] = -msg->torque.x * 4 - bias_force[3];
//    force[4] = msg->torque.y * 4 - bias_force[4];
//    force[5] = msg->torque.z * 4 - bias_force[5];

    getForce = true;

}


void startPosCallback(const sensor_msgs::JointState::ConstPtr& msg){


   for(int i = 0; i < 6; i++)
	 startPos[i] = msg->position[i];

	if(isSim){
			startPos[0] = 0; startPos[1] = -1.5707963331877837;     startPos[2] = 3.14;
			startPos[3] = 0;    startPos[4] = 1.570785397386886;     startPos[5] = 0;
	}

   if(startPosOK == false) {

        startPos.push_back(0);       
        startPosOK = true;
		for(int i = 0;  i< 6; i++){
			bias_force[i] = force[i];
		}
        //startPos = std::vector<double>(6);
    }


}


void robotStausCallback(const industrial_msgs::RobotStatusConstPtr& msg){

	if((!robotStatus ) || isSim )
	{		
		if( msg->in_motion.val == 0)
		{	robotStatus = true;
        	robot_servo_status = true;
		}	
	}

	if(msg->in_error.val != 0 || msg->in_motion.val == 0){
		robot_servo_status = false;
		std::cout << "msg->in_motion.val : "<< std::to_string(msg->in_motion.val)<<std::endl;
	}
	//msg->

}

/***********************算法初始化**********************************/
void initImped(){

//    double Mass[6] = { 1., 1.,1., 1., 1., 1. };
    double Mass[6] = { 1.6, 1.6, 3.2, 0.4, 0.3, 0.2 }; //M
//    double Damping[6] = { 5., 5., 5., 5., 5., 5. };	 //B
    double Damping[6] = { 4.8, 4.8, 3.6, 1.6, 1.2, 0.6 };	 //B

    double Stiffness[6] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 }; //K
//    double Stiffness[6] = { 0.0,0.0, 0.0, 0.0, 0.0, 0.0 };
	force = std::vector<double>(6);
	//force[2] = -15;
	CartesianImpedance = Impedance();
    CartesianImpedance.setMass(Mass);
	CartesianImpedance.setStiffness(Stiffness); //K
	CartesianImpedance.setDamping(Damping); //B
}

/***********************四元数转RPY**********************************/
// x y z w A B C
void QtoE(double q1, double q2, double q3, double q0, double& A, double &B, double &C){
	A = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
	B = asin(2*(q0*q2-q1*q3));
	C = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}


/***********************初始化运动学**********************************/
void initKinematic(){
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
	kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	joint_model_group = kinematic_model->getJointModelGroup(group);	
}

/***********************通过算法计算出实际关节角**********************************/
void doit(const std::vector<double> &in, std::vector<double> &out){

	double A_offset = 0, B_offset = 0, C_offset = 0, X_offset = 0,  Y_offset = 0,  Z_offset = 0;
	std::vector<double> Xa(6);
	int ERROR_ImpAcc , ERROR_ImpVel;
	//force[0] = 0; force[1] = 1; force[10] = 0; force[3] = 0; force[4] = 0; force[5] = 0;
	
	// 计算偏移值
	//force[2] += Z_ERR;
	CartesianImpedance.reImpedance(force, ERROR_ImpAcc, ERROR_ImpVel, Xa);

    X_offset = Xa[0];
    Y_offset = Xa[1];
    Z_offset = Xa[2];
//    A_offset = Xa[3];B_offset = Xa[4];C_offset = Xa[5];


	// 获取当前的末端姿态
	kinematic_state->setJointGroupPositions(joint_model_group, in);
	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(end_link);
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(end_effector_state, pose);

	dumpDVec(in, 6,"current joint pose: ");
    dumpDVec(force, 6, "current force ");
#ifdef DEBUG
	    ROS_INFO_STREAM("compute offset pose: "<< X_offset<<" "<< Y_offset<<" "<< Z_offset<<" "<< A_offset<<" "<< B_offset<<" "<< C_offset<<" ");
#endif
	// 四元数转RPY
	double A, B, C;
	QtoE(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, A, B, C);
#ifdef DEBUG
	ROS_INFO_STREAM("current effector cart pose XYZ ABC: "<< pose.position.x<<" "<< pose.position.y<<" "<< pose.position.z   <<" "
                                                   <<A<<" "<< B<<" "<<C);
    ROS_INFO_STREAM("current effector cart pose XYZ xyzw: "<< pose.position.x<<" "<< pose.position.y<<" "<< pose.position.z   <<" "
                                                       <<pose.orientation.x<<" "<< pose.orientation.y<<" "<<pose.orientation.z<<" "<<pose.orientation.w);

#endif

	A += A_offset; B += B_offset; B += B_offset;
	pose.position.x += X_offset; pose.position.y += Y_offset; pose.position.z += Z_offset;
        ROS_INFO_STREAM("compute effector cart pose XYZ ABC: "<< pose.position.x<<" "<< pose.position.y<<" "<< pose.position.z   <<" "
                                                           <<A<<" "<< B<<" "<<C);
	// 偏移后RPY转四元数
	tf::Quaternion q;
	q.setEulerZYX(C, B, A);
	tf::Vector3 axis = q.getAxis();


	pose.orientation.x = axis.getX();
	pose.orientation.y = axis.getY();
	pose.orientation.z = axis.getZ();
	pose.orientation.w = q.getW();

    ROS_INFO_STREAM("compute effector cart pose XYZ xyzw: "<< pose.position.x<<" "<< pose.position.y<<" "<< pose.position.z   <<" "
                                                       <<pose.orientation.x<<" "<< pose.orientation.y<<" "<<pose.orientation.z<<" "<<pose.orientation.w);
/***********************************/

    //    pose.position.z  -=1;
    // 四元数转关节角 joint_model_group, pose, end_link, 10, 0.1
//    std::string tips = "link6";
    if(!kinematic_state->setFromIK(joint_model_group, pose,end_link, 10, 0.1)){
		std::cout << "运动学逆解失败" << std::endl;
		out = lastJoint;
                dumpDVec(out, 6, " outPose ");
		return;
	}
/***********************************/
	// 返回计算后的关节角	
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	dumpDVec(joint_values, 6, "<----- copyJointGroupPositions : ");
	// 当不处于虚拟环境时 发送的是关节角偏移值
	//if(!isSim)
	//	for(int i = 0; i < 6; i ++)
	//		joint_values[i] -= in[i];

	out = joint_values;
	lastJoint = joint_values;

}

int main(int argc, char **argv){

	ros::init(argc, argv, "hsr_impedance");
	ros::NodeHandle n;

	ros::AsyncSpinner spinner(3);
  	spinner.start();



	startPosOK = false;
	isSim = false;

	if(argc == 2)
		if(strcmp(argv[1], "true") == 0){
			// 当处于虚拟环境时  直接使用指定初始点即可
			isSim = true;
			startPosOK = true;
			ROS_INFO("虚拟环境");	
		}else if(strcmp(argv[1], "false") != 0)
			ROS_ERROR("未知参数:Sim will set be false!!! \n");

	// 获取参数
	n.param<std::string>("impedance_end_link", end_link, "link6");
	n.param<std::string>("impedance_group", group, "arm");


	// 发布关节角数据
	if(isSim)
		joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	else
		joint_state_pub = n.advertise<sensor_msgs::JointState>("impedance_err", 1);

	// 初始化算法
	initImped();
	
	// 初始化运动学
	initKinematic();
	

	// 测试数据
	std::vector<double> out(6);
    //startPos[0] = 0; startPos[1] = -1.5707963331877837;     startPos[2] = 3.14;
    //startPos[3] = 0;    startPos[4] = 1.570785397386886;     startPos[5] = 0;

	sensor_msgs::JointState joint_state;

	// 获取关节名
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	//std::cout << "joint size" << joint_names.size() << std::endl;
	//for(int i = 0; i < joint_names.size(); i++)
	//	std::cout << "joint" << i << " = " << joint_names[i] << std::endl;	




	// 订阅机械臂起始关节角
    startPosOK = false;
	//ros::Subscriber start_pos_sub = n.subscribe("start_pos", 1000, startPosCallback);
    ros::Subscriber start_pos_sub = n.subscribe("/joint_states", 1, startPosCallback);
	// 订阅传感器数据
	ros::Subscriber pose_sub = n.subscribe("/daq_data", 1, foruceCallback);	

	std::cout << "bias_force ..." << std::endl;
	bias_force  = std::vector<double>(6);
	for(int i = 0; i< 6;i++)
	{
		bias_force[i] = 0;
	}


	ros::Rate loop_rate2(0.1);
	//等待获取初始点
    while(startPosOK == false){
		ROS_INFO_STREAM(" waiting ... ");
	}		

    if(isSim == false){
       	start_pos_sub.shutdown();
		sensor_msgs::JointState sensor_compute_robot_state;
    	sensor_compute_robot_state.header.stamp = ros::Time::now();
    	sensor_compute_robot_state.name.resize(7);

   		sensor_compute_robot_state.position = startPos;
    	joint_state_pub.publish(sensor_compute_robot_state);
        ROS_INFO_STREAM( "sensor_compute_robot_state size: " << sensor_compute_robot_state.position.size() );
    	dumpDVec(sensor_compute_robot_state.position, 7,"sensor_compute_robot_state: ");

   }


	//等待初始状态与机器人状态获取成功
    robotStatus = false;
    ROS_INFO_STREAM("ready the robot status ok ...");
	
	ros::Subscriber robot_status_sub = n.subscribe("/robot_status", 1, robotStausCallback);
 	//等待机器人状态成功
    while(robot_servo_status == false);	


    ROS_INFO_STREAM("get startPosOK...."<< startPosOK <<" "<<robotStatus);

	while(ros::ok()){
        //pose_sub.shutdown();
		//start_pos_sub.shutdown();
		auto start = std::chrono::system_clock::now();
		if(!robot_servo_status)
			break;
		doit(startPos, out);	
        //pose_sub = n.subscribe("daq_data", 1000, foruceCallback);  
  		//start_pos_sub = n.subscribe("joint_states", 1000, startPosCallback);

		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(6);
		joint_state.position = out;
		
		for(int i = 0; i < 6; i++)
			joint_state.name[i] =  joint_names[i];
		

	    joint_state_pub.publish(joint_state);
        ROS_ERROR_STREAM("<---------------------------------------------------->");
		//loop_rate.sleep(); start +
		std::this_thread::sleep_until( start +std::chrono::milliseconds(40));
		//break;
	}
    sleep(0.1);
	ROS_ERROR_STREAM("robot_servo_status error exit! ");

	// 进入消息循环
	//ros::spin();
}
