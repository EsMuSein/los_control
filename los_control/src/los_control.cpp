#include "ros/ros.h"
#include "std_msgs/String.h"
#include "los_control/los_control.h"

//template <typename T>
class los_control
{
private:
    ros::NodeHandle nh;
    bool robotArriveDstPose;       //是否到达目标点标志
    bool firstRobotStartDealDst;    //机器人开始处理目标点标志  且是第一次
    bool robotStartDealDst;         //机器人开始处理目标点标志
    bool firstRobotRotate;
    bool rotateArriveAngle;
    tf::TransformListener listen;
    tf::StampedTransform transform;

    /*PID param*/
    double Kp,Kd;
    double currentError,LastError,errorError;
    double deltaPDTime,lastTimeOfPD,currentTimeOfPD;
    geometry_msgs::TwistPtr cmd_;
    int sleepNum;

    double rotateAngleThreshold;
    double distThreshold;

    ros::Publisher pubRobotSpeed;
    ros::ServiceServer taskServer;

//    ros::Subscriber

    taskState currentState;
    taskState lastState;
    motionState currentMotion;
    goalState currentGoalState;
    rotateState currentRotateState;
    int loopNum,currentGoalIndex;

public:
    PointTypePose robotPoseWorld;
    double distAngle,robotToDistAngle,startToDistAngle,startRotateAngle;
    PointType startPoint,distPoint;
    vector<PointType> distPoints,goalPoints;
    double speedLiner,speedAngular,speedAngularStatic,speedLinerStatic;


public:
    los_control():
        nh("~")
    {
        robotArriveDstPose = true;
        firstRobotStartDealDst = true;
        robotStartDealDst = false;
        firstRobotRotate = true;
        speedLinerStatic = 0.3;
        speedAngular = 0.4;
        speedAngularStatic = 0.4;
        Kp = 0.5;
        Kd = 0;
        sleepNum = 0;
        currentGoalIndex = 0;
        rotateAngleThreshold = 3;
        distThreshold = 0.1;

        currentState = taskState::TASKNORMAL;


        cmd_.reset(new geometry_msgs::Twist ());
        
        taskServer = nh.advertiseService("los_control",&los_control::taskProcess,this);
        pubRobotSpeed = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1,true);


    }
    bool taskProcess(defineMessage::task_reseive::Request  &req,
                     defineMessage::task_reseive::Response &res){
        currentState = (taskState)req.task_mode;
        if(currentState == taskState::TASKNORMAL){
            for (int i=0;i<req.fixtack_poses.poses.size();i++) {
                PointType goalPose;
                goalPose._PointXYZI::x = req.fixtack_poses.poses[0].position.x;
                goalPose._PointXYZI::y = req.fixtack_poses.poses[0].position.y;
                goalPose._PointXYZI::z = req.fixtack_poses.poses[0].position.z;
                distPoints.push_back(goalPose);
            }
            speedLinerStatic = req.speed;
            goalPoints.resize(distPoints.size());
            goalPoints = distPoints;
        }

    }

    void moveRobotAsWorld(const double &speed_linerx ,const double &speed_angular){
        double speedL = speed_linerx;
        double speedA = speed_angular;
        if(currentMotion == MOTION_BACK){
            speedL = -speed_linerx;
            speedA = -speed_angular;
        }
        cmd_->linear.x = speedL;
        cmd_->linear.y = 0;
        cmd_->linear.z = 0;
        cmd_->angular.x = 0;
        cmd_->angular.y = 0;
        cmd_->angular.z = speedA;
        cout<<"speed_angular"<<speedA<<endl;
        pubRobotSpeed.publish(cmd_);
    }

    double getTwoPointAngleInWorld(const PointType &start, const PointType &end){
        double angle;
        if(start._PointXYZI::x < end._PointXYZI::x && start._PointXYZI::y > end._PointXYZI::y )
        {//第四象限
            angle = - round((atan2(fabs(end.y-start.y),fabs(end.x-start.x))) / PI * 180);
        }
        else if (start._PointXYZI::x < end._PointXYZI::x && start._PointXYZI::y <end._PointXYZI::y )
        {//第一象限
            angle = round((atan2(fabs(end.y-start.y),fabs(end.x-start.x))) / PI * 180);
        }
        else if (start._PointXYZI::x > end._PointXYZI::x && start._PointXYZI::y >end._PointXYZI::y )
        {//第三象限
            angle = -180 + round((atan2(fabs(end.y-start.y),fabs(end.x-start.x))) / PI * 180);
        }
        else if (start._PointXYZI::x > end._PointXYZI::x && start._PointXYZI::y <end._PointXYZI::y )
        {//第二象限
            angle = 180 - round((atan2(fabs(end.y-start.y),fabs(end.x-start.x))) / PI * 180);
        }
        else if (start._PointXYZI::x == end._PointXYZI::x &&start._PointXYZI::y < end._PointXYZI::y)
        {
            angle = 90;
        }
        else if (start._PointXYZI::x == end._PointXYZI::x && start._PointXYZI::y > end._PointXYZI::y)
        {
            angle = -90;
        }
        else if (start._PointXYZI::x > end._PointXYZI::x && start._PointXYZI::y == end._PointXYZI::y)
        {
            angle = -180;
        }
        else if (start._PointXYZI::x <= end._PointXYZI::x && start._PointXYZI::y == end._PointXYZI::y)
        {
            angle = 0;
        }
        return angle;
    }
    double getTwoPointAngleInWorld(const PointTypePose &robotPose, const PointType &end){
        PointType start;
        double angle;
        start._PointXYZI::x = robotPose.x;
        start._PointXYZI::y = robotPose.y;
        start._PointXYZI::z = robotPose.z;
        angle = getTwoPointAngleInWorld(start,end);
        return  angle;
         cout<<"angle"<<angle<<endl;
    }

    double getAngleBetweenTwoAnglesIn_180To180(const double &angle1, const double &angle2){
        double deltAngle = angle1 - angle2;
        if(deltAngle > 180) deltAngle -=360;
        else if (deltAngle < -180) deltAngle += 360;
        return deltAngle;
    }
    void listenRobotPose(){
        if(listen.waitForTransform("map","base_link",ros::Time(0),ros::Duration(0.05))){
            listen.lookupTransform("map","base_link",ros::Time(0),transform);
            double pitch,roll,yaw;
            transform.getBasis().getRPY(pitch,roll,yaw);
            robotPoseWorld.x = transform.getOrigin().x();
            robotPoseWorld.y = transform.getOrigin().y();
            robotPoseWorld.yaw = yaw / PI * 180;
//            cout<<"**robotPoseWorld.yaw"<<robotPoseWorld.yaw <<endl;
        }
    }
    double getDist_robotToDist(PointTypePose &robotPose, PointType &distPoint){
        double dist_exep = (robotPose.x - distPoint._PointXYZI::x) * (robotPose.x - distPoint._PointXYZI::x) +
                           (robotPose.y - distPoint._PointXYZI::y) * (robotPose.y - distPoint._PointXYZI::y);
        double dist = sqrt(dist_exep);
        return  dist;
    }
    bool robotArriveGoal(){
        double dist = getDist_robotToDist(robotPoseWorld,distPoint);
        cout<<"dist"<<dist<<endl;
        if(dist < 0.1){
            /*机器人到达目标点*/
            ROS_INFO("robot have arrived goal");
            robotStartDealDst = false;

            robotArriveDstPose = true;
            currentGoalState = HAVE_ARRIVED_GOAL;

            firstRobotStartDealDst = true;
            moveRobotAsWorld(0,0);
            return  true;
        }else {
            currentGoalState = HAVE_NOT_ARRIVED_GOAL;
            return false;
        }
    }
    bool robotRotateGoal(){
        ROS_INFO("robotRotateGoal");
//        distPoint = goal;

        startRotateAngle = getTwoPointAngleInWorld(robotPoseWorld,distPoint);
        double rotateAngle = getAngleBetweenTwoAnglesIn_180To180(startRotateAngle,robotPoseWorld.yaw);
        if(currentMotion == MOTION_BACK) {
            rotateAngle = 180 - rotateAngle;
            if(rotateAngle > 180) rotateAngle -=360;
            else if (rotateAngle < -180) rotateAngle += 360;
        }
        cout<<"startRotateAngle"<<startRotateAngle<<endl;
        cout<<"rotateAngle"<<rotateAngle<<endl;
        cout<<"robotPoseWorld.yaw"<<robotPoseWorld.yaw<<endl;
        if(fabs(rotateAngle) < 12){
            rotateArriveAngle = true;
            currentRotateState = HAVE_ROTATE_ANGLE;
            moveRobotAsWorld(0,0);
            return true;
        }else {
            rotateArriveAngle = false;
            currentRotateState = HAVE_NOT_ROTATE_ANGLE;
        }
        cout<<"speedAngularStatic"<<speedAngularStatic<<endl;
        if( rotateAngle > 0) moveRobotAsWorld(0,speedAngularStatic);
        else if (rotateAngle < 0) moveRobotAsWorld(0,-speedAngularStatic);




    }
    void PDcontrol(){
        currentError = distAngle;
        double outPut = Kp * currentError + Kd * errorError;
        LastError = distAngle;
        cout<<"currentError"<<currentError<<endl;
        cout<<"outPut"<<outPut<<endl;
        speedAngular = outPut / 180 * PI;

    }
    bool startProcess(){
        ROS_INFO("startProcess");
        if(robotRotateGoal()){
            moveRobotAsWorld(speedLiner,0);
             firstRobotStartDealDst = false;
            return true;

        }

    }
    void goalPoseProcess(){
        if(distPoints.empty()){
            currentState = taskState::TASKSTOP;
            return;
        }
        if(goalPoints.empty()){
            goalPoints = distPoints;
            loopNum ++;
        }
        if(goalPoints.empty())return;
        distPoint = goalPoints[0];
        goalPoints.erase(goalPoints.begin());
        double startToDistAngle_ = getTwoPointAngleInWorld(robotPoseWorld,distPoint);
        double robot_startToDistAngle_ = getAngleBetweenTwoAnglesIn_180To180(startToDistAngle_,robotPoseWorld.yaw);
        if(fabs(robot_startToDistAngle_)<90) {
            speedLiner = speedLinerStatic;
            currentMotion = MOTION_AHEAD;
        }
        else {
            currentMotion = MOTION_BACK;
//            speedLiner = -speedLinerStatic;
        }
        robotArriveDstPose = false;


    }

    void process(){
        robotToDistAngle = getTwoPointAngleInWorld(robotPoseWorld,distPoint);
        distAngle = getAngleBetweenTwoAnglesIn_180To180(robotToDistAngle,robotPoseWorld.yaw);
        if(currentMotion == MOTION_BACK) {
            distAngle = 180 - distAngle;
            if(distAngle > 180) distAngle -=360;
            else if (distAngle < -180) distAngle += 360;
        }
        PDcontrol();
        moveRobotAsWorld(speedLiner,speedAngular);
        //            cout<<"robotToDistAngle"<<robotToDistAngle<<endl;
        //            cout<<"distAngle"<<distAngle<<endl;
                    cout<<"distPoint._PointXYZI::x"<<distPoint._PointXYZI::x<<endl;
                    cout<<"distPoint._PointXYZI::y"<<distPoint._PointXYZI::y<<endl;
                    cout<<"robotPoseWorld.x"<<robotPoseWorld.x<<endl;
                    cout<<"robotPoseWorld.y"<<robotPoseWorld.y<<endl;
    }
    void run(){
//        distPoint._PointXYZI::x = 4.5;
//        distPoint._PointXYZI::y = 0;


        currentTimeOfPD = ros::Time::now().toSec();
        deltaPDTime = currentTimeOfPD - lastTimeOfPD;
        lastTimeOfPD = ros::Time::now().toSec();

//        listenRobotPose();
//        if(sleepNum < 10){
//            sleepNum++;
//            return;
//        }
//        cout<<"run_robotPoseWorld.yaw"<<robotPoseWorld.yaw <<endl;
        switch (currentState) {
        case taskState::TASKNORMAL :{
            if(robotArriveDstPose)
                goalPoseProcess();
            if(firstRobotStartDealDst)
            robotRotateGoal();
            if(rotateArriveAngle == false)
                break;
            if(firstRobotStartDealDst)
            {
                moveRobotAsWorld(speedLiner,0);
                firstRobotStartDealDst = false;
            }
            robotArriveGoal();
            process();

            break;
        }
        case taskState::TASKSTOP :{
            moveRobotAsWorld(0,0);
            break;
        }
        case taskState::TASKCONTINUE:{
            currentState = taskState::TASKNORMAL;
            break;
        }
        default:
            break;

        }






        LastError = distAngle;
    }
    void listenRobotPoseThread(){
        ros::Rate rate(10);
        while (ros::ok()) {

            if(listen.waitForTransform("map","base_link",ros::Time(0),ros::Duration(0.05))){
                listen.lookupTransform("map","base_link",ros::Time(0),transform);
                double pitch,roll,yaw;
                transform.getBasis().getRPY(pitch,roll,yaw);
                robotPoseWorld.x = transform.getOrigin().x();
                robotPoseWorld.y = transform.getOrigin().y();
                robotPoseWorld.yaw = yaw / PI * 180;
            }
//            if(currentMotion == MOTION_BACK)
//                robotPoseWorld.yaw  = -robotPoseWorld.yaw ;
//            cout<<"robotPoseWorld.x"<<robotPoseWorld.x<<endl;
//            cout<<"robotPoseWorld.y"<<robotPoseWorld.y<<endl;
//            cout<<"robotPoseWorld.yaw"<<robotPoseWorld.yaw <<endl;

            rate.sleep();
        }
    }

};








int main(int argc, char **argv)
{
    ros::init(argc, argv, "los_control");
    los_control los;
    std::thread listentPoseThread (&los_control::listenRobotPoseThread,&los);
    sleep(1);
    ros::Rate rate(10);
    PointType testPoint,testPoint2;
    testPoint._PointXYZI::x = 4.5;
    testPoint._PointXYZI::y = 0;
    testPoint2._PointXYZI::x = 0;
    testPoint2._PointXYZI::y =0;
    los.distPoints.push_back(testPoint);
    los.distPoints.push_back(testPoint2);

    while (ros::ok()) {
        ros::spinOnce();
        los.run();
        rate.sleep();
    }
    listentPoseThread.join();


    return 0;
}
