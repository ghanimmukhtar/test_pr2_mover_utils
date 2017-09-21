#include <pr2_mover_utils/pr2_mover.hpp>
//#include <pr2_mover_utils/pr2_mover_utils/pr2_mover.hpp>
//#include <moveit/move_group_interface/move_group.h>

using namespace pr2_mover;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_pr2_utils");
    ros::NodeHandle node;
    PR2_Mover::Ptr _my_test;
    _my_test.reset(new PR2_Mover(node));
    moveit::planning_interface::MoveGroup::Plan _group_plan;

    double x, y, z, xmin = 0.4, xmax = 0.8, ymin = 0.12, ymax = 1.0, zmin = 0.2, zmax = 1.3;
    node.getParam("X", x);
    node.getParam("Y", y);
    node.getParam("Z", z);

//    ROS_WARN_STREAM("TEST: The planning time is: " << _my_test->group->getPlanningTime());
//    ROS_WARN_STREAM("TEST: The eef pose is: ");
//    ROS_WARN_STREAM("TEST: X: " << _my_test->group->getCurrentPose().pose.position.x);
//    ROS_WARN_STREAM("TEST: Y: " << _my_test->group->getCurrentPose().pose.position.y);
//    ROS_WARN_STREAM("TEST: Z: " << _my_test->group->getCurrentPose().pose.position.z);
//    ROS_WARN_STREAM("TEST: RX: " << _my_test->group->getCurrentPose().pose.orientation.x);
//    ROS_WARN_STREAM("TEST: RY: " << _my_test->group->getCurrentPose().pose.orientation.y);
//    ROS_WARN_STREAM("TEST: RZ: " << _my_test->group->getCurrentPose().pose.orientation.z);
//    ROS_WARN_STREAM("TEST: RW: " << _my_test->group->getCurrentPose().pose.orientation.w);
    //_my_test->group->setPlannerId("RRTConnectkConfigDefault");
   // _my_test->group->setPlanningTime(50);

    _my_test->group->setPositionTarget(x, y, z);
    if(_my_test->group->plan(_group_plan))
        _my_test->group->execute(_group_plan);

    while(ros::ok()){
        moveit::planning_interface::MoveGroup::Plan _revers_plan;
        x = (xmax - xmin) * ( (double)rand() / (double)RAND_MAX ) + xmin;
        y = (ymax - ymin) * ( (double)rand() / (double)RAND_MAX ) + ymin;
        z = (zmax - zmin) * ( (double)rand() / (double)RAND_MAX ) + zmin;

        ROS_INFO_STREAM("TEST : I am planning for position, X : " << x << ", Y : " << y << ", Z : " << z);
        _my_test->group->setPositionTarget(x, y, z);
        if(_my_test->group->plan(_group_plan))
            _my_test->group->execute(_group_plan);

        //then try to execute it in reverse
        //_revers_plan = _group_plan;
        _revers_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        _revers_plan.trajectory_.joint_trajectory.header.frame_id = "world";
        _revers_plan.trajectory_.joint_trajectory.joint_names = _group_plan.trajectory_.joint_trajectory.joint_names;

        int j = _group_plan.trajectory_.joint_trajectory.points.size() - 1;
        for(size_t i = 0; i < _group_plan.trajectory_.joint_trajectory.points.size() && j >= 0; i++){

            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions = _group_plan.trajectory_.joint_trajectory.points[j].positions;
            pt.velocities.resize(7, 0.0);
            pt.accelerations.resize(7, 0.0);
            pt.effort.resize(7, 0.0);
            pt.time_from_start = _group_plan.trajectory_.joint_trajectory.points[i].time_from_start;
            //_revers_plan.trajectory_.joint_trajectory.points[i] = _group_plan.trajectory_.joint_trajectory.points[j];
            //_revers_plan.trajectory_.joint_trajectory.points[i].velocities.resize(7, 0);
            //_revers_plan.trajectory_.joint_trajectory.points[i].accelerations.resize(7, 0);
            //_revers_plan.trajectory_.joint_trajectory.points[i].effort.resize(7, 0);
            //_revers_plan.trajectory_.joint_trajectory.points[i].time_from_start = _group_plan.trajectory_.joint_trajectory.points[i].time_from_start;
            _revers_plan.trajectory_.joint_trajectory.points.push_back(pt);
            j--;
        }

//        for(size_t i = 0; i < _my_test->group->getCurrentJointValues().size(); i++)
//            ROS_INFO_STREAM("TESTER PR2 MOVER : Joint " << i << " value is :  " << _my_test->group->getCurrentJointValues()[i]);

        _my_test->group->execute(_revers_plan);
        std::cin.ignore();
    }

    //ros::waitForShutdown();
    return 0;
}
