#include <pr2_mover_utils/pr2_mover.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>

using namespace pr2_mover;

int main(int argc, char **argv)
    {
        ros::init(argc, argv, "test_pr2_utils");
        ros::NodeHandle node;
        PR2_Mover::Ptr _my_test;

        _my_test.reset(new PR2_Mover(node));
        moveit::planning_interface::MoveGroup::Plan _group_plan;

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();


        double x, y, z, xmin = 0.4, xmax = 0.8, ymin = 0.12, ymax = 1.0, zmin = 0.2, zmax = 1.3;
        node.getParam("X", x);
        node.getParam("Y", y);
        node.getParam("Z", z);

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
                if(_my_test->group->plan(_group_plan)){
                        _my_test->group->execute(_group_plan);
                        //then try to execute it in reverse
                        _revers_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
                        _revers_plan.trajectory_.joint_trajectory.header.frame_id = _group_plan.trajectory_.joint_trajectory.header.frame_id;
                        _revers_plan.trajectory_.joint_trajectory.joint_names = _group_plan.trajectory_.joint_trajectory.joint_names;

                        int j = _group_plan.trajectory_.joint_trajectory.points.size() - 1;
                        trajectory_processing::IterativeParabolicTimeParameterization time_param;
                        robot_trajectory::RobotTrajectory my_robot_trajectory(robot_model, _my_test->group->getName());
                        robot_state::RobotState r_state_holder(robot_model);
                        for(size_t i = 0; i < _group_plan.trajectory_.joint_trajectory.points.size() && j >= 0; i++){
                                ROS_WARN_STREAM("TEST : In the for loop trying to construct the inverse trajectory with size : " << _group_plan.trajectory_.joint_trajectory.points.size());
                                moveit::core::jointTrajPointToRobotState(_group_plan.trajectory_.joint_trajectory, j, r_state_holder);
                                my_robot_trajectory.insertWayPoint(i, r_state_holder, 0.1);
                                j--;
                            }

                        if(!time_param.computeTimeStamps(my_robot_trajectory))
                            ROS_WARN("TEST : Time parametrization for the solution path failed.");
                        my_robot_trajectory.getRobotTrajectoryMsg(_revers_plan.trajectory_);
                        moveit::core::robotStateToRobotStateMsg(*_my_test->group->getCurrentState(), _revers_plan.start_state_);
                        _my_test->group->execute(_revers_plan);
                        std::cin.ignore();
                    }
            }
        return 0;
    }
