#include <pr2_mover_utils/pr2_mover.hpp>

using namespace pr2_mover;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_user");
    ros::NodeHandle node;

    ros::ServiceClient mover = node.serviceClient<pr2_mover_utils::move_pr2_arm>("move_pr2_arm", 1);

    //ros::AsyncSpinner my_spinner(1);
    //my_spinner.start();

    pr2_mover_utils::move_pr2_arm::Request request;
    pr2_mover_utils::move_pr2_arm::Response response;
    request.arm = "right";
    request.type = "position";
    request.goal = {0.0, -0.9, 0.1};

    mover.call(request, response);



    return 0;
}
