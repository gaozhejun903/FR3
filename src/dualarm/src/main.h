#pragma once
int L_fix(
    std::shared_ptr<RobotMain>& node,
    std::vector<double, std::allocator<double>>& orientation_increment,
    double& total_angle_diff,
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart_Request>& fix_request,
    std::shared_ptr<robo_ctrl::srv::RobotMoveCart_Response>& fix_response,
    bool& retFlag
);
