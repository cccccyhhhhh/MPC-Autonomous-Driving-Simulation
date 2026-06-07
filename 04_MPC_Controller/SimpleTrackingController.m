function wheelForces = SimpleTrackingController(currentState, refTrajectory, v_ref, vehicle)
    % SimpleTrackingController - 简单的轨迹跟踪控制器
    %
    % 用于A*规划阶段（前80%）的基础控制器
    % 基于PID的简单反馈控制
    %
    % 输入:
    %   currentState: [x, y, theta, v] 当前状态
    %   refTrajectory: [N, 2] 参考轨迹段
    %   v_ref: 目标速度
    %   vehicle: 车辆模型
    %
    % 输出:
    %   wheelForces: [4, 1] 四轮驱动力
    
    % 获取最近参考点
    if isempty(refTrajectory) || length(refTrajectory) < 1
        wheelForces = [0; 0; 0; 0];
        return;
    end
    
    ref_point = refTrajectory(1, :);
    
    % 计算参考方向角
    if length(refTrajectory) > 1
        ref_theta = atan2(refTrajectory(2, 2) - ref_point(2), ...
                         refTrajectory(2, 1) - ref_point(1));
    else
        ref_theta = currentState(3);
    end
    
    %% 纵向控制（速度PID）
    Kp_v = 40;
    v_error = v_ref - currentState(4);
    F_long = Kp_v * v_error;
    F_long = max(min(F_long, 100), -100);
    
    %% 横向控制（轨迹跟踪）
    % 计算横向误差
    lateral_error = -sin(ref_theta) * (currentState(1) - ref_point(1)) + ...
                   cos(ref_theta) * (currentState(2) - ref_point(2));
    
    % 方向角误差
    theta_error = ref_theta - currentState(3);
    theta_error = atan2(sin(theta_error), cos(theta_error));
    
    % 横向控制增益
    Kp_lat = 15;
    Kp_theta = 8;
    
    F_lateral = Kp_lat * lateral_error + Kp_theta * theta_error;
    
    %% 四轮力分配
    F_left = (F_long + F_lateral) / 2;
    F_right = (F_long - F_lateral) / 2;
    
    wheelForces = [F_left; F_right; F_long*0.8; F_long*0.8];
    
    % 限幅
    wheelForces = max(min(wheelForces, 100), -100);
end