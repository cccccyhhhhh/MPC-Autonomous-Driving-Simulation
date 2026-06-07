classdef MPCController_FourWheel
    % MPCController_FourWheel - 四轮独立驱动MPC控制器
    %
    % 功能：
    %   基于四轮独立驱动模型的模型预测控制器
    %   支持轨迹跟踪、碰撞避免、速度控制
    %
    % 输出:
    %   四轮驱动力 [FL, FR, RL, RR] (N)
    %
    % 作者: CCCCCYHHHHH
    % 日期: 2025年1月
    
    properties
        vehicle                % 车辆模型
        params                 % MPC参数
        config                 % 全局配置
    end
    
    methods
        
        %% 构造函数
        function obj = MPCController_FourWheel(vehicle, mpcParams, config)
            obj.vehicle = vehicle;
            obj.params = mpcParams;
            obj.config = config;
        end
        
        %% MPC求解函数
        function [wheelForces, status] = solve(obj, currentState, refTrajectory, refVelocity, v_ref, lidarPoints)
            % 求解MPC最优控制
            %
            % 输入:
            %   currentState: [x, y, theta, v] 当前状态
            %   refTrajectory: [N, 2] 参考轨迹
            %   refVelocity: [N, 1] 参考速度
            %   v_ref: 目标速度 (m/s)
            %   lidarPoints: [M, 2] LiDAR检测点
            %
            % 输出:
            %   wheelForces: [4, 1] 四轮驱动力
            %   status: 求解状态 (1=成功, 0=失败)
            
            status = 1;  % 默认成功
            
            % 简化的MPC控制策略（实际应用可使用quadprog或fmincon）
            % 这里使用基于轨迹跟踪的反馈控制
            
            % 获取最近参考点
            [minDist, nearestIdx] = min(sqrt(sum((refTrajectory - currentState(1:2)).^2, 2)));
            
            if nearestIdx + 1 <= length(refTrajectory)
                ref_point = refTrajectory(nearestIdx, :);
                ref_point_next = refTrajectory(min(nearestIdx + 1, end), :);
            else
                ref_point = refTrajectory(end, :);
                ref_point_next = refTrajectory(end, :);
            end
            
            %% 纵向控制（速度PID）
            Kp_v = 30;  % 速度比例系数
            Ki_v = 5;   % 速度积分系数
            Kd_v = 5;   % 速度微分系数
            
            v_error = v_ref - currentState(4);
            
            % 简化的PID
            F_long = Kp_v * v_error;
            F_long = max(min(F_long, obj.params.maxForce), -obj.params.maxForce);
            
            %% 横向控制（方向追踪）
            % 计算参考方向角
            if norm(ref_point_next - ref_point) > 0.01
                ref_theta = atan2(ref_point_next(2) - ref_point(2), ...
                                 ref_point_next(1) - ref_point(1));
            else
                ref_theta = currentState(3);
            end
            
            % 横向误差
            lateral_error = -sin(ref_theta) * (currentState(1) - ref_point(1)) + ...
                           cos(ref_theta) * (currentState(2) - ref_point(2));
            
            % 方向角误差
            theta_error = ref_theta - currentState(3);
            % 角度规范化到[-pi, pi]
            theta_error = atan2(sin(theta_error), cos(theta_error));
            
            % 差动转向控制
            Kp_lat = 20;
            Kp_theta = 10;
            
            lateral_force = Kp_lat * lateral_error + Kp_theta * theta_error;
            
            % 分配到左右轮
            F_left = (F_long + lateral_force) / 2;
            F_right = (F_long - lateral_force) / 2;
            
            %% 碰撞避免
            if ~isempty(lidarPoints)
                % 检测前方障碍物
                front_angle_range = [-pi/4, pi/4];  % 前方±45°范围
                front_lidar_mask = lidarPoints(:, 2) > currentState(2) - 1;
                
                if any(front_lidar_mask)
                    % 前方有障碍物，降速
                    obstacle_dist = min(sqrt(sum((lidarPoints(front_lidar_mask, :) - currentState(1:2)).^2, 2)));
                    
                    if obstacle_dist < obj.params.collisionMargin
                        % 紧急停止
                        F_long = -obj.params.maxForce;
                        F_left = -obj.params.maxForce;
                        F_right = -obj.params.maxForce;
                        status = 2;  % 碰撞避免状态
                    elseif obstacle_dist < 2 * obj.params.collisionMargin
                        % 减速
                        F_long = F_long * 0.5;
                        F_left = F_left * 0.5;
                        F_right = F_right * 0.5;
                    end
                end
            end
            
            %% 四轮力分配
            % 四轮独立驱动：前后平衡分配
            F_front = (F_left + F_right) / 2;
            F_rear = (F_left + F_right) / 2;
            
            wheelForces = [F_left; F_right; F_rear; F_rear];
            
            % 最终限幅
            wheelForces = max(min(wheelForces, obj.params.maxForce), -obj.params.maxForce);
        end
        
        %% 获取性能指标
        function metrics = getMetrics(obj, state, refState)
            % 计算跟踪误差指标
            metrics.position_error = norm(state(1:2) - refState(1:2));
            metrics.angle_error = abs(state(3) - refState(3));
            metrics.velocity_error = abs(state(4) - refState(4));
        end
        
    end
end