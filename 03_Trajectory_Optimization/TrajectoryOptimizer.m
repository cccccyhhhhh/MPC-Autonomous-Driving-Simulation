classdef TrajectoryOptimizer
    % TrajectoryOptimizer - 轨迹光滑化和优化
    %
    % 功能：
    %   将A*规划的离散路点转化为光滑的参考轨迹
    %   使用样条插值或多项式拟合
    %
    % 作者: CCCCCYHHHHH
    % 日期: 2025年1月
    
    properties
        maxVelocity
        maxAccel
        samplingResolution
    end
    
    methods
        
        %% 构造函数
        function obj = TrajectoryOptimizer(config)
            obj.maxVelocity = config.maxVelocity;
            obj.maxAccel = config.maxAccel;
            obj.samplingResolution = 0.1;  % 0.1m采样
        end
        
        %% 主优化函数
        function [refTrajectory, refVelocity] = optimize(obj, waypoints, maxVel, maxAccel)
            % 优化轨迹
            %
            % 输入:
            %   waypoints: [Nx2] 离散路点
            %   maxVel: 最大速度 (m/s)
            %   maxAccel: 最大加速度 (m/s²)
            %
            % 输出:
            %   refTrajectory: [Mx2] 光滑参考轨迹
            %   refVelocity: [M,1] 对应的速度值
            
            % 样条插值（使用MATLAB内置函数）
            if length(waypoints) >= 3
                % 参数化曲线
                t = 0:length(waypoints)-1;
                
                % 使用分段三次样条
                pp_x = spline(t, waypoints(:, 1));
                pp_y = spline(t, waypoints(:, 2));
                
                % 细化采样
                t_fine = 0:0.1:(length(waypoints)-1);
                traj_x = ppval(pp_x, t_fine);
                traj_y = ppval(pp_y, t_fine);
                
                refTrajectory = [traj_x', traj_y'];
            else
                % 点数过少，直接线性插值
                totalDist = obj.calcPathLength(waypoints);
                nPoints = ceil(totalDist / obj.samplingResolution);
                refTrajectory = interp1(waypoints, linspace(1, length(waypoints), nPoints));
            end
            
            % 生成速度曲线
            refVelocity = obj.generateVelocityProfile(refTrajectory, maxVel, maxAccel);
        end
        
        %% 生成速度曲线
        function velProfile = generateVelocityProfile(obj, trajectory, maxVel, maxAccel)
            % 生成遵循物理约束的速度曲线
            % 使用梯形速度曲线（加速-匀速-减速）
            
            n = length(trajectory);
            velProfile = zeros(n, 1);
            
            % 计算路径长度
            distances = [0; cumsum(sqrt(sum(diff(trajectory).^2, 2)))];
            totalDist = distances(end);
            
            % 计算最大可能速度
            % v_max = sqrt(a_max * r_min) 转弯半径限制
            % 这里简化处理
            
            % 三角形速度曲线
            accelDist = maxVel^2 / (2 * maxAccel);
            
            if 2 * accelDist >= totalDist
                % 无法达到最大速度
                peakVel = sqrt(maxAccel * totalDist / 2);
                accelDist = peakVel^2 / (2 * maxAccel);
            else
                peakVel = maxVel;
            end
            
            % 加速段
            accelEnd = accelDist;
            decelStart = totalDist - accelDist;
            
            for i = 1:n
                d = distances(i);
                
                if d <= accelEnd
                    % 加速段
                    velProfile(i) = sqrt(2 * maxAccel * d);
                elseif d >= decelStart
                    % 减速段
                    remainDist = totalDist - d;
                    velProfile(i) = sqrt(2 * maxAccel * remainDist);
                else
                    % 匀速段
                    velProfile(i) = peakVel;
                end
            end
            
            % 平滑速度曲线
            velProfile = smooth(velProfile, 5);
            
            % 确保终点速度为0
            velProfile(end) = 0;
        end
        
        %% 计算路径长度
        function len = calcPathLength(obj, trajectory)
            diffs = diff(trajectory, 1, 1);
            lens = sqrt(sum(diffs.^2, 2));
            len = sum(lens);
        end
        
    end
    
end