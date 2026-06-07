%% ========== 分体式飞行汽车底盘控制仿真主程序 ==========
% 
% 功能：
%   - 四轮独立驱动底盘模型
%   - A*路径规划
%   - 轨迹光滑化优化
%   - MPC精细控制（最后20%）
%   - 传感器仿真与避障
%   - 实时可视化与视频输出
%
% 场景：城市环境，密集障碍物，T字路口/U形调头/狭窄街道/圆环
% 视频时长：10秒以内
% 精度要求：位置<1cm，角度<0.5°
%
% 作者：CCCCCYHHHHH
% 日期：2025年1月

clear; close all; clc;

%% ========== 1. 系统初始化 ==========
fprintf('\n========== 飞行汽车底盘控制仿真系统 ==========\n');

% 添加所有子路径
addpath(genpath(pwd));

% 全局参数配置
GlobalConfig = InitializeGlobalConfig();

%% ========== 2. 车辆模型初始化 ==========
fprintf('\n[1/7] 初始化四轮独立驱动车辆模型...\n');

vehicle = FourWheelIndependentVehicle(...
    'length', 0.5, ...
    'width', 0.5, ...
    'height', 0.4, ...
    'mass', 15, ...           % kg
    'maxSteeringAngle', deg2rad(30), ...
    'maxWheelForce', 100, ...  % N per wheel
    'wheelRadius', 0.08, ...   % m
    'Ts', GlobalConfig.Ts);

fprintf('  ✓ 车辆模型初始化完成\n');
fprintf('    - 尺寸: %.2f × %.2f × %.2f m\n', vehicle.length, vehicle.width, vehicle.height);
fprintf('    - 质量: %.1f kg\n', vehicle.mass);
fprintf('    - 采样时间: %.3f s\n', vehicle.Ts);

%% ========== 3. 地图与障碍物生成 ==========
fprintf('\n[2/7] 生成地图与密集障碍物...\n');

[mapData, obstacles] = GenerateUrbanMap(...
    'mapSize', GlobalConfig.mapSize, ...
    'obstacleCount', GlobalConfig.obstacleCount, ...
    'minObstacleSize', GlobalConfig.minObstacleSize, ...
    'maxObstacleSize', GlobalConfig.maxObstacleSize);

fprintf('  ✓ 地图生成完成\n');
fprintf('    - 地图大小: %.1f × %.1f m\n', GlobalConfig.mapSize, GlobalConfig.mapSize);
fprintf('    - 障碍物数量: %d\n', length(obstacles));

%% ========== 4. 选择场景 ==========
fprintf('\n[3/7] 选择仿真场景...\n');

sceneType = 1;  % 1:T字路口 2:U形调头 3:狭窄街道 4:圆环
[startPose, goalPose, mapData] = SelectScene(sceneType, mapData, GlobalConfig);

fprintf('  ✓ 场景选择完成\n');
fprintf('    - 起点: (%.2f, %.2f) 角度: %.2f°\n', startPose(1), startPose(2), rad2deg(startPose(3)));
fprintf('    - 终点: (%.2f, %.2f) 角度: %.2f°\n', goalPose(1), goalPose(2), rad2deg(goalPose(3)));

% 初始化车辆位置
vehicle.setState(startPose(1), startPose(2), startPose(3), 0);

%% ========== 5. A*路径规划 ==========
fprintf('\n[4/7] 执行A*路径规划...\n');

astarPlanner = AStarPathPlanner(mapData, GlobalConfig);
[pathWaypoints, planningTime] = astarPlanner.plan(startPose(1:2), goalPose(1:2));

if isempty(pathWaypoints)
    error('路径规划失败！无法找到可行路径。');
end

fprintf('  ✓ 路径规划完成\n');
fprintf('    - 规划耗时: %.3f s\n', planningTime);
fprintf('    - 路点数量: %d\n', length(pathWaypoints));
fprintf('    - 路径长度: %.2f m\n', CalcPathLength(pathWaypoints));

%% ========== 6. 轨迹优化与光滑化 ==========
fprintf('\n[5/7] 轨迹光滑化优化...\n');

trajectoryOptimizer = TrajectoryOptimizer(GlobalConfig);
[refTrajectory, refVelocity] = trajectoryOptimizer.optimize(...
    pathWaypoints, ...
    GlobalConfig.maxVelocity, ...
    GlobalConfig.maxAccel);

fprintf('  ✓ 轨迹优化完成\n');
fprintf('    - 参考点数: %d\n', length(refTrajectory));
fprintf('    - 最大速度: %.2f m/s\n', max(refVelocity));

%% ========== 7. MPC控制器初始化 ==========
fprintf('\n[6/7] 初始化MPC精细控制器...\n');

mpcParams = MPCParameters_FourWheel(...
    'predictionHorizon', GlobalConfig.predictionHorizon, ...
    'controlHorizon', GlobalConfig.controlHorizon, ...
    'Ts', GlobalConfig.Ts);

mpcController = MPCController_FourWheel(vehicle, mpcParams, GlobalConfig);

fprintf('  ✓ MPC控制器初始化完成\n');
fprintf('    - 预测时域: %d 步 (%.2f s)\n', ...
    GlobalConfig.predictionHorizon, ...
    GlobalConfig.predictionHorizon * GlobalConfig.Ts);
fprintf('    - 控制时域: %d 步\n', GlobalConfig.controlHorizon);

%% ========== 8. 传感器仿真器初始化 ==========
fprintf('\n[7/7] 初始化传感器仿真...\n');

sensorSimulator = SensorSimulator(...
    'lidarRange', GlobalConfig.lidarRange, ...
    'lidarResolution', GlobalConfig.lidarResolution, ...
    'radarRange', GlobalConfig.radarRange);

fprintf('  ✓ 传感器仿真器初始化完成\n');

%% ========== 9. 仿真循环 ==========
fprintf('\n========== 开始仿真循环 ==========\n');

% 时间参数
simTime = GlobalConfig.maxSimTime;
Ts = GlobalConfig.Ts;
timeVector = 0:Ts:simTime;
N = length(timeVector);

% 记录数据
trajectoryRecord = zeros(N, 3);  % [x, y, theta]
velocityRecord = zeros(N, 1);
stateRecord = zeros(N, 4);       % [x, y, v, theta]
controlRecord = zeros(N, 4);     % 四轮驱动力
mpcStatusRecord = zeros(N, 1);   % MPC求解成功标志

% 初始化可视化
videoWriter = VideoWriter(fullfile(pwd, 'simulation_output.mp4'), 'MPEG-4');
videoWriter.FrameRate = 30;
videoWriter.Quality = 95;
open(videoWriter);

% 创建图形窗口
fig = figure('Position', [100, 100, 1200, 900]);

% 计算每个路点的进度（用于确定MPC阶段）
pathLength = CalcPathLength(refTrajectory);
mpcStartIdx = round(0.8 * length(refTrajectory));  % 最后20%

% 仿真主循环
for k = 1:N-1
    t_current = timeVector(k);
    
    % 当前状态
    currentState = vehicle.getState();
    trajectoryRecord(k, :) = [currentState(1), currentState(2), currentState(3)];
    stateRecord(k, :) = [currentState(1), currentState(2), vehicle.velocity, currentState(3)];
    
    % 找到参考轨迹上的最近点
    [~, nearestIdx] = min(sqrt(sum((refTrajectory - currentState(1:2)).^2, 2)));
    
    % 确定是否进入MPC精细控制阶段
    useMPC = (nearestIdx >= mpcStartIdx);
    
    % 计算参考速度
    if nearestIdx + 1 <= length(refVelocity)
        v_ref = refVelocity(nearestIdx);
    else
        v_ref = 0;  % 接近终点时减速到0
    end
    
    % 获取传感器数据（检测附近障碍物）
    [lidarPoints, radarDist] = sensorSimulator.scan(currentState(1:2), obstacles);
    
    % 控制算法
    if useMPC && nearestIdx < length(refTrajectory) - 1
        % MPC精细控制（最后20%）
        % 获取参考轨迹段
        trajSegment = refTrajectory(nearestIdx:min(nearestIdx + GlobalConfig.predictionHorizon, end), :);
        velSegment = refVelocity(nearestIdx:min(nearestIdx + GlobalConfig.predictionHorizon, end));
        
        % MPC求解
        [wheelForces, mpcStatus] = mpcController.solve(...
            currentState, ...
            trajSegment, ...
            velSegment, ...
            v_ref, ...
            lidarPoints);
        
        mpcStatusRecord(k) = mpcStatus;
    else
        % 标准跟踪控制（前80%）
        wheelForces = SimpleTrackingController(...
            currentState, ...
            refTrajectory(nearestIdx:min(nearestIdx+5, end), :), ...
            v_ref, ...
            vehicle);
        
        mpcStatusRecord(k) = -1;  % 未使用MPC
    end
    
    % 执行控制（限幅）
    wheelForces = max(min(wheelForces, 100), -100);
    controlRecord(k, :) = wheelForces;
    
    % 更新车辆状态
    vehicle.update(wheelForces);
    
    % 可视化（每个仿真步长更新一次）
    if mod(k, 2) == 0  % 每2个步长绘制一次（加快速度）
        clf(fig);
        
        % 绘制地图和障碍物
        subplot(2, 2, [1, 3]);
        hold on; axis equal;
        
        % 绘制障碍物
        for i = 1:length(obstacles)
            obs = obstacles{i};
            rectangle('Position', obs, 'FaceColor', [0.5 0.5 0.5], ...
                'EdgeColor', 'k', 'LineWidth', 1);
        end
        
        % 绘制起点和终点
        plot(startPose(1), startPose(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
        plot(goalPose(1), goalPose(2), 'r*', 'MarkerSize', 20, 'DisplayName', '终点');
        
        % 绘制参考轨迹
        plot(refTrajectory(:, 1), refTrajectory(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '参考轨迹');
        
        % 绘制已完成的实际轨迹
        if k > 1
            plot(trajectoryRecord(1:k, 1), trajectoryRecord(1:k, 2), 'r-', 'LineWidth', 1.5, 'DisplayName', '实际轨迹');
        end
        
        % 绘制车辆（矩形）
        vehicleSize = [vehicle.width, vehicle.length];
        vehiclePos = currentState(1:2) - vehicleSize ./ 2;
        
        % 根据方向旋转
        [vx, vy] = deal(vehicleSize(1)/2, vehicleSize(2)/2);
        corners = [
            -vx, -vy;
             vx, -vy;
             vx,  vy;
            -vx,  vy;
            -vx, -vy
        ];
        
        % 旋转矩阵
        R = [cos(currentState(3)), -sin(currentState(3));
             sin(currentState(3)),  cos(currentState(3))];
        corners_rot = corners * R';
        corners_rot = corners_rot + repmat(currentState(1:2), size(corners_rot, 1), 1);
        
        fill(corners_rot(:, 1), corners_rot(:, 2), 'y', 'EdgeColor', 'k', 'LineWidth', 2);
        
        % 绘制传感器范围
        circle(currentState(1), currentState(2), GlobalConfig.lidarRange, 'b--', 0.5);
        
        % 绘制参考点
        if nearestIdx <= length(refTrajectory)
            plot(refTrajectory(nearestIdx, 1), refTrajectory(nearestIdx, 2), 'bs', 'MarkerSize', 8);
        end
        
        xlim([0, GlobalConfig.mapSize]);
        ylim([0, GlobalConfig.mapSize]);
        xlabel('X (m)', 'FontSize', 12);
        ylabel('Y (m)', 'FontSize', 12);
        
        if useMPC
            title(sprintf('仿真时间: %.2f s | 状态: MPC精细控制 (最后20%%) | 车速: %.2f m/s', ...
                t_current, vehicle.velocity), 'FontSize', 12, 'Color', 'red');
        else
            title(sprintf('仿真时间: %.2f s | 状态: 标准跟踪 | 车速: %.2f m/s', ...
                t_current, vehicle.velocity), 'FontSize', 12, 'Color', 'blue');
        end
        
        legend('Location', 'NorthEast');
        grid on;
        
        % 绘制速度曲线
        subplot(2, 2, 2);
        if k > 1
            plot(timeVector(1:k), stateRecord(1:k, 2), 'b-', 'LineWidth', 1.5);
        end
        xlabel('时间 (s)', 'FontSize', 10);
        ylabel('速度 (m/s)', 'FontSize', 10);
        title('车速曲线', 'FontSize', 11);
        grid on;
        
        % 绘制误差
        subplot(2, 2, 4);
        if k > 1 && nearestIdx <= length(refTrajectory)
            error_pos = sqrt((trajectoryRecord(1:k, 1) - refTrajectory(nearestIdx, 1)).^2 + ...
                            (trajectoryRecord(1:k, 2) - refTrajectory(nearestIdx, 2)).^2);
            plot(timeVector(1:k), error_pos * 100, 'r-', 'LineWidth', 1.5);  % 转换为cm
        end
        xlabel('时间 (s)', 'FontSize', 10);
        ylabel('位置误差 (cm)', 'FontSize', 10);
        title('跟踪误差', 'FontSize', 11);
        grid on;
        
        % 刷新图形
        drawnow;
        
        % 记录视频帧
        frame = getframe(fig);
        writeVideo(videoWriter, frame);
    end
    
    % 终点判定
    dist2goal = sqrt((currentState(1) - goalPose(1))^2 + (currentState(2) - goalPose(2))^2);
    if dist2goal < 0.2 && vehicle.velocity < 0.1
        fprintf('\n✓ 到达目标点！仿真结束。\n');
        break;
    end
    
    % 超时检查
    if t_current > simTime
        fprintf('\n仿真时间上限达到。\n');
        break;
    end
end

% 关闭视频
close(videoWriter);

fprintf('\n✓ 视频已保存: simulation_output.mp4\n');

%% ========== 10. 性能分析 ==========
fprintf('\n========== 性能分析 ==========\n');

% 计算性能指标
simLength = k;
actualPath = trajectoryRecord(1:simLength, 1:2);
goalError = sqrt((trajectoryRecord(simLength, 1) - goalPose(1))^2 + ...
                 (trajectoryRecord(simLength, 2) - goalPose(2))^2);

fprintf('仿真结果:\n');
fprintf('  - 总仿真步长: %d\n', simLength);
fprintf('  - 总仿真时间: %.2f s\n', timeVector(simLength));
fprintf('  - 路径长度: %.2f m\n', CalcPathLength(actualPath));
fprintf('  - 终点误差: %.4f m (%.2f cm)\n', goalError, goalError * 100);
fprintf('  - 最大速度: %.2f m/s\n', max(stateRecord(1:simLength, 3)));
fprintf('  - 平均速度: %.2f m/s\n', mean(stateRecord(1:simLength, 3)));
fprintf('\n');

%% ========== 子函数 ==========

function config = InitializeGlobalConfig()
    % 初始化全局配置参数
    config.mapSize = 100;                    % 地图大小 (m)
    config.obstacleCount = 100;              % 障碍物数量（密集）
    config.minObstacleSize = 1;              % 最小障碍物尺寸 (m)
    config.maxObstacleSize = 3;              % 最大障碍物尺寸 (m)
    config.maxVelocity = 5;                  % 最大速度 (m/s)
    config.maxAccel = 2;                     % 最大加速度 (m/s²)
    config.Ts = 0.05;                        % 采样时间 (s)
    config.maxSimTime = 10;                  % 最大仿真时间 (s)
    config.predictionHorizon = 20;           % MPC预测时域
    config.controlHorizon = 5;               % MPC控制时域
    config.lidarRange = 5;                   % LiDAR范围 (m)
    config.lidarResolution = 36;             % LiDAR分辨率 (点数)
    config.radarRange = 10;                  % Radar范围 (m)
end

function pathLength = CalcPathLength(trajectory)
    % 计算轨迹长度
    diff_vec = diff(trajectory, 1, 1);
    distances = sqrt(sum(diff_vec.^2, 2));
    pathLength = sum(distances);
end

function [mapData, obstacles] = GenerateUrbanMap(varargin)
    % 生成城市地图和随机障碍物
    p = inputParser;
    addParameter(p, 'mapSize', 100);
    addParameter(p, 'obstacleCount', 50);
    addParameter(p, 'minObstacleSize', 0.5);
    addParameter(p, 'maxObstacleSize', 3);
    parse(p, varargin{:});
    
    mapSize = p.Results.mapSize;
    obstacleCount = p.Results.obstacleCount;
    minSize = p.Results.minObstacleSize;
    maxSize = p.Results.maxObstacleSize;
    
    % 生成随机障碍物
    obstacles = {};
    for i = 1:obstacleCount
        x = rand * (mapSize - maxSize);
        y = rand * (mapSize - maxSize);
        w = minSize + rand * (maxSize - minSize);
        h = minSize + rand * (maxSize - minSize);
        obstacles{i} = [x, y, w, h];
    end
    
    % 地图数据结构
    mapData.size = mapSize;
    mapData.gridResolution = 0.2;
    mapData.obstacles = obstacles;
end

function [startPose, goalPose, mapData] = SelectScene(sceneType, mapData, config)
    % 选择特定场景
    mapSize = config.mapSize;
    
    switch sceneType
        case 1  % T字路口
            startPose = [5, 5, 0];
            goalPose = [mapSize-5, mapSize-5, 0];
            
        case 2  % U形调头
            startPose = [10, 10, 0];
            goalPose = [10, mapSize-10, pi];
            
        case 3  % 狭窄街道
            startPose = [5, mapSize/2, 0];
            goalPose = [mapSize-5, mapSize/2, 0];
            
        case 4  % 圆环
            startPose = [mapSize/2, 5, 0];
            goalPose = [mapSize/2, mapSize-5, pi];
            
        otherwise
            startPose = [5, 5, 0];
            goalPose = [mapSize-5, mapSize-5, 0];
    end
end

function circle(cx, cy, r, style, alpha)
    % 绘制圆形
    theta = linspace(0, 2*pi, 100);
    x = cx + r * cos(theta);
    y = cy + r * sin(theta);
    plot(x, y, style, 'AlphaData', alpha);
end

function wheelForces = SimpleTrackingController(currentState, refTrajectory, v_ref, vehicle)
    % 简单的轨迹跟踪控制器（PID）
    % 输出：四轮驱动力 [FL, FR, RL, RR]
    
    % 获取最近参考点
    ref_point = refTrajectory(1, :);
    
    % 纵向速度控制（PID）
    Kp_v = 50;
    v_error = v_ref - vehicle.velocity;
    F_long = Kp_v * v_error;
    
    % 均匀分配到四个轮子
    wheelForces = [F_long; F_long; F_long; F_long];
    
    % 限幅
    wheelForces = max(min(wheelForces, 100), -100);
end
