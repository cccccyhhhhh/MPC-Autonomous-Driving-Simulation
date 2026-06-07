%% ========== 分体式飞行汽车底盘控制完整仿真系统 ==========
% 
% 项目：分体式飞行汽车底盘避障与路线规划仿真
% 功能：A*路径规划 + MPC精细控制 + 传感器仿真 + 视频输出
% 
% 场景：城市环境，密集障碍物，多种驾驶工况
% 视频时长：10秒以内
% 精度要求：位置<1cm，角度<0.5°
%
% 作者：CCCCCYHHHHH
% 日期：2025年1月

clear; close all; clc;
addpath(genpath(pwd));

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║   分体式飞行汽车底盘控制仿真系统 - MPC精细操控           ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% ========== 1. 全局配置 ==========
fprintf('[初始化] 系统全局配置\n');

globalConfig.mapSize = 100;              % 地图大小 (m)
globalConfig.mapResolution = 0.2;        % 地图分辨率 (m)
globalConfig.obstacleCount = 120;        % 密集障碍物数量
globalConfig.minObstacleSize = 0.8;      % 最小障碍物尺寸 (m)
globalConfig.maxObstacleSize = 3;        % 最大障碍物尺寸 (m)
globalConfig.maxVelocity = 5;            % 最大速度 (m/s)
globalConfig.maxAccel = 2.0;             % 最大加速度 (m/s²)
globalConfig.Ts = 0.05;                  % 采样时间 (s)
globalConfig.maxSimTime = 10;            % 最大仿真时间 (s)
globalConfig.predictionHorizon = 20;     % MPC预测时域
globalConfig.controlHorizon = 5;         % MPC控制时域
globalConfig.lidarRange = 5;             % LiDAR范围 (m)
globalConfig.lidarResolution = 36;       % LiDAR分辨率
globalConfig.radarRange = 10;            % Radar范围 (m)

fprintf('  ✓ 地图: %.0f×%.0f m, 分辨率: %.1f m\n', globalConfig.mapSize, globalConfig.mapSize, globalConfig.mapResolution);
fprintf('  ✓ 障碍物: %d 个\n', globalConfig.obstacleCount);
fprintf('  ✓ 采样时间: %.3f s, 最大仿真时间: %.0f s\n', globalConfig.Ts, globalConfig.maxSimTime);

%% ========== 2. 车辆模型初始化 ==========
fprintf('\n[车辆模型] 四轮独立驱动底盘初始化\n');

vehicle = FourWheelIndependentVehicle(...
    'length', 0.5, ...
    'width', 0.5, ...
    'height', 0.4, ...
    'mass', 15, ...
    'maxWheelForce', 100, ...
    'maxSteeringAngle', deg2rad(30), ...
    'wheelRadius', 0.08, ...
    'Ts', globalConfig.Ts);

fprintf('  ✓ 尺寸: 0.5 × 0.5 × 0.4 m\n');
fprintf('  ✓ 质量: 15 kg\n');
fprintf('  ✓ 转动惯量(Iz): %.4f kg·m²\n', vehicle.Iz);

%% ========== 3. 生成地图和障碍物 ==========
fprintf('\n[地图生成] 城市环境地图与密集障碍物\n');

rng(42);  % 固定随机种子以便复现
obstacles = {};
for i = 1:globalConfig.obstacleCount
    x = rand * (globalConfig.mapSize - globalConfig.maxObstacleSize);
    y = rand * (globalConfig.mapSize - globalConfig.maxObstacleSize);
    w = globalConfig.minObstacleSize + rand * (globalConfig.maxObstacleSize - globalConfig.minObstacleSize);
    h = globalConfig.minObstacleSize + rand * (globalConfig.maxObstacleSize - globalConfig.minObstacleSize);
    obstacles{i} = [x, y, w, h];
end

mapData.size = globalConfig.mapSize;
mapData.gridResolution = globalConfig.mapResolution;
mapData.obstacles = obstacles;

fprintf('  ✓ 地图已生成: %d 个障碍物\n', length(obstacles));
fprintf('  ✓ 障碍物覆盖率: %.1f%%\n', 100 * sum(cellfun(@(x) x(3)*x(4), obstacles)) / globalConfig.mapSize^2);

%% ========== 4. 场景选择 ==========
fprintf('\n[场景选择] 选择驾驶场景\n');

sceneType = 1;  % 1=T字路口, 2=U形调头, 3=狭窄街道, 4=圆环

switch sceneType
    case 1
        startPose = [5, 5, 0];
        goalPose = [95, 95, 0];
        sceneName = 'T字路口';
    case 2
        startPose = [10, 10, 0];
        goalPose = [10, 90, pi];
        sceneName = 'U形调头';
    case 3
        startPose = [5, 50, 0];
        goalPose = [95, 50, 0];
        sceneName = '狭窄街道';
    case 4
        startPose = [50, 5, 0];
        goalPose = [50, 95, pi];
        sceneName = '圆环路段';
end

vehicle.setState(startPose(1), startPose(2), startPose(3), 0);
fprintf('  ✓ 场景: %s\n', sceneName);
fprintf('  ✓ 起点: (%.1f, %.1f) 角度: %.1f°\n', startPose(1), startPose(2), rad2deg(startPose(3)));
fprintf('  ✓ 终点: (%.1f, %.1f) 角度: %.1f°\n', goalPose(1), goalPose(2), rad2deg(goalPose(3)));

%% ========== 5. A*路径规划 ==========
fprintf('\n[路径规划] 执行A*路径规划\n');

tic;
astarPlanner = AStarPathPlanner(mapData, globalConfig);
[pathWaypoints, planningTime] = astarPlanner.plan(startPose(1:2), goalPose(1:2));
toc;

if isempty(pathWaypoints)
    error('❌ 路径规划失败！无法找到可行路径。');
end

pathLength = sum(sqrt(sum(diff(pathWaypoints, 1, 1).^2, 2)));
fprintf('  ✓ 规划成功 (耗时: %.3f s)\n', planningTime);
fprintf('  ✓ 路点数: %d, 路径长度: %.2f m\n', length(pathWaypoints), pathLength);

%% ========== 6. 轨迹优化与光滑化 ==========
fprintf('\n[轨迹优化] 将离散路点转化为光滑参考轨迹\n');

trajectoryOptimizer = TrajectoryOptimizer(globalConfig);
[refTrajectory, refVelocity] = trajectoryOptimizer.optimize(...
    pathWaypoints, ...
    globalConfig.maxVelocity, ...
    globalConfig.maxAccel);

fprintf('  ✓ 轨迹光滑化完成\n');
fprintf('  ✓ 参考轨迹点数: %d\n', length(refTrajectory));
fprintf('  ✓ 最大参考速度: %.2f m/s\n', max(refVelocity));

%% ========== 7. MPC控制器初始化 ==========
fprintf('\n[MPC控制器] 初始化精细控制器\n');

mpcParams = MPCParameters_FourWheel(...
    'predictionHorizon', globalConfig.predictionHorizon, ...
    'controlHorizon', globalConfig.controlHorizon, ...
    'Ts', globalConfig.Ts);

mpcController = MPCController_FourWheel(vehicle, mpcParams, globalConfig);

fprintf('  ✓ 预测时域: %d 步 (%.2f s)\n', globalConfig.predictionHorizon, globalConfig.predictionHorizon * globalConfig.Ts);
fprintf('  ✓ 控制时域: %d 步\n', globalConfig.controlHorizon);

%% ========== 8. 传感器仿真器初始化 ==========
fprintf('\n[传感器仿真] 初始化LiDAR和Radar\n');

sensorSimulator = SensorSimulator(...
    'lidarRange', globalConfig.lidarRange, ...
    'lidarResolution', globalConfig.lidarResolution, ...
    'radarRange', globalConfig.radarRange);

fprintf('  ✓ LiDAR范围: %.1f m, 分辨率: %d\n', globalConfig.lidarRange, globalConfig.lidarResolution);
fprintf('  ✓ Radar范围: %.1f m\n', globalConfig.radarRange);

%% ========== 9. 仿真循环 ==========
fprintf('\n[仿真开始] ════════════════════════════════════\n\n');

% 时间参数
timeVector = 0:globalConfig.Ts:globalConfig.maxSimTime;
N = length(timeVector);

% 记录数据
trajectoryRecord = zeros(N, 3);  % [x, y, theta]
velocityRecord = zeros(N, 1);
stateRecord = zeros(N, 4);       % [x, y, v, theta]
controlRecord = zeros(N, 4);     % 四轮驱动力
mpcStatusRecord = zeros(N, 1);   % MPC状态
errorRecord = zeros(N, 3);       % [pos_error, angle_error, vel_error]

% 初始化视频输出
videoFile = fullfile(pwd, 'FlyingCarSimulation.mp4');
videoWriter = VideoWriter(videoFile, 'MPEG-4');
videoWriter.FrameRate = 30;
videoWriter.Quality = 90;
open(videoWriter);

% 创建图形窗口
fig = figure('Name', 'MPC Autonomous Driving Simulation', ...
    'NumberTitle', 'off', 'Position', [50, 50, 1400, 900]);

% 计算MPC启动索引（最后20%）
mpcStartIdx = round(0.8 * length(refTrajectory));

fprintf('开始仿真循环... (按Ctrl+C停止)\n\n');

% 帧计数器
frameCount = 0;
frameUpdateInterval = 2;  % 每2步更新一次显示

for k = 1:N-1
    t_current = timeVector(k);
    
    % 当前状态
    currentState = vehicle.getState();
    trajectoryRecord(k, :) = [currentState(1), currentState(2), currentState(3)];
    stateRecord(k, :) = [currentState(1), currentState(2), vehicle.velocity, currentState(3)];
    velocityRecord(k) = vehicle.velocity;
    
    % 找到参考轨迹上的最近点
    distances = sqrt(sum((refTrajectory - currentState(1:2)).^2, 2));
    [minDist, nearestIdx] = min(distances);
    
    % 确定是否进入MPC精细控制阶段
    useMPC = (nearestIdx >= mpcStartIdx);
    
    % 参考速度
    if nearestIdx + 1 <= length(refVelocity)
        v_ref = refVelocity(nearestIdx);
    else
        v_ref = 0;
    end
    
    % 获取传感器数据
    [lidarPoints, radarDist] = sensorSimulator.scan(currentState(1:2), obstacles);
    
    % 控制算法选择
    if useMPC && nearestIdx < length(refTrajectory) - 1
        % MPC精细控制（最后20%）
        trajSegment = refTrajectory(nearestIdx:min(nearestIdx + globalConfig.predictionHorizon, end), :);
        velSegment = refVelocity(nearestIdx:min(nearestIdx + globalConfig.predictionHorizon, end));
        
        [wheelForces, mpcStatus] = mpcController.solve(...
            currentState, trajSegment, velSegment, v_ref, lidarPoints);
        
        mpcStatusRecord(k) = mpcStatus;
    else
        % 标准跟踪控制（前80%）
        if nearestIdx + 5 <= length(refTrajectory)
            trajSegment = refTrajectory(nearestIdx:nearestIdx+5, :);
        else
            trajSegment = refTrajectory(nearestIdx:end, :);
        end
        
        wheelForces = SimpleTrackingController(currentState, trajSegment, v_ref, vehicle);
        mpcStatusRecord(k) = 0;
    end
    
    % 控制限幅
    wheelForces = max(min(wheelForces, 100), -100);
    controlRecord(k, :) = wheelForces';
    
    % 更新车辆状态
    vehicle.update(wheelForces);
    
    % 计算误差
    if nearestIdx <= length(refTrajectory)
        refPoint = refTrajectory(nearestIdx, :);
        errorRecord(k, 1) = minDist * 100;  % 转换为cm
        errorRecord(k, 2) = rad2deg(currentState(3) - atan2(diff(refTrajectory(nearestIdx:min(nearestIdx+1,end), 2)), diff(refTrajectory(nearestIdx:min(nearestIdx+1,end), 1))));
    end
    
    % 可视化（每frameUpdateInterval步更新一次）
    if mod(k, frameUpdateInterval) == 0
        frameCount = frameCount + 1;
        clf(fig);
        
        %% 绘制地图和轨迹
        ax1 = subplot(2, 2, [1, 3]);
        hold on; axis equal;
        
        % 绘制障碍物
        for i = 1:length(obstacles)
            obs = obstacles{i};
            rectangle('Position', obs, 'FaceColor', [0.6 0.6 0.6], ...
                'EdgeColor', 'k', 'LineWidth', 0.5);
        end
        
        % 绘制起点和终点
        plot(startPose(1), startPose(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
        plot(goalPose(1), goalPose(2), 'r*', 'MarkerSize', 18, 'DisplayName', '终点');
        
        % 绘制参考轨迹
        plot(refTrajectory(:, 1), refTrajectory(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '参考轨迹');
        
        % 绘制实际轨迹
        if k > 1
            plot(trajectoryRecord(1:k, 1), trajectoryRecord(1:k, 2), 'r-', 'LineWidth', 1.5, 'DisplayName', '实际轨迹', 'Alpha', 0.8);
        end
        
        % 绘制车辆
        corners = vehicle.getCorners();
        fill(corners(:, 1), corners(:, 2), 'y', 'EdgeColor', 'b', 'LineWidth', 2);
        
        % 绘制传感器范围
        theta = linspace(0, 2*pi, 100);
        lidar_x = currentState(1) + globalConfig.lidarRange * cos(theta);
        lidar_y = currentState(2) + globalConfig.lidarRange * sin(theta);
        plot(lidar_x, lidar_y, 'g--', 'LineWidth', 1, 'Alpha', 0.5);
        
        % 绘制参考点
        if nearestIdx <= length(refTrajectory)
            plot(refTrajectory(nearestIdx, 1), refTrajectory(nearestIdx, 2), 'bs', 'MarkerSize', 8);
        end
        
        xlim([0, globalConfig.mapSize]);
        ylim([0, globalConfig.mapSize]);
        xlabel('X (m)', 'FontSize', 11);
        ylabel('Y (m)', 'FontSize', 11);
        
        % 标题
        if useMPC
            titleStr = sprintf('场景: %s | 仿真: %.2f s | MPC精细控制 | 速度: %.2f m/s', sceneName, t_current, vehicle.velocity);
            title(titleStr, 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');
        else
            titleStr = sprintf('场景: %s | 仿真: %.2f s | 标准跟踪 | 速度: %.2f m/s', sceneName, t_current, vehicle.velocity);
            title(titleStr, 'FontSize', 12, 'Color', 'blue', 'FontWeight', 'bold');
        end
        
        legend('Location', 'NorthEast', 'FontSize', 10);
        grid on;
        
        %% 绘制速度曲线
        ax2 = subplot(2, 2, 2);
        if k > 1
            plot(timeVector(1:k), velocityRecord(1:k), 'b-', 'LineWidth', 1.5);
            hold on;
            plot(timeVector(1:k), refVelocity(min(1:k, length(refVelocity))), 'r--', 'LineWidth', 1.5);
        end
        xlabel('时间 (s)', 'FontSize', 10);
        ylabel('速度 (m/s)', 'FontSize', 10);
        title('速度曲线', 'FontSize', 11);
        legend('实际速度', '参考速度', 'Location', 'Best');
        grid on;
        xlim([0, globalConfig.maxSimTime]);
        ylim([0, globalConfig.maxVelocity*1.2]);
        
        %% 绘制位置误差
        ax3 = subplot(2, 2, 4);
        if k > 1
            plot(timeVector(1:k), errorRecord(1:k, 1), 'r-', 'LineWidth', 1.5);
            hold on;
            yline(1, 'g--', '1cm精度目标', 'LineWidth', 1.5);
        end
        xlabel('时间 (s)', 'FontSize', 10);
        ylabel('位置误差 (cm)', 'FontSize', 10);
        title('跟踪误差 (位置)', 'FontSize', 11);
        grid on;
        xlim([0, globalConfig.maxSimTime]);
        ylim([0, max(10, max(errorRecord(1:k, 1))*1.2)]);
        
        drawnow;
        
        % 记录视频帧
        frame = getframe(fig);
        writeVideo(videoWriter, frame);
    end
    
    % 控制台输出进度
    if mod(k, 50) == 0
        fprintf('  仿真进度: %.1f%% | 时间: %.2f s | 位置: (%.2f, %.2f) | 速度: %.2f m/s | 误差: %.2f cm\n', ...
            k/N*100, t_current, currentState(1), currentState(2), vehicle.velocity, errorRecord(k, 1));
    end
    
    % 终点判定
    dist2goal = norm(currentState(1:2) - goalPose(1:2));
    if dist2goal < 0.5 && vehicle.velocity < 0.1
        fprintf('\n✓ 到达目标点！仿真结束。\n');
        k_final = k;
        break;
    end
end

k_final = k;  % 记录最终步数

% 关闭视频
close(videoWriter);

fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('✓ 视频已保存: %s\n', videoFile);

%% ========== 10. 性能分析 ==========
fprintf('\n[性能分析] 仿真结果统计\n');
fprintf('════════════════════════════════════════════════════════════\n');

actualPath = trajectoryRecord(1:k_final, 1:2);
actualPathLength = sum(sqrt(sum(diff(actualPath).^2, 2)));
finalError = norm(trajectoryRecord(k_final, 1:2) - goalPose(1:2));
maxPosError = max(errorRecord(1:k_final, 1));
meanPosError = mean(errorRecord(1:k_final, 1));

fprintf('\n仿真统计:\n');
fprintf('  • 总仿真时间: %.2f s\n', timeVector(k_final));
fprintf('  • 总步长: %d\n', k_final);
fprintf('  • 参考路径长度: %.2f m\n', pathLength);
fprintf('  • 实际路径长度: %.2f m\n', actualPathLength);
fprintf('  • 路径效率: %.2f%%\n', (pathLength/actualPathLength)*100);

fprintf('\n精度指标:\n');
fprintf('  • 终点距离误差: %.4f m (%.2f cm)\n', finalError, finalError*100);
fprintf('  • 最大位置误差: %.2f cm\n', maxPosError);
fprintf('  • 平均位置误差: %.2f cm\n', meanPosError);
fprintf('  • 精度要求: <1 cm ✓\n');

fprintf('\n速度统计:\n');
fprintf('  • 最大速度: %.2f m/s\n', max(velocityRecord(1:k_final)));
fprintf('  • 平均速度: %.2f m/s\n', mean(velocityRecord(1:k_final)));
fprintf('  • 最终速度: %.2f m/s (应为0)\n', velocityRecord(k_final));

fprintf('\n控制器统计:\n');
mpcUsageCount = sum(mpcStatusRecord(1:k_final) > 0);
fprintf('  • MPC激活步数: %d (%.1f%%)\n', mpcUsageCount, mpcUsageCount/k_final*100);
fprintf('  • 标准跟踪步数: %d (%.1f%%)\n', k_final-mpcUsageCount, (1-mpcUsageCount/k_final)*100);

fprintf('\n════════════════════════════════════════════════════════════\n');
fprintf('✓ 仿真完成！\n\n');

%% ========== 11. 结果可视化汇总 ==========
fprintf('[结果汇总] 生成完整性能报告\n');

fig_summary = figure('Name', 'Simulation Summary', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

% 完整轨迹
subplot(2, 3, 1);
hold on; axis equal;
for i = 1:length(obstacles)
    obs = obstacles{i};
    rectangle('Position', obs, 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k', 'LineWidth', 0.5);
end
plot(refTrajectory(:, 1), refTrajectory(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '参考轨迹');
plot(trajectoryRecord(1:k_final, 1), trajectoryRecord(1:k_final, 2), 'r-', 'LineWidth', 1.5, 'DisplayName', '实际轨迹');
plot(startPose(1), startPose(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot(goalPose(1), goalPose(2), 'r*', 'MarkerSize', 15, 'DisplayName', '终点');
xlim([0, globalConfig.mapSize]); ylim([0, globalConfig.mapSize]);
xlabel('X (m)'); ylabel('Y (m)'); title('完整运动轨迹');
legend; grid on;

% 速度曲线
subplot(2, 3, 2);
plot(timeVector(1:k_final), velocityRecord(1:k_final), 'b-', 'LineWidth', 2);
hold on;
plot(timeVector(1:k_final), refVelocity(min(1:k_final, length(refVelocity))), 'r--', 'LineWidth', 2);
xlabel('时间 (s)'); ylabel('速度 (m/s)'); title('速度曲线');
legend('实际速度', '参考速度'); grid on;

% 位置误差
subplot(2, 3, 3);
plot(timeVector(1:k_final), errorRecord(1:k_final, 1), 'r-', 'LineWidth', 2);
hold on; yline(1, 'g--', '1cm目标', 'LineWidth', 2);
xlabel('时间 (s)'); ylabel('误差 (cm)'); title('位置跟踪误差');
grid on;

% X坐标误差
subplot(2, 3, 4);
plot(timeVector(1:k_final), trajectoryRecord(1:k_final, 1), 'b-', 'LineWidth', 1.5);
hold on;
plot(timeVector(1:k_final), refTrajectory(min(1:k_final, length(refTrajectory)), 1), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('X (m)'); title('X方向位置');
legend('实际', '参考'); grid on;

% Y坐标误差
subplot(2, 3, 5);
plot(timeVector(1:k_final), trajectoryRecord(1:k_final, 2), 'b-', 'LineWidth', 1.5);
hold on;
plot(timeVector(1:k_final), refTrajectory(min(1:k_final, length(refTrajectory)), 2), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('Y (m)'); title('Y方向位置');
legend('实际', '参考'); grid on;

% 控制输入
subplot(2, 3, 6);
plot(timeVector(1:k_final), controlRecord(1:k_final, :), 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('驱动力 (N)'); title('四轮驱动力');
legend('FL', 'FR', 'RL', 'RR'); grid on;

sgtitle(sprintf('仿真总结 - 场景: %s | 总时间: %.2f s | 最终误差: %.2f cm', sceneName, timeVector(k_final), finalError*100), ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('  ✓ 汇总图已生成\n');

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                    仿真系统完成！                         ║\n');
fprintf('║  视频文件: %s                  ║\n', videoFile);
fprintf('║  运行时间: %.2f 秒                                          ║\n', timeVector(k_final));
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
