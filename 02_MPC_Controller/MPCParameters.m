classdef MPCParameters
    % MPCParameters - MPC控制器参数配置
    %
    % 描述：
    %   该类集中管理MPC控制器的所有参数，便于快速调整和对比分析
    %
    % 包含内容：
    %   - MPC预测和控制时域参数
    %   - 成本函数权重
    %   - 物理约束条件
    %   - 车辆参数
    %
    % 作者: CCCCCYHHHHH
    % 日期: 2025年1月
    
    properties
        
        %% ========== MPC时域参数 ==========
        PredictionHorizon      % 预测时域步数 (steps)
        ControlHorizon         % 控制时域步数 (steps)
        SamplingTime           % 采样周期 (s)
        
        %% ========== 成本函数权重 ==========
        % 状态权重（跟踪误差）
        Q_x                    % x位置权重
        Q_y                    % y位置权重
        Q_theta                % 方向角权重
        Q_v                    % 速度权重
        
        % 控制输入权重
        R_delta                % 转向角权重
        R_a                    % 加速度权重
        
        % 终端成本权重
        P_x                    % 终端x权重
        P_y                    % 终端y权重
        P_theta                % 终端方向权重
        
        %% ========== 物理约束 ==========
        % 转向约束
        DeltaMax               % 最大转向角 (rad)
        DeltaMin               % 最小转向角 (rad)
        DeltaRateMax           % 最大转向速率 (rad/s)
        
        % 速度约束
        VelocityMax            % 最大速度 (m/s)
        VelocityMin            % 最小速度 (m/s)
        
        % 加速度约束
        AccelMax               % 最大加速度 (m/s²)
        AccelMin               % 最大减速度 (m/s²)
        JerkMax                % 最大加速度变化率 (m/s³)
        
        % 舒适性约束
        MaxYawRate             % 最大横摆角速度 (rad/s)
        
        %% ========== 车辆参数 ==========
        VehicleLength          % 车辆长度 (m)
        VehicleWidth           % 车辆宽度 (m)
        Wheelbase              % 轴距 (m)
        VehicleMass            % 车辆质量 (kg)
        
        %% ========== 环境参数 ==========
        LaneWidth              % 车道宽度 (m)
        SafetyMargin           % 安全距离 (m)
        
    end
    
    methods
        
        %% 构造函数 - 默认参数配置
        function params = MPCParameters(varargin)
            % 初始化MPC参数
            %
            % 语法:
            %   params = MPCParameters()
            %   params = MPCParameters('PredictionHorizon', 25, 'SamplingTime', 0.05)
            
            %% ========== 默认MPC时域参数 ==========
            params.PredictionHorizon = 20;      % 预测2秒（20步 * 0.1s）
            params.ControlHorizon = 5;          % 控制5步，其后保持不变
            params.SamplingTime = 0.1;          % 100ms采样周期
            
            %% ========== 默认成本函数权重 ==========
            % 状态权重 - 跟踪精度要求
            params.Q_x = 100;                   % 位置x跟踪权重
            params.Q_y = 100;                   % 位置y跟踪权重
            params.Q_theta = 50;                % 方向角跟踪权重
            params.Q_v = 10;                    % 速度跟踪权重
            
            % 控制权重 - 控制输入平滑性
            params.R_delta = 10;                % 转向输入权重（避免剧烈转向）
            params.R_a = 5;                     % 加速度权重（避免剧烈加速减速）
            
            % 终端权重 - 保证最后阶段跟踪
            params.P_x = 150;
            params.P_y = 150;
            params.P_theta = 100;
            
            %% ========== 默认物理约束 ==========
            % 转向约束（汽车转向范围通常±35°）
            params.DeltaMax = deg2rad(35);      % 35度最大转向角
            params.DeltaMin = -deg2rad(35);     % -35度最小转向角
            params.DeltaRateMax = deg2rad(30);  % 30度/秒转向速率限制
            
            % 速度约束
            params.VelocityMax = 20;            % 20 m/s ≈ 72 km/h
            params.VelocityMin = 0;             % 停止
            
            % 加速度约束
            params.AccelMax = 2;                % 2 m/s² 加速
            params.AccelMin = -3;               % -3 m/s² 制动
            params.JerkMax = 1;                 % 1 m/s³ 加速度变化率
            
            % 舒适性约束
            params.MaxYawRate = deg2rad(45);    % 最大横摆速度
            
            %% ========== 默认车辆参数 ==========
            params.VehicleLength = 4.5;         % 标准乘用车长度
            params.VehicleWidth = 1.8;          % 标准乘用车宽度
            params.Wheelbase = 2.7;             % 轴距
            params.VehicleMass = 1500;          % 车辆质量
            
            %% ========== 默认环境参数 ==========
            params.LaneWidth = 3.75;            % 标准车道宽度
            params.SafetyMargin = 0.3;          % 30cm安全裕度
            
            %% ========== 解析可变参数 ==========
            if nargin > 0
                p = inputParser;
                
                % 添加所有可能的参数
                addParameter(p, 'PredictionHorizon', params.PredictionHorizon);
                addParameter(p, 'ControlHorizon', params.ControlHorizon);
                addParameter(p, 'SamplingTime', params.SamplingTime);
                addParameter(p, 'Q_x', params.Q_x);
                addParameter(p, 'Q_y', params.Q_y);
                addParameter(p, 'Q_theta', params.Q_theta);
                addParameter(p, 'Q_v', params.Q_v);
                addParameter(p, 'R_delta', params.R_delta);
                addParameter(p, 'R_a', params.R_a);
                addParameter(p, 'DeltaMax', params.DeltaMax);
                addParameter(p, 'AccelMax', params.AccelMax);
                addParameter(p, 'VelocityMax', params.VelocityMax);
                
                parse(p, varargin{:});
                
                % 更新参数
                params.PredictionHorizon = p.Results.PredictionHorizon;
                params.ControlHorizon = p.Results.ControlHorizon;
                params.SamplingTime = p.Results.SamplingTime;
                params.Q_x = p.Results.Q_x;
                params.Q_y = p.Results.Q_y;
                params.Q_theta = p.Results.Q_theta;
                params.Q_v = p.Results.Q_v;
                params.R_delta = p.Results.R_delta;
                params.R_a = p.Results.R_a;
                params.DeltaMax = p.Results.DeltaMax;
                params.AccelMax = p.Results.AccelMax;
                params.VelocityMax = p.Results.VelocityMax;
            end
            
        end
        
        %% 显示参数信息
        function displayParameters(params)
            % 以可读的形式显示所有参数
            
            fprintf('\n========== MPC参数配置 ==========\n');
            fprintf('预测时域: %d 步 (%.1f秒)\n', params.PredictionHorizon, ...
                    params.PredictionHorizon * params.SamplingTime);
            fprintf('控制时域: %d 步\n', params.ControlHorizon);
            fprintf('采样周期: %.3f 秒\n\n', params.SamplingTime);
            
            fprintf('成本函数权重:\n');
            fprintf('  Q_x=%.1f, Q_y=%.1f, Q_theta=%.1f, Q_v=%.1f\n', ...
                    params.Q_x, params.Q_y, params.Q_theta, params.Q_v);
            fprintf('  R_delta=%.1f, R_a=%.1f\n\n', params.R_delta, params.R_a);
            
            fprintf('物理约束:\n');
            fprintf('  转向角: [%.1f, %.1f]°\n', ...
                    rad2deg(params.DeltaMin), rad2deg(params.DeltaMax));
            fprintf('  速度: [%.1f, %.1f] m/s\n', params.VelocityMin, params.VelocityMax);
            fprintf('  加速度: [%.1f, %.1f] m/s²\n\n', params.AccelMin, params.AccelMax);
            
            fprintf('车辆参数:\n');
            fprintf('  轴距: %.2f m, 质量: %.0f kg\n', params.Wheelbase, params.VehicleMass);
            fprintf('=====================================\n\n');
            
        end
        
        %% 预设：高速公路参数
        function params = highwayPreset(params)
            % 为高速驾驶调整参数
            
            params.PredictionHorizon = 30;      % 更长的预测时域
            params.ControlHorizon = 10;
            params.SamplingTime = 0.05;         % 更短的采样周期
            
            params.Q_y = 50;                    % 降低横向精度要求
            params.Q_theta = 30;
            params.R_delta = 5;                 % 更平滑的转向
            
            params.VelocityMax = 35;            % 125 km/h
            params.AccelMax = 1;
            params.AccelMin = -2;
            
        end
        
        %% 预设：城市道路参数
        function params = urbanPreset(params)
            % 为城市驾驶调整参数
            
            params.PredictionHorizon = 15;      % 较短的预测时域
            params.ControlHorizon = 5;
            params.SamplingTime = 0.1;
            
            params.Q_y = 150;                   % 提高横向精度
            params.Q_theta = 80;
            params.R_a = 10;                    % 更平滑的加速减速
            
            params.VelocityMax = 15;            % 54 km/h
            params.AccelMax = 2.5;
            params.AccelMin = -3.5;
            
        end
        
        %% 预设：泊车参数
        function params = parkingPreset(params)
            % 为自动泊车调整参数
            
            params.PredictionHorizon = 50;      % 很长的预测时域
            params.ControlHorizon = 10;
            params.SamplingTime = 0.05;
            
            params.Q_x = 200;                   % 非常高的位置精度
            params.Q_y = 200;
            params.Q_theta = 150;
            params.R_delta = 50;                % 最大化平滑性
            
            params.VelocityMax = 3;             % 低速
            params.AccelMax = 0.5;
            params.AccelMin = -1;
            
        end
        
    end
    
end
