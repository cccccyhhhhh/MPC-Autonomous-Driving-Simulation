classdef MPCParameters_FourWheel
    % MPCParameters_FourWheel - 四轮独立驱动MPC参数配置
    %
    % 功能：
    %   针对四轮独立驱动车辆的MPC控制器参数配置
    %   支持多个预设场景（高速/城市/泊车）
    %
    % 作者: CCCCCYHHHHH
    % 日期: 2025年1月
    
    properties
        %% MPC时域参数
        predictionHorizon       % 预测时域步数
        controlHorizon          % 控制时域步数
        Ts                      % 采样周期 (s)
        
        %% 成本函数权重
        Q_x                     % x位置权重
        Q_y                     % y位置权重
        Q_theta                 % 方向角权重
        Q_v                     % 速度权重
        R_force                 % 驱动力权重
        P_x                     % 终端x权重
        P_y                     % 终端y权重
        P_theta                 % 终端方向角权重
        
        %% 约束条件
        maxForce                % 每轮最大驱动力 (N)
        maxVelocity             % 最大速度 (m/s)
        minVelocity             % 最小速度 (m/s)
        maxAccel                % 最大加速度 (m/s²)
        minAccel                % 最大减速度 (m/s²)
        
        %% 安全约束
        collisionMargin         % 碰撞避免安全距离 (m)
        
    end
    
    methods
        
        %% 构造函数
        function params = MPCParameters_FourWheel(varargin)
            % 初始化MPC参数
            %
            % 语法:
            %   params = MPCParameters_FourWheel()
            %   params = MPCParameters_FourWheel('predictionHorizon', 25)
            
            p = inputParser;
            addParameter(p, 'predictionHorizon', 20);
            addParameter(p, 'controlHorizon', 5);
            addParameter(p, 'Ts', 0.05);
            parse(p, varargin{:});
            
            %% 默认参数
            params.predictionHorizon = p.Results.predictionHorizon;
            params.controlHorizon = p.Results.controlHorizon;
            params.Ts = p.Results.Ts;
            
            % 成本函数权重
            params.Q_x = 100;           % 强化位置跟踪
            params.Q_y = 100;
            params.Q_theta = 50;        % 方向角权重
            params.Q_v = 10;            % 速度权重
            params.R_force = 5;         % 控制输入平滑性
            params.P_x = 150;           % 终端权重
            params.P_y = 150;
            params.P_theta = 100;
            
            % 约束条件
            params.maxForce = 100;      % N
            params.maxVelocity = 5;     % m/s
            params.minVelocity = 0;     % m/s
            params.maxAccel = 2.0;      % m/s²
            params.minAccel = -3.0;     % m/s²
            
            % 安全约束
            params.collisionMargin = 0.5;  % 50cm安全距离
        end
        
        %% 显示参数
        function display(params)
            fprintf('\n========== MPC参数配置 (四轮驱动) ==========\n');
            fprintf('预测时域: %d 步 (%.2f s)\n', ...
                params.predictionHorizon, params.predictionHorizon * params.Ts);
            fprintf('控制时域: %d 步\n', params.controlHorizon);
            fprintf('采样周期: %.3f s\n\n', params.Ts);
            
            fprintf('成本函数权重:\n');
            fprintf('  Q = [%.1f, %.1f, %.1f, %.1f] (x, y, theta, v)\n', ...
                params.Q_x, params.Q_y, params.Q_theta, params.Q_v);
            fprintf('  R = %.1f (驱动力)\n', params.R_force);
            fprintf('  P = [%.1f, %.1f, %.1f] (终端)\n\n', ...
                params.P_x, params.P_y, params.P_theta);
            
            fprintf('约束条件:\n');
            fprintf('  速度: [%.1f, %.1f] m/s\n', params.minVelocity, params.maxVelocity);
            fprintf('  加速度: [%.1f, %.1f] m/s²\n', params.minAccel, params.maxAccel);
            fprintf('  最大驱动力: %.1f N\n\n', params.maxForce);
            fprintf('=========================================\n\n');
        end
        
    end
end