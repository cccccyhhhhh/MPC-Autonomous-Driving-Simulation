classdef FourWheelIndependentVehicle
    % FourWheelIndependentVehicle - 四轮独立驱动车辆模型
    %
    % 描述：
    %   分体式飞行汽车底盘的四轮独立驱动动力学模型
    %   每个轮子可以独立控制驱动力
    %
    % 参数：
    %   - 长×宽×高: 0.5m × 0.5m × 0.4m
    %   - 质量: 10-20kg（默认15kg）
    %   - 最大转向角: ±30°
    %   - 每轮最大驱动力: 100N
    %
    % 作者: CCCCCYHHHHH
    % 日期: 2025年1月
    
    properties
        % 车辆几何参数
        length              % 车长 (m)
        width               % 车宽 (m)
        height              % 车高 (m)
        wheelRadius         % 轮半径 (m)
        
        % 动力学参数
        mass                % 车辆质量 (kg)
        Iz                  % 绕z轴转动惯量 (kg*m²)
        maxWheelForce       % 每轮最大驱动力 (N)
        maxSteeringAngle    % 最大转向角 (rad)
        
        % 状态变量
        x                   % 全局坐标x位置 (m)
        y                   % 全局坐标y位置 (m)
        theta               % 方向角 (rad)
        velocity            % 车速 (m/s)
        omega               % 角速度 (rad/s)
        
        % 采样时间
        Ts                  % 采样周期 (s)
        
        % 历史记录
        history             % 状态历史
    end
    
    methods
        
        %% 构造函数
        function obj = FourWheelIndependentVehicle(varargin)
            % 初始化四轮独立驱动车辆模型
            %
            % 语法:
            %   vehicle = FourWheelIndependentVehicle('length', 0.5, 'mass', 15, 'Ts', 0.05)
            
            % 参数解析
            p = inputParser;
            addParameter(p, 'length', 0.5);
            addParameter(p, 'width', 0.5);
            addParameter(p, 'height', 0.4);
            addParameter(p, 'mass', 15);
            addParameter(p, 'wheelRadius', 0.08);
            addParameter(p, 'maxWheelForce', 100);
            addParameter(p, 'maxSteeringAngle', deg2rad(30));
            addParameter(p, 'Ts', 0.05);
            parse(p, varargin{:});
            
            % 几何参数
            obj.length = p.Results.length;
            obj.width = p.Results.width;
            obj.height = p.Results.height;
            obj.wheelRadius = p.Results.wheelRadius;
            
            % 动力学参数
            obj.mass = p.Results.mass;
            obj.maxWheelForce = p.Results.maxWheelForce;
            obj.maxSteeringAngle = p.Results.maxSteeringAngle;
            
            % 估算转动惯量 (假设均匀分布)
            % Iz ≈ (1/12) * m * (L² + W²)
            obj.Iz = (1/12) * obj.mass * (obj.length^2 + obj.width^2);
            
            % 采样时间
            obj.Ts = p.Results.Ts;
            
            % 初始状态
            obj.x = 0;
            obj.y = 0;
            obj.theta = 0;
            obj.velocity = 0;
            obj.omega = 0;
            
            % 历史记录
            obj.history = struct('x', [], 'y', [], 'theta', [], 'v', [], 'omega', []);
        end
        
        %% 获取状态
        function state = getState(obj)
            % 获取当前状态 [x, y, theta, v]
            state = [obj.x; obj.y; obj.theta; obj.velocity];
        end
        
        %% 设置状态
        function obj = setState(obj, x, y, theta, v)
            % 设置车辆状态
            obj.x = x;
            obj.y = y;
            obj.theta = atan2(sin(theta), cos(theta));  % 归一化到[-pi, pi]
            obj.velocity = v;
        end
        
        %% 更新车辆动力学
        function obj = update(obj, wheelForces)
            % 更新车辆状态
            % wheelForces: [FL, FR, RL, RR] 四轮驱动力 (N)
            
            if length(wheelForces) ~= 4
                error('输入必须为4个轮子的驱动力');
            end
            
            % 限幅
            wheelForces = max(min(wheelForces, obj.maxWheelForce), -obj.maxWheelForce);
            
            % 记录历史
            obj.history.x = [obj.history.x; obj.x];
            obj.history.y = [obj.history.y; obj.y];
            obj.history.theta = [obj.history.theta; obj.theta];
            obj.history.v = [obj.history.v; obj.velocity];
            obj.history.omega = [obj.history.omega; obj.omega];
            
            % 计算四轮的轮周速度
            % v_wheel = F / (mass/4) 的近似
            wheelVelocities = wheelForces * obj.wheelRadius / obj.mass;
            
            % 纵向速度: 平均四轮速度
            avg_wheel_vel = mean(wheelVelocities);
            
            % 角速度: 由左右轮速差产生
            left_avg = (wheelVelocities(1) + wheelVelocities(3)) / 2;
            right_avg = (wheelVelocities(2) + wheelVelocities(4)) / 2;
            
            % 差动转向
            omega_dot = (right_avg - left_avg) * obj.wheelRadius / obj.width * 2;
            
            % 加速度限制（实际物理约束）
            max_accel = 2.0;  % m/s²
            accel = (avg_wheel_vel - obj.velocity) / obj.Ts;
            if abs(accel) > max_accel
                accel = sign(accel) * max_accel;
            end
            
            % 状态更新（欧拉法）
            obj.velocity = obj.velocity + accel * obj.Ts;
            obj.omega = obj.omega + omega_dot * obj.Ts;
            
            % 位置更新
            obj.x = obj.x + obj.velocity * cos(obj.theta) * obj.Ts;
            obj.y = obj.y + obj.velocity * sin(obj.theta) * obj.Ts;
            obj.theta = obj.theta + obj.omega * obj.Ts;
            
            % 角度归一化
            obj.theta = atan2(sin(obj.theta), cos(obj.theta));
            
            % 速度约束
            obj.velocity = max(min(obj.velocity, 5), -1);
        end
        
        %% 获取车身四个角的位置
        function corners = getCorners(obj)
            % 获取车身在全局坐标系中的四个角位置
            % 输出: [4x2] 矩阵，每行表示一个角的(x,y)坐标
            
            % 局部坐标系中的四个角
            half_len = obj.length / 2;
            half_width = obj.width / 2;
            
            corners_local = [
                -half_width, -half_len;
                 half_width, -half_len;
                 half_width,  half_len;
                -half_width,  half_len
            ];
            
            % 旋转矩阵
            R = [cos(obj.theta), -sin(obj.theta);
                 sin(obj.theta),  cos(obj.theta)];
            
            % 全局坐标
            corners = corners_local * R' + repmat([obj.x, obj.y], 4, 1);
        end
        
        %% 获取车身包围盒
        function bbox = getBoundingBox(obj)
            % 获取车身的矩形包围盒
            % 输出: [x_min, x_max, y_min, y_max]
            
            corners = obj.getCorners();
            bbox = [min(corners(:,1)), max(corners(:,1)), ...
                    min(corners(:,2)), max(corners(:,2))];
        end
        
        %% 检查碰撞
        function isColliding = checkCollision(obj, obstacle)
            % 检查是否与矩形障碍物碰撞
            % obstacle: [x, y, width, height]
            
            bbox = obj.getBoundingBox();
            
            % 简单的AABB碰撞检测（使用包围盒）
            isColliding = ~(bbox(2) < obstacle(1) || ...
                           bbox(1) > obstacle(1) + obstacle(3) || ...
                           bbox(4) < obstacle(2) || ...
                           bbox(3) > obstacle(2) + obstacle(4));
        end
        
        %% 重置历史
        function obj = resetHistory(obj)
            % 清空历史记录
            obj.history = struct('x', [], 'y', [], 'theta', [], 'v', [], 'omega', []);
        end
        
    end
    
end