classdef SensorSimulator
    % SensorSimulator - 传感器仿真器（LiDAR/Radar）
    %
    % 功能：
    %   模拟LiDAR和Radar传感器检测周围障碍物
    %
    % 参数：
    %   lidarRange: LiDAR检测范围 (m)
    %   radarRange: Radar检测范围 (m)
    %
    % 作者: CCCCCYHHHHH
    % 日期: 2025年1月
    
    properties
        lidarRange
        lidarResolution
        radarRange
        radarResolution
    end
    
    methods
        
        %% 构造函数
        function obj = SensorSimulator(varargin)
            p = inputParser;
            addParameter(p, 'lidarRange', 5);
            addParameter(p, 'lidarResolution', 36);
            addParameter(p, 'radarRange', 10);
            addParameter(p, 'radarResolution', 16);
            parse(p, varargin{:});
            
            obj.lidarRange = p.Results.lidarRange;
            obj.lidarResolution = p.Results.lidarResolution;
            obj.radarRange = p.Results.radarRange;
            obj.radarResolution = p.Results.radarResolution;
        end
        
        %% 传感器扫描
        function [lidarPoints, radarDist] = scan(obj, vehiclePos, obstacles)
            % 执行传感器扫描
            %
            % 输入:
            %   vehiclePos: [x, y] 车辆位置
            %   obstacles: 障碍物列表
            %
            % 输出:
            %   lidarPoints: [Nx2] LiDAR检测到的点坐标
            %   radarDist: [M,1] Radar检测到的距离
            
            % LiDAR扫描
            angles = linspace(0, 2*pi, obj.lidarResolution + 1);
            angles = angles(1:end-1);
            
            lidarPoints = [];
            
            for i = 1:length(angles)
                angle = angles(i);
                
                % 射线从车辆位置出发
                ray_end_x = vehiclePos(1) + obj.lidarRange * cos(angle);
                ray_end_y = vehiclePos(2) + obj.lidarRange * sin(angle);
                
                % 检测与障碍物的交点
                minDist = obj.lidarRange;
                hitPoint = [ray_end_x, ray_end_y];
                
                for j = 1:length(obstacles)
                    obs = obstacles{j};
                    
                    % 检查射线与矩形障碍物的交点
                    [intersection, dist] = obj.rayRectIntersection(...
                        vehiclePos, [ray_end_x, ray_end_y], obs);
                    
                    if intersection && dist < minDist
                        minDist = dist;
                        hitPoint = vehiclePos + dist * [cos(angle), sin(angle)];
                    end
                end
                
                if minDist < obj.lidarRange
                    lidarPoints = [lidarPoints; hitPoint];
                end
            end
            
            % Radar扫描（简化版：只返回前方距离）
            radarDist = [];
            for i = 1:length(obstacles)
                obs = obstacles{i};
                obs_center = [obs(1) + obs(3)/2, obs(2) + obs(4)/2];
                dist = norm(obs_center - vehiclePos);
                
                if dist <= obj.radarRange
                    radarDist = [radarDist; dist];
                end
            end
        end
        
        %% 射线与矩形相交检测
        function [intersect, minDist] = rayRectIntersection(obj, rayStart, rayEnd, rect)
            % 检测射线是否与矩形相交
            % rect: [x, y, width, height]
            
            intersect = false;
            minDist = inf;
            
            % 矩形的四条边
            rect_x1 = rect(1);
            rect_x2 = rect(1) + rect(3);
            rect_y1 = rect(2);
            rect_y2 = rect(2) + rect(4);
            
            % 检查射线与四条边的交点
            edges = [
                [rect_x1, rect_y1], [rect_x2, rect_y1];  % 下边
                [rect_x2, rect_y1], [rect_x2, rect_y2];  % 右边
                [rect_x2, rect_y2], [rect_x1, rect_y2];  % 上边
                [rect_x1, rect_y2], [rect_x1, rect_y1]   % 左边
            ];
            
            for i = 1:4
                p1 = squeeze(edges(i, 1, :))';
                p2 = squeeze(edges(i, 2, :))';
                
                [intersect_point, t] = obj.lineIntersection(rayStart, rayEnd, p1, p2);
                
                if ~isempty(intersect_point) && t >= 0 && t <= 1
                    dist = norm(intersect_point - rayStart);
                    if dist < minDist
                        minDist = dist;
                        intersect = true;
                    end
                end
            end
        end
        
        %% 直线相交
        function [point, t] = lineIntersection(obj, p1, p2, p3, p4)
            % 计算两条直线的交点
            % p1-p2 是射线，p3-p4 是线段
            
            point = [];
            t = [];
            
            d = (p2 - p1)' * [-p4(2) + p3(2); p4(1) - p3(1)];
            if abs(d) < 1e-10
                return;  % 平行
            end
            
            t = (p3 - p1)' * [-p4(2) + p3(2); p4(1) - p3(1)] / d;
            point = p1 + t * (p2 - p1);
        end
        
    end
    
end