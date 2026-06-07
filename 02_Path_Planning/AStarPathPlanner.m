classdef AStarPathPlanner
    % AStarPathPlanner - A*路径规划算法
    %
    % 功能：
    %   在栅格地图上使用A*算法进行最优路径规划
    %
    % 参数：
    %   mapData: 地图数据结构
    %   config: 配置参数
    %
    % 作者: CCCCCYHHHHH
    % 日期: 2025年1月
    
    properties
        mapData
        gridSize
        gridResolution
        startPos
        goalPos
        openList
        closedList
    end
    
    methods
        
        %% 构造函数
        function obj = AStarPathPlanner(mapData, config)
            obj.mapData = mapData;
            obj.gridResolution = mapData.gridResolution;
            obj.gridSize = ceil(mapData.size / obj.gridResolution);
        end
        
        %% 主规划函数
        function [waypoints, planningTime] = plan(obj, startPos, goalPos)
            % 执行A*路径规划
            %
            % 输入:
            %   startPos: [x, y] 起点坐标
            %   goalPos: [x, y] 终点坐标
            %
            % 输出:
            %   waypoints: [Nx2] 路径点坐标
            %   planningTime: 规划耗时 (s)
            
            tic;
            
            obj.startPos = startPos;
            obj.goalPos = goalPos;
            
            % 将坐标转换为栅格索引
            startGrid = obj.pos2grid(startPos);
            goalGrid = obj.pos2grid(goalPos);
            
            % 检查起点和终点的有效性
            if ~obj.isGridValid(startGrid) || ~obj.isGridValid(goalGrid)
                error('起点或终点在障碍物内');
            end
            
            % 初始化
            obj.openList = {startGrid};
            obj.closedList = {};
            
            gScore = inf(obj.gridSize, obj.gridSize);
            fScore = inf(obj.gridSize, obj.gridSize);
            cameFrom = {};
            
            % 初始化得分
            gScore(startGrid(1), startGrid(2)) = 0;
            fScore(startGrid(1), startGrid(2)) = obj.heuristic(startGrid, goalGrid);
            
            % A*主循环
            maxIterations = 10000;
            iterations = 0;
            found = false;
            
            while ~isempty(obj.openList) && iterations < maxIterations
                iterations = iterations + 1;
                
                % 找到fScore最小的点
                [~, currentIdx] = min(obj.getFScores(fScore));
                current = obj.openList{currentIdx};
                
                % 到达目标
                if isequal(current, goalGrid)
                    found = true;
                    break;
                end
                
                % 从openList移除当前点
                obj.openList(currentIdx) = [];
                obj.closedList{end+1} = current;
                
                % 遍历邻接点（8方向）
                neighbors = obj.getNeighbors(current);
                
                for i = 1:length(neighbors)
                    neighbor = neighbors{i};
                    
                    % 跳过已关闭的点
                    if obj.isInList(neighbor, obj.closedList)
                        continue;
                    end
                    
                    % 检查有效性
                    if ~obj.isGridValid(neighbor)
                        continue;
                    end
                    
                    % 计算得分
                    if abs(neighbor(1) - current(1)) + abs(neighbor(2) - current(2)) == 1
                        moveCost = 1;  % 直线移动
                    else
                        moveCost = sqrt(2);  % 对角线移动
                    end
                    
                    tentativeG = gScore(current(1), current(2)) + moveCost;
                    
                    % 如果发现更优路径
                    if tentativeG < gScore(neighbor(1), neighbor(2))
                        cameFrom{neighbor(1), neighbor(2)} = current;
                        gScore(neighbor(1), neighbor(2)) = tentativeG;
                        fScore(neighbor(1), neighbor(2)) = tentativeG + obj.heuristic(neighbor, goalGrid);
                        
                        % 添加到openList（如果不存在）
                        if ~obj.isInList(neighbor, obj.openList)
                            obj.openList{end+1} = neighbor;
                        end
                    end
                end
            end
            
            planningTime = toc;
            
            % 提取路径
            if found
                path = {goalGrid};
                current = goalGrid;
                
                while ~isequal(current, startGrid)
                    if isempty(cameFrom) || ~iskey(containers.Map(cellfun(@(x) mat2str(x), cameFrom, 'UniformOutput', false), 1:length(cameFrom)), mat2str(current))
                        break;
                    end
                    % 简化的回溯（实际实现可能需要更复杂的数据结构）
                    current = cameFrom{current(1), current(2)};
                    path = [{current}, path];
                end
                
                % 将栅格坐标转换回实际坐标
                waypoints = zeros(length(path), 2);
                for i = 1:length(path)
                    waypoints(i, :) = obj.grid2pos(path{i});
                end
                
                % 简化路径
                waypoints = obj.simplifyPath(waypoints);
            else
                % 规划失败
                waypoints = [];
            end
        end
        
        %% 辅助函数
        
        function h = heuristic(obj, node1, node2)
            % 启发式函数��欧几里得距离）
            h = sqrt((node1(1) - node2(1))^2 + (node1(2) - node2(2))^2);
        end
        
        function grid = pos2grid(obj, pos)
            % 将实际坐标转换为栅格索引
            grid = [floor(pos(2) / obj.gridResolution) + 1, ...
                    floor(pos(1) / obj.gridResolution) + 1];
            grid = max(1, min(grid, obj.gridSize));
        end
        
        function pos = grid2pos(obj, grid)
            % 将栅格索引转换为实际坐标
            pos = [(grid(2) - 1) * obj.gridResolution + obj.gridResolution/2, ...
                   (grid(1) - 1) * obj.gridResolution + obj.gridResolution/2];
        end
        
        function valid = isGridValid(obj, grid)
            % 检查栅格是否有效（不在障碍物内）
            if grid(1) < 1 || grid(1) > obj.gridSize || grid(2) < 1 || grid(2) > obj.gridSize
                valid = false;
                return;
            end
            
            pos = obj.grid2pos(grid);
            valid = true;
            
            % 检查是否与任何障碍物碰撞
            for i = 1:length(obj.mapData.obstacles)
                obs = obj.mapData.obstacles{i};
                if pos(1) >= obs(1) && pos(1) <= obs(1) + obs(3) && ...
                   pos(2) >= obs(2) && pos(2) <= obs(2) + obs(4)
                    valid = false;
                    return;
                end
            end
        end
        
        function neighbors = getNeighbors(obj, node)
            % 获取8方向邻接点
            directions = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];
            neighbors = {};
            
            for i = 1:size(directions, 1)
                neighbor = node + directions(i, :);
                if neighbor(1) >= 1 && neighbor(1) <= obj.gridSize && ...
                   neighbor(2) >= 1 && neighbor(2) <= obj.gridSize
                    neighbors{end+1} = neighbor;
                end
            end
        end
        
        function inList = isInList(obj, node, list)
            % 检查节点是否在列表中
            inList = false;
            for i = 1:length(list)
                if isequal(node, list{i})
                    inList = true;
                    return;
                end
            end
        end
        
        function fscores = getFScores(obj, fScore)
            % 获取openList中所有点的fScore
            fscores = [];
            for i = 1:length(obj.openList)
                node = obj.openList{i};
                fscores = [fscores, fScore(node(1), node(2))];
            end
        end
        
        function simplified = simplifyPath(obj, path)
            % 简化路径（移除不必要的中间点）
            if length(path) <= 2
                simplified = path;
                return;
            end
            
            simplified = [path(1, :)];
            
            for i = 2:length(path)-1
                % 检查能否直接连接i-1和i+1
                p1 = path(i-1, :);
                p2 = path(i+1, :);
                p = path(i, :);
                
                % 计算点到线段的距离
                dist = abs((p2(1)-p1(1))*(p1(2)-p(2)) - (p1(1)-p(1))*(p2(2)-p1(2))) / ...
                       sqrt((p2(1)-p1(1))^2 + (p2(2)-p1(2))^2);
                
                % 如果距离较大，保留该点
                if dist > 0.5
                    simplified = [simplified; path(i, :)];
                end
            end
            
            simplified = [simplified; path(end, :)];
        end
        
    end
    
end