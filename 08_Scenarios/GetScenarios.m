function scenarios = GetScenarios()
    % GetScenarios - 返回所有可用的仿真场景
    
    scenarios = struct();
    
    % 场景1: T字路口
    scenarios.T_Intersection.name = 'T字路口';
    scenarios.T_Intersection.start = [5, 5, 0];
    scenarios.T_Intersection.goal = [95, 95, 0];
    scenarios.T_Intersection.description = '从左下角起点到右上角终点，需要复杂避障';
    
    % 场景2: U形调头
    scenarios.U_Shape.name = 'U形调头';
    scenarios.U_Shape.start = [10, 10, 0];
    scenarios.U_Shape.goal = [10, 90, pi];
    scenarios.U_Shape.description = '从左下角调头到左上角，需要大幅转向';
    
    % 场景3: 狭窄街道
    scenarios.Narrow_Street.name = '狭窄街道';
    scenarios.Narrow_Street.start = [5, 50, 0];
    scenarios.Narrow_Street.goal = [95, 50, 0];
    scenarios.Narrow_Street.description = '沿着中线穿过密集障碍物';
    
    % 场景4: 圆环路段  
    scenarios.Roundabout.name = '圆环路段';
    scenarios.Roundabout.start = [50, 5, 0];
    scenarios.Roundabout.goal = [50, 95, pi];
    scenarios.Roundabout.description = '围绕中心点进行复杂轨迹';
    
end