function angleByPlanningJ = moveJ(angleInit,poseFinal,speedRate)%#codegen
%功能： 大范围关节角转动（关节空间）
%输入-------------angleInit：初始关节角1x7
%                 poseFinal：最终位姿4x4
%                 speedRate：机械臂最大速度比例，取值0~1，用于调节机械臂最大速度
%输出-------------angleByPlanningL：返回多组离散关节角，每0.01s发送给驱动器一组

%先求出位姿对应的关节角
workMode = 1;               %静态大范围求解模式
betaScanInterval = 0.01;    %β扫描间隔
betaInit = 0;               %避免程序出错，随便给它赋值
angleByPlanning = PlanningAngleNew(workMode,angleInit,angleInit,poseFinal,betaInit,betaScanInterval);%1x8:7个关节角+此时的β
angleFinal = angleByPlanning(1,2:8);

%初始化
Tmax = 0; 
workMode = 0;   %workMode = 0：此时在关节空间对7个关节进行规划

lineNumber = size(angleInit,2);
para = zeros(lineNumber,16);     %每一个关节规划包含16个参数：[Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min]
T = zeros(1,lineNumber);   

for i = 1:lineNumber 
    para(i,:) = STrajectoryPara(angleInit(i),angleFinal(i),workMode,i,speedRate);
%     para(i,1)+para(i,2)+para(i,3)
    T(i) = para(i,1)+para(i,2)+para(i,3);
    if Tmax<T(i)
        Tmax = T(i);
    end    
end

angleByPlanningJ = zeros(lineNumber,floor(Tmax/0.01)+1); %一般情况下不能得到整数个离散点，向下取整，并添加一个离散点，设其为最终目标点
for i = 1:lineNumber
    t = 0;
    for j =1:floor(Tmax/0.01)
        angleByPlanningJ(i,j) = S_position(t,para(i,:));
        t = t+0.01*T(i)/Tmax;
    end
    sigma = sign(angleFinal(i)-angleInit(i));  %S_position函数的前提是q1>q2，所以最后需要考虑q1、q2的大小关系
    angleByPlanningJ(i,:) = sigma*angleByPlanningJ(i,:);
    angleByPlanningJ(i,floor(Tmax/0.01)+1) = angleFinal(i);   %最后一个离散点为最终目标点
end
end