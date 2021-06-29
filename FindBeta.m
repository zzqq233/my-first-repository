function findBeta = FindBeta(angle)
%功能------------------计算出当前关节角所对应的β角，通过两法向量夹角计算
%输入------------------angle：当前关节角1x7
%输出------------------findBeta：当前关节角所对应的β角
%转C初始化要求

T01 = CoordinateTrans(0,0,angle(1),0);      
T12 = CoordinateTrans(pi/2,0,angle(2),0);
T23 = CoordinateTrans(-pi/2,0,angle(3),449.5);
T03 = T01*T12*T23;
P0E = T03(1:3,4)';
%已知BW的前提下，计算假设机械臂平面在竖直平面下，E的坐标
poseInit= ForwardKinematics(angle);
P0W = poseInit(1:3,4)';      %TCP中心点坐标，同时为基坐标系指向该点的向量1x3

nVertical  = cross(P0W,[-1,0,0]);     %(2)TCP中心点与竖直向上向量(-1,0,0)求解零位面的法向量n 
nReal = cross(P0E,P0W);
cos_angle = nVertical*nReal'/(sqrt(nVertical(1,1)^2+nVertical(1,2)^2+nVertical(1,3)^2)*sqrt(nReal(1,1)^2+nReal(1,2)^2+nReal(1,3)^2));
if cos_angle>1
    cos_angle = 1;
elseif cos_angle<-1
    cos_angle = -1;
end
nAngle = acos(cos_angle);
beta = pi-nAngle;
%y依据E点所在空间象限确定beta的符号
if P0W(1)==P0W(2) && P0W(2)==P0W(3) && P0W(1)==0
%     disp('TCP中心点不能和原点重合');
    findBeta = -10;
    return
elseif P0W(3)==0  %此时直线BW在xy平面,排除P0W(3)=0的情况，使y的除数不为0
    if P0E(3)>=0
        findBeta = -beta;
    else
        findBeta = beta;
    end
    return
else 
    y = P0E(2)-P0W(2)/P0W(3)*P0E(3);%y为正表示点在平面上方
    if abs(y) < 0.0001 %浮点数运算在0附近可能会出错，在一定范围强制为0
        y = 0;
    end
    if y >= 0
        findBeta = -beta;
    else
        findBeta = beta;
    end
end
end