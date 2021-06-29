function angleByPlanningJ = moveJ(angleInit,poseFinal,speedRate)%#codegen
%���ܣ� ��Χ�ؽڽ�ת�����ؽڿռ䣩
%����-------------angleInit����ʼ�ؽڽ�1x7
%                 poseFinal������λ��4x4
%                 speedRate����е������ٶȱ�����ȡֵ0~1�����ڵ��ڻ�е������ٶ�
%���-------------angleByPlanningL�����ض�����ɢ�ؽڽǣ�ÿ0.01s���͸�������һ��

%�����λ�˶�Ӧ�Ĺؽڽ�
workMode = 1;               %��̬��Χ���ģʽ
betaScanInterval = 0.01;    %��ɨ����
betaInit = 0;               %������������������ֵ
angleByPlanning = PlanningAngleNew(workMode,angleInit,angleInit,poseFinal,betaInit,betaScanInterval);%1x8:7���ؽڽ�+��ʱ�Ħ�
angleFinal = angleByPlanning(1,2:8);

%��ʼ��
Tmax = 0; 
workMode = 0;   %workMode = 0����ʱ�ڹؽڿռ��7���ؽڽ��й滮

lineNumber = size(angleInit,2);
para = zeros(lineNumber,16);     %ÿһ���ؽڹ滮����16��������[Ta, Tv, Td, Tj1, Tj2, q_0, q_1, v_0, v_1, vlim, a_max, a_min, a_lima, a_limd, j_max, j_min]
T = zeros(1,lineNumber);   

for i = 1:lineNumber 
    para(i,:) = STrajectoryPara(angleInit(i),angleFinal(i),workMode,i,speedRate);
%     para(i,1)+para(i,2)+para(i,3)
    T(i) = para(i,1)+para(i,2)+para(i,3);
    if Tmax<T(i)
        Tmax = T(i);
    end    
end

angleByPlanningJ = zeros(lineNumber,floor(Tmax/0.01)+1); %һ������²��ܵõ���������ɢ�㣬����ȡ���������һ����ɢ�㣬����Ϊ����Ŀ���
for i = 1:lineNumber
    t = 0;
    for j =1:floor(Tmax/0.01)
        angleByPlanningJ(i,j) = S_position(t,para(i,:));
        t = t+0.01*T(i)/Tmax;
    end
    sigma = sign(angleFinal(i)-angleInit(i));  %S_position������ǰ����q1>q2�����������Ҫ����q1��q2�Ĵ�С��ϵ
    angleByPlanningJ(i,:) = sigma*angleByPlanningJ(i,:);
    angleByPlanningJ(i,floor(Tmax/0.01)+1) = angleFinal(i);   %���һ����ɢ��Ϊ����Ŀ���
end
end