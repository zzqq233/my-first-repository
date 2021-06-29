function findBeta = FindBeta(angle)
%����------------------�������ǰ�ؽڽ�����Ӧ�Ħ½ǣ�ͨ�����������нǼ���
%����------------------angle����ǰ�ؽڽ�1x7
%���------------------findBeta����ǰ�ؽڽ�����Ӧ�Ħ½�
%תC��ʼ��Ҫ��

T01 = CoordinateTrans(0,0,angle(1),0);      
T12 = CoordinateTrans(pi/2,0,angle(2),0);
T23 = CoordinateTrans(-pi/2,0,angle(3),449.5);
T03 = T01*T12*T23;
P0E = T03(1:3,4)';
%��֪BW��ǰ���£���������е��ƽ������ֱƽ���£�E������
poseInit= ForwardKinematics(angle);
P0W = poseInit(1:3,4)';      %TCP���ĵ����꣬ͬʱΪ������ϵָ��õ������1x3

nVertical  = cross(P0W,[-1,0,0]);     %(2)TCP���ĵ�����ֱ��������(-1,0,0)�����λ��ķ�����n 
nReal = cross(P0E,P0W);
cos_angle = nVertical*nReal'/(sqrt(nVertical(1,1)^2+nVertical(1,2)^2+nVertical(1,3)^2)*sqrt(nReal(1,1)^2+nReal(1,2)^2+nReal(1,3)^2));
if cos_angle>1
    cos_angle = 1;
elseif cos_angle<-1
    cos_angle = -1;
end
nAngle = acos(cos_angle);
beta = pi-nAngle;
%y����E�����ڿռ�����ȷ��beta�ķ���
if P0W(1)==P0W(2) && P0W(2)==P0W(3) && P0W(1)==0
%     disp('TCP���ĵ㲻�ܺ�ԭ���غ�');
    findBeta = -10;
    return
elseif P0W(3)==0  %��ʱֱ��BW��xyƽ��,�ų�P0W(3)=0�������ʹy�ĳ�����Ϊ0
    if P0E(3)>=0
        findBeta = -beta;
    else
        findBeta = beta;
    end
    return
else 
    y = P0E(2)-P0W(2)/P0W(3)*P0E(3);%yΪ����ʾ����ƽ���Ϸ�
    if abs(y) < 0.0001 %������������0�������ܻ������һ����Χǿ��Ϊ0
        y = 0;
    end
    if y >= 0
        findBeta = -beta;
    else
        findBeta = beta;
    end
end
end