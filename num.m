% clear variables; close all; clc;
% nuem = [];
% m = 1;
% theta1 = -pi:pi/180:pi;
% theta2 = -pi:pi/180:pi;
% theta3 = -pi:pi/180:pi;
% for i = 1:360
%     for j = 1:360
%         for k = 1:360
%             theta1_i = theta1(i);
%             theta2_j = theta2(j);
%             theta3_k = theta3(k);
%             temp = D_matrix(theta1_i,theta2_j,theta3_k,0);
%             nuem(m) = temp(1,1) + temp(2,2) + temp(3,3);
%             m = m + 1;
%         end
%     end
% end


%运行后发现theta1的角度对惯量矩阵没有影响
clear variables; close all; clc;
nuem = [];
m = 1;
theta1 = 0;
theta2 = -40*pi/180:pi/180:120*pi/180;
theta3 = -50*pi/180:pi/180:60*pi/180;

% for j = 1:161
%     for k = 1:111
%         theta2_j = theta2(j);
%         theta3_k = theta3(k);
%         temp = D_matrix(0,theta2_j,theta3_k,0);
%         nuem(m) = temp(1,1) + temp(2,2) + temp(3,3);
%         m = m + 1;
%     end
% end


for j = 1:161
    theta2_j = theta2(j);
    temp = D_matrix(pi,theta2_j,pi/4,0);
    nuem(m) = temp(1,1) + temp(2,2) + temp(3,3);
    m = m + 1;
end

x = 1:1:161;
y = nuem(x);
plot(x,y)