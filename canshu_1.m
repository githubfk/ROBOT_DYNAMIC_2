clear all; close all; clc;
%����Ԥ׼��
% T01 T02 T03 T04
%     T12 T13 T14
%         T23 T24
%             T34
%������D-H����
theta1 = 0; theta2 = 0; theta3 = 0; theta4 = 0;
d1 = 352; a2 = 1300; d4 = 1406+200;
%���Ƚ���ǰ�Ĺؽڵ��˶�ѧ
T01 = Chuandi_matrix(pi/2,  0 ,   d1, theta1     );
T12 = Chuandi_matrix(0   ,  a2,   0 , theta2+pi/2);
T23 = Chuandi_matrix(pi/2,  0 ,   0 , theta3     );
T34 = Chuandi_matrix(0   ,  0 ,   d4, theta4     );

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;

T13 = T12*T23;
T14 = T13*T34;

T24 = T23*T34;