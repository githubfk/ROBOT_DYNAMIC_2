%功能：读取excel运动学数据，进行动力学分析
clear variables; close all; clc;
%读取excel数据
read_t = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\zhixian_shuiping_1.xlsx','sheet1','A1:A200');
read_q = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\zhixian_shuiping_1.xlsx','sheet1','G1:I200');
read_dq = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\zhixian_shuiping_1.xlsx','sheet1','L1:N199');
read_ddq = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\zhixian_shuiping_1.xlsx','sheet1','Q1:S198');

% read_t_Adams =  xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\hh.xlsx','sheet1','A3:A503');
% Torque1_Adams = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\hh.xlsx','sheet1','B3:B503');
% Torque2_Adams = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\hh.xlsx','sheet1','C3:C503');
% Torque3_Adams = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\hh.xlsx','sheet1','D3:D503');


% read_t = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\zhixian_chuizhi_6.xlsx','sheet1','A1:A200');
% read_q = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\zhixian_chuizhi_6.xlsx','sheet1','G1:I200');
% read_dq = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\zhixian_chuizhi_6.xlsx','sheet1','L1:N199');
% read_ddq = xlsread('D:\fk_file_important\MATLAB_CODE\DYNAMIC\zhixian_chuizhi_6.xlsx','sheet1','Q1:S198');

Torque = [];
for i = 1:198
    q = read_q(i,:);
    dq = read_dq(i,:);
    ddq = read_ddq(i,:);
    
    Torque(i,:) = Dynamic_trajectory(q, dq, ddq);                                   %计算运动过程中各个关节所受力矩的大小
    
    Inertia_force(i,:) = D_matrix(q, ddq);                                          %计算运动过程中各个杆件的惯性力对关节力矩的作用
    
    Centrifugal_force(i,:) = h_matrix(q,dq);                                        %计算运动过程中各个杆件的离心力和哥氏力对关节力矩的作用
    
    Gravity_force_temp(i,:) = C_matrix(q);                                               
    Gravity_force(i,:) = Gravity_force_temp(i,1:4);                            %计算运动过程中各个关节在重力作用下的力矩
    
    temp_JOINT_1_yaobu_torque(i,:)  = Gravity_force_temp(i,5);
    temp_JOINT_1_dabi_torque(i,:)   = Gravity_force_temp(i,6);
    temp_JOINT_1_xiaobi_torque(i,:) = Gravity_force_temp(i,7);
    
%     Inertia_force_contribute(i,:) = Inertia_force(i,:) ./ Torque(i,:) *100;              %惯性力的贡献率  1x4的向量分别代表4个关节
%     
%     Centrifugal_force_contribute(i,:) = Centrifugal_force(i,:) ./ Torque(i,:) *100;      %离心力和哥氏力的贡献率  1x4的向量分别代表4个关节
%     
%     Gravity_force_contribute(i,:) = Gravity_force(i,:) ./ Torque(i,:) *100;              %重力的贡献率  1x4的向量分别代表4个关节
    
%     if abs(Torque(i,1)) <= 5
%         Inertia_force_contribute(i,1) = 0;
%         Centrifugal_force_contribute(i,1) = 0;
%         Gravity_force_contribute(i,1) = 0;
%     end
% 
%     if abs(Torque(i,2)) <= 5 
%         Inertia_force_contribute(i,2) = 0;
%         Centrifugal_force_contribute(i,2) = 0;
%         Gravity_force_contribute(i,2) = 0;
%     end
%     
%     if abs(Torque(i,3)) <= 5
%         Inertia_force_contribute(i,3) = 0;
%         Centrifugal_force_contribute(i,3) = 0;
%         Gravity_force_contribute(i,3) = 0;
%     end
  

end

t = read_t(1:198);
figure(1);
plot(t', Torque(:,1),'b:','LineWidth',3); %关节1随时间变化的力矩
hold on
plot(t', Torque(:,2),'r-','LineWidth',3); %关节2随时间变化的力矩
hold on
plot(t', Torque(:,3),'g--','LineWidth',3); %关节3随时间变化的力矩
hold on
% plot(t', abs(Torque(:,4)),'k-.','LineWidth',3); %关节4随时间变化的力矩


% read_t_Adams  = read_t_Adams(1:12:end);
% Torque1_Adams = Torque1_Adams(1:12:end);
% Torque2_Adams = Torque2_Adams(1:12:end);
% Torque3_Adams = Torque3_Adams(1:12:end);
% 
% plot(read_t_Adams', Torque1_Adams,'*'); %关节1随时间变化的力矩from ADAMS
% hold on
% plot(read_t_Adams', Torque2_Adams,'*'); %关节2随时间变化的力矩from ADAMS
% hold on
% plot(read_t_Adams', Torque3_Adams,'*'); %关节3随时间变化的力矩from ADAMS
% hold on

title('zhixian trajectory dynamic','FontSize',15);
xlabel('t(s)','FontSize',15); ylabel('torque(newton-meter)','FontSize',15);
legend_set = legend('Torque of Joint 1','Torque of Joint 2','Torque of Joint 3');
set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
grid on;
hold off;

% figure(2);
% plot(t', Torque(:,1),'b:'); %关节1随时间变化的力矩
% title('zhixian trajectory dynamic');
% xlabel('t(s)'); ylabel('torque(newton-meter)');
% legend_set1 = legend('Torque of Joint 1');
% set(legend_set1,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% grid on;
% 
% figure(3);
% plot(t', Torque(:,2),'r-'); %关节2随时间变化的力矩
% title('zhixian trajectory dynamic');
% xlabel('t(s)'); ylabel('torque(newton-meter)');
% legend_set2 = legend('Torque of Joint 2');
% set(legend_set2,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% grid on;
% 
% figure(4);
% plot(t', Torque(:,3),'g--'); %关节3随时间变化的力矩
% title('zhixian trajectory dynamic');
% xlabel('t(s)'); ylabel('torque(newton-meter)');
% legend_set3 = legend('Torque of Joint 3');
% set(legend_set3,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% grid on;

% figure(5);
% plot(t', Torque(:,4),'k-.'); %关节4随时间变化的力矩
% title('zhixian trajectory dynamic');
% xlabel('t(s)'); ylabel('torque(newton-meter)');
% grid on;

% figure(6);
% plot(t', Torque(:,1),'k-','LineWidth',3);             %关节1随时间变化时关节力矩
% hold on
% plot(t', Inertia_force(:,1),'b:','LineWidth',2);      %关节1随时间变化时惯性力
% hold on
% plot(t', Centrifugal_force(:,1),'r-.','LineWidth',2); %关节1随时间变化时离心力和哥氏力
% hold on
% plot(t', Gravity_force(:,1),'g--','LineWidth',2);     %关节1随时间变化时重力
% title('Torque of Joint 1','FontSize',15);
% xlabel('t(s)','FontSize',15); ylabel('force of Joint1(newton-meter)','FontSize',15);
% legend_set = legend('Torque force','Inertia force','Centrifugal force','Gravity force');
% set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% grid on;
% hold off;
% 
% 
% 
% figure(7);
% plot(t', Torque(:,2),'k-','LineWidth',3);             %关节2随时间变化时关节力矩
% hold on
% plot(t', Inertia_force(:,2),'b:','LineWidth',2);      %关节2随时间变化时惯性力
% hold on
% plot(t', Centrifugal_force(:,2),'r-.','LineWidth',2);  %关节2随时间变化时离心力和哥氏力
% hold on
% plot(t', Gravity_force(:,2),'g--','LineWidth',2);     %关节2随时间变化时重力
% title('Torque of Joint 2','FontSize',15);
% xlabel('t(s)','FontSize',15); ylabel('force of Joint2(newton-meter)','FontSize',15);
% legend_set = legend('Torque force','Inertia force','Centrifugal force','Gravity force');
% set(legend_set,'Fontname', 'Times New Roman','FontSize',15);     %设置legend字体大小
% grid on;
% hold off;
% 
% 
% 
% 
% figure(8);
% plot(t', Torque(:,3),'k-','LineWidth',3);             %关节3随时间变化时关节力矩
% hold on
% plot(t', Inertia_force(:,3),'b:','LineWidth',2);      %关节3随时间变化时惯性力
% hold on
% plot(t', Centrifugal_force(:,3),'r-.','LineWidth',2);  %关节3随时间变化时离心力和哥氏力
% hold on
% plot(t', Gravity_force(:,3),'g--','LineWidth',2);     %关节3随时间变化时重力
% title('Torque of Joint 3','FontSize',15);
% xlabel('t(s)','FontSize',15); ylabel('force of Joint3(newton-meter)','FontSize',15);
% legend_set = legend('Torque force','Inertia force','Centrifugal force','Gravity force');
% set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% grid on;
% hold off;




% % %各个关节角度随时间的变化
% % figure(9);
% % plot(t', read_q(1:198,1),'b:','LineWidth',2);       %关节1随时间变化时角度值的变化
% % hold on
% % plot(t', read_q(1:198,2),'r-','LineWidth',2);       %关节2随时间变化时角度值的变化
% % hold on
% % plot(t', read_q(1:198,3),'g--','LineWidth',2);      %关节3随时间变化时角度值的变化
% % hold on;
% % title('joint angles varying with time','FontSize',15);
% % xlabel('t(s)','FontSize',15); ylabel('joint angles(度)','FontSize',15);
% % legend_set = legend('joint angles of Joint 1','joint angles of Joint 2','joint angles of Joint 3');
% % set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% % grid on;
% % 
% % 
% % %各个关节角速度随时间的变化
% % figure(10);
% % plot(t', read_dq(1:198,1),'b:','LineWidth',2);       %关节1随时间变化时角速度值的变化
% % hold on
% % plot(t', read_dq(1:198,2),'r-','LineWidth',2);       %关节2随时间变化时角速度值的变化
% % hold on
% % plot(t', read_dq(1:198,3),'g--','LineWidth',2);      %关节3随时间变化时角速度值的变化
% % hold on;
% % title('angular velocity varying with time','FontSize',15);
% % xlabel('t(s)','FontSize',15); ylabel('angular velocity(度/秒)','FontSize',15);
% % legend_set = legend('angular velocity of Joint 1','angular velocity of Joint 2','angular velocity of Joint 3');
% % set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% % grid on;
% % 
% % 
% % 
% % %各个关节角加速度随时间的变化
% % figure(11);
% % plot(t', read_ddq(1:198,1),'b:','LineWidth',2);       %关节1随时间变化时角加速度值的变化
% % hold on
% % plot(t', read_ddq(1:198,2),'r-','LineWidth',2);       %关节2随时间变化时角加速度值的变化
% % hold on
% % plot(t', read_ddq(1:198,3),'g--','LineWidth',2);      %关节3随时间变化时角加速度值的变化
% % hold on;
% % title('angular acceleration of varying with time','FontSize',15);
% % xlabel('t(s)','FontSize',15); ylabel('angular acceleration(度/(秒^2）)','FontSize',15);
% % legend_set = legend('angular acceleration of Joint 1','angular acceleration of Joint 2','angular acceleration of Joint 3');
% % set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% % grid on;


figure(12);
% subplot(1,2,1);
plot(t', Torque(:,1),'k-','LineWidth',3);             %关节1随时间变化时关节力矩
% hold on
% plot(t', Inertia_force(:,1),'b:','LineWidth',2);      %关节1随时间变化时惯性力
% hold on
% plot(t', Centrifugal_force(:,1),'r-.','LineWidth',2); %关节1随时间变化时离心力和哥氏力
hold on
plot(t', Gravity_force(:,1),'g-','LineWidth',2);     %关节1随时间变化时重力

hold on
plot(t', temp_JOINT_1_yaobu_torque,'b:','LineWidth',2);    
hold on
plot(t', temp_JOINT_1_dabi_torque,'r-.','LineWidth',2);     
hold on
plot(t', temp_JOINT_1_xiaobi_torque,'g--','LineWidth',2);    



title('Torque of Joint 1','FontSize',15);
xlabel('t(s)','FontSize',15); ylabel('force of Joint1(newton-meter)','FontSize',15);
% legend_set = legend('Torque force','Inertia force','Centrifugal force','Gravity force');
legend_set = legend('Torque force','Gravity force','Gravity from yaobu','Gravity for dabi','Gravity for xiaobi');
set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
grid on;

% subplot(1,2,2);
% plot(t', read_q(1:198,1)*180/pi,'b:','LineWidth',2);       %关节1随时间变化时角度值的变化
% hold on
% plot(t', read_dq(1:198,1)*180/pi,'r-.','LineWidth',2);      %关节1随时间变化时角速度值的变化
% hold on
% plot(t', read_ddq(1:198,1)*180/pi,'g--','LineWidth',2);     %关节1随时间变化时角加速度值的变化
% title('joint angles 1 varying with time','FontSize',15);
% xlabel('t(s)','FontSize',15); ylabel('joint angles','FontSize',15);
% legend_set = legend('joint angles of Joint 1','angular velocity of Joint 1','angular acceleration of Joint 1');
% set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
% grid on;




figure(13);
subplot(1,2,1);
plot(t', Torque(:,2),'k-','LineWidth',3);             %关节2随时间变化时关节力矩
hold on
plot(t', Inertia_force(:,2),'b:','LineWidth',2);      %关节2随时间变化时惯性力
hold on
plot(t', Centrifugal_force(:,2),'r-.','LineWidth',2);  %关节2随时间变化时离心力和哥氏力
hold on
plot(t', Gravity_force(:,2),'g--','LineWidth',2);     %关节2随时间变化时重力
title('Torque of Joint 2','FontSize',15);
xlabel('t(s)','FontSize',15); ylabel('force of Joint2(newton-meter)','FontSize',15);
legend_set = legend('Torque force','Inertia force','Centrifugal force','Gravity force');
set(legend_set,'Fontname', 'Times New Roman','FontSize',15);     %设置legend字体大小
grid on;
hold off;

subplot(1,2,2);
plot(t', read_q(1:198,2)*180/pi,'b:','LineWidth',2);       %关节2随时间变化时角度值的变化
hold on
plot(t', read_dq(1:198,2)*180/pi,'r-.','LineWidth',2);      %关节2随时间变化时角速度值的变化
hold on
plot(t', read_ddq(1:198,2)*180/pi,'g--','LineWidth',2);     %关节2随时间变化时角加速度值的变化
title('joint angles 2 varying with time','FontSize',15);
xlabel('t(s)','FontSize',15); ylabel('joint angles','FontSize',15);
legend_set = legend('joint angles of Joint 2','angular velocity of Joint 2','angular acceleration of Joint 2');
set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
grid on;


figure(14);
subplot(1,2,1);
plot(t', Torque(:,3),'k-','LineWidth',3);             %关节3随时间变化时关节力矩
hold on
plot(t', Inertia_force(:,3),'b:','LineWidth',2);      %关节3随时间变化时惯性力
hold on
plot(t', Centrifugal_force(:,3),'r-.','LineWidth',2);  %关节3随时间变化时离心力和哥氏力
hold on
plot(t', Gravity_force(:,3),'g--','LineWidth',2);     %关节3随时间变化时重力
title('Torque of Joint 3','FontSize',15);
xlabel('t(s)','FontSize',15); ylabel('force of Joint3(newton-meter)','FontSize',15);
legend_set = legend('Torque force','Inertia force','Centrifugal force','Gravity force');
set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
grid on;
hold off;

subplot(1,2,2);
plot(t', read_q(1:198,3)*180/pi,'b:','LineWidth',2);       %关节3随时间变化时角度值的变化
hold on
plot(t', read_dq(1:198,3)*180/pi,'r-.','LineWidth',2);      %关节3随时间变化时角速度值的变化
hold on
plot(t', read_ddq(1:198,3)*180/pi,'g--','LineWidth',2);     %关节3随时间变化时角加速度值的变化
title('joint angles 3 varying with time','FontSize',15);
xlabel('t(s)','FontSize',15); ylabel('joint angles','FontSize',15);
legend_set = legend('joint angles of Joint 3','angular velocity of Joint 3','angular acceleration of Joint 3');
set(legend_set,'Fontname', 'Times New Roman','FontSize',15); %设置legend字体大小
grid on;




%将各个贡献率写入到excel表格中
Joint_1_force = [Torque(:,1) Inertia_force(:,1) Centrifugal_force(:,1) Gravity_force(:,1)];
Joint_2_force = [Torque(:,2) Inertia_force(:,2) Centrifugal_force(:,2) Gravity_force(:,2)];
Joint_3_force = [Torque(:,3) Inertia_force(:,3) Centrifugal_force(:,3) Gravity_force(:,3)];

% Joint_1_Contribute = [Inertia_force_contribute(:, 1) Centrifugal_force_contribute(:,1) Gravity_force_contribute(:, 1)];
% Joint_2_Contribute = [Inertia_force_contribute(:, 2) Centrifugal_force_contribute(:,2) Gravity_force_contribute(:, 2)];
% Joint_3_Contribute = [Inertia_force_contribute(:, 3) Centrifugal_force_contribute(:,3) Gravity_force_contribute(:, 3)];


k = ' ';
xlswrite('D:\fk_file_important\MATLAB_CODE\DYNAMIC\gongxianlv.xlsx', k, 'sheet2','A1:Z100000');%清空表格数据
%写入各个关节所受力的大小
xlswrite('D:\fk_file_important\MATLAB_CODE\DYNAMIC\gongxianlv.xlsx', Joint_1_force, 'sheet1','A2:D201');
xlswrite('D:\fk_file_important\MATLAB_CODE\DYNAMIC\gongxianlv.xlsx', Joint_2_force, 'sheet1','F2:I201');
xlswrite('D:\fk_file_important\MATLAB_CODE\DYNAMIC\gongxianlv.xlsx', Joint_3_force, 'sheet1','K2:N201');


% %写入各个关节所受力的大小的贡献率
% xlswrite('D:\fk_file_important\MATLAB_CODE\DYNAMIC\gongxianlv.xlsx', Joint_1_Contribute, 'sheet1', 'P2:R201');%写入水平直线1末端坐标数据于gongxianlv.xlsx表格中
% xlswrite('D:\fk_file_important\MATLAB_CODE\DYNAMIC\gongxianlv.xlsx', Joint_2_Contribute, 'sheet1', 'T2:V201');%写入水平直线1求逆解之后各个关节角度随时间变化的数据
% xlswrite('D:\fk_file_important\MATLAB_CODE\DYNAMIC\gongxianlv.xlsx', Joint_3_Contribute, 'sheet1', 'X2:Z201');%写入水平直线1求逆解之后各个关节角速度随时间变化的数据
% 
