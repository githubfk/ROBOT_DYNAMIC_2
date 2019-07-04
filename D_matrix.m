%机器人动力学推导
% 在此，仅推导前4个关节，将手腕视为一个整体
% Ti 第i个关节的总力矩（广义力）  4x1矩阵
% Dik 对称阵，与加速度相关的惯性矩阵项  4x4矩阵
% h(q,dq) 与离心力和哥氏力有关的项 4x1 矩阵
% C(q) 重力项 4x1矩阵
%函数功能：输出惯量矩阵D

function D = D_matrix(q, ddq)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据预准备1
% T01 T02 T03 T04
%     T12 T13 T14
%         T23 T24
%             T34
%机器人D-H参数
% theta1 = 0; theta2 = 0; theta3 = 0; theta4 = 0;
theta1=q(1);theta2=q(2);theta3=q(3);theta4=0;
ddq1=ddq(1);ddq2=ddq(2);ddq3=ddq(3);ddq4=0;
DDq = [ddq1; ddq2; ddq3; ddq4];
d1 = 0.0; a2 = 1.300; d4 = 1.400 + 0.32545;%(增加了手腕的长度）
%首先建立前四关节的运动学
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据预准备2

%基座相对于其转动轴axis_2的惯性张量（千克*平方米）
%质量(千克）、重心（米）（相对于转动轴axis_2)
mi_1 = 182.10327824;
X1 =  0.00611769;
Y1 =  -0.04846321;
Z1 =  -0.03074003;
Ixx1 = 8.36437974;	Ixy1 = 0.32139229;	    Ixz1 = -0.06182520;
                    Iyy1 = 8.67685895;      Iyz1 = -0.63734847;
                                            Izz1 =  8.73345233;
                                                
%大臂相对于其转动轴axis_3的惯性张量（千克*平方米）
%质量(千克）、重心（米）（相对于转动轴axis_3)
mi_2 = 78.58091036;
X2 = -0.60328009;
Y2 = -0.00002796;
Z2 = -0.33729352;
Ixx2 =  10.46642806; Ixy2 =  0.00299179;	Ixz2 =  15.36927940;
                     Iyy2 =  54.84217054;	Iyz2 =  0.00055155;
                                            Izz2 =  46.48243403;
                                                 
%小臂相对于其转动轴axis_8的惯性张量（千克*平方米）
%质量(千克）、重心（米）（相对于转动轴axis_4)
mi_3 = 82.07360471 ;
X3 = -0.00159643;
Y3 = -0.06444111;
Z3 =  0.64843688;
Ixx3 = 64.06840389; Ixy3 = 0.00416677;    Ixz3 = -0.17322494; 
                    Iyy3 =  63.92197457;  Iyz3 = -1.72967587;
                                          Izz3 =  1.74053164;
                                                
%手腕部分（暂时没考虑）固定到小臂上了。。。
mi_4 = 0;
X4 = 0;
Y4 = 0;
Z4 = 0;
Ixx4 = 0;	             Ixy4 = 0;               Ixz4 = 0;
                         Iyy4 = 0;               Iyz4 = 0;
                                                 Izz4 = 0;
                                            
                                               
I1 = Weiguanliang_matrix(Ixx1, Iyy1, Izz1, Ixy1, Ixz1, Iyz1, X1, Y1, Z1, mi_1);
I2 = Weiguanliang_matrix(Ixx2, Iyy2, Izz2, Ixy2, Ixz2, Iyz2, X2, Y2, Z2, mi_2);
I3 = Weiguanliang_matrix(Ixx3, Iyy3, Izz3, Ixy3, Ixz3, Iyz3, X3, Y3, Z3, mi_3);
I4 = Weiguanliang_matrix(Ixx4, Iyy4, Izz4, Ixy4, Ixz4, Iyz4, X4, Y4, Z4, mi_4);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据预准备3
% （1） 关于Uij和Uijk的求解
Q1 = [0 -1 0 0;
      1  0 0 0;
      0  0 0 0;
      0  0 0 0];
Q2 = [0 -1 0 0;
      1  0 0 0;
      0  0 0 0;
      0  0 0 0];
Q3 = [0 -1 0 0;
      1  0 0 0;
      0  0 0 0;
      0  0 0 0];
Q4 = [0 -1 0 0;
      1  0 0 0;
      0  0 0 0;
      0  0 0 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



U11 = Q1*T01;

U21 = Q1*T02;
U22 = T01*Q2*T12;

U31 = Q1*T03;
U32 = T01*Q2*T13;
U33 = T02*Q3*T23;

U41 = Q1*T04;
U42 = T01*Q2*T14;
U43 = T02*Q3*T24;
U44 = T03*Q4*T34;


%首先计算与加速度相关的惯性矩阵项Dik
% Dik = [D11 D12 D13 D14; D21 D22 D23 D24; D31 D32 D33 D34; D41 D42 D43 D44]

D11 = trace(U11*I1*transpose(U11)) + trace(U21*I2*transpose(U21)) + trace(U31*I3*transpose(U31)) + trace(U41*I4*transpose(U41));
D12 =                                trace(U22*I2*transpose(U21)) + trace(U32*I3*transpose(U31)) + trace(U42*I4*transpose(U41));
D13 =                                                               trace(U33*I3*transpose(U31)) + trace(U43*I4*transpose(U41));
D14 =                                                                                              trace(U44*I4*transpose(U41));

D21 = D12;
D22 =                                trace(U22*I2*transpose(U22)) + trace(U32*I3*transpose(U32)) + trace(U42*I4*transpose(U42));
D23 =                                                               trace(U33*I3*transpose(U32)) + trace(U43*I4*transpose(U42));
D24 =                                                                                              trace(U44*I4*transpose(U42));

D31 = D13;
D32 = D23;
D33 =                                                              trace(U33*I3*transpose(U33)) +  trace(U43*I4*transpose(U43));
D34 =                                                                                              trace(U44*I4*transpose(U43));

D41 = D14;
D42 = D24;
D43 = D34;
D44 =                                                                                              trace(U44*I4*transpose(U44));


D = [D11 D12 D13 D14;
     D21 D22 D23 D24;
     D31 D32 D33 D34;
     D41 D42 D43 D44];
 
D = D * DDq;