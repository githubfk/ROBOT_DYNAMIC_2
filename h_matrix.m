%机器人动力学推导
% 在此，仅推导前4个关节，将手腕视为一个整体
% Ti 第i个关节的总力矩（广义力）  4x1矩阵
% Dik 对称阵，与加速度相关的惯性矩阵项  4x4矩阵
% h(q,dq) 与离心力和哥氏力有关的项 4x1 矩阵
% C(q) 重力项 4x1矩阵

%输出h矩阵，是角度q和角速度dq的函数。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据预准备1
function h = h_matrix(q,dq)
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据预准备2
% T01 T02 T03 T04
%     T12 T13 T14
%         T23 T24
%             T34
%机器人D-H参数
theta1 = q(1); theta2 = q(2); theta3 = q(3); theta4 = 0;
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
%伪惯量矩阵的计算


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



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%数据预准备4
%dq1 dq2 dq3 dq4的计算
dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);
dq4 = 0;

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


U111 = Q1*Q1*T01;

U211 = Q1*Q1*T02;
U212 = Q1*T01*Q2*T12;
U221 = Q1*T01*Q2*T12;
U222 = T01*Q2*Q2*T12; 

U311 = Q1*Q1*T03;
U312 = Q1*T01*Q2*T13;
U313 = Q1*T02*Q3*T23;
U321 = Q1*T01*Q2*T13;
U322 = T01*Q2*Q2*T13; 
U323 = T01*Q2*T12*Q3*T23;
U331 = Q1*T02*Q3*T23;
U332 = T01*Q2*T12*Q3*T23;
U333 = T02*Q3*Q3*T23;

U411 = Q1*Q1*T04;
U412 = Q1*T01*Q2*T14;
U413 = Q1*T02*Q3*T24;
U414 = Q1*T03*Q4*T34;
U421 = Q1*T01*Q2*T14;
U422 = T01*Q2*Q2*T14;
U423 = T01*Q2*T12*Q3*T24;
U424 = T01*Q2*T13*Q4*T34;
U431 = Q1*T02*Q3*T24;
U432 = T01*Q2*T12*Q3*T24;
U433 = T02*Q3*Q3*T24;
U434 = T02*Q3*T23*Q4*T34;
U441 = Q1*T03*Q4*T34;
U442 = T01*Q2*T13*Q4*T34;
U443 = T02*Q3*T23*Q4*T34;
U444 = T03*Q4*Q4*T34;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%计算h(q,dq)
%hi = [h1; h2; h3; h4]

%h1 = h111*dq1*dq1 + h112*dq1*dq2 + h113*dq1*dq3 + h114*dq1*dq4 + h121*dq2*dq1 + h122*dq2*dq2 + h123*dq2*dq3 + h124*dq2*dq4 + h131*dq3*dq1 + h132*dq3*dq2 + h133*dq3*dq3 + h134*dq3*dq4 + h141*dq4*dq1 + h142*dq4*dq2 + h143*dq4*dq3 + h144*dq4*dq4;

h111 = trace(U111*I1*transpose(U11)) + trace(U211*I2*transpose(U21)) + trace(U311*I3*transpose(U31)) + trace(U411*I4*transpose(U41));
h112 =                                 trace(U212*I2*transpose(U21)) + trace(U312*I3*transpose(U31)) + trace(U412*I4*transpose(U41));
h113 =                                                                 trace(U313*I3*transpose(U31)) + trace(U413*I4*transpose(U41));
h114 =                                                                                                 trace(U414*I4*transpose(U41));

h121 =                                 trace(U221*I2*transpose(U21)) + trace(U321*I3*transpose(U31)) + trace(U421*I4*transpose(U41));
h122 =                                 trace(U222*I2*transpose(U21)) + trace(U322*I3*transpose(U31)) + trace(U422*I4*transpose(U41));
h123 =                                                                 trace(U323*I3*transpose(U31)) + trace(U423*I4*transpose(U41));
h124 =                                                                                                 trace(U424*I4*transpose(U41));

h131 =                                                                 trace(U331*I3*transpose(U31)) + trace(U431*I4*transpose(U41));
h132 =                                                                 trace(U332*I3*transpose(U31)) + trace(U432*I4*transpose(U41));
h133 =                                                                 trace(U333*I3*transpose(U31)) + trace(U433*I4*transpose(U41));
h134 =                                                                                                 trace(U434*I4*transpose(U41));

h141 =                                                                                                 trace(U441*I4*transpose(U41));
h142 =                                                                                                 trace(U442*I4*transpose(U41));
h143 =                                                                                                 trace(U443*I4*transpose(U41));
h144 =                                                                                                 trace(U444*I4*transpose(U41));


%h2 = h211*dq1*dq1 + h212*dq1*dq2 + h213*dq1*dq3 + h214*dq1*dq4 + h221*dq2*dq1 + h222*dq2*dq2 + h223*dq2*dq3 + h224*dq2*dq4 + h231*dq3*dq1 + h232*dq3*dq2 + h233*dq3*dq3 + h234*dq3*dq4 + h241*dq4*dq1 + h242*dq4*dq2 + h243*dq4*dq3 + h244*dq4*dq4;

h211 =                                  trace(U211*I2*transpose(U22)) + trace(U311*I3*transpose(U32)) + trace(U411*I4*transpose(U42));
h212 =                                  trace(U212*I2*transpose(U22)) + trace(U312*I3*transpose(U32)) + trace(U412*I4*transpose(U42));
h213 =                                                                  trace(U313*I3*transpose(U32)) + trace(U413*I4*transpose(U42));
h214 =                                                                                                  trace(U414*I4*transpose(U42));

h221 =                                  trace(U221*I2*transpose(U22)) + trace(U321*I3*transpose(U32)) + trace(U421*I4*transpose(U42));
h222 =                                  trace(U222*I2*transpose(U22)) + trace(U322*I3*transpose(U32)) + trace(U422*I4*transpose(U42));
h223 =                                                                  trace(U323*I3*transpose(U32)) + trace(U423*I4*transpose(U42));
h224 =                                                                                                  trace(U424*I4*transpose(U42));

h231 =                                                                  trace(U331*I3*transpose(U32)) + trace(U431*I4*transpose(U42));
h232 =                                                                  trace(U332*I3*transpose(U32)) + trace(U432*I4*transpose(U42));
h233 =                                                                  trace(U333*I3*transpose(U32)) + trace(U433*I4*transpose(U42));
h234 =                                                                                                  trace(U434*I4*transpose(U42));

h241 =                                                                                                  trace(U441*I4*transpose(U42));
h242 =                                                                                                  trace(U442*I4*transpose(U42));
h243 =                                                                                                  trace(U443*I4*transpose(U42));
h244 =                                                                                                  trace(U444*I4*transpose(U42));


%h3 = h311*dq1*dq1 + h312*dq1*dq2 + h313*dq1*dq3 + h314*dq1*dq4 + h321*dq2*dq1 + h322*dq2*dq2 + h323*dq2*dq3 + h324*dq2*dq4 + h331*dq3*dq1 + h332*dq3*dq2 + h333*dq3*dq3 + h334*dq3*dq4 + h341*dq4*dq1 + h342*dq4*dq2 + h343*dq4*dq3 + h344*dq4*dq4;

h311 =                                                                  trace(U311*I3*transpose(U33)) + trace(U411*I4*transpose(U43));
h312 =                                                                  trace(U312*I3*transpose(U33)) + trace(U412*I4*transpose(U43));
h313 =                                                                  trace(U313*I3*transpose(U33)) + trace(U413*I4*transpose(U43));
h314 =                                                                                                  trace(U414*I4*transpose(U43));

h321 =                                                                  trace(U321*I3*transpose(U33)) + trace(U421*I4*transpose(U43));
h322 =                                                                  trace(U322*I3*transpose(U33)) + trace(U422*I4*transpose(U43));
h323 =                                                                  trace(U323*I3*transpose(U33)) + trace(U423*I4*transpose(U43));
h324 =                                                                                                  trace(U424*I4*transpose(U43));

h331 =                                                                  trace(U331*I3*transpose(U33)) + trace(U431*I4*transpose(U43));
h332 =                                                                  trace(U332*I3*transpose(U33)) + trace(U432*I4*transpose(U43));
h333 =                                                                  trace(U333*I3*transpose(U33)) + trace(U433*I4*transpose(U43));
h334 =                                                                                                  trace(U434*I4*transpose(U43));

h341 =                                                                                                  trace(U441*I4*transpose(U43));
h342 =                                                                                                  trace(U442*I4*transpose(U43));
h343 =                                                                                                  trace(U443*I4*transpose(U43));
h344 =                                                                                                  trace(U444*I4*transpose(U43));


%h4 = h411*dq1*dq1 + h412*dq1*dq2 + h413*dq1*dq3 + h414*dq1*dq4 + h421*dq2*dq1 + h422*dq2*dq2 + h423*dq2*dq3 + h424*dq2*dq4 + h431*dq3*dq1 + h432*dq3*dq2 + h433*dq3*dq3 + h434*dq3*dq4 + h441*dq4*dq1 + h442*dq4*dq2 + h443*dq4*dq3 + h444*dq4*dq4;

h411 =                                                                                                  trace(U411*I4*transpose(U44));
h412 =                                                                                                  trace(U412*I4*transpose(U44));
h413 =                                                                                                  trace(U413*I4*transpose(U44));
h414 =                                                                                                  trace(U414*I4*transpose(U44));

h421 =                                                                                                  trace(U421*I4*transpose(U44));
h422 =                                                                                                  trace(U422*I4*transpose(U44));
h423 =                                                                                                  trace(U423*I4*transpose(U44));
h424 =                                                                                                  trace(U424*I4*transpose(U44));

h431 =                                                                                                  trace(U431*I4*transpose(U44));
h432 =                                                                                                  trace(U432*I4*transpose(U44));
h433 =                                                                                                  trace(U433*I4*transpose(U44));
h434 =                                                                                                  trace(U434*I4*transpose(U44));

h441 =                                                                                                  trace(U441*I4*transpose(U44));
h442 =                                                                                                  trace(U442*I4*transpose(U44));
h443 =                                                                                                  trace(U443*I4*transpose(U44));
h444 =                                                                                                  trace(U444*I4*transpose(U44));

h1 = h111*dq1*dq1 + h112*dq1*dq2 + h113*dq1*dq3 + h114*dq1*dq4 + h121*dq2*dq1 + h122*dq2*dq2 + h123*dq2*dq3 + h124*dq2*dq4 + h131*dq3*dq1 + h132*dq3*dq2 + h133*dq3*dq3 + h134*dq3*dq4 + h141*dq4*dq1 + h142*dq4*dq2 + h143*dq4*dq3 + h144*dq4*dq4;
h2 = h211*dq1*dq1 + h212*dq1*dq2 + h213*dq1*dq3 + h214*dq1*dq4 + h221*dq2*dq1 + h222*dq2*dq2 + h223*dq2*dq3 + h224*dq2*dq4 + h231*dq3*dq1 + h232*dq3*dq2 + h233*dq3*dq3 + h234*dq3*dq4 + h241*dq4*dq1 + h242*dq4*dq2 + h243*dq4*dq3 + h244*dq4*dq4;
h3 = h311*dq1*dq1 + h312*dq1*dq2 + h313*dq1*dq3 + h314*dq1*dq4 + h321*dq2*dq1 + h322*dq2*dq2 + h323*dq2*dq3 + h324*dq2*dq4 + h331*dq3*dq1 + h332*dq3*dq2 + h333*dq3*dq3 + h334*dq3*dq4 + h341*dq4*dq1 + h342*dq4*dq2 + h343*dq4*dq3 + h344*dq4*dq4;
h4 = h411*dq1*dq1 + h412*dq1*dq2 + h413*dq1*dq3 + h414*dq1*dq4 + h421*dq2*dq1 + h422*dq2*dq2 + h423*dq2*dq3 + h424*dq2*dq4 + h431*dq3*dq1 + h432*dq3*dq2 + h433*dq3*dq3 + h434*dq3*dq4 + h441*dq4*dq1 + h442*dq4*dq2 + h443*dq4*dq3 + h444*dq4*dq4;

h = [h1; h2; h3; h4];