%数据预准备
% I1 I2 I3 I4
% 连杆i的伪惯量矩阵Ii
% (X,Y,Z) 为质心坐标在相对坐标系下的表示， mi为杆件i的质量
function Ii = Weiguanliang_matrix(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, X, Y, Z, mi)
Ii = [(-Ixx+Iyy+Izz)/2  Ixy               Ixz              mi*X;
       Ixy             (Ixx-Iyy+Izz)/2    Iyz              mi*Y;
       Ixz              Iyz              (Ixx+Iyy-Izz)/2   mi*Z;
       mi*X             mi*Y              mi*Z             mi ];
 


