%����Ԥ׼��
% I1 I2 I3 I4
% ����i��α��������Ii
% (X,Y,Z) Ϊ�����������������ϵ�µı�ʾ�� miΪ�˼�i������
function Ii = Weiguanliang_matrix(Ixx, Iyy, Izz, Ixy, Ixz, Iyz, X, Y, Z, mi)
Ii = [(-Ixx+Iyy+Izz)/2  Ixy               Ixz              mi*X;
       Ixy             (Ixx-Iyy+Izz)/2    Iyz              mi*Y;
       Ixz              Iyz              (Ixx+Iyy-Izz)/2   mi*Z;
       mi*X             mi*Y              mi*Z             mi ];
 


