%����ϵ���ڸ˼���ǰ��
function Ti = Chuandi_matrix(alphai, ai, di, thetai)
Ti = [cos(thetai) -cos(alphai)*sin(thetai)  sin(alphai)*sin(thetai) ai*cos(thetai);
      sin(thetai)  cos(alphai)*cos(thetai) -sin(alphai)*cos(thetai) ai*sin(thetai);
      0            sin(alphai)              cos(alphai)             di            ;
      0            0                        0                       1             ;];
  