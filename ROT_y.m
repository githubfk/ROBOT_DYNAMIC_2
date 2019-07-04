function Position = ROT_y(Position, theta)
Rot_y_theta = [ cos(theta)  0   sin(theta)  
                0           1   0           
               -sin(theta)  0   cos(theta) ];
Position = Rot_y_theta * Position;
           