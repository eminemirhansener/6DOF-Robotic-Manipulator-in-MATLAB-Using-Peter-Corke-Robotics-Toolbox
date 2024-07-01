function motor_angle = ikine(P, d, a, alpha, beta, gama, phi)
    
    %% Parameters
    Px = P(1);
    Py = P(2);
    Pz = P(3);

    syms q1 q2 q3 q4 q5 q6
    DH_table=[q1       d(1)   a(1)    alpha(1);
              q2-pi/2  d(2)   a(2)    alpha(2);
              q3       d(3)   a(3)    alpha(3);
              q4       d(4)   a(4)    alpha(4);
              q5       d(5)   a(5)    alpha(5);
              q6       d(6)   a(6)    alpha(6)];
    
    A1=[cos(DH_table(1,1)) -sin(DH_table(1,1))*cos(DH_table(1,4)) sin(DH_table(1,1))*sin(DH_table(1,4)) DH_table(1,3)*cos(DH_table(1,1));
        sin(DH_table(1,1)) cos(DH_table(1,1))*cos(DH_table(1,4)) -cos(DH_table(1,1))*sin(DH_table(1,4)) DH_table(1,3)*sin(DH_table(1,1));
        0 sin(DH_table(1,4)) cos(DH_table(1,4)) DH_table(1,2);
        0 0 0 1];
    
    A2=[cos(DH_table(2,1)) -sin(DH_table(2,1))*cos(DH_table(2,4)) sin(DH_table(2,1))*sin(DH_table(2,4)) DH_table(2,3)*cos(DH_table(2,1));
        sin(DH_table(2,1)) cos(DH_table(2,1))*cos(DH_table(2,4)) -cos(DH_table(2,1))*sin(DH_table(2,4)) DH_table(2,3)*sin(DH_table(2,1));
        0 sin(DH_table(2,4)) cos(DH_table(2,4)) DH_table(2,2);
        0 0 0 1];
    
    A3=[cos(DH_table(3,1)) -sin(DH_table(3,1))*cos(DH_table(3,4)) sin(DH_table(3,1))*sin(DH_table(3,4)) DH_table(3,3)*cos(DH_table(3,1));
        sin(DH_table(3,1)) cos(DH_table(3,1))*cos(DH_table(3,4)) -cos(DH_table(3,1))*sin(DH_table(3,4)) DH_table(3,3)*sin(DH_table(3,1));
        0 sin(DH_table(3,4)) cos(DH_table(3,4)) DH_table(3,2);
        0 0 0 1];
    
    A4=[cos(DH_table(4,1)) -sin(DH_table(4,1))*cos(DH_table(4,4)) sin(DH_table(4,1))*sin(DH_table(4,4)) DH_table(4,3)*cos(DH_table(4,1));
        sin(DH_table(4,1)) cos(DH_table(4,1))*cos(DH_table(4,4)) -cos(DH_table(4,1))*sin(DH_table(4,4)) DH_table(4,3)*sin(DH_table(4,1));
        0 sin(DH_table(4,4)) cos(DH_table(4,4)) DH_table(4,2);
        0 0 0 1];
    
    A5=[cos(DH_table(5,1)) -sin(DH_table(5,1))*cos(DH_table(5,4)) sin(DH_table(5,1))*sin(DH_table(5,4)) DH_table(5,3)*cos(DH_table(5,1));
        sin(DH_table(5,1)) cos(DH_table(5,1))*cos(DH_table(5,4)) -cos(DH_table(5,1))*sin(DH_table(5,4)) DH_table(5,3)*sin(DH_table(5,1));
        0 sin(DH_table(5,4)) cos(DH_table(5,4)) DH_table(5,2);
        0 0 0 1];
    
    A6=[cos(DH_table(6,1)) -sin(DH_table(6,1))*cos(DH_table(6,4)) sin(DH_table(6,1))*sin(DH_table(6,4)) DH_table(6,3)*cos(DH_table(6,1));
        sin(DH_table(6,1)) cos(DH_table(6,1))*cos(DH_table(6,4)) -cos(DH_table(6,1))*sin(DH_table(6,4)) DH_table(6,3)*sin(DH_table(6,1));
        0 sin(DH_table(6,4)) cos(DH_table(6,4)) DH_table(6,2);
        0 0 0 1];
    
    T30=A1*A2*A3;
    
    O60=[Px;Py;Pz];
    
    beta = beta * (pi/180);
    gama = gama * (pi/180);
    phi = phi * (pi/180);

    Rx = [1 0 0; 0 cos(beta) -sin(beta); 0 sin(beta) cos(beta)];
    Ry = [cos(gama) 0 sin(gama); 0 1 0; -sin(gama) 0 cos(gama)];
    Rz = [cos(phi) -sin(phi) 0,; sin(phi) cos(phi) 0; 0 0 1];
    
    R60 = Rz*Ry*Rx;
    
    % inverse position problem: calculating the first 3 joint variables using the wrist position
    Oc0=O60 - d(6)*R60(:,3);
    xc=Oc0(1);
    yc=Oc0(2);
    zc=Oc0(3);
    disp(xc);
    disp(yc);
    disp(zc);
    
    theta1 = atan2(yc,xc);
    
    A = power(xc,2) + power(yc,2) + power(zc-d(1),2);
    B = 2 * power(d(4),2);
    theta3 = asin((-A/B)+1);
    
    AA = d(4)*cos(theta3);
    BB = d(4)*(1-sin(theta3));
    CC = sqrt(power(xc,2)+power(yc,2));
    sol1 = acos(roots([power(AA,2) + power(BB,2), -2*AA*CC, power(CC,2)-power(BB,2)]));
    theta2 = sol1(1);
    
    % inverse orientation problem: calculating the last 3 joint variables using R63
    q1=theta1;
    q2=theta2;
    q3=theta3;
    temp=eval(T30);
    R30=temp(1:3,1:3);
    
    R63=R30.'*R60;
    
    theta4=atan2(-R63(2,3),-R63(1,3));
    theta5=atan2(sqrt(R63(3,1)^2+R63(3,2)^2),R63(3,3));
    theta6=atan2(-R63(3,2),-R63(3,1));
    
    q4=theta4;
    q5=theta5;
    q6=theta6;
    
    theta1_Deg = theta1 * 180 / pi;
    theta2_Deg = theta2 * 180 / pi;
    theta3_Deg = theta3 * 180 / pi;
    theta4_Deg = theta4 * 180 / pi;
    theta5_Deg = theta5 * 180 / pi;
    theta6_Deg = theta6 * 180 / pi;
    
    motor_angle = [theta1_Deg, theta2_Deg, theta3_Deg, theta4_Deg, theta5_Deg, theta6_Deg];
end
