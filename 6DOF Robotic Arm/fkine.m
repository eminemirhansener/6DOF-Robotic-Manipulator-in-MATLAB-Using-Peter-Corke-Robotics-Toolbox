function Tr = fkine(d, theta, a, alpha, n)
    
    theta = theta * (pi/180);
    
    %%%%%%%%%%%%%%%%%%   DH Parameters   %%%%%%%%%%%%%%%%%%
    DH = [d(1), theta(1), a(1), alpha(1);
          d(2), theta(2), a(2), alpha(2);
          d(3), theta(3), a(3), alpha(3);
          d(4), theta(4), a(4), alpha(4);
          d(5), theta(5), a(5), alpha(5);
          d(6), theta(6), a(6), alpha(6)];
    
    T = zeros(4,4,n); % Preallocating the T matrix
    
    %%%%%%%%%%%%%%%%% Transformation matrices are calculated for each link %%%%%%%%%%%%%%%%%
    % General Transformation Matrix = Rotation(z, theta)*Translation(a,0,d)*Rotation(x,alpha)
    for i = 1:n
        T(:,:,i) = [cos(DH(i,2)), -sin(DH(i,2))*cos(DH(i,4)), sin(DH(i,2))*sin(DH(i,4)), DH(i,3)*cos(DH(i,2));
                    sin(DH(i,2)), cos(DH(i,2))*cos(DH(i,4)), -cos(DH(i,2))*sin(DH(i,4)), DH(i,3)*sin(DH(i,2));
                    0, sin(DH(i,4)), cos(DH(i,4)), DH(i,1);
                    0, 0, 0, 1];
    end
    
    Tr = T(:,:,1);

    for i = 2:n
        Tr = Tr * T(:,:,i);
    end

    
end