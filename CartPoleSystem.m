function dx = CartPoleSystem(t, x)

    global m M l I g;
    
    L = (I + m*l^2)/(m*l);
    Mt = M + m;

    s = x(1);
    ds = x(2);
    theta = x(3);
    dtheta = x(4);
    

    %% PUT YOUR CODE HERE
    
    
    
    dx = [ds; dds; dtheta; ddtheta];
    
end
