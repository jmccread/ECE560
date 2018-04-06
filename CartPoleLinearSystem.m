function dx = CartPoleLinearSystem(t, x)

    global m M l I g x_d K A B;

    L = (I + m*l^2)/(m*l);
    Mt = M + m;
    
    %% PUT YOUR CODE HERE
    u = -K*(x-x_d);
    
    dx = A*x + B*u;
    
end
