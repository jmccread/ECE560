function dx = CartPoleLinearSystem(t, x)

    global m M l I g;

    L = (I + m*l^2)/(m*l);
    Mt = M + m;
    
    %% PUT YOUR CODE HERE
    
    
    
    
    dx = A*x + B*u;
    
end
