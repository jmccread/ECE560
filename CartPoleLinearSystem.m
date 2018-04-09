function dx = CartPoleLinearSystem(t, x)

    global m M l I g x_d K A B lin_state;

    L = (I + m*l^2)/(m*l);
    Mt = M + m;
    
    %% PUT YOUR CODE HERE
    %u = -K*(x-x_d); % wrong
    
    dx = A*(x-lin_state); %+ B*u;
    
end
