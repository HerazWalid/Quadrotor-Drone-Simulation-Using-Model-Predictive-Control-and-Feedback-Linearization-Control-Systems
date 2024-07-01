%% Project Made By HERAZ Walid as a bachelor graduation project at the Institute of Electrical and Electronic Engineering IGEE_ex_INELEC 2024, Algeria
%% Github : https://github.com/HerazWalid
%% LinkdIn : https://www.linkedin.com/in/walid-heraz-94337a240/



function constants=init_constants()
    
    % Constants 
    Ix = 0.0034; %kg*m^2
    Iy = 0.0034; %kg*m^2
    Iz  = 0.006; %kg*m^2
    m  = 0.698; %kg
    g  = 9.81; %m/s^2
    Jtp=1.302*10^(-6); %N*m*s^2=kg*m^2
    Ts=0.1; %s
    
    % Matrix weights for the cost function (They must be diagonal)
    Q=[10 0 0;0 10 0;0 0 10]; % weights for outputs (output x output)
    S=[10 0 0;0 10 0;0 0 10]; % weights for the final horizon outputs (output x output)
    R=[10 0 0;0 10 0;0 0 10]; % weights for inputs (input x input)
    
    ct = 7.6184*10^(-8)*(60/(2*pi))^2; %N*s^2
    cq = 2.6839*10^(-9)*(60/(2*pi))^2; %N*m^2
    l = 0.171; %m;
    
    controlled_states=3;
    hz = 4; % horizon period
    
    innerDyn_length=4; % Number of inner control loop iterations
    
    px=[-1 -2];
    py=[-1 -2];
    pz=[-1 -2];
    
    % Choose your trajectory (1,2,3,4)
    trajectory=4;
    
    constants={Ix Iy Iz m g Jtp Ts Q S R ct cq l controlled_states hz innerDyn_length px py pz trajectory};

end
