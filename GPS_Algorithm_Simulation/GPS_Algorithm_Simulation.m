% ECE 174 PA 2 GPS Algorithm
% Sam Vineyard
% A11457474

% Program: GPS.m 
% Purpose: Apply the Steepest Descent Algorithm and Gauss-Newton Algorithm
% to locate a vehicle for the zero noise case. 

clear all
close all

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Generate synthetic data for simulations yl = Rl + b                     %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Actual receiver (s) and satellite (sl) positions in units of ER
s  = [1.000000000 0.000000000 0.000000000]';
s1 = [3.585200000 2.070000000 0.000000000]';
s2 = [2.927400000 2.927400000 0.000000000]';
s3 = [2.661200000 0.000000000 3.171200000]';
s4 = [1.415900000 0.000000000 3.890400000]';

% Create cell array sl for satellite locations 
sl = {s1,s2,s3,s4};

% Actual clock bias error in units of ER
b = 2.354788068*10^-3;

% True range Rl between satellite and receiver for l = 1,2,3,4
% Rl and yl are initialized as array of zeros.
Rl = zeros(1,4);
yl = zeros(4,1);

for l = 1:4
    delta_s = s - sl{l};
    R(l) = sqrt((delta_s)'*delta_s);
    yl(l) =  R(l) + b;
end

% Now we need to use the generated synthetic data and the mathematical 
% model with S a free vector variable to obtain an estimate of the true 
% (assumed unknown) position s0.

% Initial estimate of vehicle location of s_hat(0) in units of ER
% This is 0.5 km above sea level and about 2330 km off actual location s
s_hat = [0.93310 0.25000 0.258819]';

% Initial estimate of clock bias b_hat(0)
b_hat = 0;

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Apply the Steepest Descent Algorithm to locate the vehicle for the zero %
% noise case. Errors in position estimates have units of meters.          %                                                            
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

%  Initialize x_hat to be column vector of s and b
% First column is zero so that while loop can start 
x_hat_1 = s_hat;
x_hat_1(4) = b_hat;

% Initialize h_hat and Jacobian
h_hat = zeros(4,1);
Jacob = zeros(4,4);

% Initialize loss function column array
loss_1 = zeros(27144,1);

% Initialize receiver position estimate error array
pos_error_1 = zeros(27144,1);

% Initialize clock bias estimate error array
clock_bias_error_1  = zeros(27144,1);

% Initialize k that will increment on each while loop
k1 = 1;

% Choose step size and termination criteria (units of ER)
a = 0.1;
E = 0.00000016;

while (k1 == 1 || term_criteria > E) && k1 < 50000 
    % loop for each satellite
    for l = 1:4
        
        % Create new s_hat and b_hat from x_hat
        s_hat = x_hat_1(1:3,k1);
        b_hat = x_hat_1(4,k1);
        
        % Calculate 4x1 matrix h_hat
        delta_s = s_hat - sl{l};
        Rl_hat = sqrt((delta_s)'*delta_s);
        h_hat(l) = Rl_hat + b_hat;
        
        % Calculate Jacobian
        Jacob(l,1:3) = ((delta_s)')/Rl_hat;
        Jacob(l,4) = 1;
    end
    
    % Compute the next x_hat using GDA
    x_hat_1(:,k1+1) = x_hat_1(:,k1) + a*(Jacob')*(yl - h_hat);
    
    % Compute loss function for analysis and plotting in the next section
    loss_1(k1) = 0.5*(yl - h_hat)'*(yl - h_hat);
    
    % Compute receiver position estimate error for analysis and plotting in
    % next section
    pos_error_1(k1) = sqrt((s_hat - s)'*(s_hat - s));
    
    % Compute clock bias estimate error for analysis and plotting in next
    % section 
    clock_bias_error_1(k1) = abs(b_hat - b);
    
    % Compute termination criteria for next loop
    delta_x = x_hat_1(:,k1+1) - x_hat_1(:,k1);
    term_criteria = sqrt((delta_x)'*delta_x);
    
    % Iterate k for next while loop
    k1 = k1 + 1;
end

% Create variable for final value of x_hat
x_hat_1_final = x_hat_1(:,k1)
    
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Apply the Gauss Newton Algorithm to locate the vehicle for the zero     %
% noise case. Errors in position estimates have units of meters.          %                                                            
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
    
%  Initialize x_hat to be column vector of s and b
% First column is zero so that while loop can start 
x_hat_2 = s_hat;
x_hat_2(4) = b_hat;
x_hat_2(:,1) = x_hat_2;

% Initialize h_hat and Jacobian
h_hat = zeros(4,1);
Jacob = zeros(4,4);

% Initialize loss function column array
loss_2 = zeros(3,1);

% Initialize receiver position estimate error array
pos_error_2 = zeros(3,1);

% Initialize clock bias estimate error array
clock_bias_error_2  = zeros(3,1);

% Initialize k that will increment on each while loop
k2 = 1;

% Choose step size and termination criteria (in ER)
a = 1;
E = 0.00000016;

while (k2 == 1 || term_criteria > E) && k2 < 1000
    % loop for each satellite
    for l = 1:4
        
        % Create new s_hat and b_hat from x_hat
        s_hat = x_hat_2(1:3,k2);
        b_hat = x_hat_2(4,k2);
        
        % Calculate 4x1 matrix h_hat
        delta_s = s_hat - sl{l};
        Rl_hat = sqrt((delta_s)'*delta_s);
        h_hat(l) = Rl_hat + b_hat;
        
        % Calculate Jacobian
        Jacob(l,1:3) = ((delta_s)')/Rl_hat;
        Jacob(l,4) = 1;
    end
    
    % Compute the next x_hat using GNA
    x_hat_2(:,k2+1) = x_hat_2(:,k2) + a*(inv(Jacob))*(yl - h_hat);
    
    % Compute loss function for analysis and plotting in the next section
    loss_2(k2) = 0.5*(yl - h_hat)'*(yl - h_hat);
    
    % Compute receiver position estimate error for analysis and plotting in
    % next section
    pos_error_2(k2) = sqrt((s_hat - s)'*(s_hat - s));
    
    % Compute clock bias estimate error for analysis and plotting in next
    % section 
    clock_bias_error_2(k2) = abs(b_hat - b);
    
    % Compute termination criteria for next loop
    delta_x = x_hat_2(:,k2+1) - x_hat_2(:,k2);
    term_criteria = sqrt((delta_x)'*delta_x);
    
    % Iterate k for next while loop
    k2 = k2 + 1;
end        
        
% Create variable for final value of x_hat
x_hat_2_final = x_hat_2(:,k2)   

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%                                                                         %
% Plot the (1) loss function, (2) receiver position error, (3) clock      %
% bias estimate error, and (4) 3D line plot of position as a function of  %
% k, our iteration, for both GDA and GNA.                                 %                                                                  %
%                                                                         %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% From earlier computations for GDA and GNA, we have (1),(2),(3) in 
% the form of column arrays:
%       GDA: loss_1
%            pos_error_1
%            clock_bias_error_1
%       GNA: loss_2
%            pos_error_2
%            clock_bias_error_2

% Convert pos_error and clock_bias_error arrays to meters for analysis and
% plotting. 1 ER = 6,370,000 meters
pos_error_1 = pos_error_1*6370000;
pos_error_2 = pos_error_2*6370000;
clock_bias_error_1 = clock_bias_error_1*6370000;
clock_bias_error_2 = clock_bias_error_2*6370000;

% Plot the loss function array vs k for GDA
figure
subplot(1,2,1)
k1_plot = 1:(k1-1);
plot(k1_plot,loss_1)
title('Loss Function for Gradient Descent Algorithm')
ylabel('Least Squares Loss')
xlabel('Iteration K')

% Plot the loss function array vs k for GNA 
subplot(1,2,2)
k2_plot = 1:k2;
plot(k2_plot,loss_2)
title('Loss Function for Gauss-Newton Algorithm')
ylabel('Least Squares Loss')
xlabel('Iteration k')

% Plot the receiver position estimate error vs k for GDA
figure
subplot(1,2,1)
plot(k1_plot,pos_error_1)
title('Receiver Postition Estimate Error for Gradient Descent Algorithm')
ylabel('Receiver Position Estimate Error')
xlabel('Iteration k')

% Plot the receiver position estimate error vs k for GNA
subplot(2,2,2)
plot(k2_plot,pos_error_2)
title('Receiver Postition Estimate Error for Gauss-Newton Algorithm')
ylabel('Receiver Position Estimate Error')
xlabel('Iteration k')

% Plot the clock bias estimate error vs k for GDA
figure
subplot(1,2,1)
plot(k1_plot,clock_bias_error_1)
title('Clock Bias Estimate Error for Gradient Descent Algorithm')
ylabel('Clock Bias Estimate Error')
xlabel('Iteration k')

% Plot the clock bias estimate error vs k for GNA
subplot(2,2,2)
plot(k2_plot,clock_bias_error_2)
title('Clock Bias Estimate Error for Gauss-Newton Algorithm')
ylabel('Clock Bias Estimate Error')
xlabel('Iteration k')

% Plot a 3D line plot of position (units ER) vs k for GDA
x_pos_1 = x_hat_1(1,:);
y_pos_1 = x_hat_1(2,:);
z_pos_1 = x_hat_1(3,:);
figure 
subplot(2,2,1)
plot3(x_pos_1,y_pos_1,z_pos_1)
title('3D Line Plot for Position (ER) vs Iterations k for GDA')
xlabel('X')
ylabel('Y')
zlabel('Z')

% Plot a 3D line plot of position (units ER) vs k for GNA
x_pos_2 = x_hat_2(1,:);
y_pos_2 = x_hat_2(2,:);
z_pos_2 = x_hat_2(3,:);
subplot(2,2,2)
plot3(x_pos_2,y_pos_2,z_pos_2)
title('3D Line Plot for Position (ER) vs Iterations k for GNA')
xlabel('X')
ylabel('Y')
zlabel('Z')




        





