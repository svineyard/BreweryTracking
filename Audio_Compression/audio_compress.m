function [MSE_y_recon_q_a, MSE_y_recon_q, MSE_yq] = MSE(r,alpha)
% To automate testing for a spectrum of quantization rates and alpha
% numbers, this entire script will be a function that outputs three values:
% 1) The MSE of the reconstructed signal from quantized residuals and
% coefficients 
% 2) The MSE of the reconstructed signal from quantized residuals
% 3) The MSE of the directly quantized signals 
% This function will be called in separate 3D and 3D plotting scripts
% with a nested for loop that runs through the range of values for alpha 
% and r. 


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 1
% Purpose: Read in audio file into y, prepare for blocking, and initialize
% matrices
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% audioread takes in speech data and returns sampled data y and rate Fs
[y,Fs] = audioread('homer_simpson.wav');
info = audioinfo('homer_simpson.wav');

% Truncate num_blocks and remove decimal portion
num_blocks = numel (y(:,1))/160;

if round(num_blocks) > num_blocks
    num_blocks = round(num_blocks - 0.5);
else num_blocks = round(num_blocks);
end

% remove last 83 data points of original audio signal
y(74401:74483) = [];

% L is a specified number of quanitzation levels 
% Tradeoff is resulting signal quality vs amount of data needed to 
% represent each sample 
L = 2^r;

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 2
% Purpose: Separate y into blocks of 160 data points and store into 
%          matrix y_blocks as individual columns. Perform quantization
%          operations on y
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% initialize variables 
y_block = zeros(160,465); 
yq = [];
N = numel(y);
MSE_yq = 0;

% Initialize intermediate variable to temporarily store block into
current_block = [];

% Initial values used to increment through each block of 160 data points 
j = 1; k = 160; 

% Loop through y and perform following task:
%   1) add new column of 160 data points for each iteration
%   2) take mean and standard deviation for that column
%   3) truncate outlier values 
%   4) compute quantized y_block
%   5) compute MSE for each block and calculate average 

for i = 1:num_blocks
    current_block = y(j:k);
    
    % set mean and standard dev for block of current iteration
    mean_yblock = mean(current_block);
    std_yblock = std(current_block);
    
    % threshold outlier values for block
    upper_thresh = mean_yblock + (alpha*std_yblock);
    lower_thresh = mean_yblock - (alpha*std_yblock);
    
    % Truncate outliers in y_block to upper and lower thresholds
    current_block(current_block > upper_thresh) = upper_thresh;
    current_block(current_block < lower_thresh) = lower_thresh;
    
    % Assign current block of data to cumulative array of blocks y_block
    y_block(:,i) = current_block;
    
    % compute quantization interval level and quantized y
    q = (max(y_block(:,i)) - min(y_block(:,i)))/(L-1);
    yq(j:k,1) = round(y_block (:,i)/q)*q;
    
    % Mean Squared-Error 
    % MSE_block = (y_block(:,i)-yq(j:k))'*(y_block(:,i)-yq(j:k))/N;
    MSE_block = immse(y(j:k), yq(j:k));
    MSE_yq = (MSE_yq + MSE_block);
    
    % increment j and k for next block 
    j = j + 160;
    k = k + 160;
end

% compute average MSE_y _yq
MSE_yq = MSE_yq/465;

% sound(yq,Fs)

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 3
% Purpose: Compute filter coefficients from autoregressive model by solving
% for a in linear inverse problem y = Aa. A is a 160 x 10 matrix using last
% 10 elements of previous block.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% initialize variables
A = [];
j = 161;
k = 320;
C = [];
R = [];

% initialize last block with zeros that will be shifted circularly to front
% of y
y(74401:74560,1) = 0;
y = circshift(y,160);

% create toeplitz matrix A used for y = Aa and store each block A in 
% multidim array indexed by 3rd parameter A(:,:,i)
for i = 1:num_blocks
    
    % set C and R for toeplitz matrix command 
    C = y(j-1:k-1);
    for n = 1:10
        R(1,n) = y(j-n);
    end
    
    A(1:160,1:10,i) = toeplitz(C,R);
    
    j = j + 160;
    k = k + 160;
end

% compute filter coefficients a(k) for each block of y

% initialize j and k and filter coeff matrix
j = 161;
k = 320;
a = [];

for i = 1:num_blocks
    a(:,i) = (A(:,:,i))\y(j:k);
    
    j = j + 160;
    k = k + 160;
end
    
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 4
% Purpose: Calculate residual error in each block due to no solution for 
% Aa = b
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% initialize j,k, and vector of errors e
j = 161;
k = 320;
e = [];

for i = 1:num_blocks
    e(:,i) = y(j:k) - A(:,:,i)*a(:,i);
    
    j = j + 160;
    k = k + 160;
end 

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 5
% Purpose: Write FIR digital filter program that computes residuals
% directly from eq(2) and y(n) using filter coefficients. Compute MSE for
% these residuals and those computed in Section 4
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% initialize variables 
e_FIR = [];
block = 1;

% test

% Using eq(2) e(n) = y - y_hat where y_hat is the summation of a(k)*y(n-k)
% over k = 1:10

for n = 161:numel(y)
    % compute y_hat for K = 1:10
    y_hat = 0;
    
    for K = 1:10
        y_hat = y_hat + a(K,block)*y(n - K,1);
    end
    
    % update FIR error array for each data point
    e_FIR(n -  160,1) = y(n) - y_hat;
    
    % increment to next block every 160 data points 
    if mod(n,160) == 0
        block = block + 1;
    end

end
    
e = reshape(e, [74400,1]);

MSE_e_FIR_e = immse(e,e_FIR);

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 6
% Purpose: Write a AR digital filter that reconstructs audio data directly
% from eq (1) using residuals computed in steps 4 & 5. For each data block,
% form MSE between y and reconstructed y. This should be equal to zero.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% initialize variables
block = 1;
y_recon = [];
y_recon(1:160,1) = 0;

for n = 161:numel(y)
    % compute y_hat for K = 1:10
    y_hat = 0;
    
    for K = 1:10
        y_hat = y_hat + a(K,block)*y(n - K,1);
    end
    
    y_recon(n,1) = y_hat + e(n - 160);
      
    % increment to next block every 160 data points 
    if mod(n,160) == 0
        block = block + 1;
    end
    
end
    
% compute MSE between reconstructed y and original y
MSE_y_recon_y = immse(y_recon,y);

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 7
% Purpose: Quantize residuals as previously done for step 2, but choice of 
% alpha is critical for successful quanitzation. Reconstruct y (similarly
% to step 6) from quantized residuals.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% intiialize quantized e array 
e_q = zeros(160,465);

% reshape residuals into blocks of 160
e = reshape(e,[160,465]);

% Initialize temporary variable used for truncation of e for current block
temp_e = zeros(160,465);

% quantize residuals in similar method as step 2
for i = 1:num_blocks
    
    % set mean and standard dev for block of current iteration
    mean_eblock = mean(e(:,i));
    std_eblock = std(e(:,i));
    
    % threshold outlier values for block
    upper_thresh = mean_eblock + (alpha*std_eblock);
    lower_thresh = mean_eblock - (alpha*std_eblock);
    
    % Use a temporary variable to truncate the current block of e
    temp_e = e(:,i);
    
    % Truncate outliers in each block of e to upper and lower thresholds
    temp_e(temp_e > upper_thresh) = upper_thresh;
    temp_e(temp_e < lower_thresh) = lower_thresh;
    
    % Assign truncated block back to e array
    e(:,i) = temp_e;
    
    % compute quantization interval level and quantized e
    q = (max(e(:,i)) - min(e(:,i)))/(L-1);
    e_q(:,i) = round(e (:,i)/q)*q;
    
end

% Reconstruct y again but this time use quantized residuals
e_q = reshape(e_q,[74400,1]);

% initialize variables
block = 1;
y_recon_q = [];
y_recon_q(1:160,1) = 0;

for n = 161:numel(y)
    % compute y_hat for K = 1:10
    y_hat = 0;
    
    for K = 1:10
        y_hat = y_hat + a(K,block)*y(n - K,1);
    end
    
    y_recon_q(n,1) = y_hat + e_q(n - 160);
      
    % increment to next block every 160 data points 
    if mod(n,160) == 0
        block = block + 1;
    end
    
end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 8
% Purpose: Compute MSEs for reconstructed y from quantized residuals and
% compare with MSEs for directly quantized y.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

MSE_y_recon_q = immse(y_recon_q,y);

% Plots for both MSEs vs alpha and r are created on separate scripts
% threedeeplot.m and twodeeplot.m

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Step 9
% Purpose: Quantize the vector of filter coeffecients determined in step
% 3.Large values of alpha favor non-distortion of large coeffecients and
% vice verse. 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% intiialize quantized filter coefficients array
a_q = zeros(10,465);

% quantize residuals in similar method as step 2
for i = 1:num_blocks
    
    % set mean and standard dev for block of current iteration
    mean_ablock = mean(a(:,i));
    std_ablock = std(a(:,i));
    
    % threshold outlier values for block
    upper_thresh = mean_ablock + (alpha*std_ablock);
    lower_thresh = mean_ablock - (alpha*std_ablock);
    
    % Use a temporary variable to truncate the current block of a
    temp_a = a(:,i);
    
    % Truncate outliers in each of a to upper and lower thresholds
    temp_a(temp_a > upper_thresh) = upper_thresh;
    temp_a(temp_a < lower_thresh) = lower_thresh;
        
    % Assign truncated block back to a array
    a(:,i) = temp_a;
    
    % compute quantization interval level and quantized y
    q = (max(a(:,i)) - min(a(:,i)))/(L-1);
    a_q(:,i) = round(a(:,i)/q)*q;
    
end

% Reconstruct y again but this time use quantized residuals and quantized
% filter coefficients 

% initialize variables
block = 1;
y_recon_q_a = zeros(74560,1);

for n = 161:numel(y)
    % compute y_hat for K = 1:10
    y_hat = 0;
    
    for K = 1:10
        y_hat = y_hat + a_q(K,block)*y(n - K,1);
    end
    
    y_recon_q_a(n,1) = y_hat + e_q(n - 160);
      
    % increment to next block every 160 data points 
    if mod(n,160) == 0
        block = block + 1;
    end
    
end

% Compute MSE for reconstructed y from quantized e and a
MSE_y_recon_q_a = immse(y_recon_q_a,y);

% end function
end

    





    
    


    


    




    

            
        