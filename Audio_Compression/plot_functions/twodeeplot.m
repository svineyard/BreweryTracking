% Script for 2D plots of MSE, r , alpha 

% start fresh with each execution
close all;
clear all;

% Initialize MSE for reconstructed y and yq arrays
MSE_yq_r = [];
MSE_y_recon_q_r = [];
MSE_yq_a = [];
MSE_y_recon_q_a = [];

% % % % % % % %  % % % % % % % % % % % % % %  % % % % % % % % % % % % % % %
%
% Call MSE function for range of quantization rate r's and then create 2D
% scatter plot for both reconstructed y(quantized e and quantized e + a) 
% and yq.
%
% % % % % % % %  % % % % % % % % % % % % % %  % % % % % % % % % % % % % % %

% Hold alpha constant at 5 to minimize its thresholding effect
alpha = 5;

for r = 1:8
    [MSE_y_recon_q_a_r(r),MSE_y_recon_q_r(r),MSE_yq_r(r)] = MSE(r,alpha);
end

% scatter plot for MSE of reconstructed y(quantized e and a) vs r
r = 1:8;
figure
scatter(r,MSE_y_recon_q_a_r)
title('MSE of reconstructured y via quantized e and a versus r')
xlabel('Quantization rate r')
ylabel('MSE')

% scatter plot for MSE of reconstructed y(quantized e only) vs r
figure
scatter(r,MSE_y_recon_q_r)
title('MSE of reconstructed y via quantized e versus r')
xlabel('Quantization rate r')
ylabel('MSE')

% scatter plot for MSE of directly quantized yq vs r
figure
scatter(r,MSE_yq_r)
title('MSE of directly quantized y versus r')
xlabel('Quantization rate r')
ylabel('MSE')

% % % % % % % %  % % % % % % % % % % % % % %  % % % % % % % % % % % % % % %
%
% Call MSE function for range of threshold constant alphas and then create
% 2D scatter plots for both reconstructed y (quantized e and quantized e +
% a) and yq.
%
% % % % % % % %  % % % % % % % % % % % % % %  % % % % % % % % % % % % % % %

% hold r constant at 8 to minimize its distortion effect 
r = 8;

% use i for indexing of MSE arrays, same number of indecies as alphas being
% used in for-loop
i = 1;

for alpha = 0:0.1:5
    [MSE_y_recon_q_a_r(i),MSE_y_recon_q_a(i),MSE_yq_a(i)] = MSE(r,alpha);
    i = i + 1;
end 
   
% scatter plot for MSE of reconstructed y(quantized e and a) vs alpha
alpha = 0:0.1:5;
figure
scatter(alpha,MSE_y_recon_q_a_r)
title('MSE of reconstructured y via quantized e and a versus alpha')
xlabel('Thresholding constant alpha')
ylabel('MSE')

% scatter plot for MSE of reconstructed y(quantized e only) vs alpha
figure
scatter(alpha,MSE_yq_a)
title('MSE of reconstructed y via quantized e versus alpha ')
xlabel('Thresholding constant alpha')
ylabel('MSE')

% scatter plot for MSE of yq vs alpha
figure
scatter(alpha,MSE_y_recon_q_a)
title('MSE of directly quantized y versus alpha')
xlabel('Thresholding constant alpha')
ylabel('MSE')




    

   

    
   
