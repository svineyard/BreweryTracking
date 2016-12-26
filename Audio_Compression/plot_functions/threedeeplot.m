% Script for 3D plots of MSE vs alpha vs r

% This script runs through a range of values for quantization rate r and
% thesholding constant alpha to produce MSE values for a 3D plot. This 
% script will call the MSE function to produce:
% (1) MSE of reconstructed audio signal from quantized residuals and coeff
% (2) MSE for the reconstructed audio signal from quantized residuals 
% (3) MSE of the directly quantized signal for all possible r and alpha 
% values.

% The 3D plot with have the r vector as its X axis, the alpha vector as its
% Y axis, and the corresponding MSE values as the Z axis. This Z matrix
% should be have r rows and alpha columns and will be a 3D matrix composed
% of two 2D matrices. These respective 2D matrices are the possible MSE
% values (1),(2),(3) mentioned previously. 

clear all

% Initialize variables 
j = 1;
Z = zeros(8,51);
Z(:,:,2) = zeros(8,51);
Z(:,:,3) = zeros(8,51);

for r = 1:8
    k = 1;
    for alpha = 0:0.1:5
        
        % Call the MSE function 
        [Z(j,k,1),Z(j,k,2),Z(j,k,3)] = MSE(r,alpha);
        
        % increment k for next loop
        k = k + 1;
    end
    % increment j for next loop
    j = j + 1;
end 

% create X and Y vectors from alpha and r ranges
Y = 1:8;
X = 0:0.1:5;

close all

figure 
MSEplot_recon_y_q_a = surf(X,Y,Z(:,:,1));
title('MSE of reconstructured y via quantized e/a vs r and alpha')
ylabel('Quantization rate r')
xlabel('Threshold constant alpha')

figure
MSEplot_recon_y_q = surf(X,Y,Z(:,:,2));
title('MSE of reconstructred y via quantized e vs r and alpha')
ylabel('Quantization rate r')
xlabel('Threshold constant alpha')

figure
MSEplot_yq = surf(X,Y,Z(:,:,3));
title('MSE of yq vs r and alpha')
ylabel('Quantization rate r')
xlabel('Threshold constant alpha')

        
        
        
        
        
    
