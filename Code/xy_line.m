function [ alpha, r, Cov] = xy_line( xy, range_data )

theta   = range_data(:,2);
p       = range_data(:,1);
sigma_p = 0.15;

x = xy(:,1);
y = xy(:,2);
w = ones(length(x),1);

%Mean
x_bar = 1/length(x)*sum(x);
y_bar = 1/length(y)*sum(y);

%Weighted Mean
x_bar_w = 1/sum(w)*sum(w.*x);
y_bar_w = 1/sum(w)*sum(w.*y);


%find the angle of the line 
N = -2*sum(w.*(y_bar_w-y).*(x_bar_w-x));
D = sum(w.*((y_bar_w-y).^2 - (x_bar_w-x).^2));

alpha = 0.5*atan2(N, D);

%Find the distance to the line
r = x_bar_w*cos(alpha)+y_bar_w*sin(alpha);
 
%Make sure r is not negative
if r < 0
    alpha = alpha + pi;
    r = -r;
end




D_N_inter1 = N*(x_bar*cos(theta)-y_bar*sin(theta)-p.*cos(2*theta))-...
               D*(x_bar*sin(theta)-y_bar*cos(theta)-p.*sin(2*theta));
D_N_inter2 = 1/(D^2+N^2);
           
d_alpha_d_P = D_N_inter1*D_N_inter2;
           
d_r_d_P = 1/length(theta)*cos(theta-alpha)+d_alpha_d_P.*(y_bar*cos(alpha)-x_bar*sin(alpha));


sigma_A_2 =D_N_inter2^2*sum(D_N_inter1.^2*sigma_p^2);

sigma_R_2 = sum(d_r_d_P.^2*sigma_p^2)  ;

sigma_AR = sum(d_alpha_d_P.*d_r_d_P*sigma_p^2);

Cov = [sigma_A_2 sigma_AR; sigma_AR sigma_R_2];









% ct = cos(theta);
% st = sin(theta);
% 
% da_dp = zeros(length(x),1);
% 
% for ii = 1:length(p)
%    delta_p_i = zeros(length(x), 1);
%    delta_p_i(ii) = 1;
%    pp1     = p + 100*eps*delta_p_i;
%    pp2     = p - 100*eps*delta_p_i;
%    xx1     = pp1.*ct;
%    xx2     = pp2.*ct;
%    yy1     = pp1.*st;
%    yy2     = pp2.*st;
%    aa1     = get_alpha2( xx1, yy1);
%    aa2     = get_alpha2( xx2, yy2);
%    da_dp(ii)= (aa1-aa2)/(200*eps);
% end
% 
% dr_dp = w.*cos(theta-alpha)/sum(w);
% 
%    
% A = N*(x_bar_w*cos(theta)-y_bar_w*sin(theta)-p.*cos(2*theta));
% B = D*(x_bar_w*sin(theta)+y_bar_w*cos(theta)-p.*sin(2*theta));
% %Sigma AA needs to be redone.
% sigma_AA = 1/(D^2+N^2)^2 * sum(w.*(N*A-D*B).^2*sigma_p^2);
% sigma_rr = sum( (w/sum(w).*cos(theta-alpha)+...
%                  da_dp.*(y_bar_w*cos(alpha)-x_bar_w*sin(alpha))).^2)*sigma_p^2;
% sigma_Ar = sum(da_dp.*dr_dp*sigma_p^2); 
%              
% %Not sure if sigma_AA and sigma_rr should be squared.             
%  Cov = [sigma_AA sigma_Ar; sigma_Ar sigma_rr];
% end
% 
