function [] = show_res( wsName )
%SHOW_RES Summary of this function goes here
%   Detailed explanation goes here

load(wsName);

figure;
plot_result( u_opt_nlp, res_nlp );


figure;
plot_result( u_opt_ltv, res_ltv );

end

