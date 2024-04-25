function [ x_n1 ] = RK4( Fxva, x_n, v_n, a_n, h )
% function for evaluating net step using Runge-Kutta-4
% Inputs: 
% Fxva - function of dx/dt using x,v,a,h(=dt)
% x_n - current parameter x value
% v_n - current parameter velocity value of x
% a_n - current parameter acceleration value of x
% h [sec] - time step (dt)
% Output: 
% x_n1 - next step value of x

k_1 = Fxva(x_n, v_n, a_n, 0);
k_2 = Fxva(x_n+0.5*h*k_1, v_n, a_n, h/2);
k_3 = Fxva(x_n+0.5*h*k_2, v_n, a_n, h/2);
k_4 = Fxva(x_n+h*k_3, v_n, a_n, h);

x_n1 = x_n + (1/6)*(k_1+k_2+k_3+k_4)*h;
end

