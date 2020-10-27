%% Ikine - Test
close all;
clear;
clc;

l1 = 100;
l2 = 300.002;
l3 = 309;
l4 = 115;
l5 = 190;
l6 = 27.35;
l7 = 40.153;
l8 = 189.795; % Spring Shin mechanism: +- 22.5 mm
l9 = 55;
l10 = 40.25;
l11 = 74.75;

l = [l1, l2, l3, l4, l5, l6, l7, l8, l9, l10, l11];

p0 = [0, 0];
p4 = [-13.988, -498.771];

[q, ikP, ikTheta] = main_ikine(l, p0, p4);
rad2deg(q)

[fkP, fkTheta] = main_fkine(l, p0, q)
%% Inverse Kinematics - Main Mechanism (Points 0 - 4)
function [q, outP, theta] = main_ikine(l, p0, p4)
    lr1 = distance(p0(1), p0(2), p4(1), p4(2));
    alpha = angleCosineRule(lr1, l(2), l(4) + l(5));
    lr2 = sideCosineRule(l(2), l(4), alpha);
    beta = angleCosineRule(lr2, l(1), l(3));
    delta1 = angleCosineRule(l(2), l(4), lr2);
    delta2 = angleCosineRule(l(1), lr2, l(3));
    gamma1 = angleCosineRule(l(4), l(2), lr2); % or PI - (alpha + delta1);
    gamma2 = angleCosineRule(l(3), lr2, l(1)); % or PI - (Beta + delta2);
    gamma = gamma1 + gamma2;
    iota = angleCosineRule(l(4) + l(5), l(2), lr1);
    
    if (p4(1) > p0(1))
        sigma = -atan((p4(2)-p0(2))/(p4(1)-p0(1)));
    else
        sigma = pi - atan((p4(2)-p0(2))/(p4(1)-p0(1)));
    end
    
    q(2) = sigma + iota;
    q(1) = q(2) - gamma;
    
    outP(1, 1) = p0(1) + l(1) * cos(q(1));
    outP(1, 2) = p0(2) - l(1) * sin(q(1));
    outP(2, 1) = p0(1) + l(2) * cos(q(2));
    outP(2, 2) = p0(2) - l(2) * sin(q(2));
    outP(3, 1) = p0(1) + lr2 * cos(q(1) + gamma2);
    outP(3, 2) = p0(2) - lr2 * sin(q(1) + gamma2);
    outP(4, 1) = p4(1);
    outP(4, 2) = p4(2);
    
    theta = q(2) - (pi - alpha); 
    outP(5, 1) = outP(2, 1) + l(11) * cos(theta);
    outP(5, 2) = outP(2, 2) - l(11) * sin(theta);
end
%% Inverse Kinematics - Shin Mechanism
function [q, outP, theta] = shin_ikine(l, inP, p8, omega, theta)
    for (i = 1:5)
        for (j = 1:2)
            outP(i, j) = inP(i, j);
        end
    end
    phi = % circle with vertical line through it on New IK3
    outP(6, 1) = 
    lr1 = distance(p(5, 1), p(5, 2), p8(1), p8(2));
    alpha1 = angleCosineRule(l(8), l(7)
end
%% Forward Kinematics - Main Mechanism (Points 0 - 4)
function [outP, theta] = main_fkine(l, p0, q)
    outP(1, 1) = p0(1) + l(1) * cos(q(1));
    outP(1, 2) = p0(2) - l(1) * sin(q(1));
    outP(2, 1) = p0(1) + l(2) * cos(q(2));
    outP(2, 2) = p0(2) - l(2) * sin(q(2));
    
    lr1 = distance(outP(1, 1), outP(1, 2), outP(2, 1), outP(2, 2));
    alpha1 = angleCosineRule(l(1), l(2), lr1);
    alpha2 = angleCosineRule(l(3), lr1, l(4));
    gamma1 = q(2) - q(1); % or angleCosineRule(lr1, l(2), l(1))
    gamma2 = angleCosineRule(lr1, l(4), l(3));
    beta1 = angleCosineRule(l(2), lr1, l(1)); % or pi - (gamma1 + alpha1)
    beta2 = angleCosineRule(l(4), lr1, l(3)); % or pi - (gamma2 + alpah2)
    alpha = alpha1 + alpha2;
    
    outP(3, 1) = outP(2, 1) + l(4) * cos(q(2) - (pi - alpha)); % = p(1, 1) + l(3) * cos(q(1) + (pi - beta))
    outP(3, 2) = outP(2, 2) - l(4) * sin(q(2) - (pi - alpha)); % = p(1, 2) - l(3) * sin(q(1) + (pi - beta))
    outP(4, 1) = outP(2, 1) + (l(4) + l(5)) * cos(q(2) - (pi - alpha));
    outP(4, 2) = outP(2, 2) - (l(4) + l(5)) * sin(q(2) - (pi - alpha));
    
    theta = q(2) - (pi - alpha);
    outP(5, 1) = outP(2, 1) + l(11) * cos(theta);
    outP(5, 2) = outP(2, 2) - l(11) * sin(theta);
end
%% Angle Cosine Rule
function A = angleCosineRule(a, b, c)
    A = acos((b^2 + c^2 - a^2) / (2 * b * c));
end
%% Side Cosine Rule
function a = sideCosineRule(b, c, A)
    a = sqrt(b^2 + c^2 - 2 * b * c * cos(A));
end
%% Distance
function dist = distance(x1, y1, x2, y2)
    dist = abs(sqrt((y1 - y2)^2 + (x1 - x2)^2));
end