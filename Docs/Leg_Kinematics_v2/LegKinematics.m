%% Ikine - Test
close all;
clear;
clc;

l1 = 100;
l2 = 300;
l3 = 303.5;
l4 = 100;
l5 = 180;

l = [l1, l2, l3, l4, l5];

p0 = [0, 0];
p4 = [-13.988, -498.771];

[q, p] = ikine(l, p0, p4);
rad2deg(q)
%% Inverse Kinematics - Main Mechanism (Points 0 - 4)
function [q, p] = ikine(l, p0, p4)
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
    
    p(1, 1) = p0(1) + l(1) * cos(q(1));
    p(1, 2) = p0(2) - l(1) * sin(q(1));
    p(2, 1) = p0(1) + l(2) * cos(q(2));
    p(2, 2) = p0(2) - l(2) * sin(q(2));
    p(3, 1) = p0(1) + lr2 * cos(q(1) + gamma2);
    p(3, 2) = p0(2) - lr2 * sin(q(1) + gamma2);
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