%% Test
close all;
clear;
clc;

l1 = 75;
l2 = 110;
l3 = 110;
l4 = 75;
l5 = 30;
l6 = 22.563;
l7 = 110;
l8 = 105;
l9 = 20;
l10 = 25;
T1 = 22.563;
T2 = 22.563;
T3 = 20;

l = [l1, l2, l3, l4, l5, l6, l7, l8, l9 ,l10, T1, T2, T3];
p0 = [10, 0];
q = [degtorad(67.752), degtorad(183.447), degtorad(226.837)];
[p, angles] = fkine(l, p0, q)

testPoint = [p(4, 1), p(4, 2)];
testTau = angles;
[outQ, outP] = ikine(l, p0, testPoint, testTau);
radtodeg(outQ)
simulate(outP);
%% Simulation - Plots all points and connects them to show leg
function simulate(p)
    plot(p(:, 1), p(:, 2), 'o');
    hold on;
    lineMap = [0 1 ; 0 2 ; 0 5 ; 1 3 ; 2 3 ; 3 4 ; 5 6 ; 6 7 ; 7 8 ; 2 7 ; 8 4 ; 2 6];
    [numRows, numCols] = size(lineMap);
    for i = 1:numRows
       line([p(lineMap(i, 1) + 1, 1), p(lineMap(i, 2) + 1, 1)], [p(lineMap(i, 1) + 1, 2), p(lineMap(i, 2) + 1, 2)]);
    end
end
%% Forward Kinematics - Maybe just add all angles as an output
function [p, tau] = fkine(l, p0, q)
    p(1, 1) = p0(1) + l(1) * cos(q(1));
    p(1, 2) = p0(2) - l(1) * sin(q(1));
    p(2, 1) = p0(1) + l(2) * cos(q(2));
    p(2, 2) = p0(2) - l(2) * sin(q(2));
    p(5, 1) = p0(1) + l(6) * cos(q(3));
    p(5, 2) = p0(1) - l(6) * sin(q(3));
    
    lr1 = distance(p(2, 1), p(1, 1), p(2, 2), p(1, 2));
    alpha(1) = angleCosineRule(l(1), l(2), lr1);
    alpha(2) = angleCosineRule(l(3), lr1, l(4));
    gamma(1) = angleCosineRule(lr1, l(2), l(1));
    beta(1) = pi - (alpha(1) + gamma(1));
    beta(2) = angleCosineRule(l(4), l(1), l(3));
    
    p(3, 1) = p(2, 1) + l(4) * cos(q(2) - (pi - (alpha(1) + alpha(2))));
    p(3, 2) = p(2, 2) - l(4) * sin(q(2) - (pi - (alpha(1) + alpha(2))));
    
    gamma(2) = pi - (alpha(2) + beta(2));
    
    p(4, 1) = p(2, 1) + (l(4) + l(5)) * cos(q(2) - (pi - (alpha(1) + alpha(2))));
    p(4, 2) = p(2, 2) - (l(4) + l(5)) * sin(q(2) - (pi - (alpha(1) + alpha(2))));
    
    lr2 = distance(p(5, 1), p(2, 1), p(5, 2), p(2, 2));
    delta(1) = angleCosineRule(l(2), l(6), lr2);
    delta(2) = angleCosineRule(l(11), l(7), lr2);
    
    p(6, 1) = p(5, 1) + l(7) * cos(q(3) - (pi - (delta(1) + delta(2))));
    p(6, 2) = p(5, 2) - l(7) * sin(q(3) - (pi - (delta(1) + delta(2))));
    
    omega(1) = angleCosineRule(l(11), l(12), l(13));
    omega(3) = angleCosineRule(l(13), l(12), l(11));
    omega(2) = pi - (omega(1) + omega(3));
    lambda = angleCosineRule(lr2, l(7), l(12));
    
    p(7, 1) = p(6, 1) + l(11) * cos(q(3) - (pi - (delta(1) + delta(2))) - (pi - (lambda + omega(3))));
    p(7, 2) = p(6, 2) - l(11) * sin(q(3) - (pi - (delta(1) + delta(2))) - (pi - (lambda + omega(3))));
    
    lr3 = distance(p(7, 1), p(4, 1), p(7, 2), p(4, 2));
    phi(1) = angleCosineRule((l(4) + l(5)), l(13), lr3);
    phi(2) = angleCosineRule(l(9), lr3, l(8));
    
    p(8, 1) = p(7, 1) + l(8) * cos(q(3) - (pi - (delta(1) + delta(2))) - (pi - (lambda + omega(3))) - (pi - (omega(2) + (phi(1) + phi(2)))));
    p(8, 2) = p(7, 2) - l(8) * sin(q(3) - (pi - (delta(1) + delta(2))) - (pi - (lambda + omega(3))) - (pi - (omega(2) + (phi(1) + phi(2)))));
    
    if (p(4, 2) > p(8, 2))
        if (p(4, 1) > p(8, 1))
            tau = abs(atan((p(4, 2) - p(8, 2)) / (p(4, 1) - p(8, 1))));
        else
            tau = pi - abs(atan((p(4, 2) - p(8, 2)) / (p(4, 1) - p(8, 1))));
        end
    else
        if (p(4, 1) > p(8, 1))
            tau = -abs(atan((p(4, 2) - p(8, 2)) / (p(4, 1) - p(8, 1))));
        else
            tau = -(pi - abs(atan((p(4, 2) - p(8, 2)) / (p(4, 1) - p(8, 1)))));
        end
    end
    p = [p0 ; p];
end
%% Inverse Kinematics
function [q, p] = ikine(l, p0, p4, tau)
    p(4, 1) = p4(1);
    p(4, 2) = p4(2);
    p(8, 1) = p(4, 1) - l(9) * cos(tau);
    p(8, 2) = p(4, 2) - l(9) * sin(tau);
    
    lr1 = distance(p0(1), p4(1), p0(2), p4(2));
    alpha = angleCosineRule(lr1, l(2), (l(4) + l(5)));
    lr2 = sideCosineRule(l(2), l(4), alpha);
    beta = angleCosineRule(lr2, l(1), l(3));
    delta(1) = angleCosineRule(l(2), l(4), lr2);
    delta(2) = angleCosineRule(l(1), lr2, l(3));
    gamma(1) = pi - (alpha + delta(1));
    gamma(2) = pi - (beta + delta(2));
    iota = angleCosineRule((l(4) + l(5)), l(2), lr1);
    sigma = abs(atan((p4(2) - p0(2)) / (p4(1) - p0(1))));
    theta = 0;
    
    if (p4(1) < p0(1))
        if (p4(2) < p0(2))
            theta = sigma - iota;
            q(2) = pi - (sigma - iota);
        else
            theta = iota + sigma;
            q(2) = pi + iota + sigma;
        end
    else
        if (p4(2) < p0(2))
            theta = pi - (sigma + iota);
            q(2) = sigma + iota;
        else
            theta = pi - (iota - sigma);
            q(2) = iota - sigma;
        end
    end
    
    q(1) = q(2) - (gamma(1) + gamma(2)); %or pi - (theta + gamma)
    
    p(1, 1) = p0(1) + l(1) * cos(q(1));
    p(1, 2) = p0(2) - l(1) * sin(q(1));
    p(2, 1) = p0(1) + l(2) * cos(q(2));
    p(2, 2) = p0(2) - l(2) * sin(q(2));
    p(3, 1) = p0(1) + lr2 * cos(q(1) + gamma(2));
    p(3, 2) = p0(2) - lr2 * sin(q(1) + gamma(2));
    
    lr3 = distance(p(2, 1), p(8, 1), p(2, 2), p(8, 2));
    w(1) = angleCosineRule((l(4) + l(5)), lr3, l(9));
    w(2) = angleCosineRule(l(13), l(8), lr3);
    
    p(7, 1) = p(8, 1) + l(8) * cos(tau + (w(1) + w(2)));
    p(7, 2) = p(8, 2) + l(8) * sin(tau + (w(1) + w(2)));
    
    omega(2) = angleCosineRule(l(12), l(11), l(13));
    omega(1) = angleCosineRule(l(11), l(12), l(13));
    omega(3) = pi - (omega(1) + omega(2));
    phi = angleCosineRule(lr3, l(13), l(8));
    
    p(6, 1) = p(7, 1) + l(11) * cos(tau + (w(1) + w(2)) - (pi - (omega(2) + phi)));
    p(6, 2) = p(7, 2) + l(11) * sin(tau + (w(1) + w(2)) - (pi - (omega(2) + phi)));
    
    lr4 = distance(p(6, 1), p0(1), p(6, 2), p0(2));
    lambda(1) = angleCosineRule(l(2), lr4, l(12));
    lambda(2) = angleCosineRule(l(6), lr4, l(7));
    
    p(5, 1) = p(6, 1) + l(7) * cos(tau + (w(1) + w(2)) - (pi - (omega(2) + phi)) - (pi - ((lambda(1) + lambda(2)) + omega(3))));
    p(5, 2) = p(6, 2) + l(7) * sin(tau + (w(1) + w(2)) - (pi - (omega(2) + phi)) - (pi - ((lambda(1) + lambda(2)) + omega(3))));
    
    epsilon = 0;
    if (p(5, 1) > p0(1))
        epsilon = pi - abs(atan((p(5, 2) - p0(2)) / (p(5, 1) - p0(1))));
    else
        epsilon = abs(atan((p(5, 2) - p0(2)) / (p(5, 1) - p0(1))));
    end
    
    if (p(5, 2) > p0(2))
        q(3) = pi + epsilon;
    else
        q(3) = pi - epsilon;
    end
    p = [p0 ; p];
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
function dist = distance(x1, x2, y1, y2)
    dist = abs(sqrt((y1 - y2)^2 + (x1 - x2)^2));
end