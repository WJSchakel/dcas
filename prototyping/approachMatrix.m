%{
This script peforms a simple simulation of a vehicle approaching a slower
leader. The leader has a constant speed. Several instances of the 
approaching vehicle are simulated. With different response to matrix signs
depending on ego-speed sensitivity. 
%}

v0 = 100/3.6;
a = 1.25;
b = 2.09;
b0 = 0.5;
T = 1.2;
s0 = 3;
dt = .5;
vLead = 10/3.6; % leader speed
sStart = 400;
vMatrix = 50/3.6;
%{
The default value for vGain is 69.6km/h. With social interactions this
changes to a log-normal distribution with mu=3.379 and sigma=0.4. This has
a mode of 25km/h. The 25th, 50th and 75th percentiles are 22.4, 29.3 and
38.4 km/h. Truck usually get a value of 50km/h with social interactions. A
value of 0 simplifies to the regular IDM without influence of matrix signs.
A value of 1000km/h is used as a dummy value for DCAS with full willingness
towards carefully approaching traffic ahead.
%}
vGain = [0, 22.4, 29.3, 38.4, 1000]/3.6; % different cases of ego-speed sensitivity approaching matrix
range = [295.0, 295.0, 295.0, 295.0, 150.0];

xb = ones(size(vGain)) .* -sStart; % b = IDM and matrix response
vb = ones(size(vGain)) .* v0;
gb = [];
xLead = 0;

while vb(end,end) > vLead(end) + 1e-2
    % IDM and matrix response at different ego-speed sensitivity
    j = size(xb,1);
    for i = 1:size(xb, 2)
        s = xLead-xb(j,i);
        dv = vb(j,i)-vLead(j);
        if s > range(i)
            s = range(i);
            dv = max(0, vb(j,i)-vMatrix);
            gIdm = idm(s0, vb(j,i), T, a, b, b0, v0, 0.0, inf);
        else
            gIdm = idm(s0, vb(j,i), T, a, b, b0, v0, dv, s);
        end
        f = min(1, vGain(i)/vMatrix);
        %f = min(1, max(0, 1-(vMatrix - vGain(i))/vb(end,i)));
        bb = b - f * (b-b0);
        gVms = max(-b, idm(s0, vb(j,i), T, a, bb, b0, v0, dv, s));
        gb(j,i) = min(gIdm, gVms);
    end

    % Numerical progression
    for i = 1:size(xb, 2)
        [xb(j+1,i), vb(j+1,i)] = extrap(xb(j,i), vb(j,i), gb(j,i), dt);
    end

    % Leader at constant speed
    [xLead, vLead(j+1)] = extrap(xLead, vLead(j), 0.0, dt);
end
% ga(end+1) = ga(end);
gb(end+1,:) = gb(end,:);
t = 0:dt:(size(vb,1)-1)*dt;

figure('Position', [100, 100, 960, 540]);
legLab = {};
for i = 1:size(xb,2)
    if vGain(i) == 0
        legLab{end+1} = 'IDM';
    elseif vGain(i) == 1000/3.6
        legLab{end+1} = 'DCAS';
    else
        legLab{end+1} = sprintf('\\itv_{gain}\\rm=%.1f', vGain(i)*3.6);
    end
end
makePlot(1, xb, vb, 'x [m]', 'v [m/s]', legLab, 'SouthWest');
makePlot(2, t, vb, 't [s]', 'v [m/s]', legLab, 'NorthEast');
makePlot(3, xb, gb, 'x [m]', 'a [m/s^2]', legLab, 'SouthWest');
makePlot(4, t, gb, 't [s]', 'a [m/s^2]', legLab, 'SouthEast');

function g = idm(s0, v, T, a, b, b0, v0, dv, s)
    ss = s0 + v*T + v*dv/(2*sqrt(a*b));
    g = a*min(max(-b0, 1-(v/v0)^4), 1-(ss/s)^2);
end

function [x1, v1] = extrap(x0, v0, g, dt)
    x1 = x0 + v0*dt + .5*g*dt^2;
    v1 = v0 + g*dt;
end

function makePlot(n, xb, yb, xlab, ylab, legLab, legLoc)
    subplot(2,2,n);
    hold on
    for i = 1:size(yb, 2)
        if size(xb,1) == 1
            x = xb;
        else
            x = xb(:,i);
        end
        plot(x, yb(:,i));
    end
    xlabel(xlab);
    ylabel(ylab);
    legend(legLab, 'Location', legLoc)
end