%{
This script calculates the equilibrium speed in situations where there is
no vehicle in range. Then, a virtual vehicle is assumed at the range and
driving at the speed indicated by the matrix sign. In this script full
carefulness (b = b0) is assumed. For human drivers this is not correct as
their effective carefulness depends on vGain. The result is a printed
Markdown table.
%}

a = 1.25;
b0 = 0.5;
T = 1.2;
s0 = 3;

range = [50, 100, 150, 200, 295];
vMatrix = [50, 70, 90];

b = b0;
c = sqrt(a*b);
d = 2*c*T;

%{
To find the equilibirum speed we need to find the speed at which the 
desired distance is equal to the range. Hence:

    s0 + v*T + v(v-vMatrix)/(2*sqrt(a*b)) - range = 0

Substituting c = sqrt(a*b), and multiplying both sides by 2*c we get:

    v^2 + (2*c*T-vMatrix)*v + 2*c*(s0 - range) = 0

Substituting d = 2*c*T we get:

    v^2 + (d-vMatrix)*v + 2*c*(s0 - range) = 0

This is a quadratic equation for which only the positive root is sensible.
Applying the ABC-formula we get:

    v = [vMatrix - d + sqrt( (d-vMatrix)^2 - 8*c*(s0 - range) )] / 2

%}
data = zeros(length(range), length(vMatrix));
for i = 1:length(range)
    R = range(i);
    for j = 1:length(vMatrix)
        V = vMatrix(j) / 3.6;
        vEq = 0.5 * (V - d + sqrt((d - V)^2 - 8*c*(s0 - R)));
        data(i,j) = vEq * 3.6;
    end
end

rows = compose('%.0fm', sRange);
columns = compose('%.0fkm/h', vMatrix);
tab = array2table(data, 'VariableNames', columns, 'RowNames', rows);
printMarkdownTable(tab, '%.1f');

% Author: Copilot
function printMarkdownTable(tab, fmt)

    % Header
    fprintf('| Range');
    if ~isempty(tab.Properties.RowNames)
        fprintf(' |');
    end
    for c = 1:width(tab)
        fprintf(' %s |', tab.Properties.VariableNames{c});
    end
    fprintf('\n');

    % Separator
    fprintf('|');
    if ~isempty(tab.Properties.RowNames)
        fprintf('---|');
    end
    for c = 1:width(tab)
        fprintf('---|');
    end
    fprintf('\n');

    % Rows
    for r = 1:height(tab)
        fprintf('|');

        if ~isempty(tab.Properties.RowNames)
            fprintf(' %s |', tab.Properties.RowNames{r});
        end

        for c = 1:width(tab)
            value = tab{r,c};

            if isnumeric(value) && isscalar(value)
                fprintf([' ' fmt ' |'], value);
            else
                fprintf(' %s |', string(value));
            end
        end

        fprintf('\n');
    end
end