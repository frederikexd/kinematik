function simulink_dump(rate_hz)
% SIMULINK_DUMP  Every logged signal of the last sm_car run in one CSV, for diagnosing a benchmark difference.
%
%   simulink_dump          % 10 samples per second
%   simulink_dump(rate_hz)
%
% Writes achilles/simulink_dump.csv: a Time column, then one column per logged signal (its path in the header row,
% its units in the next), resampled onto a common time base. A signal with several elements (a vector) gets one
% column per element, path(1), path(2), ... Signals that hold a single value are left out.
if nargin < 1 || isempty(rate_hz), rate_hz = 10; end
if ~evalin('base', 'exist(''logsout_sm_car'', ''var'')')
    error('simulink_dump:log', 'No logsout_sm_car in the workspace: run the lap in sm_car first.');
end
ds = evalin('base', 'logsout_sm_car');
paths = {};
tss = {};
for i = 1:ds.numElements
    el = ds.getElement(i);
    [paths, tss] = walk(paths, tss, el.Values, el.Name);
end
t0 = inf; t1 = -inf;
for k = 1:numel(tss)
    t0 = min(t0, double(tss{k}.Time(1)));
    t1 = max(t1, double(tss{k}.Time(end)));
end
tq = (t0:1 / rate_hz:t1)';
cols = {tq};
names = {'Time'};
unitrow = {'s'};
for k = 1:numel(tss)
    ts = tss{k};
    t = double(ts.Time(:));
    y = double(squeeze(ts.Data));
    if size(y, 1) ~= numel(t), y = y.'; end
    [t, iu] = unique(t);
    y = y(iu, :);
    u = '';
    try, u = char(ts.DataInfo.Units); catch, end
    for j = 1:size(y, 2)
        cols{end+1} = interp1(t, y(:, j), tq, 'linear', NaN); %#ok<AGROW>
        if size(y, 2) > 1
            names{end+1} = sprintf('%s(%d)', paths{k}, j); %#ok<AGROW>
        else
            names{end+1} = paths{k}; %#ok<AGROW>
        end
        unitrow{end+1} = u; %#ok<AGROW>
    end
end
if ~exist('achilles', 'dir'), mkdir('achilles'); end
fid = fopen(fullfile('achilles', 'simulink_dump.csv'), 'w');
fprintf(fid, '%s\n', strjoin(names, ','));
fprintf(fid, '%s\n', strjoin(unitrow, ','));
fprintf(fid, [strjoin(repmat({'%.6g'}, 1, numel(cols)), ',') '\n'], [cols{:}]');
fclose(fid);
d = dir(fullfile('achilles', 'simulink_dump.csv'));
fprintf('written: achilles/simulink_dump.csv, %d signals, %d rows, %.1f MB\n', numel(names) - 1, numel(tq), d.bytes / 1e6);
end

function [paths, tss] = walk(paths, tss, x, path)
if isa(x, 'timeseries')
    if isnumeric(x.Data) && ~isempty(x.Data) && numel(unique(x.Time)) >= 2 && numel(x.Data) / numel(x.Time) <= 16
        paths{end+1, 1} = path;
        tss{end+1, 1} = x;
    end
elseif isstruct(x)
    f = fieldnames(x);
    for j = 1:numel(f)
        [paths, tss] = walk(paths, tss, x(1).(f{j}), [path '.' f{j}]);
    end
end
end
