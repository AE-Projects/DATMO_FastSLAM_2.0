clear
clc
close all;

simOutFile = 'so_sym2_nk_wmov.txt';
mapFile = 'map_sym2.txt';
load('mov_obj_pos.mat')
datasetBase = 'DataSets/';
%Start by reading in the true map into a matrix
d = load([datasetBase mapFile]);

map = d(:,2:3)';
%map = cat(1,map,d(:,1)'); % Add id of the landmarks to map matrix
mapIds = d(:,1)';

% Now for the measurment data we have different lengths for each line in
% the file so we must work harder to load them line by line.
fid = fopen([datasetBase simOutFile],'r');
if fid <= 0
  fprintf('Failed to open simoutput file "%s"\n',simOutFile);
  return
end
simSteps = {};
while 1
    line = fgetl(fid);
    if ~ischar(line)
        break
    end
    values = sscanf(line, '%f');
    %notice that we call the creator from the SimStep Class here to parse
    %this into a 'SimStep' object.
    simSteps = {simSteps{:} SimStep(values)}; %#ok
end
fclose(fid);

n_timesteps = size(simSteps,2);
pose_true = zeros(3, n_timesteps-1);
for t = 2:n_timesteps
    z_t = simSteps{t}.seenLandmarks;
    z_t = z_t(1:3,:);
    mov_idx = find(z_t(3,:) < 0);
    if ~isempty(mov_idx)
        first_detect = 1;
    end
    z_t_m = z_t(1:2, mov_idx);
    z_t_s = z_t(1:2,:);
    z_t_s(:,mov_idx) = [];
    plot_mov_dataset(map, simSteps{t}.truePose, z_t_s, z_t_m, [xm(1,t);ym(1,t)])
    drawnow;
    hold on
end