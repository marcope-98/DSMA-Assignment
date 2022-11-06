
function scans = LidarScan(pose, map, lidarsettings)
    
    a = lidarsettings.maxRange;
    s = lidarsettings.ScanSize;
    angles = lidarsettings.Angles;
    ranges = a*ones(s,1);
    pos = pose(1:2);
    yaw = pose(3);
    rotMat = [cos(yaw), -sin(yaw);...
              sin(yaw), +cos(yaw)];
    % for each ray in the scan line
%     figure
%     plot(map)
%     hold on;
    for i = 1 : s
        lineseg = [pos; (pos' + a*rotMat*[cos(angles(i)); sin(angles(i))])'];
        [in,~] = intersect(map, lineseg);

%       [in,out] = intersect(map, lineseg);
%       plot(in(:,1), in(:,2), 'r',out(:,1), out(:,2), 'b')
        if numel(in) ~= 0 && norm(in(1,:) - pos) < a
            ranges(i) = norm(in(1,:) - pos);
        end
    end
    hold off;

    scans = lidarScan(ranges, angles);
end