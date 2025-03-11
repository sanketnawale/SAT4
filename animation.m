clc; clear; close all;

% 🌍 Define Ground Stations (Rome, Milan, NodeRM)
nodes = [12.4964, 41.9028;  % Rome
         9.1900, 45.4642;    % Milan
         12.5000, 41.9000];  % NodeRM
nodeNames = {'Rome', 'Milan', 'NodeRM'};

% 🛰️ Define Satellites (Randomly Plotted for Animation)
numSats = 5;
satLat = 20 + rand(1, numSats) * 140;  % Random latitude
satLon = -180 + rand(1, numSats) * 360; % Random longitude
satNames = arrayfun(@(x) sprintf('Sat-%d', x), 1:numSats, 'UniformOutput', false);

% 🎥 Setup Animation Figure
figure;
hold on; grid on;
xlim([-180, 180]); ylim([-90, 90]);
xlabel('Longitude (°)'); ylabel('Latitude (°)');
title('🛰️ Satellite Communication Animation');

% 🏠 Plot Ground Stations
scatter(nodes(:,1), nodes(:,2), 150, 'r', 'filled');
text(nodes(:,1) + 3, nodes(:,2), nodeNames, 'FontSize', 12);

% 🛰️ Plot Satellites
satPlots = scatter(satLon, satLat, 100, 'b', 'filled');
text(satLon + 3, satLat, satNames, 'FontSize', 10);

% 🚀 Animate Packet Transmissions
numSteps = 30;  % Animation steps
for t = 1:numSteps
    % 🎯 Select a Random Satellite for Each Node
    nodeToSat = randi(numSats, [size(nodes,1), 1]);
    
    % 📡 Draw Transmission Arrows
    for n = 1:size(nodes,1)
        satIdx = nodeToSat(n);
        satPos = [satLon(satIdx), satLat(satIdx)];
        nodePos = [nodes(n,1), nodes(n,2)];
        
        % 🎨 Decide if the packet is successful (80% success rate)
        if rand() > 0.2
            color = 'g';  % ✅ Success (Green)
        else
            color = 'r';  % ❌ Failure (Red)
        end

        % ➡️ Draw Arrow for Packet Transmission
        arrow = quiver(nodePos(1), nodePos(2), ...
                       satPos(1) - nodePos(1), satPos(2) - nodePos(2), ...
                       0, color, 'LineWidth', 2, 'MaxHeadSize', 0.5);
    end

    % Pause for Animation Effect
    pause(0.3);
    
    % Remove Arrows Before Next Frame
    delete(arrow);
end

hold off;
