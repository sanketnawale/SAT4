function [SuccessRate, OverallSuccessRate] = LoRa_Random_Access(Nodes, Pkct_Per_Hour, Simulation_T, MonteCarlo, Time_Vector, Visibility)

    %% 🎯 Constants
    Pkt_Rate = Pkct_Per_Hour / 3600;  % Packets per second
    ToA = 0.5;  % Time-on-air for a packet (seconds)
    
    SuccessRate = zeros(Nodes, length(Time_Vector));  
    OverallSuccessRate = zeros(Nodes, 1);

    %% 📡 Monte Carlo Simulation
    for t = 2:length(Time_Vector)
        dt = Time_Vector(t) - Time_Vector(t-1);  

        % 🛰️ Create satellite packet reception log
        Sat_Packet_Log = zeros(size(Visibility, 2), 1); 

        for n = 1:Nodes
            Visible_Sats = find(Visibility(n, :, t));  % Get visible satellites
            
            if ~isempty(Visible_Sats)
                num_packets = MonteCarlo;  % Packets to transmit  
                Packet_Times = rand(1, num_packets) * Simulation_T;  

                for p = 1:num_packets
                    % Pick a random satellite from visible ones
                    target_sat = Visible_Sats(randi(length(Visible_Sats))); 
                    
                    % Check for collision (another packet within ToA)
                    if Sat_Packet_Log(target_sat) > 0
                        Sat_Packet_Log(target_sat) = Sat_Packet_Log(target_sat) + 1;
                    else
                        Sat_Packet_Log(target_sat) = 1;
                    end
                end

                % Calculate success rate (only 1 packet per sat is successful)
                Successful_Packets = sum(Sat_Packet_Log == 1);
                Collisions = sum(Sat_Packet_Log > 1);

                % Debugging information
                fprintf('⏳ Time %.2f sec: Node %d -> Packets: %d, Success: %d, Collisions: %d\n', ...
                        Time_Vector(t), n, MonteCarlo, Successful_Packets, Collisions);
                
                SuccessRate(n, t) = max(0, Successful_Packets / MonteCarlo);
            end
        end
    end

    %% 🏆 Compute Overall Success Rate
    OverallSuccessRate = sum(SuccessRate, 2) ./ sum(SuccessRate > 0, 2);

end
