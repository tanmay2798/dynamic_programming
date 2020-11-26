function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    G = zeros(K,5);
    
    for i=1:K
        for next_dir = [NORTH, SOUTH, EAST, WEST, HOVER]
            if(i==TERMINAL_STATE_INDEX)
                G(i,next_dir)=0;
            else
                [next_pos,crash_status] = crash(i,next_dir,stateSpace);
                if(crash_status)
                    G(i,next_dir) = inf;
                else
                    P_CRASHED = shotProbability(next_pos,map,stateSpace);
                    G(i,next_dir) = G(i,next_dir) + (1 - P_WIND)*(1 - P_CRASHED);
                    G(i,next_dir) = G(i,next_dir) + (1 - P_WIND)*(P_CRASHED)*Nc;
                    
                    for wind = [NORTH, SOUTH, EAST, WEST]
                        [final_pos,crash_status] = crash(next_pos,wind,stateSpace);
                        if(crash_status)
                            G(i,next_dir) = G(i,next_dir) + (P_WIND/4)*Nc;
                        else
                            P_CRASHED = shotProbability(final_pos,map,stateSpace);
                            G(i,next_dir) = G(i,next_dir) + (P_WIND/4)*(1 - P_CRASHED);
                            G(i,next_dir) = G(i,next_dir) + (P_WIND/4)*(P_CRASHED)*Nc;                          
                        end
                        
                    end
                    
                end
            end
        end
    end

end


function [state,crash] = crash(pos,direction,stateSpace)
global NORTH SOUTH EAST WEST HOVER K
    temp = stateSpace;
    if(direction==NORTH)
        temp(pos,2)=temp(pos,2)+1;
    elseif(direction==SOUTH)
        temp(pos,2)=temp(pos,2)-1;
    elseif(direction==EAST)
        temp(pos,1)=temp(pos,1)+1;
    elseif(direction==WEST)
        temp(pos,1)=temp(pos,1)-1;
    end
    
    for i = 1:K
%         s = [stateSpace(i,1),stateSpace(i,2),stateSpace(i,3),i]
        if(stateSpace(i,1)==temp(pos,1) && stateSpace(i,2)==temp(pos,2) && stateSpace(i,3)==temp(pos,3))
            crash=false;
            state = i;
            return
        end
    end
    crash=true;
    state = i;
    return
end

function shotProbability = shotProbability(pos,map,stateSpace)
global SHOOTER R GAMMA
[m,n] = size(map);
not_shot_prod = 1;
for i =1:m
    for j=1:n
        if(map(i,j)==SHOOTER)
            d = abs(i-stateSpace(pos,1))+abs(j-stateSpace(pos,2));
            if(d<=R && d>=0)
                not_shot_prob = 1-(GAMMA/(d+1));
                not_shot_prod = not_shot_prod*not_shot_prob;
            end
        end
    end
end

shotProbability = 1 - not_shot_prod;
end
