function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

baseindex = base_index(map,stateSpace)
ik=0;
P = zeros(K,K,5);
for i = 1:K
    current = [stateSpace(i,1),stateSpace(i,2),stateSpace(i,3)];
    for j=1:K
        for k = [NORTH, SOUTH, EAST, WEST, HOVER]
            if(check_validity(i,j,stateSpace,k))
                ik=ik+1;
                next = [stateSpace(j,1),stateSpace(j,2),stateSpace(j,3)];
                P_SHOT = shotProbability(next,map);
                if(i==TERMINAL_STATE_INDEX)
                    P(i,j,k)=0;
                else
                    P(i,j,k) = P(i,j,k)+ (1-P_WIND)*(1-P_SHOT);
                    if(i==1 && j==1)
                        P(i,j,k)
                        P_SHOT
                        k
                    end
                    P(i,baseindex,k) = P(i,baseindex,k)+(1-P_WIND)*(P_SHOT);
                
                    for wind = [NORTH, SOUTH, EAST, WEST]
                        [wind_new_pos,crash_status] = crash(j,wind,stateSpace);
                        if(crash_status)
                            P(i,baseindex,k) = P(i,baseindex,k)+(P_WIND/4);
                        else
                            next = [stateSpace(wind_new_pos,1),stateSpace(wind_new_pos,2),stateSpace(wind_new_pos,3)];
                            P_SHOT = shotProbability(next,map);
                            P(i,wind_new_pos,k) = P(i,wind_new_pos,k) + (1-P_SHOT)*(P_WIND/4);
                            P(i,baseindex,k) = P(i,baseindex,k) + (P_SHOT)*(P_WIND/4);
                        end
                    end
                end
            end
        end
    end
end
ik
pickup = pick_up(map,stateSpace);
P2 = P;

for i = 1:K
    for j=1:K
        for k = [NORTH, SOUTH, EAST, WEST, HOVER]
            if(j==pickup)
                P2(i,pickup+1,k) = P2(i,pickup+1,k) + 1*P(i,pickup,k);
                P2(i,pickup,k) = 0;
            end
        end
    end
end

P=P2;

end


function [state,crash] = crash(pos,direction,stateSpace)
global NORTH SOUTH EAST WEST HOVER
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
    
    for i = 1:size(stateSpace,1)
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



function pickup = pick_up(map,stateSpace)
global PICK_UP
    [m,n] = size(map);
    vals = [];
    for i = 1:m
        for j=1:n
            if(map(i,j)==PICK_UP)
                vals = [vals;i,j];
            end
        end
    end
    if(size(vals)==0)
        error('not exist')
    elseif(size(vals)>1)
        error('multiple')
    else
        for i = 1:size(stateSpace,1)
    %         s = [stateSpace(i,1),stateSpace(i,2),stateSpace(i,3),i]
            if(stateSpace(i,1)==vals(1) && stateSpace(i,2)==vals(2) && stateSpace(i,3)==1)
                pickup = i;
                return
            end
        end
    end
end


function baseindex = base_index(map,stateSpace)
global BASE
    [m,n] = size(map);
    vals = [];
    for i = 1:m
        for j=1:n
            if(map(i,j)==BASE)
                vals = [vals;i,j];
            end
        end
    end
    if(size(vals)==0)
        error('not exist')
    elseif(size(vals)>1)
        error('multiple')
    else
        for i = 1:size(stateSpace,1)
    %         s = [stateSpace(i,1),stateSpace(i,2),stateSpace(i,3),i]
            if(stateSpace(i,1)==vals(1) && stateSpace(i,2)==vals(2) && stateSpace(i,3)==0)
                baseindex = i;
                return
            end
        end
    end
end

function check_validity = check_validity(i,j,stateSpace,dir)
    global NORTH SOUTH EAST WEST HOVER
    current = [stateSpace(i,1),stateSpace(i,2),stateSpace(i,3)];
    next = [stateSpace(j,1),stateSpace(j,2),stateSpace(j,3)];
    dif_x = next(1) - current(1);
    dif_y = next(2) - current(2);
    packMove = next(3) - current(3);
    if ( packMove ~= 0 ) % should not pick package yet
        check_validity = false;
    elseif(dif_x==1 && dif_y==0 && dir == EAST) 
        check_validity=true;
    elseif(dif_x==-1 && dif_y==0 && dir == WEST) 
        check_validity=true;
    elseif(dif_x==0 && dif_y==1 && dir == NORTH) 
        check_validity=true;
    elseif(dif_x==0 && dif_y==-1 && dir == SOUTH) 
        check_validity=true;
    elseif(dif_x==0 && dif_y==0 && dir == HOVER) 
        check_validity=true;
    else
        check_validity=false;
    end
end



function shotProbability = shotProbability(pos,map,stateSpace)
global SHOOTER R GAMMA
[m,n] = size(map);
shooter = [];
not_shot_prod = 1;
for i =1:m
    for j=1:n
        if(map(i,j)==SHOOTER)
            d = abs(i-pos(1))+abs(j-pos(2));
            if(d<R && d>0)
                not_shot_prob = 1-(GAMMA/(d+1));
                not_shot_prod = not_shot_prod*not_shot_prob;
            end
        end
    end
end

shotProbability = 1 - not_shot_prod;
end
