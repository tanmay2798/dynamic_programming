function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

global DROP_OFF
[m,n] = size(map);
vals = [];
for i = 1:m
    for j=1:n
        if(map(i,j)==DROP_OFF)
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
            stateIndex = i;
            return
        end
    end
end
                  
end
