%	This function calculates q4 for the do bot based on q2 and q3
%   Author: Chamath E 
%   content from UTS canvas

function [q4] = constrain_joint4(q2, q3)

q4 = pi - q2 - q3;

end

