function newDoub = uint8todouble(sign,varargin)
% Takes a set number of bytes (2,4, or 8) , up to a uint64 value, as 
% needed to turn data from uint8 to double

% Error out if only 1 input argument since need at least 2 to run
if nargin == 1
    error('Need to have more than 1 bit-based input for function to run')
end

% Setting up temporary holding place for values once they've been converted
% to uint16's and bit shifted according to the position
convNums = zeros(nargin,1);
% Converts values to uint64 storage then shifts according to position
for k = 1:nargin
    convNums(k) = bitshift(uint64(varargin{k}),(8*(k-1)));
end

% Does big bitor function over all bit inputs
bitMess = uint64(0);
for j = 1:nargin
    bitMess = bitor(bitMess,convNums(j));
end

% Check how to convert uint64 to int32, not specifically, just if it
% happens to be a 4 byte number and it IS signed. Can it be done just
% through int32(X) or does if have to be adjusted first using some
% functions and nargin?

% sign==1 indicates variable is pos, sign==0 indicates neg
if sign == 1
    newDoub = double(bitMess);
elseif sign == 0
    newDoub = -double(bitMess);
end

end

