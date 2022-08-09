function newDoub = uint8todouble(signFlag,floatFlag,varargin)
% Takes a set number of bytes (2,4, or 8), up to a uint64 value, as
% needed to turn data from uint8 to double

% Setting up var with our num of vararg's since nargin counts all arg's in
n = length(varargin);
% Error out the function if not given the proper number of inputs
if n ~= 2 && n ~= 4 && n ~= 8
    error('Cannot accept this number of information inputs. This function requires 2, 4, or 8 bytes of information after declaring the sign.')
end

% Setting up temporary holding place for values once they've been converted
% to uint16's and bit shifted according to the position
convNums = uint64(zeros(length(varargin{1}),n));

% Converts values to uint64 storage then shifts according to position
for k = 1:n
    convNums(:,k) = bitshift(uint64(varargin{k}),(8*(k-1)));
end

% Does big bitor function over all bit inputs by bitor'ing each convNums
% column of data against the single bitMess column which originally is
% setup as a uint64 column of zeros
bitMess = uint64(zeros(length(varargin{1}),1));
for j = 1:n
    bitMess(:,1) = bitor(bitMess(:,1),convNums(:,j));
end

% Now start converting the bitMess into a double requires knowing if its a
% signed or unsigned integer first

% sign==1 indicates variable is signed, sign==0 indicates unsigned
if floatFlag == 1
    if n == 4
        single(bitMess);
    elseif n == 8
        double(bitMess);
    else
        error('Float flag indicates a floating point number but an improper number of bytes were supplied');
    end
else
    if signFlag == 1
        % Then need to also check the size of the actual message in bytes being sent
        % since the last bit is the signed bit. That means checking the size of
        % the message and reducing it's package size to only be as big as the
        % original input message (Eg: If original varargin was 2 bytes in, then
        % reduce to uint16 or int16 message; if varargin was 4 bytes, then
        % uint32 or int32, etc.
        if n == 2
            signCheckedMess = int16(bitMess);
        elseif n == 4
            signCheckedMess = int32(bitMess);
        elseif n == 8
            signCheckedMess = int64(bitMess);
        end
    elseif signFlag == 0
        if n == 2
            signCheckedMess = uint16(bitMess);
        elseif n == 4
            signCheckedMess = int32(bitMess);
        elseif n == 8
            signCheckedMess = int64(bitMess);
        end
    else
        error('Unrecognized sign input. Give first input as either 1 to indicate signed value or 0 to indicate unsigned value')
    end
end

newDoub = double(signCheckedMess);

end

