function rosmsgOut = Int16(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Data = int16(slBusIn.Data);
end
