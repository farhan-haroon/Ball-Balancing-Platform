function rosmsgOut = UInt16(slBusIn, rosmsgOut)
%#codegen
%   Copyright 2021 The MathWorks, Inc.
    rosmsgOut.Data = uint16(slBusIn.Data);
end
