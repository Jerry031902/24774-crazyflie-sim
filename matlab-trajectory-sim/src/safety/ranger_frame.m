function frame = ranger_frame(ranges, sample_time, delivery_time, pos, psi, sequence)
%RANGER_FRAME  Sensor transport metadata; positive Inf is a valid no-return.
% status: 0 not received, 1 hit, 2 no return, 3 invalid/fault.
frame = struct('ranges', ranges(:), 'sample_time', sample_time, ...
    'delivery_time', delivery_time, 'pos', pos(:), 'psi', psi, 'seq', sequence);
frame.status = uint8(3 * ones(4, 1));
frame.status(isfinite(frame.ranges) & frame.ranges >= 0) = 1;
frame.status(frame.ranges == Inf) = 2;
if sequence == 0
    frame.status(:) = 0;
end
end
