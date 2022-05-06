function [qmatrix] = calc_trap_qmatrix(q0, q1, steps)
    s = lspb(0, 1, steps);
    qmatrix = nan(steps, 5);
    for i=1:steps
        tmp = (1 - s(i)) * q0 + s(i) * q1;
        tmp(4) = constrain_joint4(tmp(2), tmp(3));
        qmatrix(i, :) = [tmp(1) tmp(2) tmp(3) tmp(4) tmp(5)];
    end
