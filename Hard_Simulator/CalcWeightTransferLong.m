function [Wf, Wr] = CalcWeightTransferLong(Fx, veh)
%CALCWEIGHTTRANSFERLONG Calculates longitudinal weight transfer using
%steady-state assumptions

Wf = veh.Wf - veh.hcg*Fx;
Wr = veh.Wr + veh.hcg*Fx;

end

