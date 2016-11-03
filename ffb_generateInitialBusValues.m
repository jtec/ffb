%% ------------------------------------------------------------------
%  You can modify the values of the fields in VortexStates_MATLABStruct
%  and evaluate this cell to create/update this structure
%  in the MATLAB base workspace.
% -------------------------------------------------------------------

%% ------------------------------------------------------------------
%  You can modify the values of the fields in VortexStates_MATLABStruct
%  and evaluate this cell to create/update this structure
%  in the MATLAB base workspace.
% -------------------------------------------------------------------


VortexStates_MATLABStruct = struct;
VortexStates_MATLABStruct.pRear_b_m = ...
[0; 0; 0];
VortexStates_MATLABStruct.posFuselageBottom_b_m = ...
[0; 0; 0];
VortexStates_MATLABStruct.timeOfLastUpdate = 0;
VortexStates_MATLABStruct.p_NED_m = ...
[0; 0; 0];
VortexStates_MATLABStruct.v_NED_mps = ...
[0; 0; 0];
VortexStates_MATLABStruct.alpha_rad = 0;
VortexStates_MATLABStruct.beta_rad = 0;
VortexStates_MATLABStruct.CL = 0;
VortexStates_MATLABStruct.qAttitude = ...
[0; 0; 0; 0];
VortexStates_MATLABStruct.b_m = 0;
VortexStates_MATLABStruct.cbar_m = 0;
VortexStates_MATLABStruct.sweep_rad = 0;
VortexStates_MATLABStruct.dihedral_rad = 0;
VortexStates_MATLABStruct.longiRefLength_m = 0;
VortexStates_MATLABStruct.pNose_b_m = ...
[0; 0; 0];
VortexStates_MATLABStruct.verticalRefLength_m = 0;
VortexStates_MATLABStruct.posFinTip_b_m = ...
[0; 0; 0];

VortexStates_MATLABStruct_array = [];
for k=1:10
    VortexStates_MATLABStruct_array = [VortexStates_MATLABStruct_array; VortexStates_MATLABStruct];
end