function eta = motor_eff(rpm, T_Nm)
% MOTOR_EFF  EMRAX 228 motor efficiency from the datasheet map (digitized fit).
%   eta = motor_eff(rpm, T_Nm)   vectorized; motor only (chain/inverter separate).
% Physics-based loss fit calibrated to the published efficiency-map contours and
% cross-validated against the free-run loss curve (references/emrax_228 pdf p.2-3):
%   P_loss = iron*rpm^2 + Rdc*I^2 + ac*(rpm*I)^2 + const,  I from the saturating
%   torque-current curve. Fit error <= ~1.5% (worst at low-rpm map pinch).

C_IRON = 1.011e-4;      % [W/rpm^2]  iron + friction (matches free-run curve)
C_DC   = 0.0164;        % [W/A^2]    DC copper
C_AC   = 1.680e-9;      % [W/(rpm*A)^2]  AC copper / stray, grows with speed
C_0    = 115;           % [W]        constant floor

T_Nm = max(T_Nm, 0);
I = T_Nm ./ 0.75;                                   % linear region [A]
sat = T_Nm > 150;
I(sat) = 200 + (T_Nm(sat) - 150) ./ 0.47;           % magnetic saturation

P_mech = T_Nm .* rpm * (2*pi/60);
P_loss = C_IRON.*rpm.^2 + C_DC.*I.^2 + C_AC.*(rpm.*I).^2 + C_0;
eta = P_mech ./ max(P_mech + P_loss, 1e-6);
eta = min(max(eta, 0.50), 0.97);                    % clamp outside fitted region
end
