function [eta, P_loss, I] = motor_eff(rpm, T_Nm)
% MOTOR_EFF  EMRAX 228 efficiency and losses from a fit to the datasheet map.
%   eta = motor_eff(rpm, T_Nm)                 vectorized; motor only
%   [eta, P_loss, I] = motor_eff(rpm, T_Nm)
%
%   eta     motor efficiency, clamped to [0.50, 0.97] (the fitted region)
%   P_loss  motor heat [W], NOT clamped. At zero torque it is the spin loss
%           (iron + friction), real heat that the coolant carries even while
%           the car coasts or brakes.
%   I       phase current [Arms]
%
% Loss model: P_loss = C_IRON*rpm^2 + C_DC*I^2 + C_AC*(rpm*I)^2 + C_0, with I
% from the torque-current curve (0.75 Nm/Arms for the MV winding, saturating
% above 150 Nm). Coefficients were fitted to the efficiency-map contours and
% the free-run loss curve in the EMRAX 228 technical data (rev 4.5, p.2-3),
% fit error about 1.5%. The published map is for the HV combined-cooled
% motor; the datasheet states the HV/MV/LV windings differ only in voltage
% and current. Chain and inverter losses are applied elsewhere.

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
% The clamp protects the energy path (E = work/eta) at the fit's edges, so
% eta and P_loss agree only inside the clamp band.
end
