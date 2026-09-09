function name = vd_car()
% VD_CAR  Selects the active car config: 'TR27' (developing) or 'TR26'
% (last year's car, for validation/calibration against its test data).
% Edit this one line to switch. Each car has its own tire artifact
% (tire_coeffs_<car>.mat); run build_tire_coeffs after switching if that
% car's artifact doesn't exist yet - vd_selftest will tell you.
name = 'TR27';
end
