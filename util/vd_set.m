function p = vd_set(p, varargin)
% VD_SET  Change one or more car INPUTS and re-derive everything downstream.
%   p2 = vd_set(p, 'm_car', 240)
%   p2 = vd_set(p, 'ClA', 4.0, 'CdA', 1.63, 'mass_dist_f', 0.47)
%   p2 = vd_set(p, struct('gear_ratio', 3.6))
%
% Use it for every "what if?" and in every sweep:
%
%     for chi = 0.38:0.02:0.52
%         q = vd_set(p, 'mass_dist_f', chi);     % derived values follow
%         ay(end+1) = ay_limit(q, 12);
%     end
%
% Assigning a field directly (q = p; q.mass_dist_f = chi) leaves a, b, the
% axle loads, Izz etc. describing the old car. Structs are copied by value,
% so p itself is never changed.
%
% Setting a DERIVED field is refused (vd_derive would overwrite it): to make
% the car heavier, set m_car, not m. Unknown field names are refused too.
%
% See also VD_DERIVE, VEHICLE_PARAMS.

DERIVED = {'m','mu_x_raw','mu_y','mu_x','a','b','Wf_static','Wr_static', ...
           'm_unsprung','m_unsprung_f','m_unsprung_r','m_sprung', ...
           'k_rot','Izz','P_max','v_max','Fz_design_lbf','N_PER_LBF'};

% Accept either name/value pairs or a single struct of overrides.
if numel(varargin) == 1 && isstruct(varargin{1})
    ov = varargin{1};
    names = fieldnames(ov);
    vals  = struct2cell(ov);
else
    assert(mod(numel(varargin),2) == 0, 'vd_set:pairs', ...
        'Arguments must be name/value pairs, or a single struct of overrides.');
    names = varargin(1:2:end);
    vals  = varargin(2:2:end);
end

for i = 1:numel(names)
    f = names{i};
    assert(ischar(f), 'vd_set:badName', 'Field name %d is not text.', i);

    if any(strcmp(f, DERIVED))
        error('vd_set:derivedField', ...
            ['''%s'' is DERIVED, not an input - vd_derive would overwrite it.\n' ...
             'Set the input it comes from instead (e.g. m_car, not m).'], f);
    end
    if ~isfield(p, f)
        error('vd_set:unknownField', ...
            ['''%s'' is not a field of the params struct. Check the spelling, or\n' ...
             'add it to cars/config_<CAR>.m if it is a new car property.'], f);
    end
    p.(f) = vals{i};
end

p = vd_derive(p);
end
