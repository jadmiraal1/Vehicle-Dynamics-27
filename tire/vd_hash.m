function h = vd_hash(files)
% VD_HASH  Content hash over files (md5, sorted by basename); Python-reproducible.
% Stamps tire_coeffs.mat; vd_selftest compares to detect a stale artifact.

names = cell(size(files));
for i = 1:numel(files)
    [~, n, e] = fileparts(files{i});
    names{i} = [n e];
end
[names, idx] = sort(names);
files = files(idx);

s = '';
for i = 1:numel(files)
    s = [s names{i} ':' md5_file(files{i}) sprintf('\n')]; %#ok<AGROW>
end
h = md5_bytes(unicode2native(s, 'UTF-8'));
end

function h = md5_file(f)
fid = fopen(f, 'rb');
if fid < 0
    error('vd_hash:missing', 'cannot open %s', f);
end
b = fread(fid, Inf, '*uint8');
fclose(fid);
h = md5_bytes(b);
end

function h = md5_bytes(b)
md = java.security.MessageDigest.getInstance('MD5');
md.update(uint8(b(:)));
d = typecast(md.digest(), 'uint8');
h = lower(sprintf('%02x', d));
end
