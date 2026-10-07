function h = vd_hash(files)
% VD_HASH  Content hash over a list of files (md5, sorted by file name).
%   h = vd_hash(files)
% Stamps tire_coeffs_<CAR>.mat; vd_selftest recomputes it to detect a stale
% artifact. Line endings are normalised (CR bytes removed) in .m files, so a
% Windows checkout (CRLF) and a Linux/macOS checkout (LF) hash the same.

names = cell(size(files));
for i = 1:numel(files)
    [~, n, e] = fileparts(files{i});
    names{i} = [n e];
end
[names, idx] = sort(names);
files = files(idx);

s = '';
for i = 1:numel(files)
    [~, ~, ext] = fileparts(files{i});
    s = [s names{i} ':' md5_file(files{i}, strcmpi(ext, '.m')) sprintf('\n')]; %#ok<AGROW>
end
h = md5_bytes(unicode2native(s, 'UTF-8'));
end

function h = md5_file(f, is_text)
fid = fopen(f, 'rb');
if fid < 0
    error('vd_hash:missing', 'cannot open %s', f);
end
b = fread(fid, Inf, '*uint8');
fclose(fid);
if is_text, b = b(b ~= 13); end      % drop CR: CRLF and LF hash alike
h = md5_bytes(b);
end

function h = md5_bytes(b)
md = java.security.MessageDigest.getInstance('MD5');
md.update(uint8(b(:)));
d = typecast(md.digest(), 'uint8');
h = lower(sprintf('%02x', d));
end
