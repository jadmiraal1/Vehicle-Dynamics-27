function h = vd_hash(files)
% VD_HASH  Reproducible content hash over a list of files.
%
%   h = vd_hash({'pacejka_fit.m', 'TTC_Data/LC0_1.mat', ...})
%
% Definition (kept deliberately simple so it can be reproduced in Python,
% which is how the artifact was bootstrapped and how it is cross-checked):
%
%   digest_input = concat over files, SORTED BY BASENAME, of
%                      sprintf('%s:%s\n', basename, md5_hex(file_bytes))
%   h            = md5_hex( UTF-8 bytes of digest_input )
%
% Python equivalent:
%   parts = []
%   for f in sorted(files, key=os.path.basename):
%       parts.append('%s:%s\n' % (os.path.basename(f),
%                    hashlib.md5(open(f,'rb').read()).hexdigest()))
%   hashlib.md5(''.join(parts).encode('utf-8')).hexdigest()
%
% Used by build_tire_coeffs (to stamp the artifact) and vd_selftest (to
% detect a STALE artifact). Sorting by basename makes the hash independent
% of directory listing order and of absolute paths.

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
