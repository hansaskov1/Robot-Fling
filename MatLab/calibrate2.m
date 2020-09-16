
function [rHat, tHat] = calibrate(pr, pw)

Cpr = (1/size(pr,2)*sum(pr.')).'
Cpw = (1/size(pw,2)*sum(pw.')).'

% size(pw,2)

qr = pr - Cpr
qw = pw - Cpw

H = qr * qw.'


[U,sigma,V] = svd(H)

rHat = V * U.';

tHat = Cpw - rHat * Cpr;
end

