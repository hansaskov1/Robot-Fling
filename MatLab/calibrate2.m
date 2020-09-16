function [rHat, tHat] = calibrate(pr, pw)

Cpr = (1/size(pr,2)*sum(pr.')).'
Cpw = (1/size(pw,2)*sum(pw.')).'

qr = pr - Cpr(1)
qw = pw - Cpw(1)

H = qr * qw.'


[U,sigma,V] = svd(H)

rHat = V * U.';

tHat = Cpw - rHat * Cpr;
end


