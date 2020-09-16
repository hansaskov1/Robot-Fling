clc;
clear;
close all;



pr1i = [382 ; -235 ; -559];
pr2i = [196.8 ; -161.5 ; -559.5];
pr3i = [-439.6 ; -421.6 ; 558.7];
pr4i = [-405.6 ; -523.5 ; -266.1];

pw1i = [0 ; 0 ; 0];
pw2i = [150 ; 150 ; 0];
pw3i = [150 ; 850 ; 0];
pw4i = [50 ; 850 ; 300];

pr = [pr1i pr2i pr3i pr4i]
pw = [pw1i pw2i pw3i pw4i]

[R T] = calibrate(pr, pw)







