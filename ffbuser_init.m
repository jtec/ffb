% Setup matlab path:
% cd('D:\j.bolting\Dropbox\UAS\phd\UAVsimulator');
% Adds the necessary files and folders to the matlab search path.
pathOfThisScript_ffbuser = mfilename('fullpath');
[pathstr, name, ext] = fileparts(pathOfThisScript_ffbuser);
startpath = pathstr;
% First add everything in the current folder.
addpath(genpath(pathstr));
depth = 3;
longcall('uavsimblockset_addtomatlabpath.m', pwd, depth);
longcall('trajectory_addtomatlabpath.m', pwd, depth);
longcall('svm_addtomatlabpath.m', pwd, depth);
longcall('ffb_addtomatlabpath.m', pwd, depth);
% Initialize ffb library:
ffb_setup;
disp([mfilename '>> Set up libraries path.'])
