function setup_paths()
%SETUP_PATHS  Add the project's src folders to the MATLAB path.
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root, 'src')));
end
