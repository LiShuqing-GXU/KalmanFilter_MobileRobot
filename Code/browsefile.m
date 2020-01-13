%% Ahmed Elsaharti - 2019

function [data] = browsefile(name)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
if nargin>0
[filename, pathname] = uigetfile('*.csv', ['Select ',name,' file']);
else
[filename, pathname] = uigetfile('*.csv', 'Select a file');  
end
if isequal(filename,0)
   error('No data file selected')
else
   disp(['File selected ', fullfile(pathname, filename)])
end
data = load(fullfile(pathname, filename));

end

