% clc
% Specify the location and name of the map to load
% addpath('../Shared Code/Mapping/Maps/Ovals')
% addpath('../Shared\development\Mapping\Maps\Bonneville Salt Flats Maps\PPIHC_folded_pieces')
addpath('../Map Data/')
% addpath('../Shared/development\Mapping\Maps\Bonneville Salt Flats Maps')
% addpath('../Shared/development/Mapping/Maps/Pikes Peak Maps')
%map_name = 'test_map4';
% map_name = 'driftkeeping';
map_name = 'driftkeeping_4';
% map_name = 'radius_20';
%map_name = 'salt_flats_oval_fat_speedway';
% map_name = 'PPIHC_folded_pieces_1';
% map_name = 'PPIHC_map_dirt';

% Set scalar parameters
errLim_prm = 0.005;
iterLim_prm = 30;
alpha_prm = 0.9;
numTerms_prm = 10;
useZ_prm = 0;
beta_prm = 1;

% Load the map as parameters
map_prm = importdata([map_name '_map.csv'], ',', 1);
map_prm = map_prm.data;
curve_prm = importdata([map_name '_curve.csv'], ',', 1);
curve_prm = curve_prm.data;
seg_prm = importdata([map_name '_seg.csv'], ',', 1);
seg_prm = seg_prm.data;
discrete_prm = importdata([map_name '_discrete.csv'], ',', 1);
discrete_prm = discrete_prm.data;

disp('Map Updated');

%% save map data
mapData.map_name=map_name;
mapData.seg_prm=seg_prm;
