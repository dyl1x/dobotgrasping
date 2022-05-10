% author: chamath edirisinhege
% brief: returns 4 points on the xy plane of the transform
% inputs: cent_tr - transformation/pose of the center point
%         s - gap between two corners of the sqaure
function P = getP(cent_tr,s)
s = s/2;
% points relative to origin (0,0,0)
trtopl = transl(-s,s,0);
trtopr = transl(s,s,0);
trbotl = transl(-s,-s,0);
trbotr = transl(s,-s,0);

% points relative to global

prg_tl = cent_tr * trtopl;
prg_tr = cent_tr * trtopr ;
prg_bl = cent_tr * trbotl;
prg_br = cent_tr * trbotr;

%

P = [prg_bl(1:3,4),prg_tl(1:3,4),prg_tr(1:3,4),prg_br(1:3,4)];
% if you want a plot
% figure
% plot_sphere(P, 0.05, 'b')
% hold on
% trplot(prg_tl, 'length', 0.5, 'color', 'g');
% trplot(prg_tr, 'length', 0.5, 'color', 'g');
% trplot(prg_bl, 'length', 0.5, 'color', 'g');
% trplot(prg_br, 'length', 0.5, 'color', 'g');
% 
% trplot(cent_tr, 'length', 1, 'color', 'r');

end