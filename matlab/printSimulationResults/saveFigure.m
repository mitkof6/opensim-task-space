function [ output_args ] = saveFigure(handle, fileName, type, siz, orientation)
% saveFigure 
%   Saves figure specified by `handle` as `fileName` in fullscreen
%   as to get around the stupid behavior.

    set(handle, 'Units', 'centimeters');
    set(handle,'PaperType', 'a4');
    set(handle,'PaperSize', siz);
    set(gcf, 'Position', get(0,'Screensize'));  
    set(handle, 'PaperPosition', [0 0 siz(1) siz(2)]);
    set(handle, 'PaperOrientation', orientation);
    print(fileName, type);
end

