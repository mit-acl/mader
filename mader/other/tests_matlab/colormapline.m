function [handle_all] = colormapline(x,y,z,colmap)
% H = colormapline(X,Y,Z,C) Plot a line using a specified colormap
%   If Z is given, a 3D plot with coordinates X, Y, Z is created.
%   If Z is not given, a 2D plot of Y over X if created.
%   If Y and Z are not given, a 2D plot of X over 1:length(X) is created.
%    The plotted line uses all colors in a given colormap in ascending
%    order. Other than with other functions in File Exchange, the color
%    does NOT depend on any coordinate.
%    Technically, length(colormap) lines are plotted, each with a different
%    color from the colormap in ascending order.
%   H is a handle to the complete colored line. It can be used to set
%    parameters such as linewidth and linestyle.
%   C is optional. If it is specified, it is used as a colormap. To use it
%   with a 2D plot, use Z=[] and Y=[] if required;
%
% Examples:
%% 2D plot
%  t=0:0.1:10*pi;
%  h=colormapline(t.*sin(t),t.*cos(t),[],jet(128));
%  set(h,'linewidth',3,'linestyle','--')
%
%% 3D plot
%  t=-20*pi:0.1:20*pi;
%  h=colormapline(t.*sin(t),t.*cos(t),sin(t),jet(128));
%  set(h,'linewidth',3)
%
%% Minimal example (2D plot with one input)
%  h=colormapline(1:1000);
%
%% 2D plot using MarkerFaceColor
% t=0:0.1:10*pi;
% h=colormapline3(t.*sin(t),t.*cos(t),[],jet(128));
% set(h,'Marker','o','LineStyle','none')

% Version 2.1, 2016-12-14
%   Change from version 2 (2013-01-29):
%    - Line property 'MarkerFaceColor' is now also set using the colormap.
%      Behaviour of previous versions can be restored by running the
%      following line after the call:
%        set(H,'MarkerFaceColor','none')
%      This change was triggered by a question at
%      https://de.mathworks.com/matlabcentral/fileexchange/39972-colormapline-color-changing-2d-or-3d-line
%
%  Version 2, 2013-01-29
%   Changes from version 1 (2013-01-23):
%    - Returns proper error messages when called
%      with input vectors of length < 3 or with different lengths.
%    - Exits with the same hold state it was started with.
%    - Works with input vectors which are shorter than the colormap.
%      For this, the colormap is compressed to the input vector length.
%    - Works with colormaps of length 1 or 2
%    - Also works with a single input, like plot(). (-> "minimal example")
%   Thanks go to Rob Campbell for his comments on some of these points.
%
%  Author: Matthias Hunstig, University of Paderborn, Germany, 2013-01-23
%          matthias.hunstig ä upb.de

holdfig=ishold; % Get hold state

% Check for length
if nargin<3 || isempty(z)
        if nargin<2 || isempty(y)  % only x given
            if length(x)<3;
                error('Input vector length must be > 2. Use plot() or plot3() for shorter vectors.')
            end
        else  % x and y given
            if min([length(x),length(y)])<3;
                error('Input vector length must be > 2. Use plot() or plot3() for shorter vectors.')
            elseif length(x)~=length(y)
                error('Input vectors must have the same length.')
            end
        end
else  % x, y, and z given
        if min([length(x),length(y),length(z)])<3;
            error('Input vector length must be > 2. Use plot() or plot3() for shorter vectors.')
        elseif not(length(x)==length(y) && length(x)==length(z))
            error('Input vectors must have the same length.')
        end
end

% Define colormap
if nargin<4
    cm=colormap;
else
    cm=colmap;
end
n=size(cm,1);

% Adopt colormap length if necessary
L=length(x);
if L-1<n; % Colormap is longer than max. number of parts possible with the input vectors
    cm=cm(round(1:(n/(L-1)):n),:);
    n=size(cm,1);
end

% Plot parts
for p=1:n
    if p<n
        part=(1+round((p-1)/n*L)):(round(p/n*L)+1);
    else
        part=(1+round((p-1)/n*L)):(round(p/n*L));
    end
    
    if nargin<3 || isempty(z)
        if nargin<2 || isempty(y)  % only x given
            hand(p)=plot(x(part),x(part),'color',cm(p,:),'MarkerFaceColor',cm(p,:));
        else    % x and y given
                hand(p)=plot(x(part),y(part),'color',cm(p,:),'MarkerFaceColor',cm(p,:));
        end
    else % x, y, and z given
            hand(p)=plot3(x(part),y(part),z(part),'color',cm(p,:),'MarkerFaceColor',cm(p,:));
    end
    hold on
end

if not(holdfig); hold off; end % Restore hold state

handle_all=hand(1:end); % Define handle
    
end

