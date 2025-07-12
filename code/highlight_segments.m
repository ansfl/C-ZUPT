function highlight_segments(tt, idx_vec, color, alpha)
% highlight_segments - Highlight nonzero segments in idx_vec
%
% Usage:
%   highlight_segments(idx_vec)
%   highlight_segments(idx_vec, color, alpha, ax)
%
% Inputs:
%   idx_vec : index vector (numeric) or logical mask
%   color   : RGB triplet, e.g. [0.9 0.2 0.2] (optional)
%   alpha   : transparency, e.g. 0.4 (optional)
%   ax      : axes handle (optional)

    if nargin < 2 || isempty(color)
        color = [0.9 0.2 0.2]; % default color (pale RED)
    end
    if nargin < 3 || isempty(alpha)
        alpha = 0.4;           % default transparency
    end
    % if nargin < 4 || isempty(ax)
    ax = gca;              % current axes by default
    % end

    % Make mask if needed
    mask = idx_vec > 0;

    % Find edges
    edges = diff([0 mask 0]); % pad to detect edges at ends
    start_idx = find(edges == 1);
    end_idx   = find(edges == -1) - 1;

    % Get y limits of the specified axes
    yl = ylim(ax);

    % Draw patches
    hold(ax, 'on')
    for k = 1:length(start_idx)
        x1 = tt(start_idx(k));
        x2 = tt(end_idx(k));
        patch(ax, [x1 x2 x2 x1], [yl(1) yl(1) yl(2) yl(2)], ...
            color, 'EdgeColor', 'none', 'FaceAlpha', alpha);
    end
end
