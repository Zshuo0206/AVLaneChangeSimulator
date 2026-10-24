function validateRoad(road)
%VALIDATEROAD Check the Phase 0 road struct without modifying its fields.
%   avlc.model.validateRoad(ROAD) requires numLanes=2, positive laneWidth,
%   originalLaneCenterY=0, targetLaneCenterY=laneWidth, and a two-element
%   increasing displayXRange vector. All numeric data must be real and
%   finite, with scalar values except the range. Distances are in m; +y
%   points toward the target lane. Target-center comparison uses relative
%   tolerance 1e-10 times laneWidth. Extra fields are allowed. Returns no
%   output on success; invalid data raises AVLC:InvalidRoad.

fields = {'numLanes', 'laneWidth', 'originalLaneCenterY', ...
    'targetLaneCenterY', 'displayXRange'};
if ~isstruct(road) || ~isscalar(road) || ~all(isfield(road, fields))
    error('AVLC:InvalidRoad', 'Road must be a scalar struct containing all five required fields.');
end
for index = 1:4
    value = road.(fields{index});
    if ~(isnumeric(value) && isreal(value) && isscalar(value) && isfinite(value))
        error('AVLC:InvalidRoad', 'Road.%s must be a finite real numeric scalar.', fields{index});
    end
end
if road.numLanes ~= 2 || road.laneWidth <= 0 || road.originalLaneCenterY ~= 0
    error('AVLC:InvalidRoad', ...
        'Phase 0 requires numLanes=2, laneWidth>0, and originalLaneCenterY=0.');
end
tolerance = 1e-10 * double(road.laneWidth);
if abs(double(road.targetLaneCenterY) - double(road.laneWidth)) > tolerance
    error('AVLC:InvalidRoad', 'Road.targetLaneCenterY must equal laneWidth within relative tolerance 1e-10.');
end
range = road.displayXRange;
if ~(isnumeric(range) && isreal(range) && isvector(range) && ...
        numel(range) == 2 && all(isfinite(range(:))) && range(1) < range(2))
    error('AVLC:InvalidRoad', 'Road.displayXRange must be a finite real numeric [xMin xMax] vector with xMin<xMax.');
end
end
