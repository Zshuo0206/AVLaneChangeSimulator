function road = createRoad(laneWidth, displayXRange)
%CREATEROAD Create the Phase 0 two-straight-parallel-lane road struct.
%   ROAD = avlc.model.createRoad(LANEWIDTH, DISPLAYXRANGE) accepts a positive
%   finite real numeric scalar width in m and a finite real numeric vector
%   [xMin xMax] in m with xMin < xMax. Stores numeric fields as doubles and
%   the display range as a 1-by-2 row. numLanes is 2, originalLaneCenterY is
%   0, and targetLaneCenterY is laneWidth, so +y points to the target lane.
%   The range is display metadata only. Invalid input raises AVLC:InvalidRoad.

road = struct();
road.numLanes = 2;
road.laneWidth = laneWidth;
road.originalLaneCenterY = 0;
road.targetLaneCenterY = laneWidth;
road.displayXRange = displayXRange;
avlc.model.validateRoad(road);

road.laneWidth = double(laneWidth);
road.targetLaneCenterY = road.laneWidth;
road.displayXRange = reshape(double(displayXRange), 1, 2);
end
