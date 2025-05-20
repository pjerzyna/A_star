dimensions = [252, 255];
res1 = 0.25;
x_size = dimensions(1);
y_size = dimensions(2);

% === FUNCTION: Fill the polygon with the occupied points ===
function filled_points = polygonToOccupiedPoints(polygon, resolution, dimensions)
    % Creating a grid of points based on cell centers
    x_centers = (0.5 : 1 : floor(dimensions(1)/resolution) - 0.5) * resolution;
    y_centers = (0.5 : 1 : floor(dimensions(2)/resolution) - 0.5) * resolution;
    [X, Y] = meshgrid(x_centers, y_centers);

    xv = polygon(1, :);
    yv = polygon(2, :);
    in = inpolygon(X, Y, xv, yv);
    filled_points = [X(in), Y(in)];
end

load('obstacle1.mat');  % obstacle_1: 2xN
load('obstacle2.mat');  % obstacle_2: 2xN
load('obstacle3.mat');  % obstacle_3: 2xN

% === Fill shapes with interior points ===
points1 = polygonToOccupiedPoints(obstacle_1, res1, dimensions);
points2 = polygonToOccupiedPoints(obstacle_2, res1, dimensions);
points3 = polygonToOccupiedPoints(obstacle_3, res1, dimensions);

% === Connect all the occupied points ===
all_obstacles_res = [points1; points2; points3];
occupied = [points1; points2; points3];

% === Grid generation ===
resolution = res1;
OccupancyGrid = generateOccupancyGrid(x_size, y_size, resolution, occupied, []);

% === Start / Goal ===
startNode = [5.2, 15.5];     % [x, y]
goalNode  = [22.7, 13.1];   

% === A* ===
allowDiagonal = true;
heuristicType = 1;

[path, visitedList] = a_star(OccupancyGrid, startNode, goalNode, allowDiagonal, heuristicType);

if isempty(path)
    disp('Brak ścieżki do celu.');
else
    disp('Znaleziono ścieżkę:');
    disp(path);
end

% === Path visualization ===
figure;
visualizeOccupancyGrid(OccupancyGrid);
grid on;
hold on;
if ~isempty(path)
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
    %plot(startNode(1), startNode(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    %plot(goalNode(1), goalNode(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
end
title('Occupancy grid with designated path A*');
legend('A* path','Start','End');


%%                   MESH GENERATION AND DRAWING FUNCTIONS       

% creating an empty occupancy grid
function empty_grid = createOccupancyGrid2D(x_size, y_size, resolution)
    empty_grid.width  = floor(x_size / resolution);
    empty_grid.height = floor(y_size / resolution);
    empty_grid.resolution = resolution;
    empty_grid.data = zeros(empty_grid.height, empty_grid.width); 
end

% fills the grid with obstacles based on the obstacle coordinates
function grid_with_obs = generateOccupancyGrid(x_size, y_size, resolution, occupied, ~)
    grid_with_obs = createOccupancyGrid2D(x_size, y_size, resolution);

    % Set obstacles as 1
    for i = 1:size(occupied, 1)
        x = occupied(i, 1);
        y = occupied(i, 2);
        % converting coordinates to grid indices
        col = floor(x / resolution) + 1;    
        row = floor(y / resolution) + 1;    
        if row >= 1 && row <= grid_with_obs.height && col >= 1 && col <= grid_with_obs.width
            grid_with_obs.data(row, col) = 1;
        end
    end
end

function visualizeOccupancyGrid(grid, path)
    % Axis values
    x = 0 : grid.resolution : (grid.width - 1) * grid.resolution;
    y = 0 : grid.resolution : (grid.height - 1) * grid.resolution;

    imagesc(x, y, grid.data, [0 1]);
    colormap(flipud(gray)); 
    %colorbar;
    axis equal;
    set(gca, 'YDir', 'normal');
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Occupancy grid (0=free, 1=occupied)');

    % Manually setting chart boundaries regardless of resolution
    xlim([0, 25]);
    ylim([0, 25]);

    % Draw a path
    if nargin > 1 && ~isempty(path)
        hold on;
        plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
    end

    % Ticks every 5 meters
    xticks(0:5:25);
    yticks(0:5:25);
end



%%                      A* ALGORITHM WITH PARENT MATRICES               %%


function [path, visitedList] = a_star(grid, startNode, goalNode, allowDiagonal, heuristicType)
    % grid - occupation grid with obstacles 
    % startNode - starting coordinates
    % goalNode - final coordinates
    % allowDiagonal - (true/false) do we allow diagonal movement
    % heuristicType - method of calculating the distance norm

    path = [];
    visitedList = [];

    %% 1. Conversion (x,y) -> (row,col)

    % we convert the positions to grid indices by dividing by the resolution
    % (size of one cell) and adding 1 because matlab indexes from 1
    startRow = floor(startNode(2) / grid.resolution) + 1;
    startCol = floor(startNode(1) / grid.resolution) + 1;
    goalRow  = floor(goalNode(2)  / grid.resolution) + 1;
    goalCol  = floor(goalNode(1)  / grid.resolution) + 1;


    %% 2. Open and closed list
    % open list - stores cells to visit
    openList = [];  % [row, col, gCost, hCost, fCost]
    % lista closed - stores visited cells
    closedList = false(size(grid.data));

    % Parent matrices
    % Every cell knows its parent - this approach makes easier
    % track recovery and it is more economical than storing whole objects
    parentRow = -1 * ones(size(grid.data));
    parentCol = -1 * ones(size(grid.data));

    % Add start to openList
    % cost of the distance traveled
    gStart = 0;
    % estimated cost to target
    hStart = calculateHeuristic(startRow, startCol, goalRow, goalCol, heuristicType);
    fStart = gStart + hStart;  
    openList = [startRow, startCol, gStart, hStart, fStart];
    parentRow(startRow, startCol) = -1;
    parentCol(startRow, startCol) = -1;

    iteration = 1;
    disp('--- START A* ---');
    fprintf('Start: (r=%d,c=%d), Goal: (r=%d,c=%d)\n', startRow, startCol, goalRow, goalCol);

    %% 3. Main loop A*
    %  node with the lowest cost f -> we add to the closed list
    % -> we generate neighbors -> if neighbor is the target = end
    while ~isempty(openList)
        % (a) Find the node with the lowest fCost
        [~, idxMin] = min(openList(:,5));
        currentNode = openList(idxMin, :);
        currentRow = currentNode(1);
        currentCol = currentNode(2);
        currentG   = currentNode(3);

        % (b) Move it from openList to closedList
        openList(idxMin,:) = [];
        closedList(currentRow, currentCol) = true;

        % Save it to visitedList (optional)
        visitedList(iteration,:) = [currentCol-1, currentRow-1];
        iteration = iteration + 1;

        % (c) Check if we have reached our destination
        if (currentRow == goalRow) && (currentCol == goalCol)
            disp('-> Goal achieved, I am replaying the track...');
            path = backtrack_byMatrix(parentRow, parentCol, goalRow, goalCol, grid.resolution, startNode, goalNode);
            return;
        end

        % (d) Get neighbors (checks if it doesn't go outside the map)
        neighbors = getNeighbors(currentRow, currentCol, size(grid.data), allowDiagonal);

        for iN = 1 : size(neighbors,1)
            nRow = neighbors(iN,1);
            nCol = neighbors(iN,2);

            % If it is an obstacle or already closed, we bypass it.
            if ~isAccessible(grid, nRow, nCol) || closedList(nRow, nCol)
                continue;
            end

            % How we update costs and build a path
            stepCost = 1;
            if allowDiagonal && (abs(nRow - currentRow)==1) && (abs(nCol - currentCol)==1)
                stepCost = sqrt(2);  %walking diagonally ~ 1.41
            end

            % The cost of reaching a neighbor
            tentativeG = currentG + stepCost;

            % Is the neighbor on the open list? (True, False)
            idxOpen = find(openList(:,1)==nRow & openList(:,2)==nCol, 1);

            if isempty(idxOpen)
                % Not in openList -> add
                h = calculateHeuristic(nRow, nCol, goalRow, goalCol, heuristicType);
                f = tentativeG + h;
                openList = [openList; nRow, nCol, tentativeG, h, f];

                % Save the parent
                parentRow(nRow, nCol) = currentRow;
                parentCol(nRow, nCol) = currentCol;

            else
                % It's already in openList -> check if the new method is better
                oldG = openList(idxOpen, 3);
                if tentativeG < oldG
                    openList(idxOpen, 3) = tentativeG;
                    openList(idxOpen, 5) = tentativeG + openList(idxOpen, 4);

                    % Update the parent
                    parentRow(nRow, nCol) = currentRow;
                    parentCol(nRow, nCol) = currentCol;
                end
            end
        end
    end

    disp('--- END: no path ---');
    path = [];
end


%% === FUNCTION THAT CHECKS IF A CELL PHONE IS AVAILABLE ===
% whether a given cell in the grid is available for movement
function accessible = isAccessible(grid, row, col)
    if row < 1 || row > size(grid.data,1) || col < 1 || col > size(grid.data,2)
        accessible = false;
        return;
    end
    if grid.data(row,col) == 1  %1 means obstacle
        accessible = false;
    else
        accessible = true;
    end
end

%% === FUNCTION RETURNING NEIGHBORS (we can move in 4 or 8) ===
function neighbors = getNeighbors(r, c, gridSize, allowDiagonal)
    % r, c - actual cell (row, col)
    % allowDiagonal - is it allowed to move diagonally
    % gridSize

    moves = [
        -1,  0;  %up
         1,  0;  %down
         0, -1;  %left
         0,  1   %right
    ];
    % adding slopes
    if allowDiagonal
        moves = [moves; -1, -1; -1, 1; 1, -1; 1, 1];
    end

    neighbors = [];  % character result - [r1, c1; r2, c2; ...];
    % checking if the neighbors are within the board boundariesy
    for i = 1:size(moves,1)
        rr = r + moves(i,1);
        cc = c + moves(i,2);
        if rr>=1 && rr<=gridSize(1) && cc>=1 && cc<=gridSize(2)
            neighbors = [neighbors; rr, cc];                    %#ok<AGROW>
        end
    end
end

%% === FUNCTION FOR CALCULATION OF HEURISTICS ===
function h = calculateHeuristic(r1, c1, r2, c2, type)
    dx = abs(r2 - r1);
    dy = abs(c2 - c1);
    switch type
        case 1
            % Manhattan
            h = dx + dy;
        case 2
            % Chebyshev
            h = max(dx, dy);
        case 3
            % Euclidean
            h = sqrt(dx^2 + dy^2);
        otherwise
            h = 0;
    end
end

%% === BACKTRACKING FROM PARENT MATRIX ===
% we go from the goal to the beginning 
% - each cell knows its parent something like LinkedList
function path = backtrack_byMatrix(parentRow, parentCol, goalR, goalC, resolution, startNode, goalNode)
    % parentRow, parentCol - parent matrices of each node
    % goalR, goalC - target indexes
    % startNode, goalNode - exact coordinates (metric) to insert 
    % the original point at the end
    
    % we start from the end and go back to the start
    path = [];
    r = goalR;
    c = goalC;

    tempPath = [];

    while ~(r == -1 || c == -1)
        % we switch to physical coordinates in meters (cell center)
        xPos = (c - 0.5) * resolution;
        yPos = (r - 0.5) * resolution;
        tempPath = [[xPos, yPos]; tempPath];  % adding to the beginning
        
        % each cell knows its predecessor - we go to the start one by one
        prevR = parentRow(r, c);
        prevC = parentCol(r, c);
        r = prevR;
        c = prevC;
    end
    
    % start and target correction - to start exactly from the specified
    % Overwrite the first and last points exactly as asked
    if ~isempty(tempPath)
        tempPath(1,:) = startNode;
        tempPath(end,:) = goalNode;
    end

    path = tempPath;
end
