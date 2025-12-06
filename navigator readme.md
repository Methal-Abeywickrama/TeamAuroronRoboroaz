# Structs
## Vec2
    a templated structure to work with coordinate points. it is used with int, double here.
    can substract and check equivalence

# Classes
## Odometry
    a class to keep track of robot movements, positions.
    ->constructor 
        needs: wheel radius, axle length and cell size
    ->update function
        takes left and right wheel angular velocity as input to update the next position.
        set dt as time step
    ->getters
        x,y, position coordinates and angle
    ->Converter Function
        getGrid converts position value to grid position value. (non relative)

## Map
    a class to map the grid while the epuck moves
    ->Constructor
        needs number of rows,columns, origin coordinates and cell width
    ->Converter Functions
        index() : as the map is 1D, coordinates are converted into a flat index . it returns the value related to the grid coordinates
    ->update function
        update() :to be used in direct input function, head_update
        updateCheckpointCount(): to keep track of checkpoints visited and to add their coordinates
    ->Input update (head_update) 
        need to give a better name
        takes direct input from sensors. north relative to epuck . (As in north is the heading direction of epuck)
        inputs: wall sensor results,odemetry, current coordinates.
    ->Print Map
        for debugging and testing purposes.
    ->Getters and Helpers
        getdim() : to get the dimension of grid
        getDest() : to get the final desitination to move after the mapping
        getValue() : to be used in path finder function for wall information
        isValid() : to check validity of coordinates

# Functions
## isUnblocked()
    to be used by A* algorithm to check wheather the path is blocked
    inputs: current point and destination point
    