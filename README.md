# NavigationMesh
A custom implementation of navmeshes to replace the default Unity navmesh.
The motivation behind this was to allow for instant navmesh updates within the same frame.

``Scripts/AStar`` contains a modified AStar algorithm for use on a NavigationMesh.
``Scripts/Algorithm`` contains the navmesh generator algorithm. Especially ``Scripts/Algorithm/3_GraphAssembly`` could be of interest for reusing this implementation in other contexts.
``Scripts/Interfaces`` contains the implementation specific interfaces for the navmesh generator.
