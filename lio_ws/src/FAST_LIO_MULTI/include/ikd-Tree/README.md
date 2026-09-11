# ikd-Tree
+ ikd-Tree is an incremental k-d tree for robotic applications.

<br>

## Coverage check branch
+ This branch is for checking if the nodes(points) are **covered(seen)** by the sensor attached to the robots.
	+ `Set_Covered_Points`, `Get_Covered_Points` methods are added.
		+ with `covered` flag in `PointType` struct
		+ `Set_Covered_Points`
			+ Old: kdtree search with division axis for each point (not accurate)
			+ Currnet: for each point, check whole tree (accurate but slow)
+ Additionally, `Downsampling` mechanism is modified for faster mapping and grid-aligned points.
	+ Original: add a point with the shortest distance to the centroid of a voxel, and delete other points in a voxel grid
		+ Searching other points, deleting other points, and comparing distance take more time than changed method
	  + (`Add_by_point` should be changed to set `covered` as existing points are removed)
	+ **Changed**: add a centroid of a voxel where the raw point is laid / if a point already exists in the voxel grid, do nothing
		+ Searching other points (only checking existence), but no deletion nor comparing
		+ `Add_Points` automatically calls `Build`, if not built yet
			+ Input points for `Build` inside `Add_Points` is also modified to use `Downsampling`
			+ Otherwise, the first input points are not grid-aligned as voxels.

#### TODO
+ Current problem (from *original repo*)
	+ Mid point in `BuildTree` is not real middle point (current: `nth_element`)
		+ So, the methods that use kdtree search with division axis but not whole tree search (`Rebuild`, `Add_Points`, `Delete_Points`) are not accurate.
+ Solution1
	+ Correct `BuildTree` with other than `nth_element`
	+ Modify nothing but implementation of `Set_Covered_Points`
+ Solution2
	+ Leave `BuildTree`, `Add_Points`, `Rebuild` (giving up)
	+ Modify `Delete_Points` to search whole tree => `Delete_Points_Accurate`
	+ Added `Delete_Points_Downsample` which deletes the all points within the voxel
