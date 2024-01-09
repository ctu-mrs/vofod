# Problems with using VoxBlox for VoFOD

## Rays assumed to be perpendicular to the measured surface
For each voxel, VoxBlox estimates is distance to the closest surface.
To do this, VoxBlox relies on the assumption that the rays coming from the sensors to the measured point are penpendicular to the surface corresponding to the point when raycasting.
If the sensor is moving, the inaccuracy due to this assumption will mostly average out, but there are cases when a voxel's distance to the closest surface can be grossly underestimated due to this.
This can lead to false detections and bad cluster classifications.

## Weight of the voxels never decreases
When updating a voxel, the new weight is always the sum of the original weight and weight of the measured point.
The confidence of voxels thus only ever increases, which means that the map reacts poorly to dynamic obstacles after some time.
