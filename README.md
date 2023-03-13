# photo2global
photo2global is an open-source repository that provides tools for reorienting photogrammetry point clouds to a global plane using Open3D library. The tool was developed as part of the Robotic Fabrication seminar at Institute for Advanced Architecture of Catalunya. 

### Technical Description
The tool is built on two loops. 

***First loop*** asseses the plane model of the segmented plane. It obtains the normal of the plane from plane model and constructs X,Y axis on the plane.
This helps to -
- Move the geometry to the origin of open 3D
- Orient the Normal of the plane with Z axis of the open3D

This operation does first fitting and aligning of the point cloud with the global co-ordinates.

***Second loop*** analyses the orientation of the point cloud plane using PCA analysis to extract eigen vectors. Eigen vectors are used to create a rotation matrix to transform the point cloud to the origin plane orientation.

Once the point cloud is oriented to the global plane any kind of segmentation operation can be performed.

### Note
- For good results it is important to remove as much noise as possible from the segmented plane of the pointcloud.
- Sometimes point clouds have to be rotated after the second loop. This is because X,Y axis of the plane is assumed in the first loop this sometimes causes the X,Y axis to be flipped.

Feel free to suggest how this can be improved. 

Cheers!
