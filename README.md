# merge_depth_cam (Robot Operating System Package)

## What is it?

This ROS package merges the depth images of two depth cameras (RGBD, Time of Flight, Structured Light etc.) into one depth image. 

It's actually twofold. One node merges two point clouds and the other converts a point cloud from a certain point of view into a depth image. By chaining multiple merge_pcl nodes/nodelets, it's possible to merge even more cameras. At a cost though. 

## Why is this interesting?

For simple RGB images it is possible to just distort and stitch two images based on the camera parameters and certain features (edges, corners) that are commonly available in both images to generate a "panorama" like image (see auto_stitch for example). For depth images this is not sufficient as the depth values (i.e. the pixel values) would also have to be adapted to the distortion. 

This node/nodelet utilizes point cloud concatenation to generate a depth image. This is done by taking the following steps:

1. generate point clouds from the depth images of both cameras
2. transform both point clouds into a common coordinate system (e.g. "map" or the coordinate system of the virtual camera; see below)
3. concatenate both transformed point clouds into one big point cloud
4. project all points of the new point into the image of a pinhole camera model based virtual camera

## Yes, but why is this a thing anyway?

I needed a way to enhance the field of view of a depth cam. Instead of fiddling with additional lenses (which would probably have worked too), I used what I had at hand: a second depth cam. Furthermore I like the idea of using a point cloud because it makes it possible to place the virtual camera practically arbitrarily and even make it able to move in the virtual space, as long as it and the other cameras see respectively generate enough points of the point cloud to get a reasonable depth image as a result. 

This makes it even possible to optimize the camera pose based on some fitness value (e.g. for people tracking this might be how well the person of interest can be seen by the virtual camera). 

## Why not use a matrix operation to do the distortion/stitching appropriately for a depth image?

Well, I could have, but... first of all, to be dead honest, using the point clouds was easier to grasp and implement. If someone comes up with a faster solution, go for it! I won't stop anyone from implementing a pure math/depth image based approach. Quite the opposite. I'd really like to see how much that approach would be faster. 

## Anything to consider?

Sure. All cameras (the two real cameras and the virtual camera) should have the same focal length. The pinhole camera furthermore should take a point of view that sees all points of the point cloud and from which the coverage of the point cloud is sufficient. 

CPU load is quite high, of course, depending on the input and output resolution.

# Usage

TBA
