# ofxHeadPoseEstimator is an addon for [openFrameworks](http://openframeworks.cc)

Actually, it's not a proper addon yet. Right now it's just an example, but I wanted to get it out there for other people to play with even before it was wrapped into an addon.

The code was originally written by [Gabriele Fanelli](http://www.vision.ee.ethz.ch/~gfanelli/head_pose/head_forest.html) and integrated with openFrameworks by [David Sans Kirbis](http://therandomlab.blogspot.com/2011/12/kinect-real-time-head-tracking-with.html).

To run the example, you will need to download the [trained trees](https://github.com/downloads/kylemcdonald/ofxHeadPoseEstimator/ofxHeadPoseEstimator-trees.zip) and put them in the `example/bin/data/trees/` directory. You can also download the [original file](http://biwinas03.ee.ethz.ch/gfanelli/kinect_head_pose_db.tgz)  from Gabriele, but be warned it's >5GB.

The HPE code is distributed under the [Microsoft Research Shared Source License](http://www.vision.ee.ethz.ch/~gfanelli/downloads/MSR-SSLA.txt), as it is an adaptation of [Hough Forest Code](http://www.vision.ee.ethz.ch/~gallju/projects/houghforest/index.html). 