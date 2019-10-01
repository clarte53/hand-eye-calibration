Hand-eye calibration
====================
Compute offset from tracker 1 to tracker 2 given sequences of matrices of every tracker.
Depends on [clarte-utils](https://github.com/clarte53/clarte-utils "clarte-utils") module and [Math.Net Numerics](https://numerics.mathdotnet.com/).

See ["Hand-eye calibration", Horaud et Dornaika, Intl. Journal of Robotics Research, Vol. 14, No 3, pp 195-210, 1995](https://hal.inria.fr/inria-00590039/document) for details.

Usage
=====
Call the following static method:

`static public bool HandEyeCalibration.Compute(List<Matrix4x4> a, List<Matrix4x4> b, out Matrix4x4 origin_offset, out Matrix4x4 tracker_offset)`

with:

**a**: Sequence of tracker A matrices (For example a robot 'hand' coordinate system). Size has to be > 3

**b**: Sequence of tracker B matrices (For example a robot 'eye' coordinate system). Size has to be > 3

outpout parms:

**origin_offset**: Transformation from A origin to B origin, expressed in A

**tracker_offset**: Transformation from tracker A to tracker B, expressed in A


It is advised to feed the method with 10+ couples of matrices, depending on the quality of the measurements.

Tested on Win10, Unity 2019, <span>Math.</span>Net v4.5.