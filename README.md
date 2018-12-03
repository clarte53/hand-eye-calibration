Hand eye calibration
====================
Compute offset from tracker 1 to tracker 2 given sequences of matrices of every tracker.
Depends on [clarte-utils](ssh://git@gitlab.clarte.asso.fr:53000/modules/clarte-utils.git "clarte-utils") module (GitHub url will be added soon for external reference) and [Math.Net Numerics](https://numerics.mathdotnet.com/) (MathNet.Numerics.dll and System.Threading.dll required).

Usage:
`static public bool Compute(List<Matrix4x4> a, List<Matrix4x4> b, out Matrix4x4 origin_offset, out Matrix4x4 tracker_offset)`

with:
a: Sequence of tracker A matrices (For example the robot 'hand' coordinate system). Size has to be > 3
b: Sequence of tracker B matrices (For example the robot 'eye' coordinate system). Size has to be > 3

outpout parms:
origin_offset: Transformation from A origin to B origin, expressed in A
tracker_offset: Transformation from tracker A to tracker B, expressed in A