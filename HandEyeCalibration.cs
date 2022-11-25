using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using UnityEngine;
using System;

namespace CoRegistration
{
	/// <summary>
	/// 
	/// Hand eye calibration: How to use it
	///
	/// Call 'Compute()' method while providing a sequence a tracker A matrices and a sequence of tracker B matrices. The method will output the matrix of the offset from tracker 1 origin to tracker 2 origin and the offset from tracker A to tracker B.
	/// 
	/// Note: 
	///   A[i] * X = Y * B[i] = B'[i] ;
	///   where B'[i] represents B[i] express in A coordinate system.
	/// 
	/// </summary>

	static public class HandEyeCalibration
	{
		public struct Matrix3x3
		{
			#region Members
			public const int size = 3;

			// mXY where X = row and Y = column
			public float m00;
			public float m10;
			public float m20;
			public float m01;
			public float m11;
			public float m21;
			public float m02;
			public float m12;
			public float m22;
			#endregion

			#region Getter / Setter
			public static Matrix3x3 Zero
			{
				get
				{
					return new Matrix3x3
					{
						m00 = 0f,
						m01 = 0f,
						m02 = 0f,
						m10 = 0f,
						m11 = 0f,
						m12 = 0f,
						m20 = 0f,
						m21 = 0f,
						m22 = 0f
					};
				}
			}

			public static Matrix3x3 Identity
			{
				get
				{
					return new Matrix3x3
					{
						m00 = 1f,
						m01 = 0f,
						m02 = 0f,
						m10 = 0f,
						m11 = 1f,
						m12 = 0f,
						m20 = 0f,
						m21 = 0f,
						m22 = 1f
					};
				}
			}
			#endregion

			#region Index accessors
			public float this[int row, int column]
			{
				get
				{
					return this[column * size + row];
				}

				set
				{
					this[column * size + row] = value;
				}
			}

			public float this[int index]
			{
				get
				{
					float result;

					switch(index)
					{
						case 0:
							result = m00;
							break;
						case 1:
							result = m10;
							break;
						case 2:
							result = m20;
							break;
						case 3:
							result = m01;
							break;
						case 4:
							result = m11;
							break;
						case 5:
							result = m21;
							break;
						case 6:
							result = m02;
							break;
						case 7:
							result = m12;
							break;
						case 8:
							result = m22;
							break;
						default:
							throw new IndexOutOfRangeException(string.Format("Invalid matrix index '{0}'", index));
					}

					return result;
				}

				set
				{
					switch(index)
					{
						case 0:
							m00 = value;
							break;
						case 1:
							m10 = value;
							break;
						case 2:
							m20 = value;
							break;
						case 3:
							m01 = value;
							break;
						case 4:
							m11 = value;
							break;
						case 5:
							m21 = value;
							break;
						case 6:
							m02 = value;
							break;
						case 7:
							m12 = value;
							break;
						case 8:
							m22 = value;
							break;
						default:
							throw new IndexOutOfRangeException(string.Format("Invalid matrix index '{0}'", index));
					}
				}
			}
			#endregion
		}

		#region Public methods
		/// <summary>
		/// Computes Hand Eye Calibration.
		/// </summary>
		/// <param name="a">Sequence of tracker A matrices (For example the robot 'hand' coordinate system). Size has to be > 3</param>
		/// <param name="b">Sequence of tracker B matrices (For example the robot 'eye' coordinate system). Size has to be > 3</param>
		/// <param name="origin_offset">Transformation from A origin to B origin, expressed in A
		/// <param name="tracker_offset">Transformation from tracker A to tracker B, expressed in A
		/// <returns>True if computation was successful, false otherwise</returns>
		static public bool Compute(List<Matrix4x4> a, List<Matrix4x4> b, out Matrix4x4 origin_offset, out Matrix4x4 tracker_offset)
		{
			origin_offset = new Matrix4x4();
			tracker_offset = new Matrix4x4();

			if(a.Count < 3)
			{
				Debug.LogError("There are too few points measured. Please add more points with 'Add' function before calling 'Compute'.");

				return false;
			}

			if(a.Count != b.Count)
			{
				Debug.LogError("a and b should contain the same number of elements");

				return false;
			}

			tracker_offset = ComputeX(a, b); ;

			origin_offset = ComputeY(a, b);

			return true;
		}
		#endregion

		#region Internal methods
		static Matrix4x4 Q(Vector4 r)
		{
			Matrix4x4 q = new Matrix4x4();
			for(int i = 0; i < 4; i++)
			{
				q[i, i] = r.w;
			}
			q[0, 1] = -r.x;
			q[0, 2] = -r.y;
			q[0, 3] = -r.z;
			q[1, 2] = -r.z;
			q[1, 3] = r.y;
			q[2, 3] = -r.x;

			for(int i = 0; i < 3; i++)
			{
				for(int j = i + 1; j < 4; j++)
				{
					q[j, i] = -q[i, j];
				}
			}
			return q;
		}

		static Matrix4x4 W(Vector4 r)
		{
			Matrix4x4 q = new Matrix4x4();
			for(int i = 0; i < 4; i++)
			{
				q[i, i] = r.w;
			}
			q[0, 1] = -r.x;
			q[0, 2] = -r.y;
			q[0, 3] = -r.z;
			q[1, 2] = r.z;
			q[1, 3] = -r.y;
			q[2, 3] = r.x;

			for(int i = 0; i < 3; i++)
			{
				for(int j = i + 1; j < 4; j++)
				{
					q[j, i] = -q[i, j];
				}
			}
			return q;
		}

		static Matrix<double> ConvertMatrix3x3IntoMathNetMatrix(Matrix3x3 m)
		{
			Matrix<double> mat = Matrix<double>.Build.Dense(3, 3);

			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					mat[i, j] = m[i, j];
				}
			}

			return mat;
		}

		static Vector<double> ConvertVector3IntoMathNetVector(Vector3 v)
		{
			Vector<double> vec = Vector<double>.Build.Dense(3);

			for(int i = 0; i < 3; i++)
			{
				vec[i] = v[i];
			}

			return vec;
		}

		static Vector3 ConvertMathNetVectorIntoVector3(Vector<double> v)
		{
			return new Vector3((float)v[0], (float)v[1], (float)v[2]);
		}

		static Matrix4x4 Matrix4x4Addition(Matrix4x4 m1, Matrix4x4 m2)
		{
			Matrix4x4 result = new Matrix4x4();

			for(int i = 0; i < 4; i++)
			{
				for(int j = 0; j < 4; j++)
				{
					result[i, j] = m1[i, j] + m2[i, j];
				}
			}
			return result;
		}

		static Matrix4x4 Ai(Vector4 vi1, Vector4 vi2)
		{
			return Matrix4x4Addition(Q(vi1), W(-vi2)).transpose * Matrix4x4Addition(Q(vi1), W(-vi2));
		}

		// Return the eigenvector associated with the smallest eigenvalue
		// and convert it into a Quaternion
		static Quaternion SmallestEigenVector(Matrix4x4 m)
		{
			Matrix<double> mCopy = Matrix<double>.Build.Dense(4, 4);
			for(int i = 0; i < 4; i++)
			{
				for(int j = 0; j < 4; j++)
				{
					mCopy[i, j] = m[i, j];
				}
			}

			Evd<double> evd = mCopy.Evd();

			System.Numerics.Complex min = evd.EigenValues[0];
			int indiceMin = 0;
			for(int i = 1; i < 4; i++)
			{
				if(evd.EigenValues[i].Imaginary == 0)
				{
					if((evd.EigenValues[i].Real > 0 && evd.EigenValues[i].Real < min.Real))
					{
						min = evd.EigenValues[i];
						indiceMin = i;
					}
				}
			}

			Quaternion result = new Quaternion();

			result.w = (float)evd.EigenVectors[0, indiceMin];

			// There could be a problem with quaternion sign, so... :
			if(result.w < 0)
			{
				result.w = -(float)evd.EigenVectors[0, indiceMin];
				result.x = -(float)evd.EigenVectors[1, indiceMin];
				result.y = -(float)evd.EigenVectors[2, indiceMin];
				result.z = -(float)evd.EigenVectors[3, indiceMin];
			}
			else
			{
				result.x = (float)evd.EigenVectors[1, indiceMin];
				result.y = (float)evd.EigenVectors[2, indiceMin];
				result.z = (float)evd.EigenVectors[3, indiceMin];
			}

			return result;
		}

		/// <summary>
		/// Compute X.
		/// X represents the transformation matrix between A measure points coordinate system and B measure points coordinate system :
		/// A[i] * X = B'[i]
		/// where B'[i] represents B[i] express in A origin coordinate system.
		/// </summary>
		/// <value>X.</value>
		static Matrix4x4 ComputeX(List<Matrix4x4> a, List<Matrix4x4> b)
		{
			List<Matrix4x4> moveA = new List<Matrix4x4>();
			List<Matrix4x4> moveB = new List<Matrix4x4>();

			for(int i = 1; i < a.Count; i++)
			{
				moveA.Add(a[i].inverse * a[i - 1]);
				moveB.Add(b[i].inverse * b[i - 1]);
			}

			Matrix4x4 x = ComputeCalibration(moveA, moveB);

			return x;
		}

		/// <summary>
		/// Compute Y.
		/// Y represents the transformation matrix between A origin coordinate system and B origin coordinate system:
		/// Y * B[i] = B'[i]
		/// where B'[i] represents B[i] express in A origin coordinate system.
		/// 
		/// Application to a robot-camera system:
		/// If A is the robot coordinate system and B the camera coordinate system,
		/// then A[i] represents the robot 'hand' coordinate system
		/// B[i] represents the camera 'eye' coordinate system
		/// hence B'[i] represents the camera 'eye' express in robot origin coordinate system.
		/// 
		/// We mostly use Y matrix rather than X.
		/// </summary>
		/// <value>Y.</value>
		static Matrix4x4 ComputeY(List<Matrix4x4> a, List<Matrix4x4> b)
		{
			List<Matrix4x4> moveA = new List<Matrix4x4>();
			List<Matrix4x4> moveB = new List<Matrix4x4>();

			for(int i = 1; i < a.Count; i++)
			{
				moveA.Add(a[i] * a[i - 1].inverse);
				moveB.Add(b[i] * b[i - 1].inverse);
			}

			Matrix4x4 y = ComputeCalibration(moveA, moveB);

			return y;
		}

		static Matrix4x4 ComputeCalibration(List<Matrix4x4> moveA, List<Matrix4x4> moveB)
		{
			Matrix4x4 sumRot = Matrix4x4.zero;

			// See "Hand-eye calibration", Horaud et Dornaika, Intl. Journal of Robotics Research, Vol. 14, No 3, pp 195-210, 1995.
			for(int i = 0; i < moveA.Count; i++)
			{
				// sum is the sum of n real-symmetrical matrix, so sum is diagonalisable
				sumRot = Matrix4x4Addition(sumRot, Ai((moveA[i].ExtractRotationQuaternion()).QuaternionAxis(), (moveB[i].ExtractRotationQuaternion()).QuaternionAxis()));
			}

			Quaternion q = SmallestEigenVector(sumRot);

			// Translation is obtained by a least-square method
			Matrix<double> matrixQ = ConvertMatrix3x3IntoMathNetMatrix(Matrix4x4.Rotate(q).ExtractRotationMatrix());
			Matrix<double> matrixI = ConvertMatrix3x3IntoMathNetMatrix(Matrix3x3.Identity);

			Matrix<double> lhsPos = Matrix<double>.Build.Dense(3, 3);
			Vector<double> rhsPos = Vector<double>.Build.Dense(3);

			Matrix<double> Ki = Matrix<double>.Build.Dense(3, 3);
			Vector<double> bi = Vector<double>.Build.Dense(3);

			for(int i = 0; i < moveA.Count; i++)
			{
				Ki = ConvertMatrix3x3IntoMathNetMatrix(moveA[i].ExtractRotationMatrix()).Subtract(matrixI);

				bi = matrixQ.Multiply(ConvertVector3IntoMathNetVector(moveB[i].ExtractTranslation())).Subtract(ConvertVector3IntoMathNetVector(moveA[i].ExtractTranslation()));

				lhsPos = lhsPos.Add(Ki.Transpose().Multiply(Ki));
				rhsPos = rhsPos.Add(Ki.Transpose().Multiply(bi));
			}

			Vector3 t = ConvertMathNetVectorIntoVector3(lhsPos.Inverse().Multiply(rhsPos));

			return Matrix4x4.TRS(t, q, Vector3.one);
		}
		#endregion

		#region Extension methods
		public static Quaternion ExtractRotationQuaternion(this Matrix4x4 matrix)
		{
			if(!matrix.IsOrthogonal())
			{
				Debug.LogWarning("Matrix is not orthogonal: conversion to quaternion doesn't make sense");
			}

			return Quaternion.LookRotation(matrix.GetColumn(2), matrix.GetColumn(1));
		}

		public static bool IsOrthogonal(this Matrix4x4 matrix)
		{
			bool ret = false;

			Vector4[] c = new Vector4[3];

			for(int i = 0; i < 3; i++)
			{
				c[i] = matrix.GetColumn(i);
			}

			float dot0 = Mathf.Abs(Vector4.Dot(c[0], c[1]));
			float dot1 = Mathf.Abs(Vector4.Dot(c[1], c[2]));
			float dot2 = Mathf.Abs(Vector4.Dot(c[0], c[2]));

			if(dot0 < Vector3.kEpsilon && dot1 < Vector3.kEpsilon && dot2 < Vector3.kEpsilon)
			{
				ret = true;
			}

			return ret;
		}

		public static Matrix3x3 ExtractRotationMatrix(this Matrix4x4 matrix)
		{
			Matrix3x3 rot = new Matrix3x3();

			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 3; j++)
				{
					rot[i, j] = matrix[i, j];
				}
			}

			return rot;
		}

		public static Vector3 ExtractTranslation(this Matrix4x4 matrix)
		{
			return matrix.GetColumn(3);
		}

		static public Vector3 QuaternionAxis(this Quaternion quaternion)
		{
			float sin = 1;

			if(quaternion.w < 1)
			{
				sin = 1 / Mathf.Sqrt(1 - quaternion.w * quaternion.w);
			}
			else
			{
				Debug.LogWarning("One the rotations was identity. Computation may be wrong.");
			}

			return new Vector3(quaternion.x * sin, quaternion.y * sin, quaternion.z * sin);
		}
		#endregion

	}
}