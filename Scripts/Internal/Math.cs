// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing
using UnityEngine;

namespace SA
{

	public partial class FullBodyIK : MonoBehaviour
	{
		[System.Serializable]
		public struct Matrix3x3
		{
			public Vector3 row0, row1, row2;

			public static readonly Matrix3x3 identity = new Matrix3x3(
				1.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 1.0f );

			public bool isFuzzyIdentity
			{
				get
				{
					if( row0.x < 1.0f - IKEpsilon || row0.y < -IKEpsilon || row0.z < -IKEpsilon ||
						row1.x < -IKEpsilon || row1.y < 1.0f - IKEpsilon || row1.z < -IKEpsilon ||
						row2.x < -IKEpsilon || row2.y < -IKEpsilon || row2.z < 1.0f - IKEpsilon ||
						row0.x > 1.0f + IKEpsilon || row0.y > IKEpsilon || row0.z > IKEpsilon ||
						row1.x > IKEpsilon || row1.y > 1.0f + IKEpsilon || row1.z > IKEpsilon ||
						row2.x > IKEpsilon || row2.y > IKEpsilon || row2.z > 1.0f + IKEpsilon ) {
						return false;
					}

					return true;
				}
			}

			public Matrix3x3 transpose
			{
				get
				{
					return new Matrix3x3(
						row0.x, row1.x, row2.x,
						row0.y, row1.y, row2.y,
						row0.z, row1.z, row2.z );
				}
			}

			public Matrix3x3(
				float _11, float _12, float _13,
				float _21, float _22, float _23,
				float _31, float _32, float _33 )
			{
				row0 = new Vector3( _11, _12, _13 );
				row1 = new Vector3( _21, _22, _23 );
				row2 = new Vector3( _31, _32, _33 );
			}

			public Matrix3x3( Vector3 r0, Vector3 r1, Vector3 r2 )
			{
				row0 = r0;
				row1 = r1;
				row2 = r2;
			}

			public Matrix3x3( ref Vector3 r0, ref Vector3 r1, ref Vector3 r2 )
			{
				row0 = r0;
				row1 = r1;
				row2 = r2;
			}

			public Matrix3x3( Vector3 axis, float cos )
			{
				row0 = Vector3.zero;
				row1 = Vector3.zero;
				row2 = Vector3.zero;
				SetRotation( ref axis, cos );
			}

			public Matrix3x3( ref Vector3 axis, float cos )
			{
				row0 = Vector3.zero;
				row1 = Vector3.zero;
				row2 = Vector3.zero;
				SetRotation( ref axis, cos );
			}

			public Matrix3x3( Matrix4x4 m )
			{
				row0 = new Vector3( m.m00, m.m01, m.m02 );
				row1 = new Vector3( m.m10, m.m11, m.m12 );
				row2 = new Vector3( m.m20, m.m21, m.m22 );
			}

			public Matrix3x3( ref Matrix4x4 m )
			{
				row0 = new Vector3( m.m00, m.m01, m.m02 );
				row1 = new Vector3( m.m10, m.m11, m.m12 );
				row2 = new Vector3( m.m20, m.m21, m.m22 );
			}

			public Matrix3x3( Quaternion q )
			{
				row0 = Vector3.zero;
				row1 = Vector3.zero;
				row2 = Vector3.zero;
				SetRotation( ref q );
			}

			public Matrix3x3( ref Quaternion q )
			{
				row0 = Vector3.zero;
				row1 = Vector3.zero;
				row2 = Vector3.zero;
				SetRotation( ref q );
			}

			public static Matrix3x3 FromColumn( Vector3 column0, Vector3 column1, Vector3 column2 )
			{
				Matrix3x3 r = new Matrix3x3();
				r.SetColumn( ref column0, ref column1, ref column2 );
				return r;
			}

			public static Matrix3x3 FromColumn( ref Vector3 column0, ref Vector3 column1, ref Vector3 column2 )
			{
				Matrix3x3 r = new Matrix3x3();
				r.SetColumn( ref column0, ref column1, ref column2 );
				return r;
			}

			public void SetValue(
				float _11, float _12, float _13,
				float _21, float _22, float _23,
				float _31, float _32, float _33 )
			{
				row0.x = _11; row0.y = _12; row0.z = _13;
				row1.x = _21; row1.y = _22; row1.z = _23;
				row2.x = _31; row2.y = _32; row2.z = _33;
			}

			public void SetValue( Vector3 r0, Vector3 r1, Vector3 r2 )
			{
				row0 = r0;
				row1 = r1;
				row2 = r2;
			}

			public void SetValue( ref Vector3 r0, ref Vector3 r1, ref Vector3 r2 )
			{
				row0 = r0;
				row1 = r1;
				row2 = r2;
			}

			public void SetValue( Matrix4x4 m )
			{
				row0.x = m.m00;
				row0.y = m.m01;
				row0.z = m.m02;

				row1.x = m.m10;
				row1.y = m.m11;
				row1.z = m.m12;

				row2.x = m.m20;
				row2.y = m.m21;
				row2.z = m.m22;
			}

			public void SetValue( ref Matrix4x4 m )
			{
				row0.x = m.m00;
				row0.y = m.m01;
				row0.z = m.m02;

				row1.x = m.m10;
				row1.y = m.m11;
				row1.z = m.m12;

				row2.x = m.m20;
				row2.y = m.m21;
				row2.z = m.m22;
			}

			public void SetColumn( int n, Vector3 c )
			{
				switch( n ) {
				case 0:
					row0.x = c.x;
					row1.x = c.y;
					row2.x = c.z;
					break;
				case 1:
					row0.y = c.x;
					row1.y = c.y;
					row2.y = c.z;
					break;
				case 2:
					row0.z = c.x;
					row1.z = c.y;
					row2.z = c.z;
					break;
				}
			}

			public void SetColumn( int n, ref Vector3 c )
			{
				switch( n ) {
				case 0:
					row0.x = c.x;
					row1.x = c.y;
					row2.x = c.z;
					break;
				case 1:
					row0.y = c.x;
					row1.y = c.y;
					row2.y = c.z;
					break;
				case 2:
					row0.z = c.x;
					row1.z = c.y;
					row2.z = c.z;
					break;
				}
			}

			public void SetColumn( Vector3 c0, Vector3 c1, Vector3 c2 )
			{
				row0.x = c0.x;
				row1.x = c0.y;
				row2.x = c0.z;

				row0.y = c1.x;
				row1.y = c1.y;
				row2.y = c1.z;

				row0.z = c2.x;
				row1.z = c2.y;
				row2.z = c2.z;
			}

			public void SetColumn( ref Vector3 c0, ref Vector3 c1, ref Vector3 c2 )
			{
				row0.x = c0.x;
				row1.x = c0.y;
				row2.x = c0.z;

				row0.y = c1.x;
				row1.y = c1.y;
				row2.y = c1.z;

				row0.z = c2.x;
				row1.z = c2.y;
				row2.z = c2.z;
			}

			public Vector3 GetColumn( int n )
			{
				switch( n ) {
				case 0:
					return new Vector3( row0.x, row1.x, row2.x );
				case 1:
					return new Vector3( row0.y, row1.y, row2.y );
				case 2:
					return new Vector3( row0.z, row1.z, row2.z );
				default:
					return Vector3.zero;
				}
			}

			public Vector3 column0
			{
				set
				{
					row0.x = value.x;
					row1.x = value.y;
					row2.x = value.z;
				}
				get
				{
					return new Vector3( row0.x, row1.x, row2.x );
				}
			}

			public Vector3 column1
			{
				set
				{
					row0.y = value.x;
					row1.y = value.y;
					row2.y = value.z;
				}
				get
				{
					return new Vector3( row0.y, row1.y, row2.y );
				}
			}

			public Vector3 column2
			{
				set
				{
					row0.z = value.x;
					row1.z = value.y;
					row2.z = value.z;
				}
				get
				{
					return new Vector3( row0.z, row1.z, row2.z );
				}
			}

			public void GetColumn( out Vector3 c0, out Vector3 c1, out Vector3 c2 )
			{
				c0.x = row0.x;
				c0.y = row1.x;
				c0.z = row2.x;

				c1.x = row0.y;
				c1.y = row1.y;
				c1.z = row2.y;

				c2.x = row0.z;
				c2.y = row1.z;
				c2.z = row2.z;
			}

			public Vector3 Multiply( Vector3 v )
			{
				return Multiply( ref v );
			}

			public Vector3 Multiply( ref Vector3 v )
			{
				return new Vector3(
					row0.x * v.x + row0.y * v.y + row0.z * v.z,
					row1.x * v.x + row1.y * v.y + row1.z * v.z,
					row2.x * v.x + row2.y * v.y + row2.z * v.z );
			}

			public static Vector3 operator *( Matrix3x3 m, Vector3 v )
			{
				return m.Multiply( ref v );
			}

			public Matrix3x3 Multiply( Matrix3x3 m )
			{
				return Multiply( ref m );
			}

			public Matrix3x3 Multiply( ref Matrix3x3 m )
			{
				return new Matrix3x3(
					row0.x * m.row0.x + row0.y * m.row1.x + row0.z * m.row2.x,
					row0.x * m.row0.y + row0.y * m.row1.y + row0.z * m.row2.y,
					row0.x * m.row0.z + row0.y * m.row1.z + row0.z * m.row2.z,

					row1.x * m.row0.x + row1.y * m.row1.x + row1.z * m.row2.x,
					row1.x * m.row0.y + row1.y * m.row1.y + row1.z * m.row2.y,
					row1.x * m.row0.z + row1.y * m.row1.z + row1.z * m.row2.z,

					row2.x * m.row0.x + row2.y * m.row1.x + row2.z * m.row2.x,
					row2.x * m.row0.y + row2.y * m.row1.y + row2.z * m.row2.y,
					row2.x * m.row0.z + row2.y * m.row1.z + row2.z * m.row2.z );
			}

			public static Matrix3x3 operator *( Matrix3x3 m1, Matrix3x3 m2 )
			{
				return m1.Multiply( ref m2 );
			}

			public static implicit operator Matrix4x4( Matrix3x3 m )
			{
				Matrix4x4 r = Matrix4x4.identity;
				r.m00 = m.row0.x;
				r.m01 = m.row0.y;
				r.m02 = m.row0.z;

				r.m10 = m.row1.x;
				r.m11 = m.row1.y;
				r.m12 = m.row1.z;

				r.m20 = m.row2.x;
				r.m21 = m.row2.y;
				r.m22 = m.row2.z;
				return r;
			}

			public static implicit operator Matrix3x3( Matrix4x4 m )
			{
				return new Matrix3x3( ref m );
			}

			public void SetRotation( Quaternion q )
			{
				SetRotation( ref q );
			}

			public void SetRotation( ref Quaternion q )
			{
				if( IsFuzzyIdentity( ref q ) ) {
					this = identity;
					return;
				}

				float d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
				float s = (d > Mathf.Epsilon) ? (2.0f / d) : 0.0f;
				float xs = q.x * s, ys = q.y * s, zs = q.z * s;
				float wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
				float xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
				float yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
				row0.x = 1.0f - (yy + zz);
				row0.y = xy - wz;
				row0.z = xz + wy;
				row1.x = xy + wz;
				row1.y = 1.0f - (xx + zz);
				row1.z = yz - wx;
				row2.x = xz - wy;
				row2.y = yz + wx;
				row2.z = 1.0f - (xx + yy);
				CheckNaN( row0 );
				CheckNaN( row1 );
				CheckNaN( row2 );
			}

			public void SetRotation( Vector3 axis, float cos )
			{
				SetRotation( ref axis, cos );
			}

			public void SetRotation( ref Vector3 axis, float cos )
			{
				float sin = 1.0f - cos * cos;
				sin = (sin < IKEpsilon) ? 0.0f : ((sin > 1.0f - IKEpsilon) ? 1.0f : (float)System.Math.Sqrt( (float)sin ));

				float axis_x_sin = axis.x * sin;
				float axis_y_sin = axis.y * sin;
				float axis_z_sin = axis.z * sin;

				// r = axis * Vector3.Dot( axis, p )
				// r += Vector3.Cross( axis, p * sinTheta )
				// r = new Vector3( vecA.y * vecB.z - vecA.z * vecB.y, vecA.z * vecB.x - vecA.x * vecB.z, vecA.x * vecB.y - vecA.y * vecB.x );

				row0.x = cos;
				row1.x = axis_z_sin;
				row2.x = -axis_y_sin;

				row0.y = -axis_z_sin;
				row1.y = cos;
				row2.y = axis_x_sin;

				row0.z = axis_y_sin;
				row1.z = -axis_x_sin;
				row2.z = cos;

				// Vector3 pV = Vector3.Dot( axis, p );
				// r += pV * (1.0f - cosTheta);
				float cosI = 1.0f - cos;
				float axis_x_cosI = axis.x * cosI;
				float axis_y_cosI = axis.y * cosI;
				float axis_z_cosI = axis.z * cosI;

				row0.x += axis.x * axis_x_cosI;
				row1.x += axis.y * axis_x_cosI;
				row2.x += axis.z * axis_x_cosI;

				row0.y += axis.x * axis_y_cosI;
				row1.y += axis.y * axis_y_cosI;
				row2.y += axis.z * axis_y_cosI;

				row0.z += axis.x * axis_z_cosI;
				row1.z += axis.y * axis_z_cosI;
				row2.z += axis.z * axis_z_cosI;

				CheckNaN( row0 );
				CheckNaN( row1 );
				CheckNaN( row2 );
			}

			public Quaternion GetRotation()
			{
				if( this.isFuzzyIdentity ) {
					return Quaternion.identity;
				}

				Quaternion q = new Quaternion();
				float t = row0.x + row1.y + row2.z;
				if( t > 0.0f ) {
					float s = Sqrt( t + 1.0f );
					CheckNaN( s );
					q.w = s * 0.5f;
					s = 0.5f / s;
					q.x = (row2.y - row1.z) * s;
					q.y = (row0.z - row2.x) * s;
					q.z = (row1.x - row0.y) * s;
					CheckNaN( q );
				} else {
					if( row0.x > row1.y && row0.x > row2.z ) {
						float s = Sqrt( row0.x - row1.y - row2.z + 1.0f );
						CheckNaN( s );
						q.x = s * 0.5f;
						s = 0.5f / s;
						q.w = (row2.y - row1.z) * s;
						q.y = (row1.x + row0.y) * s;
						q.z = (row2.x + row0.z) * s;
						CheckNaN( q );
					} else if( row1.y > row2.z ) {
						float s = Sqrt( row1.y - row0.x - row2.z + 1.0f );
						CheckNaN( s );
						q.y = s * 0.5f;
						s = 0.5f / s;
						q.w = (row0.z - row2.x) * s;
						q.z = (row2.y + row1.z) * s;
						q.x = (row0.y + row1.x) * s;
						CheckNaN( q );
					} else {
						float s = Sqrt( row2.z - row0.x - row1.y + 1.0f );
						CheckNaN( s );
						q.z = s * 0.5f;
						s = 0.5f / s;
						q.w = (row1.x - row0.y) * s;
						q.x = (row0.z + row2.x) * s;
						q.y = (row1.z + row2.y) * s;
						CheckNaN( q );
					}
				}

				return q;
			}

			public static implicit operator Quaternion( Matrix3x3 m )
			{
				return m.GetRotation();
			}

			public static implicit operator Matrix3x3( Quaternion q )
			{
				return new Matrix3x3( ref q );
			}

			public override string ToString()
			{
				System.Text.StringBuilder str = new System.Text.StringBuilder();
				str.Append( row0.ToString() );
				str.Append( " : " );
				str.Append( row1.ToString() );
				str.Append( " : " );
				str.Append( row2.ToString() );
				return str.ToString();
			}

			public string ToString( string format )
			{
				System.Text.StringBuilder str = new System.Text.StringBuilder();
				str.Append( row0.ToString( format ) );
				str.Append( " : " );
				str.Append( row1.ToString( format ) );
				str.Append( " : " );
				str.Append( row2.ToString( format ) );
				return str.ToString();
			}

			public string ToStringColumn()
			{
				System.Text.StringBuilder str = new System.Text.StringBuilder();
				str.Append( column0.ToString() );
				str.Append( "(" );
				str.Append( column0.magnitude );
				str.Append( ") : " );
				str.Append( column1.ToString() );
				str.Append( "(" );
				str.Append( column1.magnitude );
				str.Append( ") : " );
				str.Append( column2.ToString() );
				str.Append( "(" );
				str.Append( column2.magnitude );
				str.Append( ")" );
				return str.ToString();
			}

			public string ToStringColumn( string format )
			{
				System.Text.StringBuilder str = new System.Text.StringBuilder();
				str.Append( column0.ToString( format ) );
				str.Append( "(" );
				str.Append( column0.magnitude );
				str.Append( ") : " );
				str.Append( column1.ToString( format ) );
				str.Append( "(" );
				str.Append( column1.magnitude );
				str.Append( ") : " );
				str.Append( column2.ToString( format ) );
				str.Append( "(" );
				str.Append( column2.magnitude );
				str.Append( ")" );
				return str.ToString();
			}

			public bool Normalize()
			{
				float n0 = Sqrt( row0.x * row0.x + row1.x * row1.x + row2.x * row2.x );
				float n1 = Sqrt( row0.y * row0.y + row1.y * row1.y + row2.y * row2.y );
				float n2 = Sqrt( row0.z * row0.z + row1.z * row1.z + row2.z * row2.z );

				bool valid = true;

				if( n0 > IKEpsilon ) {
					n0 = 1.0f / n0;
					row0.x *= n0;
					row1.x *= n0;
					row2.x *= n0;
				} else {
					valid = false;
					row0.x = 1.0f;
					row1.x = 0.0f;
					row2.x = 0.0f;
				}

				if( n1 > IKEpsilon ) {
					n1 = 1.0f / n1;
					row0.y *= n1;
					row1.y *= n1;
					row2.y *= n1;
				} else {
					valid = false;
					row0.y = 0.0f;
					row1.y = 1.0f;
					row2.y = 0.0f;
				}

				if( n2 > IKEpsilon ) {
					n2 = 1.0f / n2;
					row0.z *= n2;
					row1.z *= n2;
					row2.z *= n2;
				} else {
					valid = false;
					row0.z = 0.0f;
					row1.z = 0.0f;
					row2.z = 1.0f;
				}

				return valid;
			}
		}

		[System.Serializable]
		public struct Matrix3x4
		{
			public Matrix3x3 basis;
			public Vector3 origin;

			public static readonly Matrix3x4 identity = new Matrix3x4( Matrix3x3.identity, Vector3.zero );

			public Matrix3x4 inverse
			{
				get
				{
					Matrix3x3 t = basis.transpose;
					Vector3 v = t * -origin;
					return new Matrix3x4( ref t, ref v );
				}
			}

			public Matrix3x4( Matrix3x3 _basis, Vector3 _origin )
			{
				basis = _basis;
				origin = _origin;
			}

			public Matrix3x4( ref Matrix3x3 _basis, ref Vector3 _origin )
			{
				basis = _basis;
				origin = _origin;
			}

			public Matrix3x4( Quaternion _q, Vector3 _origin )
			{
				basis = _q;
				origin = _origin;
			}

			public Matrix3x4( ref Quaternion _q, ref Vector3 _origin )
			{
				basis = _q;
				origin = _origin;
			}

			public Matrix3x4( Matrix4x4 m )
			{
				basis = new Matrix3x3( ref m );
				origin = new Vector3( m.m03, m.m13, m.m23 );
			}

			public Matrix3x4( ref Matrix4x4 m )
			{
				basis = new Matrix3x3( ref m );
				origin = new Vector3( m.m03, m.m13, m.m23 );
			}

			public static implicit operator Matrix4x4( Matrix3x4 t )
			{
				Matrix4x4 m = Matrix4x4.identity;
				m.m00 = t.basis.row0.x;
				m.m01 = t.basis.row0.y;
				m.m02 = t.basis.row0.z;

				m.m10 = t.basis.row1.x;
				m.m11 = t.basis.row1.y;
				m.m12 = t.basis.row1.z;

				m.m20 = t.basis.row2.x;
				m.m21 = t.basis.row2.y;
				m.m22 = t.basis.row2.z;

				m.m03 = t.origin.x;
				m.m13 = t.origin.y;
				m.m23 = t.origin.z;
				return m;
			}

			public static implicit operator Matrix3x4( Matrix4x4 m )
			{
				return new Matrix3x4( ref m );
			}

			public Vector3 Multiply( Vector3 v )
			{
				return basis * v + origin;
			}

			public Vector3 Multiply( ref Vector3 v )
			{
				return basis * v + origin;
			}

			public static Vector3 operator *( Matrix3x4 t, Vector3 v )
			{
				return t.basis * v + t.origin;
			}

			public Matrix3x4 Multiply( Matrix3x4 t )
			{
				return new Matrix3x4( basis * t.basis, basis * t.origin + origin );
			}

			public Matrix3x4 Multiply( ref Matrix3x4 t )
			{
				return new Matrix3x4( basis * t.basis, basis * t.origin + origin );
			}

			public static Matrix3x4 operator *( Matrix3x4 t1, Matrix3x4 t2 )
			{
				return new Matrix3x4( t1.basis * t2.basis, t1.basis * t2.origin + t1.origin );
			}

			public override string ToString()
			{
				System.Text.StringBuilder str = new System.Text.StringBuilder();
				str.Append( "basis: " );
				str.Append( basis.ToString() );
				str.Append( " origin: " );
				str.Append( origin.ToString() );
				return str.ToString();
			}

			public string ToString( string format )
			{
				System.Text.StringBuilder str = new System.Text.StringBuilder();
				str.Append( "basis: " );
				str.Append( basis.ToString( format ) );
				str.Append( " origin: " );
				str.Append( origin.ToString( format ) );
				return str.ToString();
			}
		}

		//--------------------------------------------------------------------------------------------------------------------

		public const float IKEpsilon = (1e-7f);
		public const float IKWritebackEpsilon = 0.01f;

		public static float Sqrt( float a )
		{
			if( a <= IKEpsilon ) { // Counts as 0
				return 0.0f;
			}

			return Mathf.Sqrt( a );
		}

		public static float _SafeAcos( float cos )
		{
			if( cos >= 1.0f - IKEpsilon ) {
				return 0.0f;
			}
			if( cos <= -1.0f + IKEpsilon ) {
				return 180.0f * Mathf.Deg2Rad;
			}

			return Mathf.Acos( cos );
		}

		public static float _SafeAsin( float sin )
		{
			if( sin >= 1.0f - IKEpsilon ) {
				return 90.0f * Mathf.Deg2Rad;
			}
			if( sin <= -1.0f + IKEpsilon ) {
				return -90.0f * Mathf.Deg2Rad;
			}

			return Mathf.Asin( sin );
		}

		public static bool IsFuzzy( float lhs, float rhs, float epsilon = IKEpsilon )
		{
			float t = lhs - rhs;
			return t >= -epsilon && t <= epsilon;
		}

		public static bool IsFuzzy( Vector3 lhs, Vector3 rhs, float epsilon = IKEpsilon )
		{
			float x = lhs.x - rhs.x;
			if( x >= -epsilon && x <= epsilon ) {
				x = lhs.y - rhs.y;
				if( x >= -epsilon && x <= epsilon ) {
					x = lhs.z - rhs.z;
					if( x >= -epsilon && x <= epsilon ) {
						return true;
					}
				}
			}

			return false;
		}

		public static bool IsFuzzy( ref Vector3 lhs, ref Vector3 rhs, float epsilon = IKEpsilon )
		{
			float x = lhs.x - rhs.x;
			if( x >= -epsilon && x <= epsilon ) {
				x = lhs.y - rhs.y;
				if( x >= -epsilon && x <= epsilon ) {
					x = lhs.z - rhs.z;
					if( x >= -epsilon && x <= epsilon ) {
						return true;
					}
				}
			}

			return false;
		}

		public static bool _IsNear( ref Vector3 lhs, ref Vector3 rhs )
		{
			return IsFuzzy( ref lhs, ref rhs, IKMoveEpsilon );
		}

		//--------------------------------------------------------------------------------------------------------------------

		public static Vector3 _Rotate( ref Vector3 dirX, ref Vector3 dirY, float cosR, float sinR )
		{
			return dirX * cosR + dirY * sinR;
		}

		public static Vector3 _Rotate( ref Vector3 dirX, ref Vector3 dirY, float r )
		{
			float cosR = Mathf.Cos( r );
			float sinR = Mathf.Sin( r );
			return dirX * cosR + dirY * sinR;
		}

		public static Vector3 _Rotate( ref Vector3 dirX, ref Vector3 dirY, ref FastAngle angle )
		{
			return dirX * angle.cos + dirY * angle.sin;
		}

		public static bool _NormalizedTranslate( ref Vector3 dir, ref Vector3 fr, ref Vector3 to )
		{
			Vector3 t = to - fr;
			float length = t.magnitude;
			if( length > IKEpsilon ) {
				dir = t * (1.0f / length);
				return true;
			}

			dir = Vector3.zero;
			return false;
		}

		public static bool _SafeNormalize( ref Vector3 dir )
		{
			float length = dir.magnitude;
			if( length > IKEpsilon ) {
				dir *= (1.0f / length);
				return true;
			}

			dir = Vector3.zero;
			return false;
		}

		public static bool _SafeNormalize( ref Vector3 dir0, ref Vector3 dir1 )
		{
			bool r0 = _SafeNormalize( ref dir0 );
			bool r1 = _SafeNormalize( ref dir1 );
			return r0 && r1;
		}

		public static bool _SafeNormalize( ref Vector3 dir0, ref Vector3 dir1, ref Vector3 dir2 )
		{
			bool r0 = _SafeNormalize( ref dir0 );
			bool r1 = _SafeNormalize( ref dir1 );
			bool r2 = _SafeNormalize( ref dir2 );
			return r0 && r1 && r2;
		}

		public static bool _SafeNormalize( ref Vector3 dir0, ref Vector3 dir1, ref Vector3 dir2, ref Vector3 dir3 )
		{
			bool r0 = _SafeNormalize( ref dir0 );
			bool r1 = _SafeNormalize( ref dir1 );
			bool r2 = _SafeNormalize( ref dir2 );
			bool r3 = _SafeNormalize( ref dir3 );
			return r0 && r1 && r2 && r3;
		}
		
		public static Quaternion Inverse( Quaternion q )
		{
			return new Quaternion( -q.x, -q.y, -q.z, q.w );
		}

		public static Quaternion Normalize( Quaternion q )
		{
			float lenSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
			if( lenSq > IKEpsilon ) {
				if( lenSq >= 1.0f - IKEpsilon && lenSq <= 1.0 + IKEpsilon ) {
					return q;
				} else {
					float s = 1.0f / Mathf.Sqrt( lenSq );
					return new Quaternion( q.x * s, q.y * s, q.z * s, q.w * s );
				}
			}

			return q; // Failsafe.
	    }

		public static bool SafeNormalize( ref Quaternion q )
		{
			float lenSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
			if( lenSq > IKEpsilon ) {
				if( lenSq >= 1.0f - IKEpsilon && lenSq <= 1.0 + IKEpsilon ) {
					return true;
				} else {
					float s = 1.0f / Mathf.Sqrt( lenSq );
					q.x *= s;
					q.y *= s;
					q.z *= s;
					q.w *= s;
					return true;
				}
			}

			return false;
		}

		public static bool IsFuzzyIdentity( Quaternion q )
		{
			return IsFuzzyIdentity( ref q );
		}

		public static bool IsFuzzyIdentity( ref Quaternion q )
		{
			return
				q.x >= -IKEpsilon && q.x <= IKEpsilon &&
				q.y >= -IKEpsilon && q.y <= IKEpsilon &&
				q.z >= -IKEpsilon && q.z <= IKEpsilon &&
				q.w >= 1.0f - IKEpsilon && q.w <= 1.0f + IKEpsilon;
		}

		//--------------------------------------------------------------------------------------------------------------------

		[System.Serializable]
		public struct FastLength
		{
			[SerializeField]
			float _length;
			[SerializeField]
			float _lengthSq;
			[SerializeField]
			int _flags;

			public float length
			{
				get {
					if( (_flags & 0x01) == 0 ) {
						_flags |= 0x01;
						if( (_flags & 0x02) != 0 ) {
							_length = (_lengthSq > IKEpsilon) ? Mathf.Sqrt( _lengthSq ) : 0.0f;
						}
					}

					return _length;
				}
			}

			public float lengthSq
			{
				get {
					if( (_flags & 0x02) == 0 ) {
						_flags |= 0x02;
						if( (_flags & 0x01) != 0 ) {
							_lengthSq = _length * _length;
						}
					}

					return _lengthSq;
				}
			}

			FastLength( float length_, float lengthSq_, int flags_ )
			{
				_length = length_;
				_lengthSq = lengthSq_;
				_flags = flags_;
			}

			public static implicit operator float( FastLength fasLength )
			{
				return fasLength.length;
			}

			public static FastLength FromLength( float length )
			{
				return new FastLength( length, 0.0f, 0x01 );
			}

			public static FastLength FromLengthSq( float lengthSq )
			{
				return new FastLength( 0.0f, lengthSq, 0x02 );
			}

			public static FastLength FromVector3( Vector3 v )
			{
				return FromLengthSq( v.sqrMagnitude );
			}

			public static bool operator<( FastLength a, FastLength b )
			{
				return a.lengthSq < b.lengthSq;
			}

			public static bool operator >( FastLength a, FastLength b )
			{
				return a.lengthSq > b.lengthSq;
			}

			public static bool operator<=( FastLength a, FastLength b )
			{
				return a.lengthSq <= b.lengthSq;
			}

			public static bool operator >=( FastLength a, FastLength b )
			{
				return a.lengthSq >= b.lengthSq;
			}
		}

		[System.Serializable]
		public struct FastAngle
		{
			public float angle;
			public float cos;
			public float sin;

			public static readonly FastAngle zero = new FastAngle( 0.0f, 1.0f, 0.0f );

			public FastAngle( float angle_ )
			{
				angle = angle_;
				cos = Mathf.Cos( angle_ );
				sin = Mathf.Sin( angle_ );
			}

			public FastAngle( float angle_, float cos_, float sin_ )
			{
				angle = angle_;
				cos = cos_;
				sin = sin_;
			}

			public void Reset()
			{
				angle = 0.0f;
				cos = 1.0f;
				sin = 0.0f;
			}

			public void Reset( float angle_ )
			{
				angle = angle_;
				cos = Mathf.Cos( angle_ );
				sin = Mathf.Sin( angle_ );
			}

			public void Reset( float angle_, float cos_, float sin_ )
			{
				angle = angle_;
				cos = cos_;
				sin = sin_;
			}
		}

		//--------------------------------------------------------------------------------------------------------------------

		public static bool _ComputeBasisFromXZLockX( out Matrix3x3 basis, Vector3 dirX, Vector3 dirZ )
		{
			return _ComputeBasisFromXZLockX( out basis, ref dirX, ref dirZ );
		}

		public static bool _ComputeBasisFromXZLockX( out Matrix3x3 basis, ref Vector3 dirX, ref Vector3 dirZ )
		{
			CheckNormalized( dirX );
			Vector3 baseY = Vector3.Cross( dirZ, dirX );
			Vector3 baseZ = Vector3.Cross( dirX, baseY );
			if( _SafeNormalize( ref baseY, ref baseZ ) ) {
				basis = Matrix3x3.FromColumn( ref dirX, ref baseY, ref baseZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		public static bool _ComputeBasisFromXYLockX( out Matrix3x3 basis, Vector3 dirX, Vector3 dirY )
		{
			return _ComputeBasisFromXYLockX( out basis, ref dirX, ref dirY );
		}

		public static bool _ComputeBasisFromXYLockX( out Matrix3x3 basis, ref Vector3 dirX, ref Vector3 dirY )
		{
			CheckNormalized( dirX );
			Vector3 baseZ = Vector3.Cross( dirX, dirY );
			Vector3 baseY = Vector3.Cross( baseZ, dirX );
			if( _SafeNormalize( ref baseY, ref baseZ ) ) {
				basis = Matrix3x3.FromColumn( ref dirX, ref baseY, ref baseZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		public static bool _ComputeBasisFromXYLockY( out Matrix3x3 basis, Vector3 dirX, Vector3 dirY )
		{
			return _ComputeBasisFromXYLockY( out basis, ref dirX, ref dirY );
		}

		public static bool _ComputeBasisFromXYLockY( out Matrix3x3 basis, ref Vector3 dirX, ref Vector3 dirY )
		{
			CheckNormalized( dirY );
			Vector3 baseZ = Vector3.Cross( dirX, dirY );
			Vector3 baseX = Vector3.Cross( dirY, baseZ );
			if( _SafeNormalize( ref baseX, ref baseZ ) ) {
				basis = Matrix3x3.FromColumn( ref baseX, ref dirY, ref baseZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		public static bool _ComputeBasisFromXZLockZ( out Matrix3x3 basis, Vector3 dirX, Vector3 dirZ )
		{
			return _ComputeBasisFromXZLockZ( out basis, ref dirX, ref dirZ );
		}

		public static bool _ComputeBasisFromXZLockZ( out Matrix3x3 basis, ref Vector3 dirX, ref Vector3 dirZ )
		{
			CheckNormalized( dirZ );
			Vector3 baseY = Vector3.Cross( dirZ, dirX );
			Vector3 baseX = Vector3.Cross( baseY, dirZ );
			if( _SafeNormalize( ref baseX, ref baseY ) ) {
				basis = Matrix3x3.FromColumn( ref baseX, ref baseY, ref dirZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		public static bool _ComputeBasisFromYZLockY( out Matrix3x3 basis, Vector3 dirY, Vector3 dirZ )
		{
			return _ComputeBasisFromYZLockY( out basis, ref dirY, ref dirZ );
        }

		public static bool _ComputeBasisFromYZLockY( out Matrix3x3 basis, ref Vector3 dirY, ref Vector3 dirZ )
		{
			CheckNormalized( dirY );
			Vector3 baseX = Vector3.Cross( dirY, dirZ );
			Vector3 baseZ = Vector3.Cross( baseX, dirY );
			if( _SafeNormalize( ref baseX, ref baseZ ) ) {
				basis = Matrix3x3.FromColumn( ref baseX, ref dirY, ref baseZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		public static bool _ComputeBasisFromYZLockZ( out Matrix3x3 basis, Vector3 dirY, Vector3 dirZ )
		{
			return _ComputeBasisFromYZLockZ( out basis, ref dirY, ref dirZ );
        }

		public static bool _ComputeBasisFromYZLockZ( out Matrix3x3 basis, ref Vector3 dirY, ref Vector3 dirZ )
		{
			CheckNormalized( dirZ );
			Vector3 baseX = Vector3.Cross( dirY, dirZ );
			Vector3 baseY = Vector3.Cross( dirZ, baseX );
			if( _SafeNormalize( ref baseX, ref baseY ) ) {
				basis = Matrix3x3.FromColumn( ref baseX, ref baseY, ref dirZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		//--------------------------------------------------------------------------------------------------------------------

		public static bool _ComputeBasisLockX( out Matrix3x3 basis, ref Vector3 dirX, ref Vector3 dirY, ref Vector3 dirZ )
		{
			Matrix3x3 basisY;
			Matrix3x3 basisZ;
			bool solveY = _ComputeBasisFromXYLockX( out basisY, ref dirX, ref dirY );
			bool solveZ = _ComputeBasisFromXZLockX( out basisZ, ref dirX, ref dirZ );
			if( solveY && solveZ ) {
				float nearY = Mathf.Abs( Vector3.Dot( dirX, dirY ) );
				float nearZ = Mathf.Abs( Vector3.Dot( dirX, dirZ ) );
				if( nearZ <= IKEpsilon ) {
					basis = basisZ;
					return true;
				} else if( nearY <= IKEpsilon ) {
					basis = basisY;
					return true;
				} else {
					_Lerp( out basis, ref basisY, ref basisZ, nearY / (nearY + nearZ) );
					return true;
				}
			} else if( solveY ) {
				basis = basisY;
				return true;
			} else if( solveZ ) {
				basis = basisZ;
				return true;
			}

			basis = Matrix3x3.identity;
			return false;
		}

		public static bool _ComputeBasisLockY( out Matrix3x3 basis, ref Vector3 dirX, ref Vector3 dirY, ref Vector3 dirZ )
		{
			Matrix3x3 basisX;
			Matrix3x3 basisZ;
			bool solveX = _ComputeBasisFromXYLockY( out basisX, ref dirX, ref dirY );
			bool solveZ = _ComputeBasisFromYZLockY( out basisZ, ref dirY, ref dirZ );
			if( solveX && solveZ ) {
				float nearX = Mathf.Abs( Vector3.Dot( dirY, dirX ) );
				float nearZ = Mathf.Abs( Vector3.Dot( dirY, dirZ ) );
				if( nearZ <= IKEpsilon ) {
					basis = basisZ;
					return true;
				} else if( nearX <= IKEpsilon ) {
					basis = basisX;
					return true;
				} else {
					_Lerp( out basis, ref basisX, ref basisZ, nearX / (nearX + nearZ) );
					return true;
				}
			} else if( solveX ) {
				basis = basisX;
				return true;
			} else if( solveZ ) {
				basis = basisZ;
				return true;
			}

			basis = Matrix3x3.identity;
			return false;
		}

		public static bool _ComputeBasisLockZ( out Matrix3x3 basis, ref Vector3 dirX, ref Vector3 dirY, ref Vector3 dirZ )
		{
			Matrix3x3 basisX;
			Matrix3x3 basisY;
			bool solveX = _ComputeBasisFromXZLockZ( out basisX, ref dirX, ref dirZ );
			bool solveY = _ComputeBasisFromYZLockZ( out basisY, ref dirY, ref dirZ );
			if( solveX && solveY ) {
				float nearX = Mathf.Abs( Vector3.Dot( dirZ, dirX ) );
				float nearY = Mathf.Abs( Vector3.Dot( dirZ, dirY ) );
				if( nearY <= IKEpsilon ) {
					basis = basisY;
					return true;
				} else if( nearX <= IKEpsilon ) {
					basis = basisX;
					return true;
				} else {
					_Lerp( out basis, ref basisX, ref basisY, nearX / (nearX + nearY) );
					return true;
				}
			} else if( solveX ) {
				basis = basisX;
				return true;
			} else if( solveY ) {
				basis = basisY;
				return true;
			}

			basis = Matrix3x3.identity;
			return false;
		}

		//--------------------------------------------------------------------------------------------------------------------

		public static bool _ComputeBasisFrom( out Matrix3x3 basis, ref Matrix3x3 rootBasis, ref Vector3 dir, _DirectionAs directionAs )
		{
			CheckNormalized( dir );

			switch( directionAs ) {
			case _DirectionAs.XPlus:
				return _ComputeBasisFromXYLockX( out basis, dir, rootBasis.column1 );
			case _DirectionAs.XMinus:
				return _ComputeBasisFromXYLockX( out basis, -dir, rootBasis.column1 );
			case _DirectionAs.YPlus:
				return _ComputeBasisFromXYLockY( out basis, rootBasis.column0, dir );
			case _DirectionAs.YMinus:
				return _ComputeBasisFromXYLockY( out basis, rootBasis.column0, -dir );
			}

			basis = Matrix3x3.identity;
			return false;
		}

		#if false
		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeBasisFrom( ref Matrix3x3 basis, ref Vector3 dir, _DirectionAs directionAs )
		{
			Vector3 dirM = -dir;
			switch( directionAs ) {
			case _DirectionAs.XPlus:
				return _ComputeBasisFromX_Z( ref basis, ref dir );
			case _DirectionAs.XMinus:
				return _ComputeBasisFromX_Z( ref basis, ref dirM );
			case _DirectionAs.YPlus:
				return _ComputeBasisFromY_Z( ref basis, ref dir );
			case _DirectionAs.YMinus:
				return _ComputeBasisFromY_Z( ref basis, ref dirM );
			}

			basis = Matrix3x3.identity;
			return false;
		}
		#endif

		#if false
		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeBasis2From( ref Matrix3x3 basis, ref Vector3 dir, _DirectionAs directionAs )
		{
			Vector3 dirM = -dir;
			switch( directionAs ) {
			case _DirectionAs.XPlus:
				return _ComputeBasisFromX_Y( ref basis, ref dir );
			case _DirectionAs.XMinus:
				return _ComputeBasisFromX_Y( ref basis, ref dirM );
			case _DirectionAs.YPlus:
				return _ComputeBasisFromY_X( ref basis, ref dir );
			case _DirectionAs.YMinus:
				return _ComputeBasisFromY_X( ref basis, ref dirM );
			}

			basis = Matrix3x3.identity;
			return false;
		}
		#endif

		#if false
		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeBasisFromX_Z( ref Matrix3x3 basis, ref Vector3 dirX )
		{
			// Memo: Z+ = front.
			Vector3 dirY = Vector3.Cross( new Vector3( 0.0f, 0.0f, 1.0f ), dirX );
			Vector3 dirZ = Vector3.Cross( dirX, dirY );
			dirY = Vector3.Cross( dirZ, dirX );
			if( _SafeNormalize( ref dirY ) && _SafeNormalize( ref dirZ ) ) {
				basis.SetColumn( ref dirX, ref dirY, ref dirZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeBasisFromX_Y( ref Matrix3x3 basis, ref Vector3 dirX )
		{
			// Memo: Y+ = up.
			Vector3 dirY = new Vector3( 0.0f, 1.0f, 0.0f );
			Vector3 dirZ = Vector3.Cross( dirX, dirY );
			dirY = Vector3.Cross( dirZ, dirX );
			if( _SafeNormalize( ref dirY ) && _SafeNormalize( ref dirZ ) ) {
				basis.SetColumn( ref dirX, ref dirY, ref dirZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeBasisFromX_Z( ref Matrix3x3 boneToSolvedBasis, ref Matrix3x3 solvedToBoneBasis, ref Vector3 dirX )
		{
			if( _ComputeBasisFromX_Z( ref boneToSolvedBasis, ref dirX ) ) {
				solvedToBoneBasis = boneToSolvedBasis.transpose;
				return true;
			}

			solvedToBoneBasis = Matrix3x3.identity;
			return false;
		}

		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeSolvedToBoneBasisFromX_Z( ref Matrix3x3 basis, ref Vector3 dirX )
		{
			if( _ComputeBasisFromX_Z( ref basis, ref dirX ) ) {
				basis = basis.transpose;
				return true;
			}

			return false;
		}

		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeBasisFromY_Z( ref Matrix3x3 basis, ref Vector3 dirY )
		{
			// Memo: Z+ = front.
			Vector3 dirX = Vector3.Cross( dirY, new Vector3( 0.0f, 0.0f, 1.0f ) );
			Vector3 dirZ = Vector3.Cross( dirX, dirY );
			dirX = Vector3.Cross( dirY, dirZ );
			if( _SafeNormalize( ref dirX ) && _SafeNormalize( ref dirZ ) ) {
				basis.SetColumn( ref dirX, ref dirY, ref dirZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeBasisFromY_X( ref Matrix3x3 basis, ref Vector3 dirY )
		{
			// Memo: X+ = right.
			Vector3 dirX = new Vector3( 1.0f, 0.0f, 0.0f );
			Vector3 dirZ = Vector3.Cross( dirX, dirY );
			dirX = Vector3.Cross( dirY, dirZ );
			if( _SafeNormalize( ref dirX ) && _SafeNormalize( ref dirZ ) ) {
				basis.SetColumn( ref dirX, ref dirY, ref dirZ );
				return true;
			} else {
				basis = Matrix3x3.identity;
				return false;
			}
		}

		// Legacy.(Unstable function, not used released version.)
		public static bool _ComputeSolvedToBoneBasisFromY_Z( ref Matrix3x3 basis, ref Vector3 dirY )
		{
			if( _ComputeBasisFromY_Z( ref basis, ref dirY ) ) {
				basis = basis.transpose;
				return true;
			}

			return false;
		}

		#endif

		//--------------------------------------------------------------------------------------------------------------------

		public static void _LerpRotateBasis( out Matrix3x3 basis, ref Vector3 axis, float cos, float rate )
		{
			if( rate <= 1.0f - IKEpsilon ) {
				if( rate <= IKEpsilon ) {
					basis = Matrix3x3.identity;
					return;
				}

				float acos = (cos >= 1.0f - IKEpsilon) ? 0.0f : ((cos <= -1.0f - IKEpsilon) ? (180.0f * Mathf.Deg2Rad) : (float)System.Math.Acos( (float)cos ));
				cos = (float)System.Math.Cos( (float)(acos * rate) );
			}

			basis = new Matrix3x3( ref axis, cos );
		}

		public static void _LerpRoateBasis(
			out Matrix3x3 basis,
			ref Vector3 axis0, float cos0, float feedbackRate0,
			ref Vector3 axis1, float cos1, float feedbackRate1,
			float lerpRate )
		{
			if( lerpRate <= IKEpsilon || feedbackRate1 <= IKEpsilon ) {
				_LerpRotateBasis( out basis, ref axis0, cos0, feedbackRate0 );
				return;
			} else if( lerpRate >= 1.0f - IKEpsilon || feedbackRate0 <= IKEpsilon ) {
				_LerpRotateBasis( out basis, ref axis1, cos1, feedbackRate1 );
				return;
			}

			if( feedbackRate0 <= 1.0f - IKEpsilon ) {
				float acos0 = (cos0 >= 1.0f - IKEpsilon) ? 0.0f : ((cos0 <= -1.0f - IKEpsilon) ? (180.0f * Mathf.Deg2Rad) : (float)System.Math.Acos( (float)cos0 ));
				cos0 = (float)System.Math.Cos( (float)(acos0 * feedbackRate0) );
			}
			if( feedbackRate1 <= 1.0f - IKEpsilon ) {
				float acos1 = (cos1 >= 1.0f - IKEpsilon) ? 0.0f : ((cos1 <= -1.0f - IKEpsilon) ? (180.0f * Mathf.Deg2Rad) : (float)System.Math.Acos( (float)cos1 ));
				cos1 = (float)System.Math.Cos( (float)(acos1 * feedbackRate1) );
			}

			float sin0 = 1.0f - cos0 * cos0;
			float sin1 = 1.0f - cos1 * cos1;
			sin0 = (sin0 < IKEpsilon) ? 0.0f : ((sin0 > 1.0f - IKEpsilon) ? 1.0f : (float)System.Math.Sqrt( (float)sin0 ));
			sin1 = (sin1 < IKEpsilon) ? 0.0f : ((sin1 > 1.0f - IKEpsilon) ? 1.0f : (float)System.Math.Sqrt( (float)sin1 ));

			float axis0_x_sin = axis0.x * sin0;
			float axis0_y_sin = axis0.y * sin0;
			float axis0_z_sin = axis0.z * sin0;

			float axis1_x_sin = axis1.x * sin1;
			float axis1_y_sin = axis1.y * sin1;
			float axis1_z_sin = axis1.z * sin1;

			Vector3 dirX0 = new Vector3( cos0, axis0_z_sin, -axis0_y_sin );
			Vector3 dirY0 = new Vector3( -axis0_z_sin, cos0, axis0_x_sin );

			Vector3 dirX1 = new Vector3( cos1, axis1_z_sin, -axis1_y_sin );
			Vector3 dirY1 = new Vector3( -axis1_z_sin, cos1, axis1_x_sin );

			float cos0I = 1.0f - cos0;
			float cos1I = 1.0f - cos1;

			float axis0_x_cosI = axis0.x * cos0I;
			float axis0_y_cosI = axis0.y * cos0I;

			float axis1_x_cosI = axis1.x * cos1I;
			float axis1_y_cosI = axis1.y * cos1I;

			dirX0.x += axis0.x * axis0_x_cosI;
			dirX0.y += axis0.y * axis0_x_cosI;
			dirX0.z += axis0.z * axis0_x_cosI;

			dirY0.x += axis0.x * axis0_y_cosI;
			dirY0.y += axis0.y * axis0_y_cosI;
			dirY0.z += axis0.z * axis0_y_cosI;

			dirX1.x += axis1.x * axis1_x_cosI;
			dirX1.y += axis1.y * axis1_x_cosI;
			dirX1.z += axis1.z * axis1_x_cosI;

			dirY1.x += axis1.x * axis1_y_cosI;
			dirY1.y += axis1.y * axis1_y_cosI;
			dirY1.z += axis1.z * axis1_y_cosI;

			Vector3 dirX = Vector3.Lerp( dirX0, dirX1, lerpRate );
			Vector3 dirY = Vector3.Lerp( dirY0, dirY1, lerpRate );
			Vector3 dirZ = Vector3.Cross( dirX, dirY );
			dirX = Vector3.Cross( dirY, dirZ );

			if( _SafeNormalize( ref dirX ) && _SafeNormalize( ref dirY ) && _SafeNormalize( ref dirZ ) ) {
				basis = Matrix3x3.FromColumn( ref dirX, ref dirY, ref dirZ );
			} else {
				basis = Matrix3x3.identity;
			}
		}

		// Stabler for over angle rotation.
		public static void _LerpRoateBasis2(
			out Matrix3x3 basis,
			ref Vector3 axis0, float cosTheta0, float feedbackRate0,
			ref Vector3 axis1, float cosTheta1, float feedbackRate1,
			float lerpRate )
		{
			Matrix3x3 tempBasis;
			_LerpRoateBasis( out tempBasis,
				ref axis0, cosTheta0, feedbackRate0 * 0.5f,
				ref axis1, cosTheta1, feedbackRate1 * 0.5f, lerpRate );
			basis = tempBasis * tempBasis;
		}

		#if false
		// Borken!!! for Use Matrix3x3( axis, theta )
		public static void _RotateBasis( out Matrix3x3 basis, ref Vector3 axis, float cosTheta, float rate )
		{
			Vector3 dirX, dirY;
			_RotateBasisXY( out dirX, out dirY, ref axis, cosTheta, rate );
			
			Vector3 dirZ = Vector3.Cross( dirX, dirY );
			dirX = Vector3.Cross( dirY, dirX );

			if( _SafeNormalize( ref dirX ) && _SafeNormalize( ref dirY ) && _SafeNormalize( ref dirZ ) ) {
				basis = new Matrix3x3();
				basis.SetColumn( dirX, dirY, dirZ );
			} else {
				basis = Matrix3x3.identity;
			}
		}

		// Borken!!! for Use Matrix3x3( axis, theta )
		public static void _RotateBasisXY( out Vector3 dirX, out Vector3 dirY, ref Vector3 axis, float cosTheta, float rate )
		{
			dirX = new Vector3( 1.0f, 0.0f, 0.0f );
			dirY = new Vector3( 0.0f, 1.0f, 0.0f );
			_RotateAxisTheta( ref dirX, ref axis, cosTheta );
			_RotateAxisTheta( ref dirY, ref axis, cosTheta );
			if( rate < 1.0f - IKEpsilon ) {
				dirX = Vector3.Lerp( dirX, new Vector3( 1.0f, 0.0f, 0.0f ), rate );
				dirY = Vector3.Lerp( dirY, new Vector3( 0.0f, 1.0f, 0.0f ), rate );
			}
		}
		#endif

		public static void _RotateAxisTheta( ref Vector3 p, ref Vector3 axis, float cosTheta )
		{
			Vector3 pV = axis * Vector3.Dot( axis, p );
			float sinTheta = Mathf.Sqrt( Mathf.Clamp01( 1.0f - cosTheta * cosTheta ) );
			p = p * cosTheta + Vector3.Cross( axis, p * sinTheta ) + pV * (1.0f - cosTheta);
		}

		//--------------------------------------------------------------------------------------------------------------------

		public static void _LerpToIdentity( ref Matrix3x3 basis, float t )
		{
			Matrix3x3 to = Matrix3x3.identity;
			_Lerp( out basis, ref basis, ref to, t );
		}

		public static void _Lerp( out Matrix3x3 basis, ref Matrix3x3 fr, ref Matrix3x3 to, float t )
		{
			if( t <= IKEpsilon ) {
				basis = fr;
				return;
			} else if( t >= 1.0f - IKEpsilon ) {
				basis = to;
				return;
			}

			Vector3 x = fr.column0;
			Vector3 y = fr.column1;
			x = x + (to.column0 - x) * t;
			y = y + (to.column1 - y) * t;

			Vector3 z = Vector3.Cross( x, y );
			x = Vector3.Cross( y, z );

			if( _SafeNormalize( ref x, ref y, ref z ) ) {
				basis = Matrix3x3.FromColumn( ref x, ref y, ref z );
			} else {
				basis = fr;
			}
		}

		//--------------------------------------------------------------------------------------------------------------------

		// Trigonometry
		public static float ComputeCosTheta(
			float lenASq,
			float lenBSq,
			float lenCSq,
			float lenB,
			float lenC )
		{
			float bc2 = lenB * lenC * 2.0f;
			if( bc2 > IKEpsilon ) {
				return (lenBSq + lenCSq - lenASq) / bc2;
			}

			return 1.0f;
		}

		// Trigonometry to A
		public static float ComputeCosTheta(
			FastLength lenA,
			FastLength lenB,
			FastLength lenC )
		{
			float bc2 = lenB.length * lenC.length * 2.0f;
			if( bc2 > IKEpsilon ) {
				return (lenB.lengthSq + lenC.lengthSq - lenA.lengthSq) / bc2;
			}

			return new FastLength();
		}

		// Trigonometry
		public static float ComputeSinTheta(
			float lenASq,
			float lenBSq,
			float lenCSq,
			float lenB,
			float lenC )
		{
			float bc2 = lenB * lenC * 2.0f;
			if( bc2 > IKEpsilon ) {
				float cs = (lenBSq + lenCSq - lenASq) / bc2;
				return Sqrt( Mathf.Clamp01( 1.0f - cs * cs ) );
			}

			return 0.0f;
		}

		// Trigonometry to A
		public static float ComputeSinTheta(
			FastLength lenA,
			FastLength lenB,
			FastLength lenC )
		{
			float bc2 = lenB.length * lenC.length * 2.0f;
			if( bc2 > IKEpsilon ) {
				float cs = (lenB.lengthSq + lenC.lengthSq - lenA.lengthSq) / bc2;
				return Sqrt( Mathf.Clamp01( 1.0f - cs * cs ) );
			}

			return new FastLength();
		}

		//--------------------------------------------------------------------------------------------------------------------

#if false // Legacy.
		public static bool _ComputeBasisFromTo(
			ref Matrix3x3 basis,
			ref Vector3 fr,
			ref Vector3 to )
		{
			float d = Vector3.Dot( fr, to );
			if( d < 1.0f - IKEpsilon ) {
				Vector3 c = Vector3.Cross( fr, to );
				if( _SafeNormalize( ref c ) ) {
					Quaternion q = Quaternion.AngleAxis( Mathf.Acos( d ) * Mathf.Rad2Deg, c );
					basis = new Matrix3x3( q );
					return true;
				}
			}

			basis = Matrix3x3.identity;
			return false;
		}

		public static bool _ComputeBasisFromTo(
			ref Matrix3x3 basis,
			ref Vector3 fr,
			ref Vector3 to,
			float r )
		{
			if( r > IKEpsilon ) {
				float d = Vector3.Dot( fr, to );
				if( d < 1.0f - IKEpsilon ) {
					Vector3 c = Vector3.Cross( fr, to );
					if( _SafeNormalize( ref c ) ) {
						Quaternion q = Quaternion.AngleAxis( Mathf.Acos( d ) * r * Mathf.Rad2Deg, c );
						basis = new Matrix3x3( q );
						return true;
					}
				}
			}

			basis = Matrix3x3.identity;
			return false;
		}
#endif

		//--------------------------------------------------------------------------------------------------------------------

		// Limb.(Return to dir.)
		public static bool _SolveLimb(
			out Vector3 solvedBeginToBendingDir,
			out Vector3 solvedBendingToEndDir,
			ref Vector3 beginPos,
			ref Vector3 bendingPos,
			ref Vector3 endPos,
			ref FastLength beginToBendingLength,
			ref FastLength bendingToEndLength )
		{
			return _SolveLimb(
				out solvedBeginToBendingDir,
				out solvedBendingToEndDir,
				ref beginPos,
				ref bendingPos,
				ref endPos,
				beginToBendingLength.length,
				beginToBendingLength.lengthSq,
				bendingToEndLength.length,
				bendingToEndLength.lengthSq );
		}

		public static bool _SolveLimb(
			out Vector3 solvedBeginToBendingDir,
			out Vector3 solvedBendingToEndDir,
			ref Vector3 beginPos,
			ref Vector3 bendingPos,
			ref Vector3 endPos,
			float beginToBendingLength,
			float beginToBendingLengthSq,
			float bendingToEndLength,
			float bendingToEndLengthSq )
		{
			solvedBeginToBendingDir = Vector3.zero;
			solvedBendingToEndDir = Vector3.zero;

			Vector3 effectorDir = endPos - beginPos;
			float effectorLen = effectorDir.magnitude;
			if( effectorLen <= IKEpsilon ) {
				return false;
			}

			effectorDir *= 1.0f / effectorLen;

			bool isSolved = false;

			{
				Vector3 beginToBendingTrans = bendingPos - beginPos;
				Vector3 intersectBendingTrans = beginToBendingTrans - effectorDir * Vector3.Dot( effectorDir, beginToBendingTrans );
				float intersectBendingLen = intersectBendingTrans.magnitude;

				if( intersectBendingLen > IKEpsilon ) {
					Vector3 intersectBendingDir = intersectBendingTrans * (1.0f / intersectBendingLen);

					float bc2 = 2.0f * beginToBendingLength * effectorLen;
					if( bc2 > IKEpsilon ) {
						float effectorCosTheta = (beginToBendingLengthSq + effectorLen * effectorLen - bendingToEndLengthSq) / bc2;
						float effectorSinTheta = Sqrt( Mathf.Clamp01( 1.0f - effectorCosTheta * effectorCosTheta ) );

						Vector3 beginToInterTranslate = effectorDir * effectorCosTheta * beginToBendingLength
														+ intersectBendingDir * effectorSinTheta * beginToBendingLength;
						Vector3 interToEndTranslate = endPos - (beginPos + beginToInterTranslate);

						if( _SafeNormalize( ref beginToInterTranslate ) && _SafeNormalize( ref interToEndTranslate ) ) {
							isSolved = true;
							solvedBeginToBendingDir = beginToInterTranslate;
							solvedBendingToEndDir = interToEndTranslate;
						}
					}
				}
			}

			if( !isSolved ) { // Failsafe.
				Vector3 bendingDir = bendingPos - beginPos;
				if( _SafeNormalize( ref bendingDir ) ) {
					Vector3 interPos = beginPos + bendingDir * beginToBendingLength;
					Vector3 endDir = endPos - interPos;
					if( _SafeNormalize( ref endDir ) ) {
						isSolved = true;
						solvedBeginToBendingDir = bendingDir;
						solvedBendingToEndDir = endDir;
					}
				}
			}

			return isSolved;
		}

		//--------------------------------------------------------------------------------------------------------------------

#if true // Legacy.
		// Limb.(Return to translate)
		public static bool _SolveLimbPosBased(
			out Vector3 solvedBeginToBending,
			out Vector3 solvedBendingToEnd,
			ref Vector3 beginPos,
			ref Vector3 bendingPos,
			ref Vector3 endPos,
			ref FastLength beginToBendingLength,
			ref FastLength bendingToEndLength )
		{
			return _SolveLimbPosBased(
				out solvedBeginToBending,
				out solvedBendingToEnd,
				ref beginPos,
				ref bendingPos,
				ref endPos,
				beginToBendingLength.length,
				beginToBendingLength.lengthSq,
				bendingToEndLength.length,
				bendingToEndLength.lengthSq );
		}

		public static bool _SolveLimbPosBased(
			out Vector3 solvedBeginToBending,
			out Vector3 solvedBendingToEnd,
			ref Vector3 beginPos,
			ref Vector3 bendingPos,
			ref Vector3 endPos,
			float beginToBendingLength,
			float beginToBendingLengthSq,
			float bendingToEndLength,
			float bendingToEndLengthSq )
		{
			solvedBeginToBending = Vector3.zero;
			solvedBendingToEnd = Vector3.zero;

			Vector3 effectorDir = endPos - beginPos;
			float effectorLen = effectorDir.magnitude;
			if( effectorLen <= IKEpsilon ) {
				return false;
			}

			effectorDir *= 1.0f / effectorLen;

			bool isSolved = false;

			{
				Vector3 beginToBendingTrans = bendingPos - beginPos;
				Vector3 intersectBendingTrans = beginToBendingTrans - effectorDir * Vector3.Dot( effectorDir, beginToBendingTrans );
				float intersectBendingLen = intersectBendingTrans.magnitude;

				if( intersectBendingLen > IKEpsilon ) {
					Vector3 intersectBendingDir = intersectBendingTrans * (1.0f / intersectBendingLen);

					float bc2 = 2.0f * beginToBendingLength * effectorLen;
					if( bc2 > IKEpsilon ) {
						float effectorCosTheta = (beginToBendingLengthSq + effectorLen * effectorLen - bendingToEndLengthSq) / bc2;
						float effectorSinTheta = Sqrt( Mathf.Clamp01( 1.0f - effectorCosTheta * effectorCosTheta ) );

						Vector3 beginToInterTranslate = effectorDir * effectorCosTheta * beginToBendingLength
														+ intersectBendingDir * effectorSinTheta * beginToBendingLength;
						Vector3 interToEndTranslate = endPos - (beginPos + beginToInterTranslate);

						isSolved = true;
						solvedBeginToBending = beginToInterTranslate;
						solvedBendingToEnd = interToEndTranslate;
					}
				}
			}

			if( !isSolved ) { // Failsafe.
				Vector3 bendingDir = bendingPos - beginPos;
				float bendingDirLen = bendingDir.magnitude;
				if( bendingDirLen > IKEpsilon ) {
					bendingDir *= (beginToBendingLength / bendingDirLen);
					Vector3 interPos = beginPos + bendingDir;
					Vector3 endDir = endPos - interPos;
					float endDirLen = endDir.magnitude;
					if( endDirLen > IKEpsilon ) {
						isSolved = true;
						endDir *= (bendingToEndLength / endDirLen);
						solvedBeginToBending = bendingDir;
						solvedBendingToEnd = endDir;
					}
				}
			}

			return isSolved;
		}
#endif
	
		//--------------------------------------------------------------------------------------------------------------------

		static bool _ComputeThetaAxis(
			ref Vector3 origPos,
			ref Vector3 fromPos,
			ref Vector3 toPos,
			out float theta,
			out Vector3 axis )
		{
			Vector3 dirFrom = fromPos - origPos;
			Vector3 dirTo = toPos - origPos;
			if( !_SafeNormalize( ref dirFrom ) || !_SafeNormalize( ref dirTo ) ) {
				theta = 0.0f;
				axis = new Vector3( 0.0f, 0.0f, 1.0f );
				return false;
			}

			return _ComputeThetaAxis( ref dirFrom, ref dirTo, out theta, out axis );
		}

		static bool _ComputeThetaAxis(
			ref Vector3 dirFrom,
			ref Vector3 dirTo,
			out float theta,
			out Vector3 axis )
		{
			axis = Vector3.Cross( dirFrom, dirTo );
			if( !_SafeNormalize( ref axis ) ) {
				theta = 0.0f;
				axis = new Vector3( 0.0f, 0.0f, 1.0f );
				return false;
			}

			theta = Vector3.Dot( dirFrom, dirTo );
			return true;
		}

		//static float _computeThetaMin = Mathf.Sin( 0.25f * Mathf.Deg2Rad );

		//--------------------------------------------------------------------------------------------------------------------

		public static Vector3 Scale( Vector3 lhs, Vector3 rhs )
		{
			return new Vector3( lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z );
		}

		public static Vector3 Scale( ref Vector3 lhs, ref Vector3 rhs )
		{
			return new Vector3( lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z );
		}

		//--------------------------------------------------------------------------------------------------------------------

		// Limited Square.
		public static bool _LimitXY_Square(
			ref Vector3 dir,                // dirZ
			float limitXMinus,               // X-
			float limitXPlus,                // X+
			float limitYMinus,               // Z-
			float limitYPlus )               // Z+
		{
			bool isXLimited = false;
			bool isYLimited = false;

			if( dir.x < -limitXMinus ) {
				dir.x = -limitXMinus;
				isXLimited = true;
			} else if( dir.x > limitXPlus ) {
				dir.x = limitXPlus;
				isXLimited = true;
			}

			if( dir.y < -limitYMinus ) {
				dir.y = -limitYMinus;
				isYLimited = true;
			} else if( dir.y > limitYPlus ) {
				dir.y = limitYPlus;
				isYLimited = true;
			}

			if( isXLimited || isYLimited ) {
				dir.z = Sqrt( 1.0f - (dir.x * dir.x + dir.y * dir.y) );
				return true;
			} else {
				if( dir.z < 0.0f ) {
					dir.z = -dir.z;
					return true;
                }
			}

			return false;
		}

		//--------------------------------------------------------------------------------------------------------------------

		public static bool _LimitXY(
			ref Vector3 dir,				// dirZ
			float limitXMinus,				// X-
			float limitXPlus,				// X+
			float limitYMinus,				// Z-
			float limitYPlus )				// Z+
		{
			bool isXPlus = (dir.x >= 0.0f);
			bool isYPlus = (dir.y >= 0.0f);
			float xLimit = isXPlus ? limitXPlus : limitXMinus;
			float yLimit = isYPlus ? limitYPlus : limitYMinus;

			bool isLimited = false;
			if( xLimit <= IKEpsilon && yLimit <= IKEpsilon ) {
				Vector3 limitedDir = new Vector3( 0.0f, 0.0f, 1.0f );
				Vector3 temp = limitedDir - dir;
				if( Mathf.Abs( temp.x ) > IKEpsilon || Mathf.Abs( temp.y ) > IKEpsilon || Mathf.Abs( temp.z ) > IKEpsilon ) {
					dir = limitedDir;
					isLimited = true;
				}
			} else {
				float inv_xLimit = (xLimit >= IKEpsilon) ? (1.0f / xLimit) : 0.0f;
				float inv_yLimit = (yLimit >= IKEpsilon) ? (1.0f / yLimit) : 0.0f;
				float localX = dir.x * inv_xLimit;
				float localY = dir.y * inv_yLimit;
				float localLen = Sqrt( localX * localX + localY * localY + dir.z * dir.z );

				float inv_localLen = (localLen > IKEpsilon) ? (1.0f / localLen) : 0.0f;
				float nrm_localX = localX * inv_localLen; // Counts as sinTheta
				float nrm_localY = localY * inv_localLen; // Counts as cosTheta

				if( localLen > 1.0f ) { // Outer circle.
					if( !isLimited ) {
						isLimited = true;
						localX = nrm_localX;
						localY = nrm_localY;
					}
				}

				float worldX = isLimited ? (localX * xLimit) : dir.x;
				float worldY = isLimited ? (localY * yLimit) : dir.y;

				bool isInverse = (dir.z < 0.0f);

				if( isLimited ) {
					float limitSinSq = (worldX * worldX + worldY * worldY);
					float limitSin = Sqrt( limitSinSq );
					float limitCos = Sqrt( 1.0f - limitSin * limitSin );
					dir.x = worldX;
					dir.y = worldY;
					dir.z = limitCos;
				} else if( isInverse ) {
					isLimited = true;
					dir.z = -dir.z;
				}
			}

			return isLimited;
		}

		public static bool _LimitXZ(
			ref Vector3 dir,				// dirY
			float limiXMinus,				// X-
			float limiXPlus,				// X+
			float limiZMinus,				// Z-
			float limiZPlus )				// Z+
		{
			bool isXPlus = (dir.x >= 0.0f);
			bool isZPlus = (dir.z >= 0.0f);
			float xLimit = isXPlus ? limiXPlus : limiXMinus;
			float zLimit = isZPlus ? limiZPlus : limiZMinus;

			bool isLimited = false;
			if( xLimit <= IKEpsilon && zLimit <= IKEpsilon ) {
				Vector3 limitedDir = new Vector3( 0.0f, 1.0f, 0.0f );
				Vector3 temp = limitedDir - dir;
				if( Mathf.Abs( temp.x ) > IKEpsilon || Mathf.Abs( temp.y ) > IKEpsilon || Mathf.Abs( temp.z ) > IKEpsilon ) {
					dir = limitedDir;
					isLimited = true;
				}
			} else {
				float inv_xLimit = (xLimit >= IKEpsilon) ? (1.0f / xLimit) : 0.0f;
				float inv_zLimit = (zLimit >= IKEpsilon) ? (1.0f / zLimit) : 0.0f;
				float localX = dir.x * inv_xLimit;
				float localZ = dir.z * inv_zLimit;
				float localLen = Sqrt( localX * localX + localZ * localZ + dir.y * dir.y );

				float inv_localLen = (localLen > IKEpsilon) ? (1.0f / localLen) : 0.0f;
				float nrm_localX = localX * inv_localLen; // Counts as sinTheta
				float nrm_localZ = localZ * inv_localLen; // Counts as cosTheta

				if( localLen > 1.0f ) { // Outer circle.
					if( !isLimited ) {
						isLimited = true;
						localX = nrm_localX;
						localZ = nrm_localZ;
					}
				}

				float worldX = isLimited ? (localX * xLimit) : dir.x;
				float worldZ = isLimited ? (localZ * zLimit) : dir.z;

				bool isInverse = (dir.y < 0.0f);

				if( isLimited ) {
					float limitSinSq = (worldX * worldX + worldZ * worldZ);
					float limitSin = Sqrt( limitSinSq );
					float limitCos = Sqrt( 1.0f - limitSin * limitSin );
					dir.x = worldX;
					dir.y = limitCos;
					dir.z = worldZ;
				} else if( isInverse ) {
					isLimited = true;
					dir.y = -dir.y;
				}
			}

			return isLimited;
		}

		public static bool _LimitYZ(
			bool isRight,
			ref Vector3 dir,					// dirX
			float limitYMinus,					// Y-
			float limitYPlus,					// Y+
			float limitZMinus,					// Z-
			float limitZPlus )					// Z+
		{
			bool isYPlus = (dir.y >= 0.0f);
			bool isZPlus = (dir.z >= 0.0f);
			float yLimit = isYPlus ? limitYPlus : limitYMinus;
			float zLimit = isZPlus ? limitZPlus : limitZMinus;

			bool isLimited = false;
			if( yLimit <= IKEpsilon && zLimit <= IKEpsilon ) {
				Vector3 limitedDir = isRight ? new Vector3( 1.0f, 0.0f, 0.0f ) : new Vector3( -1.0f, 0.0f, 0.0f );
				Vector3 temp = limitedDir - dir;
				if( Mathf.Abs( temp.x ) > IKEpsilon || Mathf.Abs( temp.y ) > IKEpsilon || Mathf.Abs( temp.z ) > IKEpsilon ) {
					dir = limitedDir;
					isLimited = true;
				}
			} else {
				float inv_yLimit = (yLimit >= IKEpsilon) ? (1.0f / yLimit) : 0.0f;
				float inv_zLimit = (zLimit >= IKEpsilon) ? (1.0f / zLimit) : 0.0f;
				float localY = dir.y * inv_yLimit;
				float localZ = dir.z * inv_zLimit;
				float localLen = Sqrt( dir.x * dir.x + localY * localY + localZ * localZ );

				float inv_localLen = (localLen > IKEpsilon) ? (1.0f / localLen) : 0.0f;
				float nrm_localY = localY * inv_localLen; // Counts as sinTheta
				float nrm_localZ = localZ * inv_localLen; // Counts as cosTheta

				if( localLen > 1.0f ) { // Outer circle.
					if( !isLimited ) {
						isLimited = true;
						localY = nrm_localY;
						localZ = nrm_localZ;
					}
				}

				float worldY = isLimited ? (localY * yLimit) : dir.y;
				float worldZ = isLimited ? (localZ * zLimit) : dir.z;

				bool isInverse = ((dir.x >= 0.0f) != isRight);

				if( isLimited ) {
					float limitSinSq = (worldY * worldY + worldZ * worldZ);
					float limitSin = Sqrt( limitSinSq );
					float limitCos = Sqrt( 1.0f - limitSin * limitSin );
					dir.x = isRight ? limitCos : -limitCos;
					dir.y = worldY;
					dir.z = worldZ;
				} else if( isInverse ) {
					isLimited = true;
					dir.x = -dir.x;
				}
			}

			return isLimited;
		}

		//--------------------------------------------------------------------------------------------------------------------

		public static Vector3 _FitToPlane( Vector3 pos, Vector3 planeDir )
		{
			float d = Vector3.Dot( pos, planeDir );
			if( d <= IKEpsilon && d >= -IKEpsilon ) {
				return pos; // Cross.
			}

			return pos - planeDir * d;
		}

#if true // Legacy unused.
		public static bool _FitToPlaneDir( ref Vector3 dir, Vector3 planeDir )
		{
			float d = Vector3.Dot( dir, planeDir );
			if( d <= IKEpsilon && d >= -IKEpsilon ) {
				return false;
			}

			Vector3 tmp = dir - planeDir * d;
			if( !_SafeNormalize( ref tmp ) ) {
				return false;
			}

			dir = tmp;
			return true;
		}
#endif

		//--------------------------------------------------------------------------------------------------------------------

		public static bool _NormalizeAndComputeBasisFromXYLockY( ref Matrix3x3 basis, ref Vector3 dirX, ref Vector3 dirY )
		{
			Vector3 x = dirX, y = dirY;
			Vector3 z = Vector3.Cross( x, y );
			x = Vector3.Cross( y, z );
			if( _SafeNormalize( ref x, ref y, ref z ) ) {
				basis.SetColumn( ref x, ref y, ref z );
				return true;
			}

			return false;
		}

		//--------------------------------------------------------------------------------------------------------------------

		public static Vector3 _LerpDir( ref Vector3 src, ref Vector3 dst, float r )
		{
			float theta;
			Vector3 axis;
			if( _ComputeThetaAxis( ref src, ref dst, out theta, out axis ) ) {
				Matrix3x3 basis;
				_LerpRotateBasis( out basis, ref axis, theta, r );
				return basis.Multiply( src );
			}

			return dst;
		}

		public static Vector3 _FastLerpDir( ref Vector3 src, ref Vector3 dst, float r )
		{
			if( r <= IKEpsilon ) {
				return src;
			} else if( r >= 1.0f - IKEpsilon ) {
				return dst;
			}

			Vector3 tmp = src + (dst - src) * r;
			float len = tmp.magnitude;
			if( len > IKEpsilon ) {
				return tmp * (1.0f / len);
			}

			return dst;
		}

		//--------------------------------------------------------------------------------------------------------------------

		// for Finger.
		public static bool _LimitFingerNotThumb(
			bool isRight,
			ref Vector3 dir, // dirX
			ref FastAngle limitYPlus,
			ref FastAngle limitYMinus,
			ref FastAngle limitZ )
		{
			bool isLimited = false;

			// Yaw
			if( limitZ.cos > IKEpsilon ) {
				// Memo: Unstable when dir.z near 1.
				if( dir.z < -limitZ.sin || dir.z > limitZ.sin ) {
					isLimited = true;
					bool isPlus = (dir.z >= 0.0f);
					float lenXY = Sqrt( dir.x * dir.x + dir.y * dir.y );
					if( limitZ.sin <= IKEpsilon ) { // Optimized.
						if( lenXY > IKEpsilon ) {
							dir.z = 0.0f;
							dir = dir * (1.0f / lenXY);
						} else { // Failsafe.
							dir.Set( isRight ? limitZ.cos : -limitZ.cos, 0.0f, isPlus ? limitZ.sin : -limitZ.sin );
						}
					} else {
						float lenZ = limitZ.sin * lenXY / limitZ.cos;
						dir.z = isPlus ? lenZ : -lenZ;

						float len = dir.magnitude;
						if( len > IKEpsilon ) {
							dir *= (1.0f / len);
						} else { // Failsafe.
							dir.Set( isRight ? limitZ.cos : -limitZ.cos, 0.0f, isPlus ? limitZ.sin : -limitZ.sin );
						}
					}
				}
			}

			// Pitch
			{
				// Memo: Not use z.( For yaw limit. )
				bool isPlus = (dir.y >= 0.0f);
				float cosPitchLimit = isPlus ? limitYPlus.cos : limitYMinus.cos;
				if( (isRight && dir.x < cosPitchLimit) || (!isRight && dir.x > -cosPitchLimit) ) {
					float lenY = Sqrt( 1.0f - (cosPitchLimit * cosPitchLimit + dir.z * dir.z) );
					dir.x = (isRight ? cosPitchLimit : -cosPitchLimit);
					dir.y = (isPlus ? lenY : -lenY);
				}
			}

			return isLimited;
		}
	}

}