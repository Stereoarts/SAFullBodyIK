// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing
using UnityEngine;

namespace SA
{

	public static class FullBodyIKUtil
	{
		public static TYPE_ SafeNew<TYPE_>( ref TYPE_ obj )
			where TYPE_ : class, new()
		{
			if( obj == null ) {
				obj = new TYPE_();
			}

			return obj;
		}

		public static void PrepareArray< TypeA, TypeB >( ref TypeA[] dstArray, TypeB[] srcArray )
		{
			if( srcArray != null ) {
				if( dstArray == null || dstArray.Length != srcArray.Length ) {
					dstArray = new TypeA[srcArray.Length];
				}
			} else {
				dstArray = null;
			}
		}

		public static void CloneArray< Type >( ref Type[] dstArray, Type[] srcArray )
		{
			if( srcArray != null ) {
				if( dstArray == null || dstArray.Length != srcArray.Length ) {
					dstArray = new Type[srcArray.Length];
				}
				for( int i = 0; i < srcArray.Length; ++i ) {
					dstArray[i] = srcArray[i];
				}
			} else {
				dstArray = null;
			}
		}
		
		public static void DestroyImmediate( ref Transform transform )
		{
			if( transform != null ) {
				Object.DestroyImmediate( transform.gameObject );
				transform = null;
			} else {
				transform = null; // Optimized. Because Object is weak reference.
			}
		}
		
		public static void DestroyImmediate( ref Transform transform, bool allowDestroyingAssets )
		{
			if( transform != null ) {
				Object.DestroyImmediate( transform.gameObject, allowDestroyingAssets );
				transform = null;
			} else {
				transform = null; // Optimized. Because Object is weak reference.
			}
		}
		
		public static bool CheckAlive< Type >( ref Type obj )
			where Type : UnityEngine.Object
		{
			if( obj != null ) {
				return true;
			} else {
				obj = null; // Optimized. Because Object is weak reference.
				return false;
			}
		}

		public static bool IsParentOfRecusively( Transform parent, Transform child )
		{
			while( child != null ) {
				if( child.parent == parent ) {
					return true;
				}

				child = child.parent;
			}

			return false;
		}
	}

}