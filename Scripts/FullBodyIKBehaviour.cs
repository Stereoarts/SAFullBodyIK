// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing

using UnityEngine;
using System.Collections.Generic;

namespace SA
{
	[ExecuteInEditMode]
	[DisallowMultipleComponent]
	public class FullBodyIKBehaviour : MonoBehaviour
	{
		[SerializeField]
		FullBodyIK _fullBodyIK;

		public FullBodyIK fullBodyIK { get { return _fullBodyIK; } }

		// Excecutable in Inspector.
		public void Prefix()
		{
			if( _fullBodyIK == null ) {
				_fullBodyIK = new FullBodyIK();
			}

			fullBodyIK.Prefix( this.transform );
		}

		void Awake()
		{
#if UNITY_EDITOR
			if( !Application.isPlaying ) {
				return;
			}
#endif
			if( _fullBodyIK == null ) {
				_fullBodyIK = new FullBodyIK();
			}

			_fullBodyIK.Initialize( this.transform );
		}

		void OnDestroy()
		{
#if UNITY_EDITOR
			if( _fullBodyIK != null ) {
				_fullBodyIK.Destroy();
            }
#endif
		}

		void LateUpdate()
		{
#if UNITY_EDITOR
			if( !Application.isPlaying ) {
				return;
			}
#endif

			FullBodyIK.Assert( _fullBodyIK != null );
			if( _fullBodyIK != null ) {
				_fullBodyIK.Update();
			}
		}

#if UNITY_EDITOR
		void OnDrawGizmos()
		{
			if( _fullBodyIK != null ) {
				_fullBodyIK.DrawGizmos();
            }
		}
#endif
	}
}