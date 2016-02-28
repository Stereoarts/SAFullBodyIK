// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing
using UnityEngine;

namespace SA
{
	public partial class FullBodyIK
	{
		public class HeadIK
		{
			Settings _settings;
			InternalValues _internalValues;

			Bone _neckBone;
			Bone _headBone;
			Bone _leftEyeBone;
			Bone _rightEyeBone;

			Effector _neckEffector;
			Effector _headEffector;
			Effector _eyesEffector;

			// for UnityChan
			Vector3 _headBoneLossyScale = Vector3.one;
			bool _isHeadBoneLossyScaleFuzzyIdentity = true;

			float _eyeInnerLimit = Mathf.Sin( 30.0f * Mathf.Deg2Rad );
			float _eyeOuterLimit = Mathf.Sin( 50.0f * Mathf.Deg2Rad );
			float _eyeUpperLimit = Mathf.Sin( 20.0f * Mathf.Deg2Rad );
			float _eyeLowerLimit = Mathf.Sin( 30.0f * Mathf.Deg2Rad );

			public HeadIK( FullBodyIK fullBodyIK )
			{
				_settings = fullBodyIK.settings;
				_internalValues = fullBodyIK.internalValues;

				_neckBone = fullBodyIK.headBones.neck;
				_headBone = fullBodyIK.headBones.head;
				_leftEyeBone = fullBodyIK.headBones.leftEye;
				_rightEyeBone = fullBodyIK.headBones.rightEye;
				_neckEffector = fullBodyIK.headEffectors.neck;
				_headEffector = fullBodyIK.headEffectors.head;
				_eyesEffector = fullBodyIK.headEffectors.eyes;

				_Prepare();
            }

			public static Vector3 _unityChan_leftEyeDefaultLocalPosition = new Vector3( -0.042531f + 0.024f, 0.048524f, 0.047682f - 0.02f );
			public static Vector3 _unityChan_rightEyeDefaultLocalPosition = new Vector3( 0.042531f - 0.024f, 0.048524f, 0.047682f - 0.02f );
			Vector3 _unityChan_leftEyeDefaultPosition = Vector3.zero;
			Vector3 _unityChan_rightEyeDefaultPosition = Vector3.zero;

			void _Prepare()
			{
				if( _settings.modelTemplate == ModelTemplate.UnityChan ) {
					if( _headBone != null && _headBone.transformIsAlive ) {
						Vector3 leftPos, rightPos;
						SAFBIKMatMultVec( out leftPos, ref _internalValues.defaultRootBasis, ref _unityChan_leftEyeDefaultLocalPosition );
						SAFBIKMatMultVec( out rightPos, ref _internalValues.defaultRootBasis,  ref _unityChan_rightEyeDefaultLocalPosition );

						_headBoneLossyScale = _headBone.transform.lossyScale;
						_isHeadBoneLossyScaleFuzzyIdentity = IsFuzzy( _headBoneLossyScale, Vector3.one );

						if( !_isHeadBoneLossyScaleFuzzyIdentity ) {
							leftPos = Scale( ref leftPos, ref _headBoneLossyScale );
							rightPos = Scale( ref rightPos, ref _headBoneLossyScale );
						}

						_unityChan_leftEyeDefaultPosition = _headBone._defaultPosition + leftPos;
						_unityChan_rightEyeDefaultPosition = _headBone._defaultPosition + rightPos;
					}
				}
			}

			public void Solve()
			{
				if( !_neckBone.transformIsAlive ||
					!_headBone.transformIsAlive ||
					_headBone.parentBone == null ||
					!_headBone.parentBone.transformIsAlive ) {
					return;
				}

				_Solve();
			}

			void _Solve()
			{
				float eyesWeight = _eyesEffector.positionEnabled ? _eyesEffector.positionWeight : 0.0f;
				if( eyesWeight > IKEpsilon ) {
					Quaternion parentBoneWorldRotation = _neckBone.parentBone.worldRotation;
					Matrix3x3 parentBasis;
					SAFBIKMatSetRotMultInv1( out parentBasis, ref parentBoneWorldRotation, ref _neckBone.parentBone._defaultRotation );
					Matrix3x3 parentBaseBasis;
					SAFBIKMatMult( out parentBaseBasis, ref parentBasis, ref _internalValues.defaultRootBasis );

					Vector3 eyesPosition, parentBoneWorldPosition = _neckBone.parentBone.worldPosition;
					SAFBIKMatMultVecPreSubAdd( out eyesPosition, ref parentBasis, ref _eyesEffector._defaultPosition, ref _neckBone.parentBone._defaultPosition, ref parentBoneWorldPosition );

					Vector3 eyesDir = _eyesEffector.worldPosition - eyesPosition; // Memo: Not normalize yet.

					Matrix3x3 neckBaseBasis = parentBaseBasis;

					bool _isSolveNeck = true;
					bool _isSolveHead = true;
					float _neckYRate = 0.75f;

					_internalValues.UpdateDebugValue( "_isSolveNeck", ref _isSolveNeck );
					_internalValues.UpdateDebugValue( "_isSolveHead", ref _isSolveHead );
					_internalValues.UpdateDebugValue( "_neckYRate", ref _neckYRate );

#if true
					if( _isSolveNeck ) {
						Vector3 localDir;
						SAFBIKMatMultVecInv( out localDir, ref parentBaseBasis, ref eyesDir );

						//_debugData.AddPoint( eyesPosition, Color.red );
						_internalValues.AddDebugPoint( eyesPosition + eyesDir * 0.25f, Color.red );

						float _neckYUpLimitAngle = 15.0f;
						float _neckYDownLimitAngle = 30.0f;
						float _neckLerpRate = 0.5f;

						_internalValues.UpdateDebugValue( "_neckYUpLimitAngle", ref _neckYUpLimitAngle );
						_internalValues.UpdateDebugValue( "_neckYDownLimitAngle", ref _neckYDownLimitAngle );

						_neckYUpLimitAngle *= Mathf.Deg2Rad;
						_neckYDownLimitAngle *= Mathf.Deg2Rad;

						localDir.y *= _neckYRate;
						SAFBIKVecNormalize( ref localDir );

						localDir.y = Mathf.Clamp( localDir.y, -Mathf.Sin( _neckYDownLimitAngle ), Mathf.Sin( _neckYUpLimitAngle ) );
						localDir.x = 0.0f;
						localDir.z = SAFBIKSqrt( 1.0f - localDir.y * localDir.y );

						SAFBIKMatMultVec( out eyesDir, ref parentBaseBasis, ref localDir );

						eyesDir = Vector3.Lerp( parentBaseBasis.column2, eyesDir, _neckLerpRate );
						SAFBIKVecNormalize( ref eyesDir );

						{
							Vector3 xDir = parentBaseBasis.column0;
							Vector3 yDir = parentBaseBasis.column1;
							Vector3 zDir = eyesDir;

							if( !SAFBIKComputeBasisLockZ( out neckBaseBasis, ref xDir, ref yDir, ref zDir ) ) {
								neckBaseBasis = parentBaseBasis;
                            }
						}

						Quaternion worldRotation;
						SAFBIKMatMultGetRot( out worldRotation, ref neckBaseBasis, ref _neckBone._baseToWorldBasis );
                        _neckBone.worldRotation = worldRotation;
                    }
#endif
					Matrix3x3 neckBasis;
					SAFBIKMatMult( out neckBasis, ref neckBaseBasis, ref _internalValues.defaultRootBasisInv );

					Vector3 neckBoneWorldPosition = _neckBone.worldPosition;
                    SAFBIKMatMultVecPreSubAdd( out eyesPosition, ref neckBasis, ref _eyesEffector._defaultPosition, ref _neckBone._defaultPosition, ref neckBoneWorldPosition );

					eyesDir = _eyesEffector.worldPosition - eyesPosition;

					Matrix3x3 headBaseBasis = neckBaseBasis;

					if( _isSolveHead ) {
						Vector3 localDir;
						SAFBIKMatMultVecInv( out localDir, ref neckBaseBasis, ref eyesDir );

						float _headXLimitAngle = 80.0f;
						float _headYUpLimitAngle = 15.0f;
						float _headYDownLimitAngle = 15.0f;
						float _headXRate = 0.8f;
						float _headYRate = 0.5f;
						float _headZOffset = 0.5f; // Lock when behind looking.

						_internalValues.UpdateDebugValue( "_headXLimitAngle", ref _headXLimitAngle );
						_internalValues.UpdateDebugValue( "_headYUpLimitAngle", ref _headYUpLimitAngle );
						_internalValues.UpdateDebugValue( "_headYDownLimitAngle", ref _headYDownLimitAngle );
						_internalValues.UpdateDebugValue( "_headXRate", ref _headXRate );
						_internalValues.UpdateDebugValue( "_headYRate", ref _headYRate );
						_internalValues.UpdateDebugValue( "_headXOffset", ref _headZOffset );

						_headXLimitAngle *= Mathf.Deg2Rad;
						_headYUpLimitAngle *= Mathf.Deg2Rad;
						_headYDownLimitAngle *= Mathf.Deg2Rad;

						localDir.x *= _headXRate;
						localDir.y *= _headYRate;

						SAFBIKVecNormalize( ref localDir );

						if( localDir.z < 0.0f ) {
							float offset = Mathf.Clamp( _headZOffset, 0.0f, 0.99f );
							if( offset > IKEpsilon ) {
								if( localDir.z > -offset ) {
									localDir.z = 0.0f;
								} else {
									localDir.z = (localDir.z + offset) / (1.0f - offset);
								}
								SAFBIKVecNormalize( ref localDir );
							}
						}

						_LimitXY( ref localDir,
							Mathf.Sin( _headXLimitAngle ),
							Mathf.Sin( _headXLimitAngle ),
							Mathf.Sin( _headYDownLimitAngle ),
							Mathf.Sin( _headYUpLimitAngle ) );

						SAFBIKMatMultVec( out eyesDir, ref neckBaseBasis, ref localDir );

						SAFBIKVecNormalize( ref eyesDir );

						{
							Vector3 xDir = neckBaseBasis.column0;
							Vector3 yDir = neckBaseBasis.column1;
							Vector3 zDir = eyesDir;

							if( !SAFBIKComputeBasisLockZ( out headBaseBasis, ref xDir, ref yDir, ref zDir ) ) {
								headBaseBasis = neckBaseBasis;
							}
						}

						Quaternion worldRotation;
						SAFBIKMatMultGetRot( out worldRotation, ref headBaseBasis, ref _headBone._baseToWorldBasis );
						_headBone.worldRotation = worldRotation;
					}

					Matrix3x3 headBasis;
					SAFBIKMatMult( out headBasis, ref headBaseBasis, ref _internalValues.defaultRootBasisInv );

					if( _settings.modelTemplate == ModelTemplate.UnityChan ) {
						_SolveEyesUnityChan( ref neckBasis, ref headBasis, ref headBaseBasis );
                    } else {
						_SolveEyes( ref neckBasis, ref headBasis, ref headBaseBasis );
					}
				}
			}

			void _SolveEyesUnityChan( ref Matrix3x3 neckBasis, ref Matrix3x3 headBasis, ref Matrix3x3 headBaseBasis )
			{
				if( _leftEyeBone.transformIsAlive || _rightEyeBone.transformIsAlive ) {
					Vector3 leftEyePosition = new Vector3( -0.042531f, 0.048524f, 0.047682f );
					Vector3 rightEyePosition = new Vector3( 0.042531f, 0.048524f, 0.047682f );

					float _eyesHorzLimitAngle = 40.0f;
					float _eyesVertLimitAngle = 4.5f;
					float _eyesXRate = 0.796f;
					float _eyesYRate = 0.28f;
					float _eyesOuterXRotRate = 0.096f;
					float _eyesInnerXRotRate = 0.065f;
					float _eyesXOffset = -0.024f;
					float _eyesYOffset = 0.0f;
					float _eyesZOffset = -0.02f;

					_internalValues.UpdateDebugValue( "_eyesHorzLimitAngle", ref _eyesHorzLimitAngle );
					_internalValues.UpdateDebugValue( "_eyesVertLimitAngle", ref _eyesVertLimitAngle );
					_internalValues.UpdateDebugValue( "_eyesXRate", ref _eyesXRate );
					_internalValues.UpdateDebugValue( "_eyesYRate", ref _eyesYRate );
					_internalValues.UpdateDebugValue( "_eyesOuterXRotRate", ref _eyesOuterXRotRate );
					_internalValues.UpdateDebugValue( "_eyesInnerXRotRate", ref _eyesInnerXRotRate );
					_internalValues.UpdateDebugValue( "_eyesXOffset", ref _eyesXOffset );
					_internalValues.UpdateDebugValue( "_eyesYOffset", ref _eyesYOffset );
					_internalValues.UpdateDebugValue( "_eyesZOffset", ref _eyesZOffset );

					float _innerMoveXRate = 0.063f;
					float _outerMoveXRate = 0.063f;

					_internalValues.UpdateDebugValue( "_innerMoveXRate", ref _innerMoveXRate );
					_internalValues.UpdateDebugValue( "_outerMoveXRate", ref _outerMoveXRate );

					_innerMoveXRate *= 0.1f;
					_outerMoveXRate *= 0.1f;

					_eyesHorzLimitAngle *= Mathf.Deg2Rad;
					_eyesVertLimitAngle *= Mathf.Deg2Rad;

					float _eyesHorzLimit = Mathf.Sin( _eyesHorzLimitAngle );
					float _eyesVertLimit = Mathf.Sin( _eyesVertLimitAngle );

					leftEyePosition.x -= _eyesXOffset;
					rightEyePosition.x += _eyesXOffset;
					leftEyePosition.y += _eyesYOffset;
					rightEyePosition.y += _eyesYOffset;
					leftEyePosition.z += _eyesZOffset;
					rightEyePosition.z += _eyesZOffset;

					Vector3 leftEyeDefaultPosition = _unityChan_leftEyeDefaultPosition;
					Vector3 rightEyeDefaultPosition = _unityChan_rightEyeDefaultPosition;
					leftEyePosition = _unityChan_leftEyeDefaultPosition;
					rightEyePosition = _unityChan_rightEyeDefaultPosition;

					Vector3 headWorldPosition, neckBoneWorldPosition = _neckBone.worldPosition;
					SAFBIKMatMultVecPreSubAdd( out headWorldPosition, ref neckBasis, ref _headBone._defaultPosition, ref _neckBone._defaultPosition, ref neckBoneWorldPosition );
					Vector3 eyesPosition;
					SAFBIKMatMultVecPreSubAdd( out eyesPosition, ref headBasis, ref _eyesEffector._defaultPosition, ref _headBone._defaultPosition, ref headWorldPosition );

					Vector3 eyesDir = _eyesEffector.worldPosition - eyesPosition;

					Matrix3x3 leftEyeBaseBasis = headBaseBasis;
					Matrix3x3 rightEyeBaseBasis = headBaseBasis;

					SAFBIKMatMultVecInv( out eyesDir, ref headBaseBasis, ref eyesDir );

					SAFBIKVecNormalize( ref eyesDir );

					_LimitXY_Square( ref eyesDir,
						Mathf.Sin( _eyesHorzLimitAngle ),
						Mathf.Sin( _eyesHorzLimitAngle ),
						Mathf.Sin( _eyesVertLimitAngle ),
						Mathf.Sin( _eyesVertLimitAngle ) );

					float moveX = Mathf.Clamp( eyesDir.x * _eyesXRate, -_eyesHorzLimit, _eyesHorzLimit );
					float moveY = Mathf.Clamp( eyesDir.y * _eyesYRate, -_eyesVertLimit, _eyesVertLimit );
					float moveZ = -Mathf.Max( 1.0f - eyesDir.z, 1.0f - _eyesVertLimit ); // Reuse _eyesVertLimit.

					eyesDir.x *= _eyesXRate;
					eyesDir.y *= _eyesYRate;
					Vector3 leftEyeDir = eyesDir;
					Vector3 rightEyeDir = eyesDir;

					if( eyesDir.x >= 0.0f ) {
						leftEyeDir.x *= _eyesInnerXRotRate;
						rightEyeDir.x *= _eyesOuterXRotRate;
					} else {
						leftEyeDir.x *= _eyesOuterXRotRate;
						rightEyeDir.x *= _eyesInnerXRotRate;
					}

					SAFBIKVecNormalize2( ref leftEyeDir, ref rightEyeDir );

					SAFBIKMatMultVec( out leftEyeDir, ref headBaseBasis, ref leftEyeDir );
					SAFBIKMatMultVec( out rightEyeDir, ref headBaseBasis, ref rightEyeDir );

					float leftXRate = (moveX >= 0.0f) ? _innerMoveXRate : _outerMoveXRate;
					float rightXRate = (moveX >= 0.0f) ? _outerMoveXRate : _innerMoveXRate;

					{
						Vector3 xDir = headBasis.column0;
						Vector3 yDir = headBasis.column1;
						Vector3 zDir = leftEyeDir;
						SAFBIKComputeBasisLockZ( out leftEyeBaseBasis, ref xDir, ref yDir, ref zDir );
					}

					{
						Vector3 xDir = headBasis.column0;
						Vector3 yDir = headBasis.column1;
						Vector3 zDir = rightEyeDir;
						SAFBIKComputeBasisLockZ( out rightEyeBaseBasis, ref xDir, ref yDir, ref zDir );
					}

					Vector3 leftEyeWorldPosition;
					Vector3 rightEyeWorldPosition;

					leftEyeWorldPosition = headBaseBasis.column0 * (leftXRate * moveX);
					rightEyeWorldPosition = headBaseBasis.column0 * (rightXRate * moveX);

					if( !_isHeadBoneLossyScaleFuzzyIdentity ) {
						leftEyeWorldPosition = Scale( ref leftEyeWorldPosition, ref _headBoneLossyScale );
						rightEyeWorldPosition = Scale( ref rightEyeWorldPosition, ref _headBoneLossyScale );
					}

					Vector3 tempVec;
					SAFBIKMatMultVecPreSubAdd( out tempVec, ref headBasis, ref _unityChan_leftEyeDefaultPosition, ref _headBone._defaultPosition, ref headWorldPosition );
					leftEyeWorldPosition += tempVec;
					SAFBIKMatMultVecPreSubAdd( out tempVec, ref headBasis, ref _unityChan_rightEyeDefaultPosition, ref _headBone._defaultPosition, ref headWorldPosition );
					rightEyeWorldPosition += tempVec;

					Matrix3x3 leftEyeBasis, rightEyeBasis;
					SAFBIKMatMult( out leftEyeBasis, ref leftEyeBaseBasis, ref _internalValues.defaultRootBasisInv );
					SAFBIKMatMult( out rightEyeBasis, ref rightEyeBaseBasis, ref _internalValues.defaultRootBasisInv );

					Vector3 worldPosition;
					Quaternion worldRotation;

					if( _leftEyeBone.transformIsAlive ) {
						SAFBIKMatMultVecPreSubAdd( out worldPosition, ref leftEyeBasis, ref _leftEyeBone._defaultPosition, ref leftEyeDefaultPosition, ref leftEyeWorldPosition );
						_leftEyeBone.worldPosition = worldPosition;
						SAFBIKMatMultGetRot( out worldRotation, ref leftEyeBaseBasis, ref _leftEyeBone._baseToWorldBasis );
						_leftEyeBone.worldRotation = worldRotation;
					}

					if( _rightEyeBone.transformIsAlive ) {
						SAFBIKMatMultVecPreSubAdd( out worldPosition, ref rightEyeBasis, ref _rightEyeBone._defaultPosition, ref rightEyeDefaultPosition, ref rightEyeWorldPosition );
						_rightEyeBone.worldPosition = worldPosition;
						SAFBIKMatMultGetRot( out worldRotation, ref rightEyeBaseBasis, ref _rightEyeBone._baseToWorldBasis );
                        _rightEyeBone.worldRotation = worldRotation;
					}
				}
			}

			void _SolveEyes( ref Matrix3x3 neckBasis, ref Matrix3x3 headBasis, ref Matrix3x3 headBaseBasis )
			{
				if( _leftEyeBone.transformIsAlive || _rightEyeBone.transformIsAlive ) {
					float _eyesHorzLimitAngle = 40.0f;
					float _eyesVertLimitAngle = 12.0f;
					float _eyesXRate = 0.796f;
					float _eyesYRate = 0.729f;
					float _eyesOuterXRotRate = 0.356f;
					float _eyesInnerXRotRate = 0.212f;

					_internalValues.UpdateDebugValue( "_eyesHorzLimitAngle", ref _eyesHorzLimitAngle );
					_internalValues.UpdateDebugValue( "_eyesVertLimitAngle", ref _eyesVertLimitAngle );
					_internalValues.UpdateDebugValue( "_eyesXRate", ref _eyesXRate );
					_internalValues.UpdateDebugValue( "_eyesYRate", ref _eyesYRate );
					_internalValues.UpdateDebugValue( "_eyesOuterXRotRate", ref _eyesOuterXRotRate );
					_internalValues.UpdateDebugValue( "_eyesInnerXRotRate", ref _eyesInnerXRotRate );

					_eyesHorzLimitAngle *= Mathf.Deg2Rad;
					_eyesVertLimitAngle *= Mathf.Deg2Rad;

					Vector3 headWorldPosition, neckBoneWorldPosition = _neckBone.worldPosition;
                    SAFBIKMatMultVecPreSubAdd( out headWorldPosition, ref neckBasis, ref _headBone._defaultPosition, ref _neckBone._defaultPosition, ref neckBoneWorldPosition );

					Vector3 eyesPosition;
					SAFBIKMatMultVecPreSubAdd( out eyesPosition, ref headBasis, ref _eyesEffector._defaultPosition, ref _headBone._defaultPosition, ref headWorldPosition );

					Vector3 eyesDir = _eyesEffector.worldPosition - eyesPosition;

					SAFBIKMatMultVecInv( out eyesDir, ref headBaseBasis, ref eyesDir );

					SAFBIKVecNormalize( ref eyesDir );

					_LimitXY_Square( ref eyesDir,
						Mathf.Sin( _eyesHorzLimitAngle ),
						Mathf.Sin( _eyesHorzLimitAngle ),
						Mathf.Sin( _eyesVertLimitAngle ),
						Mathf.Sin( _eyesVertLimitAngle ) );

					eyesDir.x *= _eyesXRate;
					eyesDir.y *= _eyesYRate;
					Vector3 leftEyeDir = eyesDir;
					Vector3 rightEyeDir = eyesDir;

					if( eyesDir.x >= 0.0f ) {
						leftEyeDir.x *= _eyesInnerXRotRate;
						rightEyeDir.x *= _eyesOuterXRotRate;
					} else {
						leftEyeDir.x *= _eyesOuterXRotRate;
						rightEyeDir.x *= _eyesInnerXRotRate;
					}

					SAFBIKVecNormalize2( ref leftEyeDir, ref rightEyeDir );

					SAFBIKMatMultVec( out leftEyeDir, ref headBaseBasis, ref leftEyeDir );
					SAFBIKMatMultVec( out rightEyeDir, ref headBaseBasis, ref rightEyeDir );

					Quaternion worldRotation;

					if( _leftEyeBone.transformIsAlive ) {
						Matrix3x3 leftEyeBaseBasis;
						SAFBIKComputeBasisLockZ( out leftEyeBaseBasis, ref headBasis.column0, ref headBasis.column1, ref leftEyeDir );
						SAFBIKMatMultGetRot( out worldRotation, ref leftEyeBaseBasis, ref _leftEyeBone._baseToWorldBasis );
						_leftEyeBone.worldRotation = worldRotation;
					}

					if( _rightEyeBone.transformIsAlive ) {
						Matrix3x3 rightEyeBaseBasis;
						SAFBIKComputeBasisLockZ( out rightEyeBaseBasis, ref headBasis.column0, ref headBasis.column1, ref rightEyeDir );
						SAFBIKMatMultGetRot( out worldRotation, ref rightEyeBaseBasis, ref _rightEyeBone._baseToWorldBasis );
						_rightEyeBone.worldRotation = worldRotation;
					}
				}
			}
			
		}
	}
}