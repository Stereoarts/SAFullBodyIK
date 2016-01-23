// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing
using UnityEngine;

namespace SA
{
	public partial class FullBodyIK : MonoBehaviour
	{
		public class HeadIK
		{
#if SAFULLBODYIK_DEBUG
			DebugData _debugData;
#endif
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
			float _eyeLimitRate = 0.5f;

			public HeadIK( FullBodyIK fullBodyIK )
			{
#if SAFULLBODYIK_DEBUG
				_debugData = fullBodyIK._debugData;
#endif
				_settings = fullBodyIK._settings;
				_internalValues = fullBodyIK._internalValues;

				_neckBone = fullBodyIK._headBones.neck;
				_headBone = fullBodyIK._headBones.head;
				_leftEyeBone = fullBodyIK._headBones.leftEye;
				_rightEyeBone = fullBodyIK._headBones.rightEye;
				_neckEffector = fullBodyIK._headEffectors.neck;
				_headEffector = fullBodyIK._headEffectors.head;
				_eyesEffector = fullBodyIK._headEffectors.eyes;

				_Prepare();
            }

			public static readonly Vector3 _unityChan_leftEyeDefaultLocalPosition = new Vector3( -0.042531f + 0.024f, 0.048524f, 0.047682f - 0.02f );
			public static readonly Vector3 _unityChan_rightEyeDefaultLocalPosition = new Vector3( 0.042531f - 0.024f, 0.048524f, 0.047682f - 0.02f );
			Vector3 _unityChan_leftEyeDefaultPosition = Vector3.zero;
			Vector3 _unityChan_rightEyeDefaultPosition = Vector3.zero;

			void _Prepare()
			{
				if( _settings.modelTemplate == ModelTemplate.UnityChan ) {
					if( _headBone != null && _headBone.transformIsAlive ) {
						Vector3 leftPos = _internalValues.defaultRootBasis.Multiply( _unityChan_leftEyeDefaultLocalPosition );
						Vector3 rightPos = _internalValues.defaultRootBasis.Multiply( _unityChan_rightEyeDefaultLocalPosition );

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
					Matrix3x3 parentBasis = _neckBone.parentBone.worldRotation * Inverse( _neckBone.parentBone._defaultRotation );
					Matrix3x3 parentBaseBasis = parentBasis * _internalValues.defaultRootBasis;

					Vector3 eyesPosition = parentBasis.Multiply( _eyesEffector.defaultPosition - _neckBone.parentBone._defaultPosition ) + _neckBone.parentBone.worldPosition;
					Vector3 eyesDir = _eyesEffector.worldPosition - eyesPosition; // Memo: Not normalize yet.

					Matrix3x3 neckBaseBasis = parentBaseBasis;

					bool _isSolveNeck = true;
					bool _isSolveHead = true;
					float _neckYRate = 0.75f;
#if SAFULLBODYIK_DEBUG
					_debugData.UpdateValue( "_isSolveNeck", ref _isSolveNeck );
					_debugData.UpdateValue( "_isSolveHead", ref _isSolveHead );
					_debugData.UpdateValue( "_neckYRate", ref _neckYRate );
#endif

#if true
					if( _isSolveNeck ) {
						Vector3 localDir = parentBaseBasis.transpose.Multiply( eyesDir );

#if SAFULLBODYIK_DEBUG
						//_debugData.AddPoint( eyesPosition, Color.red );
						_debugData.AddPoint( eyesPosition + eyesDir * 0.25f, Color.red );
#endif
						float _neckYUpLimitAngle = 15.0f;
						float _neckYDownLimitAngle = 30.0f;
						float _neckLerpRate = 0.5f;

#if SAFULLBODYIK_DEBUG
						_debugData.UpdateValue( "_neckYUpLimitAngle", ref _neckYUpLimitAngle );
						_debugData.UpdateValue( "_neckYDownLimitAngle", ref _neckYDownLimitAngle );
#endif
						_neckYUpLimitAngle *= Mathf.Deg2Rad;
						_neckYDownLimitAngle *= Mathf.Deg2Rad;

						localDir.y *= _neckYRate;
						_SafeNormalize( ref localDir );

						localDir.y = Mathf.Clamp( localDir.y, -Mathf.Sin( _neckYDownLimitAngle ), Mathf.Sin( _neckYUpLimitAngle ) );
						localDir.x = 0.0f;
						localDir.z = Sqrt( 1.0f - localDir.y * localDir.y );

						eyesDir = parentBaseBasis.Multiply( localDir );

						eyesDir = Vector3.Lerp( parentBaseBasis.column2, eyesDir, _neckLerpRate );
						_SafeNormalize( ref eyesDir );

						{
							Vector3 xDir = parentBaseBasis.column0;
							Vector3 yDir = parentBaseBasis.column1;
							Vector3 zDir = eyesDir;

							if( !_ComputeBasisLockZ( out neckBaseBasis, ref xDir, ref yDir, ref zDir ) ) {
								neckBaseBasis = parentBaseBasis;
                            }
						}

                        _neckBone.worldRotation = (neckBaseBasis * _neckBone._baseToWorldBasis).GetRotation();
                    }
#endif
					Matrix3x3 neckBasis = neckBaseBasis * _internalValues.defaultRootBasisInv;

					eyesPosition = neckBasis.Multiply( _eyesEffector.defaultPosition - _neckBone._defaultPosition ) + _neckBone.worldPosition;
					eyesDir = _eyesEffector.worldPosition - eyesPosition;

					Matrix3x3 headBaseBasis = neckBaseBasis;

					if( _isSolveHead ) {
						Vector3 localDir = neckBaseBasis.transpose.Multiply( eyesDir );

						float _headXLimitAngle = 80.0f;
						float _headYUpLimitAngle = 15.0f;
						float _headYDownLimitAngle = 15.0f;
						float _headXRate = 0.8f;
						float _headYRate = 0.5f;
						float _headZOffset = 0.5f; // Lock when behind looking.
#if SAFULLBODYIK_DEBUG
						_debugData.UpdateValue( "_headXLimitAngle", ref _headXLimitAngle );
						_debugData.UpdateValue( "_headYUpLimitAngle", ref _headYUpLimitAngle );
						_debugData.UpdateValue( "_headYDownLimitAngle", ref _headYDownLimitAngle );
						_debugData.UpdateValue( "_headXRate", ref _headXRate );
						_debugData.UpdateValue( "_headYRate", ref _headYRate );
						_debugData.UpdateValue( "_headXOffset", ref _headZOffset );
#endif
						_headXLimitAngle *= Mathf.Deg2Rad;
						_headYUpLimitAngle *= Mathf.Deg2Rad;
						_headYDownLimitAngle *= Mathf.Deg2Rad;

						localDir.x *= _headXRate;
						localDir.y *= _headYRate;

						_SafeNormalize( ref localDir );

						if( localDir.z < 0.0f ) {
							float offset = Mathf.Clamp( _headZOffset, 0.0f, 0.99f );
							if( offset > IKEpsilon ) {
								if( localDir.z > -offset ) {
									localDir.z = 0.0f;
								} else {
									localDir.z = (localDir.z + offset) / (1.0f - offset);
								}
								_SafeNormalize( ref localDir );
							}
						}

						_LimitXY( ref localDir,
							Mathf.Sin( _headXLimitAngle ),
							Mathf.Sin( _headXLimitAngle ),
							Mathf.Sin( _headYDownLimitAngle ),
							Mathf.Sin( _headYUpLimitAngle ) );

						eyesDir = neckBaseBasis.Multiply( localDir );

						_SafeNormalize( ref eyesDir );

						{
							Vector3 xDir = neckBaseBasis.column0;
							Vector3 yDir = neckBaseBasis.column1;
							Vector3 zDir = eyesDir;

							if( !_ComputeBasisLockZ( out headBaseBasis, ref xDir, ref yDir, ref zDir ) ) {
								headBaseBasis = neckBaseBasis;
							}
						}

						_headBone.worldRotation = (headBaseBasis * _headBone._baseToWorldBasis).GetRotation();
					}

					Matrix3x3 headBasis = headBaseBasis * _internalValues.defaultRootBasisInv;

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
#if SAFULLBODYIK_DEBUG
					_debugData.UpdateValue( "_eyesHorzLimitAngle", ref _eyesHorzLimitAngle );
					_debugData.UpdateValue( "_eyesVertLimitAngle", ref _eyesVertLimitAngle );
					_debugData.UpdateValue( "_eyesXRate", ref _eyesXRate );
					_debugData.UpdateValue( "_eyesYRate", ref _eyesYRate );
					_debugData.UpdateValue( "_eyesOuterXRotRate", ref _eyesOuterXRotRate );
					_debugData.UpdateValue( "_eyesInnerXRotRate", ref _eyesInnerXRotRate );
					_debugData.UpdateValue( "_eyesXOffset", ref _eyesXOffset );
					_debugData.UpdateValue( "_eyesYOffset", ref _eyesYOffset );
					_debugData.UpdateValue( "_eyesZOffset", ref _eyesZOffset );
#endif
					float _innerMoveXRate = 0.063f;
					float _outerMoveXRate = 0.063f;
#if SAFULLBODYIK_DEBUG
					_debugData.UpdateValue( "_innerMoveXRate", ref _innerMoveXRate );
					_debugData.UpdateValue( "_outerMoveXRate", ref _outerMoveXRate );
#endif
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

					Vector3 headWorldPosition = neckBasis.Multiply( _headBone._defaultPosition - _neckBone._defaultPosition ) + _neckBone.worldPosition;
					Vector3 eyesPosition = headBasis.Multiply( _eyesEffector.defaultPosition - _headBone._defaultPosition ) + headWorldPosition;
					Vector3 eyesDir = _eyesEffector.worldPosition - eyesPosition;

					Matrix3x3 leftEyeBaseBasis = headBaseBasis;
					Matrix3x3 rightEyeBaseBasis = headBaseBasis;

					eyesDir = headBaseBasis.transpose.Multiply( eyesDir );

					_SafeNormalize( ref eyesDir );

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

					_SafeNormalize( ref leftEyeDir );
					_SafeNormalize( ref rightEyeDir );

					leftEyeDir = headBaseBasis.Multiply( leftEyeDir );
					rightEyeDir = headBaseBasis.Multiply( rightEyeDir );

					float leftXRate = (moveX >= 0.0f) ? _innerMoveXRate : _outerMoveXRate;
					float rightXRate = (moveX >= 0.0f) ? _outerMoveXRate : _innerMoveXRate;

					{
						Vector3 xDir = headBasis.column0;
						Vector3 yDir = headBasis.column1;
						Vector3 zDir = leftEyeDir;
						_ComputeBasisLockZ( out leftEyeBaseBasis, ref xDir, ref yDir, ref zDir );
					}

					{
						Vector3 xDir = headBasis.column0;
						Vector3 yDir = headBasis.column1;
						Vector3 zDir = rightEyeDir;
						_ComputeBasisLockZ( out rightEyeBaseBasis, ref xDir, ref yDir, ref zDir );
					}

					Vector3 leftEyeWorldPosition;
					Vector3 rightEyeWorldPosition;

					leftEyeWorldPosition = headBaseBasis.column0 * (leftXRate * moveX);
					rightEyeWorldPosition = headBaseBasis.column0 * (rightXRate * moveX);

					if( !_isHeadBoneLossyScaleFuzzyIdentity ) {
						leftEyeWorldPosition = Scale( ref leftEyeWorldPosition, ref _headBoneLossyScale );
						rightEyeWorldPosition = Scale( ref rightEyeWorldPosition, ref _headBoneLossyScale );
					}

					leftEyeWorldPosition += headBasis.Multiply( _unityChan_leftEyeDefaultPosition - _headBone._defaultPosition ) + headWorldPosition;
					rightEyeWorldPosition += headBasis.Multiply( _unityChan_rightEyeDefaultPosition - _headBone._defaultPosition ) + headWorldPosition;

					Matrix3x3 leftEyeBasis = leftEyeBaseBasis * _internalValues.defaultRootBasisInv;
					Matrix3x3 rightEyeBasis = rightEyeBaseBasis * _internalValues.defaultRootBasisInv;

					if( _leftEyeBone.transformIsAlive ) {
						_leftEyeBone.worldPosition = leftEyeBasis.Multiply( _leftEyeBone._defaultPosition - leftEyeDefaultPosition ) + leftEyeWorldPosition;
						_leftEyeBone.worldRotation = (leftEyeBaseBasis * _leftEyeBone._baseToWorldBasis).GetRotation();
					}

					if( _rightEyeBone.transformIsAlive ) {
						_rightEyeBone.worldPosition = rightEyeBasis.Multiply( _rightEyeBone._defaultPosition - rightEyeDefaultPosition ) + rightEyeWorldPosition;
						_rightEyeBone.worldRotation = (rightEyeBaseBasis * _rightEyeBone._baseToWorldBasis).GetRotation();
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
#if SAFULLBODYIK_DEBUG
					_debugData.UpdateValue( "_eyesHorzLimitAngle", ref _eyesHorzLimitAngle );
					_debugData.UpdateValue( "_eyesVertLimitAngle", ref _eyesVertLimitAngle );
					_debugData.UpdateValue( "_eyesXRate", ref _eyesXRate );
					_debugData.UpdateValue( "_eyesYRate", ref _eyesYRate );
					_debugData.UpdateValue( "_eyesOuterXRotRate", ref _eyesOuterXRotRate );
					_debugData.UpdateValue( "_eyesInnerXRotRate", ref _eyesInnerXRotRate );
#endif
					_eyesHorzLimitAngle *= Mathf.Deg2Rad;
					_eyesVertLimitAngle *= Mathf.Deg2Rad;

					Vector3 headWorldPosition = neckBasis.Multiply( _headBone._defaultPosition - _neckBone._defaultPosition ) + _neckBone.worldPosition;
					Vector3 eyesPosition = headBasis.Multiply( _eyesEffector.defaultPosition - _headBone._defaultPosition ) + headWorldPosition;
					Vector3 eyesDir = _eyesEffector.worldPosition - eyesPosition;

					Matrix3x3 leftEyeBaseBasis = headBaseBasis;
					Matrix3x3 rightEyeBaseBasis = headBaseBasis;

					eyesDir = headBaseBasis.transpose.Multiply( eyesDir );

					_SafeNormalize( ref eyesDir );

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

					_SafeNormalize( ref leftEyeDir );
					_SafeNormalize( ref rightEyeDir );

					leftEyeDir = headBaseBasis.Multiply( leftEyeDir );
					rightEyeDir = headBaseBasis.Multiply( rightEyeDir );

					if( _leftEyeBone.transformIsAlive ) {
						{
							Vector3 xDir = headBasis.column0;
							Vector3 yDir = headBasis.column1;
							Vector3 zDir = leftEyeDir;
							_ComputeBasisLockZ( out leftEyeBaseBasis, ref xDir, ref yDir, ref zDir );
						}

						Matrix3x3 leftEyeBasis = leftEyeBaseBasis * _internalValues.defaultRootBasisInv;
						_leftEyeBone.worldRotation = (leftEyeBaseBasis * _leftEyeBone._baseToWorldBasis).GetRotation();
					}

					if( _rightEyeBone.transformIsAlive ) {
						{
							Vector3 xDir = headBasis.column0;
							Vector3 yDir = headBasis.column1;
							Vector3 zDir = rightEyeDir;
							_ComputeBasisLockZ( out rightEyeBaseBasis, ref xDir, ref yDir, ref zDir );
						}

						Matrix3x3 rightEyeBasis = rightEyeBaseBasis * _internalValues.defaultRootBasisInv;
						_rightEyeBone.worldRotation = (rightEyeBaseBasis * _rightEyeBone._baseToWorldBasis).GetRotation();
					}
				}
			}
			
		}
	}
}