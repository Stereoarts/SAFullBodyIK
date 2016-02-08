// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing

//#define _ENABLE_LIMBIK_FORCEFIX

using UnityEngine;

namespace SA
{
	public partial class FullBodyIK : MonoBehaviour
	{
		public class LimbIK
		{
			const float _EffectorMaxLengthRate = 0.9999f;

			const float _LegEffectorMinLengthRate = 0.5f;

			const float _ElbowBasisForcefixEffectorLengthRate = 0.99f;
			const float _ElbowBasisForcefixEffectorLengthLerpRate = 0.03f;

#if SAFULLBODYIK_DEBUG
			DebugData _debugData;
#endif

			Settings _settings;

			public LimbIKLocation _limbIKLocation;
			LimbIKType _limbIKType;
			Side _limbIKSide;

			Bone _beginBone;
			Bone _bendingBone;
			Bone _endBone;
			Effector _bendingEffector;
			Effector _endEffector;

			Bone[] _armTwistBones;
			Bone[] _handTwistBones;

			float _beginToBendingLength;
			float _beginToBendingLengthSq;
			float _bendingToEndLength;
			float _bendingToEndLengthSq;
			float _beginToEndLength;
			float _beginToEndLengthSq;
			Matrix3x3 _solvedToBeginBoneBasis = Matrix3x3.identity;
			Matrix3x3 _beginBoneToSolvedBasis = Matrix3x3.identity;
			Matrix3x3 _solvedToBendingBoneBasis = Matrix3x3.identity;

			Matrix3x3 _beginToBendingBoneBasis = Matrix3x3.identity;
			Quaternion _endEffectorToWorldRotation = Quaternion.identity;

			Matrix3x3 _effectorToBeginBoneBasis = Matrix3x3.identity;
			float _defaultSinTheta = 0.0f;
			float _defaultCosTheta = 1.0f;

			float _effectorMaxLength = 0.0f;
			float _legEffectorMinLength = 0.0f; // for Test.

			InternalValues _internalValues;

			float _leg_upperLimitLegAngle = 60.0f * Mathf.Deg2Rad;
			float _leg_upperLimitKneeAngle = 45.0f * Mathf.Deg2Rad;
			float _leg_upperLimitNearCircleZ = 0.0f;
			float _leg_upperLimitNearCircleY = 0.0f;

			float _arm_elbowBasisForcefixEffectorLengthBegin = 0.0f;
			float _arm_elbowBasisForcefixEffectorLengthEnd = 0.0f;

			public LimbIK( FullBodyIK fullBodyIK, LimbIKLocation limbIKLocation )
			{
				Assert( fullBodyIK != null );
				if( fullBodyIK == null ) {
					return;
				}

#if SAFULLBODYIK_DEBUG
				_debugData = fullBodyIK._debugData;
#endif

				_settings = fullBodyIK._settings;

				_internalValues = fullBodyIK._internalValues;

				_limbIKLocation = limbIKLocation;
				_limbIKType = ToLimbIKType( limbIKLocation );
				_limbIKSide = ToLimbIKSide( limbIKLocation );

				if( _limbIKType == LimbIKType.Leg ) {
					var legBones = (_limbIKSide == Side.Left) ? fullBodyIK._leftLegBones : fullBodyIK._rightLegBones;
					var legEffectors = (_limbIKSide == Side.Left) ? fullBodyIK._leftLegEffectors : fullBodyIK._rightLegEffectors;
					_beginBone = legBones.leg;
					_bendingBone = legBones.knee;
					_endBone = legBones.foot;
					_bendingEffector = legEffectors.knee;
					_endEffector = legEffectors.foot;
				} else if( _limbIKType == LimbIKType.Arm ) {
					var armBones = (_limbIKSide == Side.Left) ? fullBodyIK._leftArmBones : fullBodyIK._rightArmBones;
					var armEffectors = (_limbIKSide == Side.Left) ? fullBodyIK._leftArmEffectors : fullBodyIK._rightArmEffectors;
					_beginBone = armBones.arm;
					_bendingBone = armBones.elbow;
					_endBone = armBones.wrist;
					_bendingEffector = armEffectors.elbow;
					_endEffector = armEffectors.wrist;
					_armTwistBones = armBones.armTwist;
					_handTwistBones = armBones.handTwist;
				}

				_Prepare( fullBodyIK );
			}

			void _Prepare( FullBodyIK fullBodyIK )
			{
				_beginToBendingLengthSq = (_bendingBone._defaultPosition - _beginBone._defaultPosition).sqrMagnitude;
				_beginToBendingLength = Sqrt( _beginToBendingLengthSq );
				_bendingToEndLengthSq = (_endBone._defaultPosition - _bendingBone._defaultPosition).sqrMagnitude;
				_bendingToEndLength = Sqrt( _bendingToEndLengthSq );
				_beginToEndLengthSq = (_endBone._defaultPosition - _beginBone._defaultPosition).sqrMagnitude;
				_beginToEndLength = Sqrt( _beginToEndLengthSq );

				Vector3 beginToEndDir = _endBone._defaultPosition - _beginBone._defaultPosition;
				if( _SafeNormalize( ref beginToEndDir ) ) {
					if( _limbIKType == LimbIKType.Arm ) {
						if( _limbIKSide == Side.Left ) {
							beginToEndDir = -beginToEndDir;
						}
						Vector3 dirY = _internalValues.defaultRootBasis.column1;
						Vector3 dirZ = _internalValues.defaultRootBasis.column2;
						if( _ComputeBasisLockX( out _effectorToBeginBoneBasis, ref beginToEndDir, ref dirY, ref dirZ ) ) {
                            _effectorToBeginBoneBasis = _effectorToBeginBoneBasis.transpose;
						}
					} else {
						beginToEndDir = -beginToEndDir;
						Vector3 dirX = _internalValues.defaultRootBasis.column0;
						Vector3 dirZ = _internalValues.defaultRootBasis.column2;
						// beginToEffectorBasis( identity to effectorDir(y) )
						if( _ComputeBasisLockY( out _effectorToBeginBoneBasis, ref dirX, ref beginToEndDir, ref dirZ ) ) {
							// effectorToBeginBasis( effectorDir(y) to identity )
							_effectorToBeginBoneBasis = _effectorToBeginBoneBasis.transpose;
						}
					}

					// effectorToBeginBasis( effectorDir(y) to _beginBone._localAxisBasis )
					_effectorToBeginBoneBasis *= _beginBone._localAxisBasis;
				}

				_defaultCosTheta = ComputeCosTheta(
					_bendingToEndLengthSq,			// lenASq
					_beginToEndLengthSq,			// lenBSq
					_beginToBendingLengthSq,		// lenCSq
					_beginToEndLength,				// lenB
					_beginToBendingLength );		// lenC

				_defaultSinTheta = Sqrt( Mathf.Clamp01( 1.0f - _defaultCosTheta * _defaultCosTheta ) );
				CheckNaN( _defaultSinTheta );

				_beginBoneToSolvedBasis = _beginBone._localAxisBasis;
				_solvedToBeginBoneBasis = _beginBone._localAxisBasisInv;
				_solvedToBendingBoneBasis = _bendingBone._localAxisBasisInv;

				_effectorMaxLength = _beginToBendingLength + Sqrt( _bendingToEndLengthSq );
				_endEffectorToWorldRotation = Inverse( _endEffector.defaultRotation ) * _endBone._defaultRotation;

				_beginToBendingBoneBasis = _beginBone._localAxisBasisInv * _bendingBone._localAxisBasis;

				if( _limbIKType == LimbIKType.Leg ) {
					if( fullBodyIK._leftLegBones != null &&
						fullBodyIK._rightLegBones != null &&
						fullBodyIK._leftLegBones.leg != null &&
						fullBodyIK._rightLegBones.leg != null ) {
						var n = fullBodyIK._rightLegBones.leg._defaultPosition - fullBodyIK._leftLegBones.leg._defaultPosition;
						_legEffectorMinLength = n.magnitude * _LegEffectorMinLengthRate;
                    }

					_leg_upperLimitNearCircleZ = _beginToBendingLength * Mathf.Cos( _leg_upperLimitLegAngle )
												+ _bendingToEndLength * Mathf.Cos( _leg_upperLimitKneeAngle );
					_leg_upperLimitNearCircleY = _beginToBendingLength * Mathf.Sin( _leg_upperLimitLegAngle )
												+ _bendingToEndLength * Mathf.Sin( _leg_upperLimitKneeAngle );
				}

				if( _limbIKType == LimbIKType.Arm ) {
					_arm_elbowBasisForcefixEffectorLengthBegin = _effectorMaxLength * (_ElbowBasisForcefixEffectorLengthRate - _ElbowBasisForcefixEffectorLengthLerpRate);
					_arm_elbowBasisForcefixEffectorLengthEnd = _effectorMaxLength * _ElbowBasisForcefixEffectorLengthRate;
				}

				_effectorMaxLength *= _EffectorMaxLengthRate;
			}

			// for animatorEnabled
			bool _isPresolvedBending = false;
			Matrix3x3 _presolvedBendingBasis = Matrix3x3.identity;
			Vector3 _presolvedEffectorDir = Vector3.zero;
			float _presolvedEffectorLength = 0.0f;

			// effectorDir to beginBoneBasis
			void _SolveBaseBasis( out Matrix3x3 baseBasis, ref Matrix3x3 parentBaseBasis, ref Vector3 effectorDir )
			{
				if( _limbIKType == LimbIKType.Arm ) {
					Vector3 dirX = (_limbIKSide == Side.Left) ? -effectorDir : effectorDir;
					Vector3 basisY = parentBaseBasis.column1;
					Vector3 basisZ = parentBaseBasis.column2;
					if( _ComputeBasisLockX( out baseBasis, ref dirX, ref basisY, ref basisZ ) ) {
						baseBasis *= _effectorToBeginBoneBasis;
					} else { // Failsafe.(Counts as default effectorDir.)
						baseBasis = parentBaseBasis * _beginBone._localAxisBasis;
					}
				} else {
					Vector3 dirY = -effectorDir;
					Vector3 basisX = parentBaseBasis.column0;
					Vector3 basisZ = parentBaseBasis.column2;
					if( _ComputeBasisLockY( out baseBasis, ref basisX, ref dirY, ref basisZ ) ) {
						baseBasis *= _effectorToBeginBoneBasis;
					} else { // Failsafe.(Counts as default effectorDir.)
						baseBasis = parentBaseBasis * _beginBone._localAxisBasis;
					}
				}
			}

			public void PresolveBeinding()
			{
				_isPresolvedBending = false;

				if( _beginBone == null ||
					!_beginBone.transformIsAlive ||
					_beginBone.parentBone == null ||
					!_beginBone.parentBone.transformIsAlive ||
					_bendingEffector == null ||
					_bendingEffector.bone == null ||
					!_bendingEffector.bone.transformIsAlive ||
					_endEffector == null ||
					_endEffector.bone == null ||
					!_endEffector.bone.transformIsAlive ) {
					return ; // Failsafe.
				}

				if( !_internalValues.animatorEnabled ) {
					return; // No require.
				}

				if( _bendingEffector.positionEnabled ) {
					return; // No require.
				}

				if( _limbIKType == LimbIKType.Leg ) {
					if( _settings.limbIK.presolveKneeRate < IKEpsilon ) {
						return; // No effect.
					}
				} else {
					if( _settings.limbIK.presolveElbowRate < IKEpsilon ) {
						return; // No effect.
					}
				}

				Vector3 beginPos = _beginBone.worldPosition;
				Vector3 bendingPos = _bendingEffector.bone.worldPosition;
				Vector3 effectorPos = _endEffector.bone.worldPosition;
				Vector3 effectorTrans = effectorPos - beginPos;
				Vector3 bendingTrans = bendingPos - beginPos;

				float effectorLen = effectorTrans.magnitude;
				float bendingLen = bendingTrans.magnitude;
				if( effectorLen <= IKEpsilon || bendingLen <= IKEpsilon ) {
					return;
				}

				Vector3 effectorDir = effectorTrans * (1.0f / effectorLen);
				Vector3 bendingDir = bendingTrans * (1.0f / bendingLen);

				Matrix3x3 parentBaseBasis = _beginBone.parentBone.worldRotation * _beginBone.parentBone._worldToBaseRotation;
				// Solve EffectorDir Based Basis.
				Matrix3x3 baseBasis;
				_SolveBaseBasis( out baseBasis, ref parentBaseBasis, ref effectorDir );

				_presolvedEffectorDir = effectorDir;
				_presolvedEffectorLength = effectorLen;

				Matrix3x3 toBasis;
				if( _limbIKType == LimbIKType.Arm ) {
					Vector3 dirX = (_limbIKSide == Side.Left) ? -bendingDir : bendingDir;
					Vector3 basisY = parentBaseBasis.column1;
					Vector3 basisZ = parentBaseBasis.column2;
					if( _ComputeBasisLockX( out toBasis, ref dirX, ref basisY, ref basisZ ) ) {
						_presolvedBendingBasis = toBasis * baseBasis.transpose;
						_isPresolvedBending = true;
					}
				} else {
					Vector3 dirY = -bendingDir;
					Vector3 basisX = parentBaseBasis.column0;
					Vector3 basisZ = parentBaseBasis.column2;
					if( _ComputeBasisLockY( out toBasis, ref basisX, ref dirY, ref basisZ ) ) {
						_presolvedBendingBasis = toBasis * baseBasis.transpose;
						_isPresolvedBending = true;
					}
				}
			}

			//------------------------------------------------------------------------------------------------------------

			bool _PrefixLegEffectorPos_UpperNear( ref Vector3 localEffectorTrans )
			{
				float y = localEffectorTrans.y - _leg_upperLimitNearCircleY;
				float z = localEffectorTrans.z;

				float rZ = _leg_upperLimitNearCircleZ;
                float rY = _leg_upperLimitNearCircleY + _legEffectorMinLength;

				if( rZ > IKEpsilon && rY > IKEpsilon ) {
					bool isLimited = false;

					z /= rZ;
					if( y > _leg_upperLimitNearCircleY ) {
						isLimited = true;
					} else {
						y /= rY;
						float len = Sqrt( y * y + z * z );
						if( len < 1.0f ) {
							isLimited = true;
						}
					}

					if( isLimited ) {
						float n = Sqrt( 1.0f - z * z );
						if( n > IKEpsilon ) { // Memo: Upper only.
							localEffectorTrans.y = -n * rY + _leg_upperLimitNearCircleY;
						} else { // Failsafe.
							localEffectorTrans.z = 0.0f;
							localEffectorTrans.y = -_legEffectorMinLength;
						}
						return true;
					}
				}

				return false;
			}

			static bool _PrefixLegEffectorPos_Circular_Far( ref Vector3 localEffectorTrans, float effectorLength )
			{
				return _PrefixLegEffectorPos_Circular( ref localEffectorTrans, effectorLength, true );
            }

			static bool _PrefixLegEffectorPos_Circular( ref Vector3 localEffectorTrans, float effectorLength, bool isFar )
			{
				float y = localEffectorTrans.y;
				float z = localEffectorTrans.z;
				float len = Sqrt( y * y + z * z );
				if( (isFar && len > effectorLength) || (!isFar && len < effectorLength) ) {
					float n = Sqrt( effectorLength * effectorLength - localEffectorTrans.z * localEffectorTrans.z );
					if( n > IKEpsilon ) { // Memo: Lower only.
						localEffectorTrans.y = -n;
					} else { // Failsafe.
						localEffectorTrans.z = 0.0f;
						localEffectorTrans.y = -effectorLength;
					}

					return true;
				}

				return false;
			}

			static bool _PrefixLegEffectorPos_Upper_Circular_Far( ref Vector3 localEffectorTrans,
				float centerPositionZ,
				float effectorLengthZ, float effectorLengthY )
			{
				if( effectorLengthY > IKEpsilon && effectorLengthZ > IKEpsilon ) {
					float y = localEffectorTrans.y;
					float z = localEffectorTrans.z - centerPositionZ;

					y /= effectorLengthY;
					z /= effectorLengthZ;

					float len = Sqrt( y * y + z * z );
					if( len > 1.0f ) {
						float n = Sqrt( 1.0f - z * z );
						if( n > IKEpsilon ) { // Memo: Upper only.
							localEffectorTrans.y = n * effectorLengthY;
						} else { // Failsafe.
							localEffectorTrans.z = centerPositionZ;
							localEffectorTrans.y = effectorLengthY;
						}

						return true;
					}
				}

				return false;
			}

			//------------------------------------------------------------------------------------------------------------

			// for Arms.

			const float _LocalDirMaxTheta = 0.99f;
			const float _LocalDirLerpTheta = 0.01f;

			static bool _NormalizeXZ( ref Vector3 localDirXZ )
			{
				float t = localDirXZ.x * localDirXZ.x + localDirXZ.z * localDirXZ.z;
				if( t > IKEpsilon ) {
					t = (float)System.Math.Sqrt( (float)t ); // Faster than Mathf.Sqrt()
					if( t > IKEpsilon ) {
						t = 1.0f / t;
						localDirXZ.x *= t;
						localDirXZ.z *= t;
						return true;
					} else {
						return false;
					}
				} else {
					return false;
				}
			}

			// Lefthand based.
			static void _ComputeLocalDirXZ( ref Vector3 localDir, out Vector3 localDirXZ )
			{
				if( localDir.y >= _LocalDirMaxTheta - IKEpsilon ) {
					localDirXZ = new Vector3( 1.0f, 0.0f, 0.0f );
				} else if( localDir.y > _LocalDirMaxTheta - _LocalDirLerpTheta - IKEpsilon ) {
					float r = (localDir.y - (_LocalDirMaxTheta - _LocalDirLerpTheta)) * (1.0f / _LocalDirLerpTheta);
					localDirXZ = new Vector3( localDir.x + (1.0f - localDir.x) * r, 0.0f, localDir.z - localDir.z * r );
					if( !_NormalizeXZ( ref localDirXZ ) ) {
						localDirXZ = new Vector3( 1.0f, 0.0f, 0.0f );
					}
				} else if( localDir.y <= -_LocalDirMaxTheta + IKEpsilon ) {
					localDirXZ = new Vector3( -1.0f, 0.0f, 0.0f );
				} else if( localDir.y < -(_LocalDirMaxTheta - _LocalDirLerpTheta - IKEpsilon) ) {
					float r = (-(_LocalDirMaxTheta - _LocalDirLerpTheta) - localDir.y) * (1.0f / _LocalDirLerpTheta);
					localDirXZ = new Vector3( localDir.x + (-1.0f - localDir.x) * r, 0.0f, localDir.z - localDir.z * r );
					if( !_NormalizeXZ( ref localDirXZ ) ) {
						localDirXZ = new Vector3( -1.0f, 0.0f, 0.0f );
					}
				} else {
					localDirXZ = new Vector3( localDir.x, 0.0f, localDir.z );
					if( !_NormalizeXZ( ref localDirXZ ) ) {
						localDirXZ = new Vector3( 1.0f, 0.0f, 0.0f );
					}
				}
			}

			static bool _NormalizeYZ( ref Vector3 localDirYZ )
			{
				float t = localDirYZ.y * localDirYZ.y + localDirYZ.z * localDirYZ.z;
				if( t > IKEpsilon ) {
					t = (float)System.Math.Sqrt( (float)t ); // Faster than Mathf.Sqrt()
					if( t > IKEpsilon ) {
						t = 1.0f / t;
						localDirYZ.y *= t;
						localDirYZ.z *= t;
						return true;
					} else {
						return false;
					}
				} else {
					return false;
				}
			}

			// Lefthand based.
			static void _ComputeLocalDirYZ( ref Vector3 localDir, out Vector3 localDirYZ )
			{
				if( localDir.x >= _LocalDirMaxTheta - IKEpsilon ) {
					localDirYZ = new Vector3( 0.0f, 0.0f, -1.0f );
				} else if( localDir.x > _LocalDirMaxTheta - _LocalDirLerpTheta - IKEpsilon ) {
					float r = (localDir.x - (_LocalDirMaxTheta - _LocalDirLerpTheta)) * (1.0f / _LocalDirLerpTheta);
					localDirYZ = new Vector3( 0.0f, localDir.y - localDir.y * r, localDir.z + (-1.0f - localDir.z) * r );
					if( !_NormalizeYZ( ref localDirYZ ) ) {
						localDirYZ = new Vector3( 0.0f, 0.0f, -1.0f );
					}
				} else if( localDir.x <= -_LocalDirMaxTheta + IKEpsilon ) {
					localDirYZ = new Vector3( 0.0f, 0.0f, 1.0f );
				} else if( localDir.x < -(_LocalDirMaxTheta - _LocalDirLerpTheta - IKEpsilon) ) {
					float r = (-(_LocalDirMaxTheta - _LocalDirLerpTheta) - localDir.x) * (1.0f / _LocalDirLerpTheta);
					localDirYZ = new Vector3( 0.0f, localDir.y - localDir.y * r, localDir.z + (1.0f - localDir.z) * r );
					if( !_NormalizeYZ( ref localDirYZ ) ) {
						localDirYZ = new Vector3( 0.0f, 0.0f, 1.0f );
					}
				} else {
					localDirYZ = new Vector3( 0.0f, localDir.y, localDir.z );
					if( !_NormalizeYZ( ref localDirYZ ) ) {
						localDirYZ = new Vector3( 0.0f, 0.0f, (localDir.x >= 0.0f) ? -1.0f : 1.0f );
					}
				}
			}

			//------------------------------------------------------------------------------------------------------------

			CachedDegreesToCos _presolvedLerpTheta = CachedDegreesToCos.zero;
			CachedDegreesToCos _automaticKneeBaseTheta = CachedDegreesToCos.zero;

			public bool Solve()
			{
				bool r = _SolveInternal();
				r |= _SolveEndRotation();

				return r;
			}

			public bool _SolveInternal()
			{
				if( !_endEffector.positionEnabled ) {
					return false;
				}

				if( _beginBone.parentBone == null || !_beginBone.parentBone.transformIsAlive ) {
					return false; // Failsafe.
				}

				Matrix3x3 parentBaseBasis = _beginBone.parentBone.worldRotation * _beginBone.parentBone._worldToBaseRotation;

				Vector3 beginPos = _beginBone.worldPosition;
				Vector3 bendingPos = _bendingEffector._hidden_worldPosition;
				Vector3 effectorPos = _endEffector._hidden_worldPosition;
				Vector3 effectorTrans = effectorPos - beginPos;

				float effectorLen = effectorTrans.magnitude;
				if( effectorLen <= IKEpsilon ) {
					return _SolveEndRotation();
				}
				if( _effectorMaxLength <= IKEpsilon ) {
					return _SolveEndRotation();
				}

				Vector3 effectorDir = effectorTrans * (1.0f / effectorLen);

				if( effectorLen > _effectorMaxLength ) {
					effectorTrans = effectorDir * _effectorMaxLength;
					effectorPos = beginPos + effectorTrans;
					effectorLen = _effectorMaxLength;
				}

				bool _isSolveLimbIK = true;
				bool _isPresolveBending = true;
				bool _isEffectorPrefixer = true;

				Vector3 localEffectorDir = new Vector3( 0.0f, 0.0f, 1.0f );

#if SAFULLBODYIK_DEBUG
				_debugData.UpdateValue( "_isEffectorPrefixer", ref _isEffectorPrefixer );
				_debugData.UpdateValue( "_isPresolveBending", ref _isPresolveBending );
				_debugData.UpdateValue( "_isSolveLimbIK", ref _isSolveLimbIK );
#endif
				if( _limbIKType == LimbIKType.Arm ) {
					// Detail map.
					float _armEffectorBackLimitAngle = 60.0f;
					float _armEffectorBackLimitAngleDepth = 0.5f;
					float _armEffectorInnerLimitMaxLengthRate = 0.8f;
					bool _armEffectorLimitEnabled = false;
#if SAFULLBODYIK_DEBUG
					_debugData.UpdateValue( "_armEffectorBackLimitAngle", ref _armEffectorBackLimitAngle );
					_debugData.UpdateValue( "_armEffectorBackLimitAngleDepth", ref _armEffectorBackLimitAngleDepth );
					_debugData.UpdateValue( "_armEffectorInnerLimitMaxLengthRate", ref _armEffectorInnerLimitMaxLengthRate );
					_debugData.UpdateValue( "_armEffectorLimitEnabled", ref _armEffectorLimitEnabled );
#endif
					CachedDegreesToCosSin _armEffectorLimitTheta = new CachedDegreesToCosSin( _armEffectorBackLimitAngle );

					Vector3 localEffectorLimitDir;
					if( _limbIKSide == Side.Left ) {
						localEffectorLimitDir = new Vector3( _armEffectorLimitTheta.cos, 0.0f, -_armEffectorLimitTheta.sin );
					} else {
						localEffectorLimitDir = new Vector3( -_armEffectorLimitTheta.cos, 0.0f, -_armEffectorLimitTheta.sin );
					}

					bool isLimited = false;
					localEffectorDir = parentBaseBasis.transpose.Multiply( ref effectorDir );
					float localEffectorLen = effectorLen;

					if( _armEffectorLimitEnabled ) {
						if( localEffectorDir.z < 0.0f ) { // Optimized: Prefilter.
							float d = Vector3.Dot( localEffectorLimitDir, localEffectorDir );
							if( d * localEffectorLen > _effectorMaxLength * _armEffectorBackLimitAngleDepth ) {
								Vector3 localEffectorTrans = localEffectorDir * localEffectorLen;
								var intersectPlane = new Plane( localEffectorLimitDir, localEffectorLimitDir * (_effectorMaxLength * _armEffectorBackLimitAngleDepth) );
								float distanceToPoint = intersectPlane.GetDistanceToPoint( localEffectorTrans );
								localEffectorTrans -= localEffectorLimitDir * distanceToPoint;
								float tempLen = localEffectorTrans.magnitude;
								if( tempLen > IKEpsilon ) {
									localEffectorLen = tempLen;
									localEffectorDir = localEffectorTrans * (1.0f / localEffectorLen);
									isLimited = true;
								}
							}
						}

						float maxLength = _effectorMaxLength;
						float localX = (_limbIKSide == Side.Left) ? localEffectorDir.x : -localEffectorDir.x;
						if( localX >= 0.0f ) {
							float r = _armEffectorInnerLimitMaxLengthRate;
							maxLength *= r + (1.0f - r) * (1.0f - localX);
						}

						if( localEffectorLen > maxLength ) {
							isLimited = true;
							localEffectorLen = maxLength;
						}
					}

					if( isLimited ) {
#if SAFULLBODYIK_DEBUG
						_debugData.AddPoint( effectorPos, Color.black, 0.05f );
#endif
						effectorDir = parentBaseBasis.Multiply( localEffectorDir );
						effectorLen = localEffectorLen;
						effectorTrans = effectorDir * effectorLen;
						effectorPos = beginPos + effectorTrans;
#if SAFULLBODYIK_DEBUG
						_debugData.AddPoint( effectorPos, Color.white, 0.05f );
#endif
					}
				}

				// pending: Detail processing for Arm too.
				if( _isEffectorPrefixer && _limbIKType == LimbIKType.Leg ) { // Override Effector Pos.
					Vector3 localEffectorTrans = parentBaseBasis.transpose * effectorTrans;

					bool isProcessed = false;
					bool isLimited = false;
					if( localEffectorTrans.z >= 0.0f ) { // Front
						if( localEffectorTrans.z >= _beginToBendingLength + _bendingToEndLength ) { // So far.
							isProcessed = true;
							localEffectorTrans.z = _beginToBendingLength + _bendingToEndLength;
							localEffectorTrans.y = 0.0f;
                        }

						if( !isProcessed &&
							localEffectorTrans.y >= -_legEffectorMinLength &&
							localEffectorTrans.z <= _leg_upperLimitNearCircleZ ) { // Upper(Near)
							isProcessed = true;
							isLimited = _PrefixLegEffectorPos_UpperNear( ref localEffectorTrans );
                        }

						if( !isProcessed &&
							localEffectorTrans.y >= 0.0f &&
							localEffectorTrans.z > _leg_upperLimitNearCircleZ ) { // Upper(Far)
							isProcessed = true;
							_PrefixLegEffectorPos_Upper_Circular_Far( ref localEffectorTrans,
								_leg_upperLimitNearCircleZ,
								_beginToBendingLength + _bendingToEndLength - _leg_upperLimitNearCircleZ,
								_leg_upperLimitNearCircleY );
                        }

						if( !isProcessed ) { // Lower
							isProcessed = true;
							isLimited = _PrefixLegEffectorPos_Circular_Far( ref localEffectorTrans, _beginToBendingLength + _bendingToEndLength );
                        }

					} else { // Back
						// Pending: Detail Processing.
						if( localEffectorTrans.y >= -_legEffectorMinLength ) {
							isLimited = true;
							localEffectorTrans.y = -_legEffectorMinLength;
                        } else {
							isLimited = _PrefixLegEffectorPos_Circular_Far( ref localEffectorTrans, _beginToBendingLength + _bendingToEndLength );
						}
					}

					if( isLimited ) {
#if SAFULLBODYIK_DEBUG
						_debugData.AddPoint( effectorPos, Color.black, 0.05f );
#endif
						effectorTrans = parentBaseBasis * localEffectorTrans;
						effectorLen = effectorTrans.magnitude;
						effectorPos = beginPos + effectorTrans;
						if( effectorLen > IKEpsilon ) {
							effectorDir = effectorTrans * (1.0f / effectorLen);
						}
#if SAFULLBODYIK_DEBUG
						_debugData.AddPoint( effectorPos, Color.white, 0.05f );
#endif
					}
				}

				Matrix3x3 baseBasis;
				_SolveBaseBasis( out baseBasis, ref parentBaseBasis, ref effectorDir );

				// Automatical bendingPos
				if( !_bendingEffector.positionEnabled ) {
					float presolvedBendingRate = (_limbIKType == LimbIKType.Leg)
						? _settings.limbIK.presolveKneeRate
						: _settings.limbIK.presolveElbowRate;

					float presolvedLerpAngle = (_limbIKType == LimbIKType.Leg)
						? _settings.limbIK.presolveKneeLerpAngle
						: _settings.limbIK.presolveElbowLerpAngle;

					float presolvedLerpLengthRate = (_limbIKType == LimbIKType.Leg)
						? _settings.limbIK.presolveKneeLerpLengthRate
						: _settings.limbIK.presolveElbowLerpLengthRate;

					Vector3 presolvedBendingPos = Vector3.zero;
					if( _isPresolveBending && _isPresolvedBending ) {
						if( _presolvedEffectorLength > IKEpsilon ) {
							float lerpLength = _presolvedEffectorLength * presolvedLerpLengthRate;
							if( lerpLength > IKEpsilon ) {
								float tempLength = Mathf.Abs( _presolvedEffectorLength - effectorLen );
								if( tempLength < lerpLength ) {
									presolvedBendingRate *= 1.0f - (tempLength / lerpLength);
                                } else {
									presolvedBendingRate = 0.0f;
								}
							} else { // Failsafe.
								presolvedBendingRate = 0.0f;
							}
						} else { // Failsafe.
							presolvedBendingRate = 0.0f;
						}

						if( presolvedBendingRate > IKEpsilon ) {
							_presolvedLerpTheta.Reset( presolvedLerpAngle );
							if( _presolvedLerpTheta < 1.0f - IKEpsilon ) { // Lerp
								float presolvedFeedbackTheta = Vector3.Dot( effectorDir, _presolvedEffectorDir );
								if( presolvedFeedbackTheta > _presolvedLerpTheta + IKEpsilon ) {
									float presolvedFeedbackRate = (presolvedFeedbackTheta - _presolvedLerpTheta) / (1.0f - _presolvedLerpTheta);
									presolvedBendingRate *= presolvedFeedbackRate;
								} else {
									presolvedBendingRate = 0.0f;
								}
							} else {
								presolvedBendingRate = 0.0f;
							}
						}

						if( presolvedBendingRate > IKEpsilon ) {
							Vector3 bendingDir;
							Matrix3x3 presolvedBendingBasis = baseBasis * _presolvedBendingBasis;
							if( _limbIKType == LimbIKType.Arm ) {
								bendingDir = (_limbIKSide == Side.Left) ? -presolvedBendingBasis.column0 : presolvedBendingBasis.column0;
							} else {
								bendingDir = -presolvedBendingBasis.column1;
							}

							presolvedBendingPos = beginPos + bendingDir * _beginToBendingLength;
							bendingPos = presolvedBendingPos; // Failsafe.
						}
					} else {
						presolvedBendingRate = 0.0f;
					}

					if( presolvedBendingRate < 1.0f - IKEpsilon ) {
						float cosTheta = ComputeCosTheta(
							_bendingToEndLengthSq,          // lenASq
							effectorLen * effectorLen,      // lenBSq
							_beginToBendingLengthSq,        // lenCSq
							effectorLen,                    // lenB
							_beginToBendingLength );        // lenC

						float sinTheta = Sqrt( Mathf.Clamp01( 1.0f - cosTheta * cosTheta ) );

						float moveC = _beginToBendingLength * (1.0f - Mathf.Max( _defaultCosTheta - cosTheta, 0.0f ));
						float moveS = _beginToBendingLength * Mathf.Max( sinTheta - _defaultSinTheta, 0.0f );

						if( _limbIKType == LimbIKType.Arm ) {
							Vector3 dirX = (_limbIKSide == Side.Left) ? -baseBasis.column0 : baseBasis.column0;
							{
#if true
								// Based localXZ
								float _armEffectorBackBeginAngle = 5.0f;
								float _armEffectorBackCoreBeginAngle = -10.0f;
								float _armEffectorBackCoreEndAngle = -30.0f;
								float _armEffectorBackEndAngle = -160.0f;

								// Based localYZ
								float _armEffectorBackCoreUpperAngle = 45.0f;
								float _armEffectorBackCoreLowerAngle = -45.0f;

								float _armElbowBaseAngle = 30.0f;
								float _armElbowLowerAngle = 90.0f;
								float _armElbowUpperAngle = 90.0f;
								float _armElbowBackUpperAngle = 180.0f;
								float _armElbowBackLowerAngle = 330.0f;

#if SAFULLBODYIK_DEBUG
								_debugData.UpdateValue( "_armEffectorBackBeginAngle", ref _armEffectorBackBeginAngle );
								_debugData.UpdateValue( "_armEffectorBackCoreBeginAngle", ref _armEffectorBackCoreBeginAngle );
								_debugData.UpdateValue( "_armEffectorBackCoreEndAngle", ref _armEffectorBackCoreEndAngle );
								_debugData.UpdateValue( "_armEffectorBackEndAngle", ref _armEffectorBackEndAngle );
								_debugData.UpdateValue( "_armEffectorBackCoreUpperAngle", ref _armEffectorBackCoreUpperAngle );
								_debugData.UpdateValue( "_armEffectorBackCoreLowerAngle", ref _armEffectorBackCoreLowerAngle );
								_debugData.UpdateValue( "_armElbowBaseAngle", ref _armElbowBaseAngle );
								_debugData.UpdateValue( "_armElbowLowerAngle", ref _armElbowLowerAngle );
								_debugData.UpdateValue( "_armElbowUpperAngle", ref _armElbowUpperAngle );
								_debugData.UpdateValue( "_armElbowBackUpperAngle", ref _armElbowBackUpperAngle );
								_debugData.UpdateValue( "_armElbowBackLowerAngle", ref _armElbowBackLowerAngle );
#endif
								float _elbowAngle = _armElbowBaseAngle;

								CachedDegreesToSin _armEffectorBackBeginTheta = new CachedDegreesToSin( _armEffectorBackBeginAngle );
								CachedDegreesToSin _armEffectorBackCoreBeginTheta = new CachedDegreesToSin( _armEffectorBackCoreBeginAngle );
								CachedDegreesToCos _armEffectorBackCoreEndTheta = new CachedDegreesToCos( _armEffectorBackCoreEndAngle );
								CachedDegreesToCos _armEffectorBackEndTheta = new CachedDegreesToCos( _armEffectorBackEndAngle );

								CachedDegreesToSin _armEffectorBackCoreUpperTheta = new CachedDegreesToSin( _armEffectorBackCoreUpperAngle );
								CachedDegreesToSin _armEffectorBackCoreLowerTheta = new CachedDegreesToSin( _armEffectorBackCoreLowerAngle );

								Vector3 localXZ; // X is reversed in RightSide.
								Vector3 localYZ;

								Vector3 localDir = (_limbIKSide == Side.Left) ? localEffectorDir : new Vector3( -localEffectorDir.x, localEffectorDir.y, localEffectorDir.z );
								_ComputeLocalDirXZ( ref localDir, out localXZ ); // Lefthand Based.
								_ComputeLocalDirYZ( ref localDir, out localYZ ); // Lefthand Based.

								if( localDir.y < 0.0f ) {
									_elbowAngle = Mathf.Lerp( _elbowAngle, _armElbowLowerAngle, -localDir.y );
								} else {
									_elbowAngle = Mathf.Lerp( _elbowAngle, _armElbowUpperAngle, localDir.y );
								}

								if( localXZ.z < _armEffectorBackBeginTheta.sin &&
									localXZ.x > _armEffectorBackEndTheta.cos ) {

									float targetAngle;
									if( localYZ.y >= _armEffectorBackCoreUpperTheta.sin ) {
										targetAngle = _armElbowBackUpperAngle;
									} else if( localYZ.y <= _armEffectorBackCoreLowerTheta.sin ) {
										targetAngle = _armElbowBackLowerAngle;
									} else {
										float t = _armEffectorBackCoreUpperTheta.sin - _armEffectorBackCoreLowerTheta.sin;
										if( t > IKEpsilon ) {
											float r = (localYZ.y - _armEffectorBackCoreLowerTheta.sin) / t;
											targetAngle = Mathf.Lerp( _armElbowBackLowerAngle, _armElbowBackUpperAngle, r );
										} else {
											targetAngle = _armElbowBackLowerAngle;
										}
									}

									if( localXZ.x < _armEffectorBackCoreEndTheta.cos ) {
										float t = _armEffectorBackCoreEndTheta.cos - _armEffectorBackEndTheta.cos;
										if( t > IKEpsilon ) {
											float r = (localXZ.x - _armEffectorBackEndTheta.cos) / t;

											if( localYZ.y >= _armEffectorBackCoreUpperTheta.sin ) {
												_elbowAngle = Mathf.Lerp( _elbowAngle, targetAngle - 360.0f, r );
											} else if( localYZ.y <= _armEffectorBackCoreLowerTheta.sin ) {
												_elbowAngle = Mathf.Lerp( _elbowAngle, targetAngle, r );
											} else {
												float angle0 = Mathf.Lerp( _elbowAngle, targetAngle, r ); // Lower
												float angle1 = Mathf.Lerp( _elbowAngle, targetAngle - 360.0f, r ); // Upper
												float t2 = _armEffectorBackCoreUpperTheta.sin - _armEffectorBackCoreLowerTheta.sin;
												if( t2 > IKEpsilon ) {
													// Weighted average.
													float r2 = (localYZ.y - _armEffectorBackCoreLowerTheta.sin) / t2;

													float w0 = (r <= IKEpsilon) ? 1.0f : (1.0f / r);
													float w1 = ((1.0f - r) <= IKEpsilon) ? 1.0f : (1.0f / (1.0f - r));
													float h0 = (r2 <= IKEpsilon) ? 1.0f : (1.0f / r2);
													float h1 = ((1.0f - r2) <= IKEpsilon) ? 1.0f : (1.0f / (1.0f - r2));

													float wh_t = w0 + w1 + h0 + h1;
													if( wh_t > IKEpsilon ) {
														_elbowAngle = (_elbowAngle * w0 + targetAngle * w1 + angle0 * h0 + angle1 * h1) / wh_t;
                                                    } else {
														_elbowAngle = angle0;
													}
												} else { // Failsafe.
													_elbowAngle = angle0;
												}
											}
										}
									} else if( localXZ.z > _armEffectorBackCoreBeginTheta.sin ) {
										float t = (_armEffectorBackBeginTheta.sin - _armEffectorBackCoreBeginTheta.sin);
										if( t > IKEpsilon ) {
											float r = (_armEffectorBackBeginTheta.sin - localXZ.z) / t;
											_elbowAngle = Mathf.Lerp( _elbowAngle, targetAngle - 360.0f, r );
										} else { // Failsafe.
											_elbowAngle = targetAngle;
										}
									} else {
										_elbowAngle = targetAngle;
									}
								}
#endif

								Vector3 dirY = parentBaseBasis.column1;
								Vector3 dirZ = Vector3.Cross( baseBasis.column0, dirY );
								dirY = Vector3.Cross( dirZ, baseBasis.column0 );
								if( !_SafeNormalize( ref dirY, ref dirZ ) ) { // Failsafe.
									dirY = parentBaseBasis.column1;
									dirZ = parentBaseBasis.column2;
								}

								CachedDegreesToCosSin _armElbowTheta = new CachedDegreesToCosSin( _elbowAngle );

								bendingPos = beginPos + dirX * moveC
									+ -dirY * moveS * _armElbowTheta.cos
									+ -dirZ * moveS * _armElbowTheta.sin;
							}
						} else { // Leg
							if( IsFuzzy( _settings.limbIK.automaticKneeBaseAngle, 0.0f ) ) {
								bendingPos = beginPos + -baseBasis.column1 * moveC + baseBasis.column2 * moveS;
							} else {
								_automaticKneeBaseTheta.Reset( _settings.limbIK.automaticKneeBaseAngle );

								float kneeSin = _automaticKneeBaseTheta.cos;
                                float kneeCos = Sqrt( 1.0f - kneeSin * kneeSin );
								if( _limbIKSide == Side.Right ) {
									if( _settings.limbIK.automaticKneeBaseAngle >= 0.0f ) {
										kneeCos = -kneeCos;
									}
								} else {
									if( _settings.limbIK.automaticKneeBaseAngle < 0.0f ) {
										kneeCos = -kneeCos;
									}
								}

								bendingPos = beginPos + -baseBasis.column1 * moveC
									+ baseBasis.column0 * moveS * kneeCos
									+ baseBasis.column2 * moveS * kneeSin;
							}
						}
					}

					if( presolvedBendingRate > IKEpsilon ) {
						bendingPos = Vector3.Lerp( bendingPos, presolvedBendingPos, presolvedBendingRate );
                    }
				}

				bool isSolved = false;
				Vector3 solvedBeginToBendingDir = Vector3.zero;
				Vector3 solvedBendingToEndDir = Vector3.zero;

				if( _isSolveLimbIK ) {
					Vector3 beginToBendingTrans = bendingPos - beginPos;
					Vector3 intersectBendingTrans = beginToBendingTrans - effectorDir * Vector3.Dot( effectorDir, beginToBendingTrans );
					float intersectBendingLen = intersectBendingTrans.magnitude;

					if( intersectBendingLen > IKEpsilon ) {
						Vector3 intersectBendingDir = intersectBendingTrans * (1.0f / intersectBendingLen);

						float bc2 = 2.0f * _beginToBendingLength * effectorLen;
						if( bc2 > IKEpsilon ) {
							float effectorCosTheta = (_beginToBendingLengthSq + effectorLen * effectorLen - _bendingToEndLengthSq) / bc2;
							float effectorSinTheta = Sqrt( Mathf.Clamp01( 1.0f - effectorCosTheta * effectorCosTheta ) );

							Vector3 beginToInterTranslate = effectorDir * effectorCosTheta * _beginToBendingLength
															+ intersectBendingDir * effectorSinTheta * _beginToBendingLength;
							Vector3 interToEndTranslate = effectorPos - (beginPos + beginToInterTranslate);

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
						Vector3 interPos = beginPos + bendingDir * _beginToBendingLength;
						Vector3 endDir = effectorPos - interPos;
						if( _SafeNormalize( ref endDir ) ) {
							isSolved = true;
							solvedBeginToBendingDir = bendingDir;
							solvedBendingToEndDir = endDir;
						}
					}
				}

				if( !isSolved ) {
					return false;
				}

				Matrix3x3 beginBasis = Matrix3x3.identity;
				Matrix3x3 bendingBasis = Matrix3x3.identity;

				if( _limbIKType == LimbIKType.Arm ) {
					// Memo: Arm Bone Based Y Axis.
					if( _limbIKSide == Side.Left ) {
						solvedBeginToBendingDir = -solvedBeginToBendingDir;
						solvedBendingToEndDir = -solvedBendingToEndDir;
					}

					Vector3 basisY = parentBaseBasis.column1;
					Vector3 basisZ = parentBaseBasis.column2;
					if( !_ComputeBasisLockX( out beginBasis, ref solvedBeginToBendingDir, ref basisY, ref basisZ ) ) {
						return false;
					}

					//_arm_elbowBasisForcefixEffectorLengthBegin = _effectorMaxLength * (_ElbowBasisForcefixEffectorLengthRate - _ElbowBasisForcefixEffectorLengthLerpRate);
					//_arm_elbowBasisForcefixEffectorLengthEnd = _effectorMaxLength * _ElbowBasisForcefixEffectorLengthRate;

					{
						if( effectorLen > _arm_elbowBasisForcefixEffectorLengthEnd ) {
							basisY = beginBasis.Multiply_Column1( ref _beginToBendingBoneBasis );
						} else {
							basisY = Vector3.Cross( -solvedBeginToBendingDir, solvedBendingToEndDir ); // Memo: Require to MaxEffectorLengthRate is less than 1.0
							if( _limbIKSide == Side.Left ) {
								basisY = -basisY;
							}

							if( effectorLen > _arm_elbowBasisForcefixEffectorLengthBegin ) {
								float t = _arm_elbowBasisForcefixEffectorLengthEnd - _arm_elbowBasisForcefixEffectorLengthBegin;
								if( t > IKEpsilon ) {
									float r = (effectorLen - _arm_elbowBasisForcefixEffectorLengthBegin) / t;
									basisY = Vector3.Lerp( basisY, beginBasis.Multiply_Column1( ref _beginToBendingBoneBasis ), r );
                                }
                            }
						}

						if( !_ComputeBasisFromXYLockX( out bendingBasis, ref solvedBendingToEndDir, ref basisY ) ) {
							return false;
						}
					}
				} else {
					// Memo: Leg Bone Based X Axis.
					solvedBeginToBendingDir = -solvedBeginToBendingDir;
					solvedBendingToEndDir = -solvedBendingToEndDir;

					Vector3 basisX = baseBasis.column0;
					Vector3 basisZ = baseBasis.column2;
					if( !_ComputeBasisLockY( out beginBasis, ref basisX, ref solvedBeginToBendingDir, ref basisZ ) ) {
						return false;
					}

					basisX = beginBasis.Multiply_Column0( ref _beginToBendingBoneBasis );

					if( !_ComputeBasisFromXYLockY( out bendingBasis, ref basisX, ref solvedBendingToEndDir ) ) {
						return false;
					}
				}

				_beginBone.worldRotation = (beginBasis * _beginBone._boneToWorldBasis).GetRotation();
				_bendingBone.worldRotation = (bendingBasis * _bendingBone._boneToWorldBasis).GetRotation();
				return true;
			}

			bool _SolveEndRotation()
			{
				if( !_endEffector.rotationEnabled ) {
					return false;
				}

				var r = _endEffector.worldRotation * _endEffectorToWorldRotation;
				_endBone.worldRotation = r;
				return true;
			}

			public void Twist()
			{
				// Test Code!!!
				// todo: Refactoring.
				if( _handTwistBones != null ) {
					int boneLength = 0;
					for( int i = 0; i < _handTwistBones.Length; ++i ) {
						if( _handTwistBones[i] != null && _handTwistBones[i].transformIsAlive ) {
							++boneLength;
						}
					}

					if( boneLength > 0 ) {
						var bendingBoneBasis = new Matrix3x3( _bendingBone.worldRotation * _bendingBone._worldToBoneRotation );
						var endBoneBasis = new Matrix3x3( _endBone.worldRotation * _endBone._worldToBaseRotation );
						endBoneBasis *= _bendingBone._baseToBoneBasis; // Attention!!!

						Vector3 dirZ = endBoneBasis.column2;
						Vector3 dirX = bendingBoneBasis.column0;
						Vector3 dirY = Vector3.Cross( dirZ, dirX );
						dirZ = Vector3.Cross( dirX, dirY );
						if( _SafeNormalize( ref dirY, ref dirZ ) ) { // Lock dirX(bendingBoneBasis.column0)
							var baseBasis = bendingBoneBasis * _bendingBone._boneToBaseBasis;
							var baseBasisTo = Matrix3x3.FromColumn( dirX, dirY, dirZ ) * _bendingBone._boneToBaseBasis;

							int boneIndex = 0;
							for( int i = 0; i < _handTwistBones.Length; ++i ) {
								if( _handTwistBones[i] != null && _handTwistBones[i].transformIsAlive ) {
									Matrix3x3 tempBasis;

									float rate = (float)(boneIndex + 1) / (float)(boneLength + 1);

									_Lerp( out tempBasis, ref baseBasis, ref baseBasisTo, rate );

									_handTwistBones[i].worldRotation = (tempBasis * _handTwistBones[i]._baseToWorldBasis).GetRotation();


									++boneIndex;
								}
							}
						}
					}
				}
			}
		}
	}
}