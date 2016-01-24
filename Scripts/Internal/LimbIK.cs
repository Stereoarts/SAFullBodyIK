// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing
using UnityEngine;

namespace SA
{
	public partial class FullBodyIK : MonoBehaviour
	{
		public class LimbIK
		{
			const float LegMinEffectorLengthRate = 0.5f;

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
						_legEffectorMinLength = n.magnitude * LegMinEffectorLengthRate;
                    }

					_leg_upperLimitNearCircleZ = _beginToBendingLength * Mathf.Cos( _leg_upperLimitLegAngle )
												+ _bendingToEndLength * Mathf.Cos( _leg_upperLimitKneeAngle );
					_leg_upperLimitNearCircleY = _beginToBendingLength * Mathf.Sin( _leg_upperLimitLegAngle )
												+ _bendingToEndLength * Mathf.Sin( _leg_upperLimitKneeAngle );
				}
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
					//Vector3 dirY = _effectorToBeginBoneBasis * -effectorDir;
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

			CachedDegreesToCos _presolvedLerpTheta = CachedDegreesToCos.zero;
			CachedDegreesToCos _automaticKneeBaseTheta = CachedDegreesToCos.zero;

			public bool Solve()
			{
				if( !_endEffector.positionEnabled ) {
					return _SolveEndRotation();
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

#if SAFULLBODYIK_DEBUG
				_debugData.UpdateValue( "_isEffectorPrefixer", ref _isEffectorPrefixer );
				_debugData.UpdateValue( "_isPresolveBending", ref _isPresolveBending );
				_debugData.UpdateValue( "_isSolveLimbIK", ref _isSolveLimbIK );
#endif

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
							Vector3 dirX = baseBasis.column0;
							Vector3 dirY = parentBaseBasis.column1; // Use parent boneTransform.
							Vector3 dirZ = Vector3.Cross( dirX, dirY );
							dirX = (_limbIKSide == Side.Left) ? -baseBasis.column0 : baseBasis.column0;
							if( _SafeNormalize( ref dirZ ) ) {
								float elbowCosTheta = Mathf.Cos( 30.0f * Mathf.Deg2Rad );
								float elbowSinTheta = Mathf.Sin( 30.0f * Mathf.Deg2Rad );
								bendingPos = beginPos + dirX * moveC
									+ -dirY * moveS * elbowCosTheta
									+ -dirZ * moveS * elbowSinTheta;
							} else { // Failsafe.
								bendingPos = beginPos + dirX * moveC + -baseBasis.column2 * moveS;
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
					return _SolveEndRotation();
				}

				Matrix3x3 beginBasis = Matrix3x3.identity;
				Matrix3x3 bendingBasis = Matrix3x3.identity;

				if( _limbIKType == LimbIKType.Arm ) {
					// Memo: Arm Bone Based Y Axis.
					if( _limbIKSide == Side.Left ) {
						solvedBeginToBendingDir = -solvedBeginToBendingDir;
						solvedBendingToEndDir = -solvedBendingToEndDir;
					}
					Vector3 dirY = Vector3.Cross( -solvedBeginToBendingDir, solvedBendingToEndDir );
					if( _limbIKSide == Side.Left ) {
						dirY = -dirY;
					}
					if( !_SafeNormalize( ref dirY ) ) {
						dirY = baseBasis.column1;
					}
					Vector3 dirZ = Vector3.Cross( solvedBeginToBendingDir, dirY );
					if( !_SafeNormalize( ref dirZ ) ) {
						dirZ = baseBasis.column2;
					}

					float rate = Mathf.Clamp01( effectorLen / _beginToEndLength );
					Vector3 basisY = Vector3.Lerp( dirY, baseBasis.column1, rate );
					Vector3 basisZ = Vector3.Lerp( dirZ, baseBasis.column2, rate );

					if( !_ComputeBasisLockX( out beginBasis, ref solvedBeginToBendingDir, ref basisY, ref basisZ ) ) {
						return _SolveEndRotation();
					}

					bendingBasis = beginBasis * _beginToBendingBoneBasis;

					dirZ = Vector3.Cross( solvedBendingToEndDir, dirY );
					if( !_SafeNormalize( ref dirZ ) ) {
						dirZ = beginBasis.column2;
					}
					basisY = Vector3.Lerp( dirY, bendingBasis.column1, rate );
					basisZ = Vector3.Lerp( dirZ, bendingBasis.column2, rate );
					if( !_ComputeBasisLockX( out bendingBasis, ref solvedBendingToEndDir, ref basisY, ref basisZ ) ) {
						return _SolveEndRotation();
					}
				} else {
					// Memo: Leg Bone Based X Axis.
					solvedBeginToBendingDir = -solvedBeginToBendingDir;
					solvedBendingToEndDir = -solvedBendingToEndDir;

					Vector3 basisX = baseBasis.column0;
					Vector3 basisZ = baseBasis.column2;
					if( !_ComputeBasisLockY( out beginBasis, ref basisX, ref solvedBeginToBendingDir, ref basisZ ) ) { // Test
						return _SolveEndRotation();
					}

					bendingBasis = beginBasis * _beginToBendingBoneBasis;

					basisX = bendingBasis.column0;
					basisZ = bendingBasis.column2;
					if( !_ComputeBasisLockY( out bendingBasis, ref basisX, ref solvedBendingToEndDir, ref basisZ ) ) { // Test
						return _SolveEndRotation();
					}
				}

				_beginBone.worldRotation = (beginBasis * _beginBone._boneToWorldBasis).GetRotation();
				_bendingBone.worldRotation = (bendingBasis * _bendingBone._boneToWorldBasis).GetRotation();

				_SolveEndRotation();
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