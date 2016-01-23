// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing

#if SAFULLBODYIK_DEBUG
//#define SAFULLBODYIK_DEBUG_DETAIL_SOLVETORSO
//#define SAFULLBODYIK_DEBUG_DETAIL_SOLVELEGS
#endif

using UnityEngine;
using Util = SA.FullBodyIKUtil;

namespace SA
{
	public partial class FullBodyIK : MonoBehaviour
	{
		public class BodyIK
		{
#if SAFULLBODYIK_DEBUG
			DebugData _debugData;
#endif

			public bool enabled = true;

			Bone _pelvisBone; // Null accepted.
			Bone[] _torsoBones; // Null accepted.
			Bone _torsoBone; // Null accepted.
			Bone _torsoUBone; // Null accepted.
			Bone _neckBone; // Null accepted.
			Bone[] _legBones; // Null accepted.
			Bone[] _shoulderBones; // Null accepted.
			Bone[] _armBones; // Null accepted.
			Effector _pelvisEffector;
			Effector _neckEffector;
			Effector _eyesEffector;
			Effector[] _armEffectors = new Effector[2];
			Effector[] _wristEffectors = new Effector[2];
			Effector[] _footEffectors = new Effector[2];

			Vector3 _defaultCenterLegPos = Vector3.zero;
			Vector3 _defaultCenterFootPos = Vector3.zero;
			Matrix3x3 _centerLegBoneBasisInv = Matrix3x3.identity;
			Matrix3x3 _centerLegToArmBasis = Matrix3x3.identity;
			Matrix3x3 _centerLegToArmBasisInv = Matrix3x3.identity;
			Matrix3x3 _centerLegToArmBaseBasis = Matrix3x3.identity; // contain rootBaseBasis.
			Matrix3x3 _centerLegToArmBaseBasisInv = Matrix3x3.identity;
			FastLength[] _shoulderToArmLength = new FastLength[2];
			FastLength[] _legEffectorMaxLength = new FastLength[2];
			FastLength[] _armEffectorMaxLength = new FastLength[2];

			Vector3 _defaultCenterArmPos = Vector3.zero;
			float _defaultCenterLegLen; // LeftLeg to RightLeg Length.
			float _defaultCenterLegHalfLen; // LeftLeg to RightLeg Length / 2.

			Vector3 _defaultCenterEyePos = Vector3.zero;

			SolverInternal _solverInternal;

			float _shoulderLimitYPlus	= Mathf.Sin( 30.0f * Mathf.Deg2Rad );
			float _shoulderLimitYMinus	= Mathf.Sin(  1.0f * Mathf.Deg2Rad );
			float _shoulderLimitZ		= Mathf.Sin( 30.0f * Mathf.Deg2Rad );

			float _torsoDirXLegToArmRate = 0.5f;

			Settings _settings;
			InternalValues _internalValues;

			public BodyIK( FullBodyIK fullBodyIK )
			{
				Assert( fullBodyIK != null );

				_settings = fullBodyIK._settings;
				_internalValues = fullBodyIK._internalValues;

#if SAFULLBODYIK_DEBUG
				_debugData = fullBodyIK._debugData;
#endif
				_pelvisBone = _PrepareBone( fullBodyIK._bodyBones.pelvis );
				_neckBone = _PrepareBone( fullBodyIK._headBones.neck );
				_pelvisEffector = fullBodyIK._bodyEffectors.pelvis;
				_neckEffector = fullBodyIK._headEffectors.neck;
				_eyesEffector = fullBodyIK._headEffectors.eyes;
				_armEffectors[0] = fullBodyIK._leftArmEffectors.arm;
				_armEffectors[1] = fullBodyIK._rightArmEffectors.arm;
				_wristEffectors[0] = fullBodyIK._leftArmEffectors.wrist;
				_wristEffectors[1] = fullBodyIK._rightArmEffectors.wrist;
				_footEffectors[0] = fullBodyIK._leftLegEffectors.foot;
				_footEffectors[1] = fullBodyIK._rightLegEffectors.foot;

				_torsoBones = _PrepareTorsoBones( fullBodyIK.bones );
				if( _torsoBones != null && _torsoBones.Length > 0 ) {
					_torsoBone = _torsoBones[0];
					_torsoUBone = _torsoBones[_torsoBones.Length - 1];
				}

				// Memo: These should be pair bones.(Necessary each side bones.)
				_legBones = _PrepareBones( fullBodyIK._leftLegBones.leg, fullBodyIK._rightLegBones.leg );
				_armBones = _PrepareBones( fullBodyIK._leftArmBones.arm, fullBodyIK._rightArmBones.arm );
				_shoulderBones = _PrepareBones( fullBodyIK._leftArmBones.shoulder, fullBodyIK._rightArmBones.shoulder );

				_Prepare( fullBodyIK );
			}

			static Bone _PrepareBone( Bone bone )
			{
				return (bone != null && bone.transformIsAlive) ? bone : null;
			}

			static Bone[] _PrepareTorsoBones( Bone[] bones )
			{
				if( bones == null || bones.Length != (int)BoneLocation.Max ) {
					Assert( false );
					return null;
				}

				int torsoLength = 0;
				for( int i = (int)BoneLocation.Torso; i <= (int)BoneLocation.TorsoU; ++i ) {
					if( bones[i] != null && bones[i].transformIsAlive ) {
						++torsoLength;
					}
				}

				if( torsoLength == 0 ) {
					return null;
				}

				Bone[] torsoBones = new Bone[torsoLength];
				int index = 0;
				for( int i = (int)BoneLocation.Torso; i <= (int)BoneLocation.TorsoU; ++i ) {
					if( bones[i] != null && bones[i].transformIsAlive ) {
						torsoBones[index] = bones[i];
						++index;
					}
				}

				return torsoBones;
			}

			static Bone[] _PrepareBones( Bone leftBone, Bone rightBone )
			{
				Assert( leftBone != null && rightBone != null );
				if( leftBone != null && rightBone != null ) {
					if( leftBone.transformIsAlive && rightBone.transformIsAlive ) {
						var bones = new Bone[2];
						bones[0] = leftBone;
						bones[1] = rightBone;
						return bones;
					}
				}

				return null;
			}

			void _Prepare( FullBodyIK fullBodyIK )
			{
				if( _legBones != null ) {
					_defaultCenterLegPos = (_legBones[0]._defaultPosition + _legBones[1]._defaultPosition) * 0.5f;
				}

				if( _armBones != null ) {
					_defaultCenterArmPos = (_armBones[1]._defaultPosition + _armBones[0]._defaultPosition) * 0.5f;
					Vector3 dirX = _armBones[1]._defaultPosition - _armBones[0]._defaultPosition;
					Vector3 dirY = _defaultCenterArmPos - _defaultCenterLegPos;
					if( _NormalizeAndComputeBasisFromXYLockY( ref _centerLegToArmBasis, ref dirX, ref dirY ) ) {
						_centerLegToArmBasisInv = _centerLegToArmBasis.transpose;
						_centerLegToArmBaseBasisInv = _centerLegToArmBasisInv * _internalValues.defaultRootBasis;
						_centerLegToArmBaseBasis = _centerLegToArmBaseBasisInv.transpose;
					}
				}

				if( _footEffectors != null ) {
					if( _footEffectors[0].bone != null && _footEffectors[1].bone != null ) {
						_defaultCenterFootPos = (_footEffectors[0].bone._defaultPosition + _footEffectors[1].bone._defaultPosition) * 0.5f;
						_defaultCenterLegLen = (_footEffectors[1].bone._defaultPosition - _footEffectors[0].bone._defaultPosition).magnitude;
						_defaultCenterLegHalfLen = _defaultCenterLegLen * 0.5f;
					}
				}

				if( _torsoBone != null && _legBones != null ) {
					Assert( _torsoBone.transformIsAlive && _legBones[0].transformIsAlive && _legBones[1].transformIsAlive ); // Already checked.
					if( _ComputeCenterLegBasis( out _centerLegBoneBasisInv,
						ref _torsoBone._defaultPosition,
						ref _legBones[0]._defaultPosition,
						ref _legBones[1]._defaultPosition ) ) {
						_centerLegBoneBasisInv = _centerLegBoneBasisInv.transpose;
					}
				}

				if( _shoulderBones != null && _armBones != null ) {
					for( int i = 0; i < 2; ++i ) {
						Assert( _shoulderBones[i].transformIsAlive && _armBones[i].transformIsAlive ); // Already checked.
						_shoulderToArmLength[i] = FastLength.FromLengthSq( (_shoulderBones[i]._defaultPosition - _armBones[i]._defaultPosition).sqrMagnitude );
					}
				}

				for( int i = 0; i < 2; ++i ) {
					Bone kneeBone = (i == 0) ? fullBodyIK._leftLegBones.knee : fullBodyIK._rightLegBones.knee;
					Bone elbowBone = (i == 0) ? fullBodyIK._leftArmBones.elbow : fullBodyIK._rightArmBones.elbow;
					float legLength = 0.0f;
					float armLength = 0.0f;
					if( _legBones != null ) {
						Assert( _legBones[i].transformIsAlive ); // Already checked. 
						if( kneeBone.transformIsAlive && _footEffectors[i].bone.transformIsAlive ) {
							legLength = (_legBones[i]._defaultPosition - kneeBone._defaultPosition).magnitude
										+ (kneeBone._defaultPosition - _footEffectors[i].bone._defaultPosition).magnitude;
						}
					}
					if( _armBones != null ) {
						Assert( _armBones[i].transformIsAlive ); // Already checked. 
						if( elbowBone.transformIsAlive && _wristEffectors[i].bone.transformIsAlive ) {
							armLength = (_armBones[i]._defaultPosition - elbowBone._defaultPosition).magnitude
										+ (elbowBone._defaultPosition - _wristEffectors[i].bone._defaultPosition).magnitude;
						}
					}
					_legEffectorMaxLength[i] = FastLength.FromLength( legLength );
					_armEffectorMaxLength[i] = FastLength.FromLength( armLength );
				}

				if( _eyesEffector != null ) {
					_defaultCenterEyePos = _eyesEffector.defaultPosition;
                }
			}

			public static bool _KeepLength( ref Vector3 posTo, ref Vector3 posFrom, ref FastLength length )
			{
				Vector3 v = posTo - posFrom;
				float len = v.magnitude;
				if( len > IKEpsilon ) {
					v = v * (length.length / len);
					posTo = posFrom + v;
					return true;
				}

				return false;
			}

#if false
			public static bool _KeepLength2(
				ref Vector3 posA,
				ref Vector3 posB,
				ref Vector3 posC,
				ref FastLength posAtoBLen,
				ref FastLength posAtoCLen )
			{
				Vector3 solvedBeginToBending;
				Vector3 solvedBendingToEnd;
				if( _SolveLimbPosBased(
					out solvedBeginToBending,
					out solvedBendingToEnd,
					ref posB,
					ref posA,
					ref posC,
					ref posAtoBLen,
					ref posAtoCLen ) ) {
					posA = posB + solvedBeginToBending;
					return true;
				}

				return false;
			}
#endif

			public bool Solve()
			{
				if( !_IsEffectorEnabled() ) {
					return false;
				}

				if( !_PrepareSolverInternal() ) {
					return false;
				}

				var temp = _solverInternal;
				
#if false
				// Test.
				temp.arms.SolveTargetBeginPos();
				temp.legs.SolveTargetBeginPos();

				if( !temp.arms.targetBeginPosEnabledAnything && !temp.legs.targetBeginPosEnabledAnything ) {
					return false;
				}
#endif

				bool neckEnabled = (_neckBone != null && _neckBone.transformIsAlive);

				// Memo: arms / legs don't setup here. (Already prepared in _PrepareSolverInternal())

				if( temp.torsoPos != null ) {
					for( int i = 0; i < temp.torsoPos.Length; ++i ) {
						temp.torsoPos[i] = _torsoBones[i].worldPosition;
					}
				}
				if( neckEnabled ) {
					temp.neckPos = _neckBone.worldPosition;
				}
				if( temp.shoulderPos != null ) {
					for( int i = 0; i < 2; ++i ) {
						temp.shoulderPos[i] = _shoulderBones[i].worldPosition;
					}
				}

				temp.SetDirtyVariables();

				if( _internalValues.resetTransforms ) {
					_ResetTransforms();
				} else if( _internalValues.animatorEnabled ) {
					_PresolvePelvis();
				}

				bool _isProcessLowerSolve = true;
				bool _isProcessUpperSolve = true;
				bool _isProcessLowerSolve2 = true;
				bool _isComputeWorldTransform = true;

#if SAFULLBODYIK_DEBUG
				_debugData.UpdateValue( "_isProcessLowerSolve", ref _isProcessLowerSolve );
				_debugData.UpdateValue( "_isProcessUpperSolve", ref _isProcessUpperSolve );
				_debugData.UpdateValue( "_isProcessLowerSolve2", ref _isProcessLowerSolve2 );
				_debugData.UpdateValue( "_isComputeWorldTransform", ref _isComputeWorldTransform );
#endif

				if( _isProcessLowerSolve ) {
					_LowerSolve( false, true );
				}

				if( _isProcessUpperSolve ) {
					_UpperSolve();
				}

				if( _isProcessLowerSolve2 ) {
					_LowerSolve( false, false );
				}

				if( _isComputeWorldTransform ) {
					_ComputeWorldTransform();
				}

#if SAFULLBODYIK_DEBUG
				_debugData.AddPoint( temp.centerLegPos );
				if( temp.torsoPos != null ) {
					for( int i = 0; i < temp.torsoPos.Length; ++i ) {
						_debugData.AddPoint( temp.torsoPos[i] );
					}
				}
				_debugData.AddPoint( temp.neckPos );
				for( int i = 0; i < 2; ++i ) {
					if( temp.shoulderPos != null ) {
						_debugData.AddPoint( temp.shoulderPos[i] );
					}
					_debugData.AddPoint( temp.armPos[i] );
					_debugData.AddPoint( temp.legPos[i] );
				}
#endif

				return true;
			}


			Vector3[] _tempArmPos = new Vector3[2];
			Vector3[] _tempArmPos2 = new Vector3[2];

			bool _UpperSolve()
			{
				var temp = _solverInternal;
				if( temp == null || temp.torsoPos == null || temp.torsoPos.Length == 0 || _wristEffectors == null ) {
					return false; // No moved.
				}

				float eyesWeight = _eyesEffector.positionEnabled ? _eyesEffector.positionWeight : 0.0f;
				float armPull0 = _wristEffectors[0].positionEnabled ? _wristEffectors[0].pull : 0.0f;
				float armPull1 = _wristEffectors[1].positionEnabled ? _wristEffectors[1].pull : 0.0f;
				if( eyesWeight <= IKEpsilon && armPull0 <= IKEpsilon && armPull1 <= IKEpsilon ) {
					return false; // No moved.
				}

#if SAFULLBODYIK_DEBUG
				//_debugData.AddPoint( _wristEffectors[0]._hidden_worldPosition, Color.black, 0.05f );
				//_debugData.AddPoint( _wristEffectors[1]._hidden_worldPosition, Color.black, 0.05f );
				//_debugData.AddPoint( _wristEffectors[1].bone.transform.position, Color.black, 0.025f );
#endif

				bool _upper_preSolve = false;

				float _upper_preTranslateRate1 = 0.2f;
				float _upper_preTranslateRate2 = 0.1f;
				float _upper_limbRotateRate1 = 0.673f; // for centerLeg
				float _upper_limbRotateRate2 = 0.775f; // for torso
				float _upper_postTranslateRate = 1.0f;
				bool _upper_solveTorso = true;
				bool _upper_solveTorso2 = true;

				float _lerpCenterLegRate = 1.0f;
				float _lerpTorsoRate = 1.0f;

				float _torsoLimitAngleY = 10.0f;
				float _torsoLimitAngleX = 40.0f;

				float _stablePreRate = 0.65f;
				float _stableCenterLegRate = 0.0f;
				float _stableTorsoRate = 0.0f;
				float _stablePostRate = 0.01f;

				float _upperEyesRate1 = 0.2f;
				float _upperEyesRate2 = 0.5f;
				float _upperEyesYUpRate = 0.25f;
				float _upperEyesYDownRate = 0.5f;
				float _upperEyesZOffset = 0.5f; // Lock when behind looking.
				float _upperEyesXAngle = 35.0f;
				float _upperEyesYUpAngle = 10.0f;
				float _upperEyesYDownAngle = 45.0f;

#if SAFULLBODYIK_DEBUG
				_debugData.UpdateValue( "_upperEyesRate1", ref _upperEyesRate1 );
				_debugData.UpdateValue( "_upperEyesRate2", ref _upperEyesRate2 );
				_debugData.UpdateValue( "_upperEyesYUpRate", ref _upperEyesYUpRate );
				_debugData.UpdateValue( "_upperEyesYDownRate", ref _upperEyesYDownRate );
				_debugData.UpdateValue( "_upperEyesZOffset", ref _upperEyesZOffset );
				_debugData.UpdateValue( "_upperEyesXAngle", ref _upperEyesXAngle );
				_debugData.UpdateValue( "_upperEyesYUpAngle", ref _upperEyesYUpAngle );
				_debugData.UpdateValue( "_upperEyesYDownAngle", ref _upperEyesYDownAngle );
				_debugData.UpdateValue( "_upper_preSolve", ref _upper_preSolve );
				_debugData.UpdateValue( "_upper_preTranslateRate1", ref _upper_preTranslateRate1 );
				_debugData.UpdateValue( "_upper_preTranslateRate2", ref _upper_preTranslateRate2 );
				_debugData.UpdateValue( "_upper_limbRotateRate1", ref _upper_limbRotateRate1 );
				_debugData.UpdateValue( "_upper_limbRotateRate2", ref _upper_limbRotateRate2 );
				_debugData.UpdateValue( "_upper_postTranslateRate", ref _upper_postTranslateRate );
				_debugData.UpdateValue( "_upper_solveTorso", ref _upper_solveTorso );
				_debugData.UpdateValue( "_upper_solveTorso2", ref _upper_solveTorso2 );
				_debugData.UpdateValue( "_torsoLimitAngleY", ref _torsoLimitAngleY );
				_debugData.UpdateValue( "_torsoLimitAngleX", ref _torsoLimitAngleX );
				_debugData.UpdateValue( "_stablePreRate", ref _stablePreRate );
				_debugData.UpdateValue( "_stableCenterLegRate", ref _stableCenterLegRate );
				_debugData.UpdateValue( "_stableTorsoRate", ref _stableTorsoRate );
				_debugData.UpdateValue( "_stablePostRate", ref _stablePostRate );
				_debugData.UpdateValue( "_lerpCenterLegRate", ref _lerpCenterLegRate );
				_debugData.UpdateValue( "_lerpTorsoRate", ref _lerpTorsoRate );
#endif

				_upper_limbRotateRate2 = Mathf.Max( _upper_limbRotateRate1, _upper_limbRotateRate2 );

				_upperEyesXAngle *= Mathf.Deg2Rad;
				_upperEyesYUpAngle *= Mathf.Deg2Rad;
				_upperEyesYDownAngle *= Mathf.Deg2Rad;

				_torsoLimitAngleY *= Mathf.Deg2Rad;
				_torsoLimitAngleX *= Mathf.Deg2Rad;

				Vector3 baseDirX = Vector3.zero;
				Vector3 baseCenterLegPos = Vector3.zero;

				bool continuousSolverEnabled = _internalValues.continuousSolverEnabled;

				// Preprocess for armPos / armPos2
				if( continuousSolverEnabled ) {
					if( _upper_preSolve ) {
						// Presolver.(for Optimized.)
						if( !temp.arms.SolveTargetBeginPos() ) {
							return false; // No moved.(Skip all steps.)
						}
					}

					_UpperSolve_Solve_GetBaseTransform( out baseDirX, out baseCenterLegPos );

#if SAFULLBODYIK_DEBUG
					_debugData.AddPoint( baseCenterLegPos, Color.gray, 0.05f );
					_debugData.AddPoint( baseCenterLegPos + baseDirX * 0.1f, Color.yellow, 0.05f );
#endif

					temp.Backup(); // for Testsolver.

					Vector3 baseDirZ = new Vector3( -baseDirX.z, 0.0f, baseDirX.x );

					Matrix3x3 centerLegBasis = new Matrix3x3( baseDirX, new Vector3( 0.0f, 1.0f, 0.0f ), baseDirZ );

					if( _torsoBones != null ) {
						for( int i = 0; i < _torsoBones.Length; ++i ) {
							temp.torsoPos[i] = centerLegBasis.Multiply( _torsoBones[i]._defaultPosition - _defaultCenterLegPos ) + baseCenterLegPos;
						}
						if( _neckBone != null ) {
							temp.neckPos = centerLegBasis.Multiply( _neckBone._defaultPosition - _defaultCenterLegPos ) + baseCenterLegPos;
						}
						for( int n = 0; n < 2; ++n ) {
							if( _shoulderBones != null ) {
								temp.shoulderPos[n] = centerLegBasis.Multiply( _shoulderBones[n]._defaultPosition - _defaultCenterLegPos ) + baseCenterLegPos;
							}
							if( _armBones != null ) {
								temp.armPos[n] = centerLegBasis.Multiply( _armBones[n]._defaultPosition - _defaultCenterLegPos ) + baseCenterLegPos;
							}
							if( _legBones != null ) {
								temp.legPos[n] = centerLegBasis.Multiply( _legBones[n]._defaultPosition - _defaultCenterLegPos ) + baseCenterLegPos;
							}
						}

						temp.SetDirtyVariables();
					}
				}

				// Solve.
				if( !temp.arms.SolveTargetBeginPos() ) {
					if( continuousSolverEnabled ) {
						// Nothing.
					} else {
						if( _upper_preSolve ) {
							return false; // No moved.
						}
					}
				}

				// PreTranslate.
				if( _upper_preTranslateRate1 > IKEpsilon ) {
					for( int i = 0; i < 2; ++i ) {
						float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
						temp.arms.ResolveTargetBeginPosRated( i, pull ); // Memo: Not contain _upper_preTranslateRate1

						if( temp.arms.targetBeginPosEnabled[i] ) {
							// for _UpperSolve_Translate()
							_UpperSolve_ShoulderToArm( i ); // Update temp.arms.beginPos(armPos)
						}
					}

					_UpperSolve_Translate( _upper_preTranslateRate1, _stablePreRate, baseCenterLegPos );

					temp.arms.SolveTargetBeginPos(); // Must call.
				}

				int torsoLength = (_torsoBones != null) ? (_torsoBones.Length) : 0;

				// Resolve for armPos.
				if( _upper_limbRotateRate1 > IKEpsilon ) {
					for( int i = 0; i < 2; ++i ) {
						float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
						temp.arms.ResolveTargetBeginPosRated( i, pull * _upper_limbRotateRate1 );
					}
				}

				if( continuousSolverEnabled ) {
					for( int i = 0; i < 2; ++i ) {
						if( !temp.arms.targetBeginPosEnabled[i] ) {
							// for _tempArmPos / _tempArmPos2
							_UpperSolve_ShoulderToArm( i ); // Update temp.arms.beginPos(armPos)
						}
					}
				} else {
					// for _tempArmPos / _tempArmPos2
					// for centerLeg / torso solvers.
					_UpperSolve_ShoulderToArm();
				}

				// Collect armPos.
				if( _upper_limbRotateRate1 > IKEpsilon ) {
					for( int i = 0; i < 2; ++i ) {
						if( temp.arms.targetBeginPosEnabled[i] ) {
							_tempArmPos[i] = temp.arms.targetBeginPosRated[i];
						} else {
							_tempArmPos[i] = temp.arms.beginPos[i];
						}
					}
				} else {
					for( int i = 0; i < 2; ++i ) {
						_tempArmPos[i] = temp.arms.beginPos[i];
					}
				}

				// Resolve for armPos(2).
				if( IsFuzzy( _upper_limbRotateRate1, _upper_limbRotateRate2 ) ) {
					for( int i = 0; i < 2; ++i ) {
						_tempArmPos2[i] = _tempArmPos[i];
					}
				} else if( _upper_limbRotateRate2 > IKEpsilon ) {
					// Memo: _upper_limbRotateRate2 should be saved to minimum value to _upper_limbRotateRate1.
					for( int i = 0; i < 2; ++i ) {
						float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
						temp.arms.ResolveTargetBeginPosRated( i, pull * _upper_limbRotateRate2 );
						if( temp.arms.targetBeginPosEnabled[i] ) {
							_tempArmPos2[i] = temp.arms.targetBeginPosRated[i];
						} else {
							_tempArmPos2[i] = temp.arms.beginPos[i];
						}
					}
				} else {
					for( int i = 0; i < 2; ++i ) {
						_tempArmPos2[i] = temp.arms.beginPos[i];
					}
				}

				Vector3 presolveCenterLegPos = temp.centerLegPos; // for continuousSolverEnabled

				float postTranslateRate = continuousSolverEnabled ? _upper_postTranslateRate : 1.0f;

				// for NeckSolver.
				Vector3 presolvedCenterLegPos2 = temp.centerLegPos;
				{
					Vector3 presolvedTranslate;
					if( _UpperSolve_PreTranslate( out presolvedTranslate, postTranslateRate, _stablePreRate, baseCenterLegPos ) ) {
						presolvedCenterLegPos2 += presolvedTranslate;
                    }
				}

				if( continuousSolverEnabled ) {
					// Salvage bone positions at end of testsolver.
					temp.Restore();

					if( !temp.arms.SolveTargetBeginPos() ) {
						if( _upper_preSolve ) {
							return false; // Failsafe.(No moved.)
						}
					}

					float stableRate = continuousSolverEnabled ? _stablePreRate : 0.0f;
					if( _upper_preTranslateRate2 > IKEpsilon || stableRate > IKEpsilon ) {
						for( int i = 0; i < 2; ++i ) {
							float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
							temp.arms.ResolveTargetBeginPosRated( i, pull ); // Memo: Not contain _upper_preTranslateRate1

							if( temp.arms.targetBeginPosEnabled[i] ) { // for _UpperSolve_Translate()
								_UpperSolve_ShoulderToArm( i ); // Update armPos(temp.arms.beginPos)
							}
						}

						_UpperSolve_Translate( _upper_preTranslateRate2, stableRate, baseCenterLegPos );

						temp.arms.SolveTargetBeginPos(); // Must call.
					}

					// for centerLeg / torso solvers.
					_UpperSolve_ShoulderToArm();
				}

				// pending: contain neck.
				Vector3 centerArmDirX = _tempArmPos[1] - _tempArmPos[0];
				Vector3 centerArmDirX2 = _tempArmPos2[1] - _tempArmPos2[0];

				if( !_SafeNormalize( ref centerArmDirX, ref centerArmDirX2 ) ) {
					return false; // Failsafe.(No moved.)
				}

#if SAFULLBODYIK_DEBUG
				for( int i = 0; i < 2; ++i ) {
					_debugData.AddPoint( _tempArmPos[i], Color.blue );
					_debugData.AddPoint( _tempArmPos2[i], Color.blue );
				}
#endif

				Vector3 centerArmPos, centerArmPos2;
				Vector3 centerArmDirY, centerArmDirY2;

				{
					// todo: Prepare Variables.
					float currentArmLen = (_armBones[1]._defaultPosition - _armBones[0]._defaultPosition).magnitude;
					float currentArmHalfLen = currentArmLen * 0.5f;
					Vector3 vecX = centerArmDirX * currentArmHalfLen;
					centerArmPos = Vector3.Lerp( _tempArmPos[0] + vecX, _tempArmPos[1] - vecX, temp.arms.lerpRate );
					centerArmPos2 = Vector3.Lerp( _tempArmPos2[0] + vecX, _tempArmPos2[1] - vecX, temp.arms.lerpRate ); // Test

#if SAFULLBODYIK_DEBUG
					_debugData.AddPoint( centerArmPos, Color.blue, 0.024f );
					_debugData.AddPoint( centerArmPos2, Color.blue, 0.024f );

					_debugData.AddPoint( _tempArmPos[0] + vecX, Color.white );
					_debugData.AddPoint( _tempArmPos[1] - vecX, Color.black );
#endif

					// pending: contain neck.
					centerArmDirY = centerArmPos - temp.centerLegPos;
					centerArmDirY2 = centerArmPos2 - temp.centerLegPos;
				}

				if( !_SafeNormalize( ref centerArmDirY, ref centerArmDirY2 ) ) {
					return false; // Failsafe.(No moved.)
				}

				if( eyesWeight > IKEpsilon ) {
					// Based on centerArmDirX2 / centerArmDirY2
					Matrix3x3 toBasis;
					_ComputeBasisFromXYLockY( out toBasis, ref centerArmDirX2, ref centerArmDirY2 );

					Matrix3x3 toBasisGlobal = toBasis * _centerLegToArmBasisInv; // Kari

					Matrix3x3 fromBasis = toBasis;

					toBasis *= _centerLegToArmBaseBasisInv;

					Vector3 eyePos = toBasisGlobal.Multiply( _defaultCenterEyePos - _defaultCenterLegPos ) + presolvedCenterLegPos2;
					Vector3 eyeDir = _eyesEffector.worldPosition - eyePos;

					{
						float _upperEyesXLimit = Mathf.Sin( _upperEyesXAngle );
						float _upperEyesYUpLimit = Mathf.Sin( _upperEyesYUpAngle );
						float _upperEyesYDownLimit = Mathf.Sin( _upperEyesYDownAngle );

						eyeDir = toBasis.transpose.Multiply( eyeDir ); // to Local

						if( eyeDir.y >= 0.0f ) {
							eyeDir.y *= _upperEyesYUpRate;
                        } else {
							eyeDir.y *= _upperEyesYDownRate;
						}

						_SafeNormalize( ref eyeDir );

						if( eyeDir.z < 0.0f ) {
                            float offset = Mathf.Clamp( _upperEyesZOffset, 0.0f, 0.99f );
							if( offset > IKEpsilon ) {
								if( eyeDir.z > -offset ) {
									eyeDir.z = 0.0f;
								} else {
									eyeDir.z = (eyeDir.z + offset) / (1.0f - offset);
								}
								_SafeNormalize( ref eyeDir );
							}
						}

						_LimitXY( ref eyeDir, _upperEyesXLimit, _upperEyesXLimit, _upperEyesYDownLimit, _upperEyesYUpLimit );

						eyeDir = toBasis.Multiply( eyeDir ); // to Global

						{
							Vector3 xDir = toBasis.column0;
							Vector3 yDir = toBasis.column1;
							Vector3 zDir = eyeDir;

							if( _ComputeBasisLockZ( out toBasis, ref xDir, ref yDir, ref zDir ) ) {
								// Nothing.
							}
						}
					}

					toBasis *= _centerLegToArmBaseBasis;

					Matrix3x3 solveBasis;
					if( _upperEyesRate2 > IKEpsilon ) {
						_Lerp( out solveBasis, ref fromBasis, ref toBasis, _upperEyesRate2 );
						centerArmDirX2 = solveBasis.column0;
						centerArmDirY2 = solveBasis.column1;
					}

					if( _upperEyesRate1 > IKEpsilon ) {
						_ComputeBasisFromXYLockY( out fromBasis, ref centerArmDirX, ref centerArmDirY );
						_Lerp( out solveBasis, ref fromBasis, ref toBasis, _upperEyesRate1 );
						centerArmDirX = solveBasis.column0;
						centerArmDirY = solveBasis.column1;
					}
				}
				
				{
					Matrix3x3 rotateBasis = Matrix3x3.identity;

					Matrix3x3 toBasis;
					_ComputeBasisFromXYLockY( out toBasis, ref centerArmDirX, ref centerArmDirY );

					if( _internalValues.animatorEnabled || _internalValues.resetTransforms ) {
						// for animatorEnabled or resetTransform(Base on armPos)
						if( continuousSolverEnabled && _stableCenterLegRate > IKEpsilon ) {
							Matrix3x3 presolveCenterLegBasis = Matrix3x3.identity;
							Vector3 solveDirY = centerArmPos - presolveCenterLegPos;
							Vector3 solveDirX = centerArmDirX;
							if( _SafeNormalize( ref solveDirY ) ) {
								_ComputeBasisFromXYLockY( out presolveCenterLegBasis, ref solveDirX, ref solveDirY );
							}

							Matrix3x3 tempBasis;
							_Lerp( out tempBasis, ref toBasis, ref presolveCenterLegBasis, _stableCenterLegRate );
							toBasis = tempBasis;
						}

						Matrix3x3 fromBasis = Matrix3x3.identity;
						Vector3 currentDirX = temp.armPos[1] - temp.armPos[0];
						Vector3 currentDirY = (temp.armPos[1] + temp.armPos[0]) * 0.5f - temp.centerLegPos;
						if( _SafeNormalize( ref currentDirX, ref currentDirY ) ) {
							_ComputeBasisFromXYLockY( out fromBasis, ref currentDirX, ref currentDirY );
						}

						rotateBasis = toBasis * fromBasis.transpose;
						if( _lerpCenterLegRate < 1.0f - IKEpsilon ) {
							_LerpToIdentity( ref rotateBasis, 1.0f - _lerpCenterLegRate );
						}
					} else { // for continuousSolverEnabled.(Base on centerLegBasis)
						toBasis *= _centerLegToArmBasisInv;

						if( continuousSolverEnabled && _stableCenterLegRate > IKEpsilon ) {
							Matrix3x3 presolveCenterLegBasis = Matrix3x3.identity;
							Vector3 solveDirY = centerArmPos - presolveCenterLegPos;
							Vector3 solveDirX = centerArmDirX;
							if( _SafeNormalize( ref solveDirY ) ) {
								if( _ComputeBasisFromXYLockY( out presolveCenterLegBasis, ref solveDirX, ref solveDirY ) ) {
									presolveCenterLegBasis *= _centerLegToArmBasisInv;
								}
							}

							Matrix3x3 tempBasis;
							_Lerp( out tempBasis, ref toBasis, ref presolveCenterLegBasis, _stableCenterLegRate );
							toBasis = tempBasis;
						}

						rotateBasis = toBasis * temp.centerLegBasis.transpose;
					}

					if( _lerpCenterLegRate < 1.0f - IKEpsilon ) {
						_LerpToIdentity( ref rotateBasis, 1.0f - _lerpCenterLegRate );
					}

					temp.UpperRotation( -1, ref rotateBasis );
				}

				{
					// Compute torsoRate.
					float torsoRate = 1.0f;
					for( int i = 0; i < 2; ++i ) {
						if( temp.arms.endPosEnabled[i] ) {
							Vector3 centerLegToArm = temp.arms.beginPos[i] - temp.centerLegPos;
							Vector3 beginToEnd = temp.arms.endPos[i] - temp.arms.beginPos[i];
							if( _SafeNormalize( ref centerLegToArm, ref beginToEnd ) ) {
								torsoRate *= Mathf.Abs( Vector3.Dot( centerLegToArm, beginToEnd ) );
							}
						}
					}

					torsoRate = 1.0f - torsoRate;
					if( torsoRate > IKEpsilon ) { // Limit centerArmDirX2 / centerArmDirY2 from torsoRate
						// Recompute centerLegToArmBoneBasisTo2( for Torso )
						{
							float fromToX = Vector3.Dot( centerArmDirX, centerArmDirX2 );
							float fromToXAng = _SafeAcos( fromToX );
							fromToXAng *= torsoRate;
							if( fromToXAng > _torsoLimitAngleX ) {
								if( fromToXAng > IKEpsilon ) {
									float balancedRate = _torsoLimitAngleX / fromToXAng;
									Vector3 dirX2Balanced = Vector3.Lerp( centerArmDirX, centerArmDirX2, balancedRate );
									if( _SafeNormalize( ref dirX2Balanced ) ) {
										centerArmDirX2 = dirX2Balanced;
									}
								}
							}
						}

						// Pending: torso stiffness.(Sin scale to balanced rate.)
						{
							float fromToY = Vector3.Dot( centerArmDirY, centerArmDirY2 );
							float fromToYAng = _SafeAcos( fromToY );
							fromToYAng *= torsoRate;
							if( fromToYAng > _torsoLimitAngleY ) {
								if( fromToYAng > IKEpsilon ) {
									float balancedRate = _torsoLimitAngleY / fromToYAng;
									Vector3 dirY2Balanced = Vector3.Lerp( centerArmDirY, centerArmDirY2, balancedRate );
									if( _SafeNormalize( ref dirY2Balanced ) ) {
										centerArmDirY2 = dirY2Balanced;
									}
								}
							}
						}
					}
				}

				if( _upper_solveTorso ) {

					//float centerLegToArmLength = (_neckBone._defaultPosition - _defaultCenterLegPos).magnitude;
					float centerLegToArmLength = (_defaultCenterArmPos - _defaultCenterLegPos).magnitude;

					int solveLength = Mathf.Min( torsoLength, 2 );
					if( !_upper_solveTorso2 ) {
						solveLength = Mathf.Min( torsoLength, 1 );
					}

					Matrix3x3 centerLegBasis = temp.centerLegBasis;

					Vector3 centerArmPosY2 = centerArmDirY2 * centerLegToArmLength + temp.centerLegPos;

					if( _internalValues.animatorEnabled || _internalValues.resetTransforms ) {
						for( int i = 0; i < solveLength; ++i ) {
							Vector3 origPos = temp.torsoPos[i];

							Vector3 currentDirX = temp.armPos[1] - temp.armPos[0];
							Vector3 currentDirY = (temp.armPos[1] + temp.armPos[0]) * 0.5f - origPos;

							Vector3 targetDirX = centerArmDirX2;
							Vector3 targetDirY = centerArmPosY2 - origPos;

							if( !_SafeNormalize( ref currentDirX, ref currentDirY, ref targetDirY ) ) {
								continue; // Skip.
							}

							Vector3 dirX = targetDirX;
							Vector3 dirY = targetDirY;

							if( i == 0 ) { // Torso
								dirX = Vector3.Lerp( currentDirX, targetDirX, _torsoDirXLegToArmRate );
								if( !_SafeNormalize( ref dirX ) ) { // Failsafe.
									dirX = currentDirX;
								}

								dirY = Vector3.Lerp( currentDirY, targetDirY, 0.5f ); // Test Rate!!!
								if( !_SafeNormalize( ref dirY ) ) { // Failsafe.
									dirY = currentDirY;
								}
							}

							Matrix3x3 toBasis;
							_ComputeBasisFromXYLockY( out toBasis, ref dirX, ref dirY );
							Matrix3x3 fromBasis;
							_ComputeBasisFromXYLockY( out fromBasis, ref currentDirX, ref currentDirY );

							Matrix3x3 rotateBasis = toBasis * fromBasis.transpose;

							if( _lerpTorsoRate < 1.0f - IKEpsilon ) {
								_LerpToIdentity( ref rotateBasis, 1.0f - _lerpTorsoRate );
							}

							temp.UpperRotation( i, ref rotateBasis );
						}
					} else {
						for( int i = 0; i < solveLength; ++i ) {
							Vector3 origPos = temp.torsoPos[i];
							Vector3 dirX = centerArmDirX2;
							Vector3 dirY = centerArmPosY2 - origPos;

							if( i == 0 ) { // Torso
								dirX = Vector3.Lerp( centerLegBasis.column0, centerArmDirX2, _torsoDirXLegToArmRate );
								if( !_SafeNormalize( ref dirX ) ) { // Failsafe.
									dirX = centerLegBasis.column0;
								}

								// todo: Prepare _torsoDefaultDirY[]
								Vector3 defaultPos = _torsoBones[i]._defaultPosition;
								Vector3 defaultChildPod = (i + 1 == torsoLength) ? _neckBone._defaultPosition : _torsoBones[i + 1]._defaultPosition;
								Vector3 baseY = centerLegBasis.Multiply( defaultChildPod - defaultPos );
								_SafeNormalize( ref baseY );

								_SafeNormalize( ref dirY );
								baseY = Vector3.Lerp( baseY, dirY, 0.5f ); // Test Rate!!!
								_SafeNormalize( ref baseY );
								dirY = baseY;
							} else { // Torso 2
								dirX = Vector3.Lerp( centerLegBasis.column0, centerArmDirX2, 0.9f ); // Test Rate!!!
								if( !_SafeNormalize( ref dirX ) ) { // Failsafe.
									dirX = centerLegBasis.column0;
								}
								if( !_SafeNormalize( ref dirY ) ) { // Failsafe.
									// todo: _torsoDefaultDirY[]
									Vector3 childPos = (i + 1 == torsoLength) ? temp.neckPos : temp.torsoPos[i + 1];
									dirY = childPos - origPos;
									if( !_SafeNormalize( ref dirY ) ) {
										continue;
									}
								}
							}
						
							Matrix3x3 toBasis;
							if( _ComputeBasisFromXYLockY( out toBasis, ref dirX, ref dirY ) ) {
								Vector3 childPos = (i + 1 == torsoLength) ? temp.neckPos : temp.torsoPos[i + 1];

								Vector3 fromY = childPos - origPos;
								Vector3 fromX = temp.nearArmPos[1] - temp.nearArmPos[0];
								if( i == 0 && torsoLength >= 2 ) {
									if( _SafeNormalize( ref fromX ) ) {
										fromX = Vector3.Lerp( centerLegBasis.column0, fromX, _torsoDirXLegToArmRate );
									} else { // Failsafe.
										fromX = centerLegBasis.column0;
									}
								}

								Vector3 fromZ = Vector3.Cross( fromX, fromY );
								fromX = Vector3.Cross( fromY, fromZ );
								if( _SafeNormalize( ref fromX, ref fromY, ref fromZ ) ) {
									Matrix3x3 fromBasis = Matrix3x3.FromColumn( ref fromX, ref fromY, ref fromZ );
									Matrix3x3 rotateBasis = toBasis * fromBasis.transpose;

									if( _lerpTorsoRate < 1.0f - IKEpsilon ) {
										_LerpToIdentity( ref rotateBasis, 1.0f - _lerpTorsoRate );
									}

									temp.UpperRotation( i, ref rotateBasis );
								}
							}
						}
					}
				}

				// Last: ShoulderToArm & Translate.
				if( !temp.arms.SolveTargetBeginPos() ) {
					if( continuousSolverEnabled ) {
						// Nothing.
					} else {
						_UpperSolve_ShoulderToArm();
						return true;
					}
				}

				for( int i = 0; i < 2; ++i ) {
					float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
					temp.arms.ResolveTargetBeginPosRated( i, pull );
				}

				_UpperSolve_ShoulderToArm();

				_UpperSolve_Translate( postTranslateRate, _stablePostRate, baseCenterLegPos );
				return true;
			}

			bool _UpperSolve_Solve_GetBaseTransform( out Vector3 baseDirX, out Vector3 baseCenterLegPos )
			{
				var temp = _solverInternal;
				Assert( temp != null );

				if( _footEffectors == null ) {
					baseDirX = new Vector3( 1.0f, 0.0f, 0.0f );
					baseCenterLegPos = temp.centerLegPos;
					return false;
				}

				if( !_footEffectors[0].bone.transformIsAlive ||
					!_footEffectors[1].bone.transformIsAlive ) {
					baseDirX = new Vector3( 1.0f, 0.0f, 0.0f );
					baseCenterLegPos = temp.centerLegPos;
					return false;
				}

				Vector3 footPos0 = _footEffectors[0].positionEnabled ? _footEffectors[0]._hidden_worldPosition : _footEffectors[0].bone.worldPosition;
				Vector3 footPos1 = _footEffectors[1].positionEnabled ? _footEffectors[1]._hidden_worldPosition : _footEffectors[1].bone.worldPosition;

				baseDirX = footPos1 - footPos0;
				baseDirX.y = 0.0f;
				if( !_SafeNormalize( ref baseDirX ) ) {
					baseDirX = new Vector3( 1.0f, 0.0f, 0.0f );
					baseCenterLegPos = temp.centerLegPos;
					return false;
				}

				Vector3 centerFootPos = (footPos1 + footPos0) * 0.5f;

				Vector3 baseDirY = new Vector3( 0.0f, 1.0f, 0.0f );
				Vector3 baseDirZ = new Vector3( -baseDirX.z, 0.0f, baseDirX.x );
				Matrix3x3 baseBasis = Matrix3x3.FromColumn( ref baseDirX, ref baseDirY, ref baseDirZ );

				baseCenterLegPos = baseBasis.Multiply( _defaultCenterLegPos - _defaultCenterFootPos ) + centerFootPos;

				Vector3 baseLegPos0 = baseBasis.Multiply( _legBones[0]._defaultPosition - _defaultCenterLegPos ) + baseCenterLegPos;
				Vector3 baseLegPos1 = baseBasis.Multiply( _legBones[1]._defaultPosition - _defaultCenterLegPos ) + baseCenterLegPos;

				bool isLimited = false;

				FastLength legLen0 = FastLength.FromVector3( baseLegPos0 - footPos0 );
				if( legLen0 > _legEffectorMaxLength[0] ) {
					_KeepLength( ref baseLegPos0, ref footPos0, ref _legEffectorMaxLength[0] );
					isLimited = true;
				}

				FastLength legLen1 = FastLength.FromVector3( baseLegPos1 - footPos1 );
				if( legLen1 > _legEffectorMaxLength[1] ) {
					_KeepLength( ref baseLegPos1, ref footPos1, ref _legEffectorMaxLength[1] );
					isLimited = true;
				}

				if( isLimited ) {
					if( _footEffectors[0].positionEnabled || _footEffectors[1].positionEnabled ) {
						Vector3 vecX = baseDirX * _defaultCenterLegHalfLen;
						baseCenterLegPos = Vector3.Lerp( baseLegPos0 + vecX, baseLegPos1 - vecX, temp.legs.lerpRate );
					} else {
						baseCenterLegPos = (baseLegPos0 + baseLegPos1) * 0.5f;
					}
				}

				return true;
			}

			void _UpperSolve_Transform( int origIndex, ref Matrix3x3 transformBasis )
			{
				var temp = _solverInternal;

				Vector3 origPos = (origIndex == -1) ? temp.centerLegPos : temp.torsoPos[origIndex];

				for( int i = 0; i < 2; ++i ) {
					temp.armPos[i] = transformBasis.Multiply( temp.armPos[i] - origPos ) + origPos;
					if( _shoulderBones != null ) {
						temp.shoulderPos[i] = transformBasis.Multiply( temp.shoulderPos[i] - origPos ) + origPos;
					}
				}

				int torsoLength = (_torsoBones != null) ? (_torsoBones.Length) : 0;
				for( int torsoIndex = origIndex + 1; torsoIndex < torsoLength; ++torsoIndex ) {
					temp.torsoPos[torsoIndex] = transformBasis.Multiply( temp.torsoPos[torsoIndex] - origPos ) + origPos;
				}

				if( _neckBone != null ) {
					temp.neckPos = transformBasis.Multiply( temp.neckPos - origPos ) + origPos;
				}

				if( origIndex == -1 ) {
					if( temp.legPos != null ) {
						for( int i = 0; i < 2; ++i ) {
							temp.legPos[i] = transformBasis.Multiply( temp.legPos[i] - origPos ) + origPos;
						}
					}
				}

				temp.SetDirtyVariables();
			}

			Vector3[] _temp_targetTranslate = new Vector3[2]; // Arms(2)

			bool _UpperSolve_PreTranslate( out Vector3 translate, float translateRate, float stableRate, Vector3 stableCenterLegPos )
			{
				// If resetTransform = false, contain targetBeginPos to default transform or modify _UpperSolve_Translate()

				// Memo: Prepare SolveTargetBeginPosRated().

				translate = Vector3.zero;

				var temp = _solverInternal;
				Assert( temp != null );

				bool continuousSolverEnabled = _internalValues.continuousSolverEnabled;

				bool translateEnabled = (continuousSolverEnabled && stableRate > IKEpsilon);
				for( int i = 0; i < 2; ++i ) {
					if( temp.arms.targetBeginPosEnabled[i] ) {
						_temp_targetTranslate[i] = temp.arms.targetBeginPosRated[i] - temp.arms.beginPos[i]; // Memo: targetBeginPos is balanced.
						translateEnabled = true;
					} else {
						_temp_targetTranslate[i] = Vector3.zero; // Clear value.
					}
				}

				if( translateEnabled ) {
					translate = Vector3.Lerp( _temp_targetTranslate[0], _temp_targetTranslate[1], temp.arms.lerpRate );
					if( translateRate < 1.0f ) {
						translate *= translateRate;
					}

					if( continuousSolverEnabled && stableRate > IKEpsilon ) {
						Vector3 extraTranslate = stableCenterLegPos - temp.centerLegPos;
						translate = Vector3.Lerp( translate, extraTranslate, stableRate );
					}

					return true;
				}

				return false;
			}

			void _UpperSolve_Translate( float translateRate, float stableRate, Vector3 stableCenterLegPos )
			{
				Vector3 translate;
				if( _UpperSolve_PreTranslate( out translate, translateRate, stableRate, stableCenterLegPos ) ) {
					var temp = _solverInternal;
					Assert( temp != null );
					temp.Translate( ref translate );
				}
			}
			
			void _LowerSolve( bool isBeginOnly, bool firstPass )
			{
				var temp = _solverInternal;
				if( temp == null || temp.torsoPos == null || temp.torsoPos.Length == 0 ) {
					return;
				}

				float limbRate = 1.0f;
				if( !firstPass ) {
					limbRate = temp.armToLegLerpRate;
				}

				if( temp.PrepareLowerRotation( 0 ) ) {
					Vector3 centerLegBoneY = temp.centerLegBasis.column1;
					for( int i = 0; i < 2; ++i ) {
						if( temp.legs.endPosEnabled[i] && temp.legs.pullEnabled[i] ) {
							Vector3 legDir = temp.legs.endPos[i] - temp.legs.beginPos[i];
							if( _SafeNormalize( ref legDir ) ) {
								float legDirFeedbackRate = Vector3.Dot( centerLegBoneY, -legDir );
#if false
								legDirFeedbackRate = Mathf.Asin( Mathf.Clamp( legDirFeedbackRate, -1.0f, 1.0f ) );
								legDirFeedbackRate *= 1.0f / (90.0f * Mathf.Deg2Rad);
								legDirFeedbackRate = Mathf.Clamp01( legDirFeedbackRate );
#else // Faster
								legDirFeedbackRate = Mathf.Clamp01( legDirFeedbackRate );
#endif
								legDirFeedbackRate = 1.0f - legDirFeedbackRate;
								temp.SetSolveFeedbackRate( i, legDirFeedbackRate * limbRate );
							}
						}
					}
					
					Quaternion origLowerRotation;
					if( temp.SolveLowerRotation( 0, out origLowerRotation ) ) {
						if( isBeginOnly ) {
							temp.LowerRotationBeginOnly( 0, ref origLowerRotation );
						} else {
							//temp.LowerRotation( 0, ref origLowerRotation, true );
							temp.LowerRotation( 0, ref origLowerRotation, false );
						}
					}
				}

				if( temp.PrepareLowerTranslate() ) {
					Vector3 origLowerTranslate;
					if( temp.SolveLowerTranslate( out origLowerTranslate ) ) {
						//if( defaultToLerpRate > 0.0f ) { // Test!!!
						//	origLowerTranslate = Vector3.Lerp( origLowerTranslate, _defaultCenterLegPos - temp.centerLegPos, defaultToLerpRate );
						//}

						//Debug.Log( "_LowerSolve: " + firstPass + " " + "limbRate: " + limbRate );

						if( limbRate < 1.0f - IKEpsilon ) {
							origLowerTranslate *= limbRate;
						}

						if( isBeginOnly ) {
							temp.LowerTranslateBeginOnly( ref origLowerTranslate );
						} else {
							temp.Translate( ref origLowerTranslate );
						}
					}
				}
			}

			void _ComputeWorldTransform()
			{
				var temp = _solverInternal;
				if( temp == null || temp.torsoPos == null || temp.torsoPos.Length == 0 ) {
					return;
				}

				// Compute worldPosition / worldRotation.
				if( _pelvisBone != null && _pelvisBone.transformIsAlive && temp.torsoPos != null && temp.torsoPos.Length > 0 && _neckBone != null && _neckBone.transformIsAlive ) {
					Vector3 pelvisToTorsoDirX = new Vector3( 1.0f, 0.0f, 0.0f );

					Vector3 dirX = temp.legs.beginPos[1] - temp.legs.beginPos[0];
					Vector3 dirY = temp.torsoPos[0] - (temp.legs.beginPos[1] + temp.legs.beginPos[0]) * 0.5f;

					Matrix3x3 boneBasis = new Matrix3x3();

					if( _SafeNormalize( ref dirY ) && _ComputeBasisFromXYLockY( out boneBasis, ref dirX, ref dirY ) ) {
						Matrix3x3 tempBasis = boneBasis * _centerLegBoneBasisInv;

						pelvisToTorsoDirX = boneBasis.column0; // Counts as baseBasis.
						_pelvisBone.worldRotation = (tempBasis * _pelvisBone._defaultBasis).GetRotation();

						if( _pelvisBone.isWritebackWorldPosition ) {
							_pelvisBone.worldPosition = temp.torsoPos[0] + tempBasis * (_pelvisBone._defaultPosition - _torsoBone._defaultPosition);
						}
					} else { // Failsafe.
						if( _SafeNormalize( ref dirX ) ) {
							pelvisToTorsoDirX = dirX;
                        }
					}


					for( int i = 0; i < temp.torsoPos.Length; ++i ) {
						if( i + 1 == temp.torsoPos.Length ) {
							dirY = temp.neckPos - temp.torsoPos[i];
							if( temp.nearArmPos != null ) {
								dirX = temp.nearArmPos[1] - temp.nearArmPos[0];
							} else { // Failsafe.
								dirX = pelvisToTorsoDirX;
                            }
						} else {
							dirY = temp.torsoPos[i + 1] - temp.torsoPos[i];
							dirX = pelvisToTorsoDirX;
							if( temp.nearArmPos != null ) {
								Vector3 dirX0 = temp.nearArmPos[1] - temp.nearArmPos[0];
								if( _SafeNormalize( ref dirX0 ) ) {
									dirX = Vector3.Lerp( dirX, dirX0, _torsoDirXLegToArmRate );
								}
							}
						}

						if( _SafeNormalize( ref dirY ) && _ComputeBasisFromXYLockY( out boneBasis, ref dirX, ref dirY ) ) {
							pelvisToTorsoDirX = boneBasis.column0;
							_torsoBones[i].worldRotation = (boneBasis * _torsoBones[i]._boneToWorldBasis).GetRotation();
							if( _torsoBones[i].isWritebackWorldPosition ) {
								_torsoBones[i].worldPosition = temp.torsoPos[i];
							}
						}
					}

					if( _shoulderBones != null ) {
						for( int i = 0; i < 2; ++i ) {
							Vector3 xDir = temp.armPos[i] - temp.shoulderPos[i];
							Vector3 yDir = temp.shoulderPos[i] - temp.torsoUPos;
							xDir = (i == 0) ? -xDir : xDir;
							Vector3 zDir = Vector3.Cross( xDir, yDir );
							yDir = Vector3.Cross( zDir, xDir );
							if( _SafeNormalize( ref xDir, ref yDir, ref zDir ) ) {
								boneBasis.SetColumn( ref xDir, ref yDir, ref zDir );
								_shoulderBones[i].worldRotation = (boneBasis * _shoulderBones[i]._boneToWorldBasis).GetRotation();
							}
						}
					}
				}
			}

			bool _IsEffectorEnabled()
			{
				if( _pelvisEffector.effectorEnabled ||
					_neckEffector.effectorEnabled ||
					_armEffectors[0].effectorEnabled ||
					_armEffectors[1].effectorEnabled ) {
					return true;
				}

				if( _eyesEffector.positionEnabled ||
					_wristEffectors[0].positionEnabled ||
					_wristEffectors[1].positionEnabled ||
					_footEffectors[0].positionEnabled ||
					_footEffectors[1].positionEnabled ) {
					return true;
				}

				return false;
			}

			bool _PrepareSolverInternal()
			{
				if( !this.enabled ) {
					_solverInternal = null;
					return false;
				}

				if( _armBones == null || _legBones == null ) {
					_solverInternal = null;
					return false;
				}

				if( _solverInternal == null ) {
					_solverInternal = new SolverInternal();
					_solverInternal._centerLegBoneBasisInv = this._centerLegBoneBasisInv;
					if( _torsoUBone != null ) {
						if( _shoulderBones != null || _armBones != null ) {
							var nearArmBones = ( _shoulderBones != null ) ? _shoulderBones : _armBones;
							Vector3 dirY = nearArmBones[1]._defaultPosition + nearArmBones[0]._defaultPosition;
							Vector3 dirX = nearArmBones[1]._defaultPosition - nearArmBones[0]._defaultPosition;
							dirY = dirY * 0.5f - _torsoUBone._defaultPosition;
							Vector3 dirZ = Vector3.Cross( dirX, dirY );
							dirX = Vector3.Cross( dirY, dirZ );
							if( _SafeNormalize( ref dirX ) && _SafeNormalize( ref dirY ) && _SafeNormalize( ref dirZ ) ) {
								Matrix3x3 localBasis = Matrix3x3.FromColumn( ref dirX, ref dirY, ref dirZ );
								_solverInternal._torsoUBoneLocalAxisBasisInv = localBasis.transpose;
							}
						}
					}
				}

				// Arms, Legs
				_solverInternal.arms.Prepare( _armBones, _wristEffectors, _armEffectorMaxLength );
				_solverInternal.legs.Prepare( _legBones, _footEffectors, _legEffectorMaxLength );

				_solverInternal.armToLegPull[0] = _solverInternal.arms.pull[0] + _solverInternal.arms.pull[1];
				_solverInternal.armToLegPull[1] = _solverInternal.legs.pull[0] + _solverInternal.legs.pull[1];
				_PullToWeight2( _solverInternal.armToLegPull, _solverInternal.armToLegWeight );

				// Shoulder bones. (Optional)
				Util.PrepareArray( ref _solverInternal.shoulderPos, _shoulderBones );

				_solverInternal.nearArmPos = (_shoulderBones != null) ? _solverInternal.shoulderPos : _solverInternal.armPos;

				Util.PrepareArray( ref _solverInternal.torsoPos, _torsoBones );

				return true;
			}

			void _UpperSolve_ShoulderToArm()
			{
				for( int i = 0; i < 2; ++i ) {
					_solverInternal.SolveShoulderToArm(
						i,
						_shoulderBones,
						_shoulderToArmLength,
						_shoulderLimitYPlus,
						_shoulderLimitYMinus,
						_shoulderLimitZ );
				}
			}

			void _UpperSolve_ShoulderToArm( int i )
			{
				_solverInternal.SolveShoulderToArm(
					i,
					_shoulderBones,
					_shoulderToArmLength,
					_shoulderLimitYPlus,
					_shoulderLimitYMinus,
					_shoulderLimitZ );
			}

			void _PresolvePelvis()
			{
				Assert( _internalValues != null && _internalValues.animatorEnabled );

				var temp = _solverInternal;
				Assert( temp != null );

				if( _pelvisEffector == null ) {
					return;
				}

				bool rotationEnabled = _pelvisEffector.rotationEnabled && _pelvisEffector.rotationWeight > IKEpsilon;
				bool positionEnabled = _pelvisEffector.positionEnabled && _pelvisEffector.positionWeight > IKEpsilon;

				if( !rotationEnabled && !positionEnabled ) {
					return;
				}

				Matrix3x3 centerLegBasis = temp.centerLegBasis;

				if( rotationEnabled ) {
					Matrix3x3 centerLegBasisTo = _pelvisEffector.worldRotation * Inverse( _pelvisEffector._defaultRotation );
					Matrix3x3 centerLegBasisFrom = centerLegBasis;

					if( _settings.pelvisEffectorKeepHrizontalRate > IKEpsilon ) {
						Vector3 dirY;

						if( _settings.pelvisEffectorKeepHrizontalRate < 1.0f - IKEpsilon ) {
							Vector3 dirYFrom = centerLegBasisFrom.column1;
							Vector3 dirYTo = centerLegBasisTo.column1;
							dirY = Vector3.Lerp( dirYTo, dirYFrom, _settings.pelvisEffectorKeepHrizontalRate );
							if( !_SafeNormalize( ref dirY ) ) {
								dirY = dirYFrom;
							}
						} else {
							dirY = centerLegBasisFrom.column1;
						}

						Matrix3x3 tempBasis;
						Vector3 dirX = centerLegBasisTo.column0;
						Vector3 dirZ = centerLegBasisTo.column2;
						if( _ComputeBasisLockY( out tempBasis, ref dirX, ref dirY, ref dirZ ) ) {
							centerLegBasisTo = tempBasis;
                        }
                    }

					Matrix3x3 centerLegRotationBasis = centerLegBasisTo * centerLegBasisFrom.transpose;

					if( _pelvisEffector.rotationWeight < 1.0f - IKEpsilon ) {
						_LerpToIdentity( ref centerLegRotationBasis, 1.0f - _pelvisEffector.rotationWeight );
					}

					temp.LowerRotation( -1, ref centerLegRotationBasis, true );
					centerLegBasis = temp.centerLegBasis;
				}

				if( positionEnabled ) {
					Vector3 centerLegPos = centerLegBasis.Multiply( _defaultCenterLegPos - _pelvisEffector.defaultPosition ) + _pelvisEffector.worldPosition;
					Vector3 translate = centerLegPos - temp.centerLegPos;
					if( _pelvisEffector.positionWeight < 1.0f - IKEpsilon ) {
						translate *= _pelvisEffector.positionWeight;
                    }

					temp.Translate( ref translate );
				}
			}

			void _ResetTransforms()
			{
				Assert( _internalValues != null && _internalValues.resetTransforms );

				Matrix3x3 centerLegBasis = Matrix3x3.identity;
				if( _pelvisEffector != null && _pelvisEffector.rotationEnabled && _pelvisEffector.rotationWeight > IKEpsilon ) {
					Quaternion centerLegRotation = _pelvisEffector.worldRotation * Inverse( _pelvisEffector._defaultRotation );
					if( _pelvisEffector.rotationWeight < 1.0f - IKEpsilon && _internalValues.rootTransformIsAlive ) {
						Quaternion rootRotation = _internalValues.rootTransform.rotation * Inverse( _internalValues.defaultRootRotation );
						centerLegBasis = Quaternion.Lerp( rootRotation, centerLegRotation, _pelvisEffector.rotationWeight );
					} else {
						centerLegBasis = centerLegRotation;
					}
				} else if( _internalValues.rootTransformIsAlive ) {
					centerLegBasis = _internalValues.rootTransform.rotation * Inverse( _internalValues.defaultRootRotation );
				}

				Vector3 centerLegPos = Vector3.zero;
				if( _pelvisEffector != null && _pelvisEffector.positionEnabled && _pelvisEffector.positionWeight > IKEpsilon ) {
					centerLegPos = centerLegBasis.Multiply( _defaultCenterLegPos - _pelvisEffector.defaultPosition ) + _pelvisEffector.worldPosition;
					if( _pelvisEffector.positionWeight < 1.0f - IKEpsilon ) {
						Vector3 rootPosition = centerLegBasis.Multiply( _defaultCenterLegPos - _internalValues.defaultRootPosition ) + _internalValues.rootTransform.position;
						centerLegPos = Vector3.Lerp( rootPosition, centerLegPos, _pelvisEffector.positionWeight );
					}
				} else if( _internalValues.rootTransformIsAlive ) {
					centerLegPos = centerLegBasis.Multiply( _defaultCenterLegPos - _internalValues.defaultRootPosition ) + _internalValues.rootTransform.position;
				}

				_ResetCenterLegTransform( ref centerLegPos, ref centerLegBasis );
			}

			void _ResetCenterLegTransform( ref Vector3 centerLegPos, ref Matrix3x3 centerLegBasis )
			{
				var temp = _solverInternal;
				Assert( temp != null );

				Vector3 defaultCenterLegPos = _defaultCenterLegPos;

				if( _legBones != null ) {
					for( int i = 0; i < 2; ++i ) {
						temp.legPos[i] = centerLegBasis.Multiply( _legBones[i]._defaultPosition - defaultCenterLegPos ) + centerLegPos;
					}
				}
				if( _torsoBones != null ) {
					for( int i = 0; i < _torsoBones.Length; ++i ) {
						temp.torsoPos[i] = centerLegBasis.Multiply( _torsoBones[i]._defaultPosition - defaultCenterLegPos ) + centerLegPos;
					}
				}
				if( _shoulderBones != null ) {
					for( int i = 0; i < 2; ++i ) {
						temp.shoulderPos[i] = centerLegBasis.Multiply( _shoulderBones[i]._defaultPosition - defaultCenterLegPos ) + centerLegPos;
					}
				}
				if( _armBones != null ) {
					for( int i = 0; i < 2; ++i ) {
						temp.armPos[i] = centerLegBasis.Multiply( _armBones[i]._defaultPosition - defaultCenterLegPos ) + centerLegPos;
					}
				}
				if( _neckBone != null ) {
					temp.neckPos = centerLegBasis.Multiply( _neckBone._defaultPosition - defaultCenterLegPos ) + centerLegPos;
				}

				temp.SetDirtyVariables();
				temp._SetCenterLegPos( ref centerLegPos ); // Optimized.
			}

			//----------------------------------------------------------------------------------------------------------------------------------------

			class SolverInternal
			{
				public class Limb
				{
					public Vector3[] beginPos = new Vector3[2];
					public bool[] targetBeginPosEnabled = new bool[2];
					public Vector3[] targetBeginPos = new Vector3[2];
					public Vector3[] targetBeginPosRated = new Vector3[2];
					public Vector3[] targetBeginToEnd = new Vector3[2];
					public float[] targetBeginToEndTempLen = new float[2];
					public bool[] endPosEnabled = new bool[2];
					public Vector3[] endPos = new Vector3[2];
					public FastLength[] targetBeginToEndLength = new FastLength[2];
					public bool[] pullEnabled = new bool[2];
					public float[] pull = new float[2];
					public float[] weight = new float[2];
					public Vector3[] backupBeginPos = new Vector3[2];

					public float lerpRate { get { return weight[1]; } }

					public bool targetBeginPosEnabledAnything
					{
						get
						{
							return targetBeginPosEnabled[0] || targetBeginPosEnabled[1];
						}
					}
					
					public void Prepare( Bone[] beginBones, Effector[] endEffectors, FastLength[] beginToEndLength )
					{
						for( int i = 0; i < 2; ++i ) {
							this.beginPos[i] = beginBones[i].worldPosition;
							this.targetBeginPos[i] = this.beginPos[i];
							this.targetBeginPosRated[i] = this.beginPos[i];
							this.targetBeginPosEnabled[i] = false;
							this.targetBeginToEnd[i] = Vector3.zero;
							this.targetBeginToEndTempLen[i] = 0.0f;

							if( endEffectors[i].positionEnabled ) {
								this.endPosEnabled[i] = true;
								this.endPos[i] = endEffectors[i]._hidden_worldPosition;
							} else if( endEffectors[i].bone.transformIsAlive ) { // for other limbs.
								this.endPosEnabled[i] = true;
								this.endPos[i] = endEffectors[i].bone.worldPosition;
							} else {
								this.endPosEnabled[i] = false;
								this.endPos[i] = Vector3.zero;
							}

							this.pullEnabled[i] = endEffectors[i].positionEnabled && endEffectors[i].pull > IKEpsilon;
							if( this.pullEnabled[i] ) {
								this.pull[i] = Mathf.Clamp01( endEffectors[i].pull );
								if( this.pull[i] >= 1.0f - IKEpsilon ) {
									this.targetBeginToEndLength[i] = beginToEndLength[i];
								} else {
									if( endEffectors[i].bone.transformIsAlive ) { // Lerp Bone to Effector position.
										float length = (endEffectors[i].bone.worldPosition - endEffectors[i]._hidden_worldPosition).magnitude;
										this.targetBeginToEndLength[i] = FastLength.FromLength( length * (1.0f - this.pull[i]) + (float)beginToEndLength[i] );
									} else { // Failsafe.
										this.pull[i] = 0.0f;
										this.targetBeginToEndLength[i] = beginToEndLength[i];
									}
								}
							} else {
								this.pull[i] = 0.0f;
								this.targetBeginToEndLength[i] = beginToEndLength[i];
							}
						}

						_PullToWeight2( this.pull, this.weight );				
					}

					public bool SolveTargetBeginPos()
					{
						bool r = false;
						for( int i = 0; i < 2; ++i ) {
							r |= _SolveTargetBeginPos( i, ref this.beginPos[i] );
						}
						return r;
					}

					public bool SolveTargetBeginPos( int i )
					{
						return _SolveTargetBeginPos( i, ref this.beginPos[i] );
					}

					public bool _SolveTargetBeginPos( int i, ref Vector3 beginPos )
					{
						this.targetBeginPos[i] = beginPos;
						this.targetBeginPosRated[i] = beginPos;
						this.targetBeginPosEnabled[i] = false;

						if( this.endPosEnabled[i] ) { // Memo: Not comprare to this.pullEnabled[i]
							Vector3 beginToEnd = (this.endPos[i] - beginPos);
							FastLength beginToEndLen = FastLength.FromVector3( beginToEnd );
							if( beginToEndLen.lengthSq > this.targetBeginToEndLength[i].lengthSq + IKEpsilon ) {
								if( beginToEndLen.lengthSq > IKEpsilon ) {
									float tempLength = beginToEndLen.length - this.targetBeginToEndLength[i].length;
									tempLength = tempLength / beginToEndLen.length;
									if( tempLength > IKMoveEpsilon ) {
										this.targetBeginPos[i] = beginPos + beginToEnd * tempLength;
										this.targetBeginPosRated[i] = this.targetBeginPos[i];
										this.targetBeginPosEnabled[i] = true;
										this.targetBeginToEnd[i] = beginToEnd;
										this.targetBeginToEndTempLen[i] = tempLength;
										return true;
									}
								}
							}
						}

						return false;
					}

					public bool ResolveTargetBeginPosRated( float limbRate )
					{
						bool r = false;
						for( int i = 0; i < 2; ++i ) {
							r |= ResolveTargetBeginPosRated( i, limbRate );
						}

						return r;
					}

					public bool ResolveTargetBeginPosRated( int i, float limbRate )
					{
						if( this.targetBeginPosEnabled[i] ) {
							if( limbRate < IKEpsilon ) {
								this.targetBeginPosRated[i] = this.beginPos[i];
							} else if( limbRate < 1.0f - IKEpsilon ) {
								float tempLength = this.targetBeginToEndTempLen[i];
								tempLength *= (1.0f - limbRate);
								this.targetBeginPosRated[i] = this.targetBeginPos[i] - this.targetBeginToEnd[i] * tempLength;
							} else {
								this.targetBeginPosRated[i] = this.targetBeginPos[i];
							}

							return true;
						}

						return false;
					}
				}

				public Limb arms = new Limb();
				public Limb legs = new Limb();

				public float[] armToLegPull = new float[2];
				public float[] armToLegWeight = new float[2];

				public float armToLegLerpRate { get { return armToLegWeight[1]; } }

				public const int MaxPullLength = 2;

				public Vector3[] origToBeginDir = new Vector3[MaxPullLength];
				public Vector3[] origToTargetBeginDir = new Vector3[MaxPullLength];
				public float[] origTheta = new float[MaxPullLength];
				public Vector3[] origAxis = new Vector3[MaxPullLength];
				public Vector3[] origTranslate = new Vector3[MaxPullLength];
				public float[] origFeedbackRate = new float[MaxPullLength];

				public Vector3[] torsoPos;

				public Vector3 neckPos;

				public Vector3[] nearArmPos;
				public Vector3[] shoulderPos;

				public Vector3[] armPos { get { return this.arms.beginPos; } }
				public Vector3[] legPos { get { return this.legs.beginPos; } }

				public Matrix3x3 _centerLegBoneBasisInv = Matrix3x3.identity; // Require setting on initialize.
				public Matrix3x3 _torsoUBoneLocalAxisBasisInv = Matrix3x3.identity; // Require setting on initialize.

				public Vector3 _centerArmPos = Vector3.zero;
				public Vector3 _centerLegPos = Vector3.zero;
				public Matrix3x3 _centerLegBasis = Matrix3x3.identity;
				public Matrix3x3 _torsoUBasis = Matrix3x3.identity;

				bool _isDirtyCenterArmPos = true;
				bool _isDirtyCenterLegPos = true;
				bool _isDirtyCenterLegBasis = true;
				bool _isDirtyTorsoUBasis = true;

				public Vector3 centerArmPos
				{
					get
					{
						if( _isDirtyCenterArmPos ) {
							_UpdateCenterArmPos();
						}

						return _centerArmPos;
					}
				}

				public Vector3 centerLegPos
				{
					get
					{
						if( _isDirtyCenterLegPos ) {
							_UpdateCenterLegPos();
						}

						return _centerLegPos;
					}
				}

				public void _UpdateCenterArmPos()
				{
					if( _isDirtyCenterArmPos ) {
						_isDirtyCenterArmPos = false;
						var nearArmPos = this.shoulderPos;
						if( nearArmPos == null ) {
							nearArmPos = this.armPos;
						}
						if( nearArmPos != null ) {
							_centerArmPos = (nearArmPos[0] + nearArmPos[1]) * 0.5f;
						}
					}
				}

				public void _UpdateCenterLegPos()
				{
					if( _isDirtyCenterLegPos ) {
						_isDirtyCenterLegPos = false;
						var legPos = this.legPos;
						if( legPos != null ) {
							_centerLegPos = (legPos[0] + legPos[1]) * 0.5f;
						}
					}
				}

				public void _SetCenterArmPos( ref Vector3 centerArmPos )
				{
					_isDirtyCenterArmPos = false;
					_centerArmPos = centerArmPos;
				}

				public void _SetCenterLegPos( ref Vector3 centerLegPos )
				{
					_isDirtyCenterLegPos = false;
					_centerLegPos = centerLegPos;
				}

				public Matrix3x3 centerLegBasis
				{
					get
					{
						if( _isDirtyCenterLegBasis ) {
							_UpdateCenterLegBasis();
						}

						return _centerLegBasis;
					}
				}

				public Matrix3x3 torsoUBasis
				{
					get
					{
						if( _isDirtyTorsoUBasis ) {
							_UpdateTorsoUBasis();
						}

						return _torsoUBasis;
					}
				}

				public void _UpdateCenterLegBasis()
				{
					if( _isDirtyCenterLegBasis ) {
						_isDirtyCenterLegBasis = false;
						var legPos = this.legPos;
						_centerLegBasis = Matrix3x3.identity;
						if( this.torsoPos != null && this.torsoPos.Length > 0 && legPos != null ) {
							Vector3 dirX = legPos[1] - legPos[0];
							Vector3 dirY = this.torsoPos[0] - this.centerLegPos;
							Vector3 dirZ = Vector3.Cross( dirX, dirY );
							dirX = Vector3.Cross( dirY, dirZ );
							if( _SafeNormalize( ref dirX, ref dirY, ref dirZ ) ) {
								_centerLegBasis.SetColumn( ref dirX, ref dirY, ref dirZ );
								_centerLegBasis *= _centerLegBoneBasisInv;
							}
						}
					}
				}

				public void _UpdateTorsoUBasis()
				{
					if( _isDirtyTorsoUBasis ) {
						_isDirtyTorsoUBasis = false;
						_torsoUBasis = Matrix3x3.identity;
						Vector3 dirY = (this.shoulderPos != null) ? (this.shoulderPos[1] + this.shoulderPos[0]) : (this.armPos[1] + this.armPos[0]);
						dirY = dirY * 0.5f - this.torsoUPos;
						Vector3 dirX = (this.shoulderPos != null) ? (this.shoulderPos[1] - this.shoulderPos[0]) : (this.armPos[1] - this.armPos[0]);
						Vector3 dirZ = Vector3.Cross( dirX, dirY );
						dirX = Vector3.Cross( dirY, dirZ );
						if( _SafeNormalize( ref dirX, ref dirY, ref dirZ ) ) {
							_torsoUBasis.SetColumn( ref dirX, ref dirY, ref dirZ );
							_torsoUBasis *= _torsoUBoneLocalAxisBasisInv;
						}
					}
				}

				public void SetDirtyVariables()
				{
					_isDirtyCenterArmPos = true;
					_isDirtyCenterLegPos = true;
					_isDirtyCenterLegBasis = true;
					_isDirtyTorsoUBasis = true;
				}

				public Vector3 torsoUPos
				{
					get
					{
						if( this.torsoPos != null && this.torsoPos.Length != 0 ) {
							return this.torsoPos[this.torsoPos.Length - 1];
						}

						return Vector3.zero;
					}
				}

				public class BackupData
				{
					public Vector3 centerArmPos;
					public Vector3 centerLegPos;
					public Matrix3x3 centerLegBasis;
					public Matrix3x3 torsoUBasis;

					public Vector3[] torsoPos;
 					public Vector3 neckPos;
					public Vector3[] shoulderPos;

					public Vector3[] armPos = new Vector3[2];
					public Vector3[] legPos = new Vector3[2];
				}

				BackupData _backupData = new BackupData();

				public void Backup()
				{
					_backupData.centerArmPos = this.centerArmPos;
					_backupData.centerLegPos = this.centerLegPos;
					_backupData.centerLegBasis = this.centerLegBasis;
					_backupData.torsoUBasis = this.torsoUBasis;
					Util.CloneArray( ref _backupData.torsoPos, this.torsoPos );
					_backupData.neckPos = this.neckPos;
					Util.CloneArray( ref _backupData.shoulderPos, this.shoulderPos );
					Util.CloneArray( ref _backupData.armPos, this.arms.beginPos );
					Util.CloneArray( ref _backupData.legPos, this.legs.beginPos );
				}

				public void Restore()
				{
					_isDirtyCenterArmPos = false;
					_isDirtyCenterLegPos = false;
					_isDirtyCenterLegBasis = false;
					_isDirtyTorsoUBasis = false;
					_centerArmPos = _backupData.centerArmPos;
					_centerLegPos = _backupData.centerLegPos;
					_centerLegBasis = _backupData.centerLegBasis;
					_torsoUBasis = _backupData.torsoUBasis;
					Util.CloneArray( ref this.torsoPos, _backupData.torsoPos );
					this.neckPos = _backupData.neckPos;
					Util.CloneArray( ref this.shoulderPos, _backupData.shoulderPos );
					Util.CloneArray( ref this.arms.beginPos, _backupData.armPos );
					Util.CloneArray( ref this.legs.beginPos, _backupData.legPos );
				}
				
				public bool PrepareLowerRotation( int origIndex )
				{
					bool r = false;
					for( int i = 0; i < 2; ++i ) {
						this.legs._SolveTargetBeginPos( i, ref this.legs.beginPos[i] );
						r |= _PrepareLimbRotation( this.legs, i, origIndex, ref this.legs.beginPos[i] );
					}
					return r;
				}

				public bool _PrepareLimbRotation( Limb limb, int i, int origIndex, ref Vector3 beginPos )
				{
					this.origTheta[i] = 0.0f;
					this.origAxis[i] = new Vector3( 0.0f, 0.0f, 1.0f );

					if( i >= 2 ) {
						return false; // Unsupported yet.(headPull)
					}

					if( !limb.targetBeginPosEnabled[i] ) {
						return false;
					}

					// Memo: limb index = orig index.

					var targetBeginPos = limb.targetBeginPosRated;

					Vector3 origPos = (origIndex == -1) ? this.centerLegPos : this.torsoPos[origIndex];

					return _ComputeThetaAxis(
						ref origPos,
						ref beginPos,
						ref targetBeginPos[i],
						out this.origTheta[i],
						out this.origAxis[i] );
				}

				public void SetSolveFeedbackRate( float feedbackRate )
				{
					for( int i = 0; i < this.origFeedbackRate.Length; ++i ) {
						this.origFeedbackRate[i] = feedbackRate;
					}
				}

				public void SetSolveFeedbackRate( int i, float feedbackRate )
				{
					this.origFeedbackRate[i] = feedbackRate;
				}
				
				public bool SolveLowerRotation( int origIndex, out Quaternion origRotation )
				{
					return _SolveLimbRotation( this.legs, origIndex, out origRotation );
				}

				bool _SolveLimbRotation( Limb limb, int origIndex, out Quaternion origRotation )
				{
					origRotation = Quaternion.identity;

					int pullIndex = -1;
					int pullLength = 0;
					for( int i = 0; i < 2; ++i ) {
						if( limb.targetBeginPosEnabled[i] ) {
							pullIndex = i;
							++pullLength;
						}
					}

					if( pullLength == 0 ) {
						return false; // Failsafe.
					}

					if( pullLength == 1 ) {
						int i0 = pullIndex;
						if( this.origTheta[i0] == 0.0f ) {
							return false;
						}

						float lerpRate = limb.lerpRate;
						if( i0 == 0 ) {
							lerpRate = 1.0f - lerpRate;
						}

						origRotation = _GetRotation( ref this.origAxis[i0], this.origTheta[i0], this.origFeedbackRate[i0] * lerpRate );
						return true;
					}

					if( pullLength == 2 ) {
						float lerpRate = limb.lerpRate;
						Quaternion origRotation0 = _GetRotation( ref this.origAxis[0], this.origTheta[0], this.origFeedbackRate[0] * 0.5f );
						Quaternion origRotation1 = _GetRotation( ref this.origAxis[1], this.origTheta[1], this.origFeedbackRate[1] * 0.5f );
						origRotation = Quaternion.Lerp( origRotation0, origRotation1, lerpRate );
						origRotation = origRotation * origRotation; // Optimized: Not normalize.
						return true;
					}

					return false;
				}

				public void LowerRotationBeginOnly( int origIndex, ref Quaternion origRotation )
				{
					_LimbRotationBeginOnly( this.legs, origIndex, ref origRotation );
					_isDirtyCenterLegBasis = true;
				}
				
				void _LimbRotationBeginOnly( Limb limb, int origIndex, ref Quaternion origRotation )
				{
					Vector3 origPos = (origIndex == -1) ? this.centerLegPos : this.torsoPos[origIndex];
					Matrix3x3 origBasis = new Matrix3x3( origRotation );

					var beginPos = limb.beginPos;
					if( beginPos != null ) {
						for( int i = 0; i < beginPos.Length; ++i ) {
							beginPos[i] = origBasis.Multiply( beginPos[i] - origPos ) + origPos;
						}
					}
				}

				public void UpperRotation( int origIndex, ref Matrix3x3 origBasis )
				{
					Vector3 origPos = (origIndex == -1) ? this.centerLegPos : this.torsoPos[origIndex];

					{
						var armPos = this.armPos;
						if( armPos != null ) {
							for( int i = 0; i < armPos.Length; ++i ) {
								armPos[i] = origBasis.Multiply( armPos[i] - origPos ) + origPos;
							}
						}
					}

					if( this.shoulderPos != null ) {
						for( int i = 0; i < this.shoulderPos.Length; ++i ) {
							this.shoulderPos[i] = origBasis.Multiply( this.shoulderPos[i] - origPos ) + origPos;
						}
					}
					
					this.neckPos = origBasis.Multiply( this.neckPos - origPos ) + origPos;

					// Legs					
					if( origIndex == -1 ) { // Rotation origin is centerLeg
						var legPos = this.legPos;
						if( legPos != null ) {
							for( int i = 0; i < legPos.Length; ++i ) {
								legPos[i] = origBasis.Multiply( legPos[i] - origPos ) + origPos;
							}
						}
						
						_isDirtyCenterLegBasis = true;
					}

					// Torso
					for( int t = (origIndex == -1) ? 0 : origIndex; t < this.torsoPos.Length; ++t ) {
						Vector3 torsoPos = this.torsoPos[t];
						torsoPos = origBasis.Multiply( torsoPos - origPos ) + origPos;
						this.torsoPos[t] = torsoPos;
					}

					_isDirtyCenterArmPos = true;
					_isDirtyTorsoUBasis = true;
				}

				public void LowerRotation( int origIndex, ref Quaternion origRotation, bool bodyRotation )
				{
					Matrix3x3 origBasis = new Matrix3x3( origRotation );
					LowerRotation( origIndex, ref origBasis, bodyRotation );
				}

				public void LowerRotation( int origIndex, ref Matrix3x3 origBasis, bool bodyRotation )
				{
					Vector3 origPos = (origIndex == -1) ? this.centerLegPos : this.torsoPos[origIndex];

					var legPos = this.legPos;
					if( legPos != null ) {
						for( int i = 0; i < 2; ++i ) {
							legPos[i] = origBasis.Multiply( legPos[i] - origPos ) + origPos;
						}
					}

					if( this.torsoPos != null ) {
						int length = bodyRotation ? this.torsoPos.Length : origIndex;
						for( int n = 0; n < length; ++n ) {
							this.torsoPos[n] = origBasis.Multiply( this.torsoPos[n] - origPos ) + origPos;
						}
					}

					_isDirtyCenterArmPos = true;
					_isDirtyCenterLegPos = true;
					_isDirtyCenterLegBasis = true;

					if( bodyRotation || this.torsoPos == null || origIndex + 1 == this.torsoPos.Length ) {
						this.neckPos = origBasis.Multiply( this.neckPos - origPos ) + origPos;

						var armPos = this.armPos;
						if( armPos != null ) {
							for( int i = 0; i < 2; ++i ) {
								armPos[i] = origBasis.Multiply( armPos[i] - origPos ) + origPos;
							}
						}

						if( this.shoulderPos != null ) {
							for( int i = 0; i < 2; ++i ) {
								this.shoulderPos[i] = origBasis.Multiply( this.shoulderPos[i] - origPos ) + origPos;
							}
						}

						_isDirtyTorsoUBasis = true;
					}
				}

				public bool PrepareLowerTranslate()
				{
					bool r = false;
					for( int i = 0; i < 2; ++i ) {
						this.legs._SolveTargetBeginPos( i, ref this.legs.beginPos[i] );
						r |= _PrepareLimbTranslate( this.legs, i, ref this.legs.beginPos[i] );
					}
					return r;
				}

				bool _PrepareLimbTranslate( Limb limb, int i, ref Vector3 beginPos )
				{
					this.origTranslate[i] = Vector3.zero;
					if( limb.targetBeginPosEnabled[i] ) {
						this.origTranslate[i] = (limb.targetBeginPos[i] - beginPos);
						return true;
					}

					return false;
				}

				public bool SolveLowerTranslate( out Vector3 translate )
				{
					return _SolveLimbTranslate( this.legs, out translate );
				}

				bool _SolveLimbTranslate( Limb limb, out Vector3 origTranslate )
				{
					origTranslate = Vector3.zero;

					if( limb.targetBeginPosEnabled[0] && limb.targetBeginPosEnabled[1] ) {
						origTranslate = Vector3.Lerp( this.origTranslate[0], this.origTranslate[1], limb.lerpRate );
					} else if( limb.targetBeginPosEnabled[0] || limb.targetBeginPosEnabled[1] ) {
						int i0 = limb.targetBeginPosEnabled[0] ? 0 : 1;
						float lerpRate1to0 = limb.targetBeginPosEnabled[0] ? (1.0f - limb.lerpRate) : limb.lerpRate;
						origTranslate = this.origTranslate[i0] * lerpRate1to0;
					}

					return (origTranslate != Vector3.zero);
				}
				
				public void LowerTranslateBeginOnly( ref Vector3 origTranslate )
				{
					_LimbTranslateBeginOnly( this.legs, ref origTranslate );
					_centerLegPos += origTranslate; // Optimized.
				}

				void _LimbTranslateBeginOnly( Limb limb, ref Vector3 origTranslate )
				{
					for( int i = 0; i < 2; ++i ) {
						limb.beginPos[i] += origTranslate;
					}
				}

				public void Translate( ref Vector3 origTranslate )
				{
					var legPos = this.legPos;
					var armPos = this.armPos;

					_centerArmPos += origTranslate;
					_centerLegPos += origTranslate;

					if( this.torsoPos != null ) {
						for( int i = 0; i < this.torsoPos.Length; ++i ) {
							this.torsoPos[i] += origTranslate;
						}
					}
					
					this.neckPos += origTranslate;
					
					for( int i = 0; i < 2; ++i ) {
						if( this.legPos != null ) {
							legPos[i] += origTranslate;
						}

						if( this.shoulderPos != null ) {
							this.shoulderPos[i] += origTranslate;
						}

						if( armPos != null ) {
							armPos[i] += origTranslate;
						}
					}
				}

				public void SolveShoulderToArm(
					int i,
					Bone[] shoulderBones,
					FastLength[] shoulderToArmLength,
					float limitYPlus,
					float limitYMinus,
					float limitZ )
				{
					if( this.shoulderPos != null ) {
						var armPos = this.arms.beginPos;
						var targetArmPos = this.arms.targetBeginPosRated;
						var targetArmPosEnabled = this.arms.targetBeginPosEnabled;
	
						if( targetArmPosEnabled[i] ) {
							if( !IsFuzzy( ref armPos[i], ref targetArmPos[i] ) ) {
								Vector3 dirX = targetArmPos[i] - this.shoulderPos[i];
								if( _SafeNormalize( ref dirX ) ) {
									Matrix3x3 worldBasis = this.torsoUBasis;
									worldBasis *= shoulderBones[i]._localAxisBasis;
									dirX = worldBasis.transpose.Multiply( dirX );
									_LimitYZ( i != 0, ref dirX, limitYMinus, limitYPlus, limitZ, limitZ );
									dirX = worldBasis.Multiply( dirX );
	
									this.armPos[i] = this.shoulderPos[i] + dirX * shoulderToArmLength[i].length;
								}
							}
						}
					}
				}
			}

			//----------------------------------------------------------------------------------------------------------------------------------------

			static bool _ComputeCenterLegBasis(
				out Matrix3x3 centerLegBasis,
				ref Vector3 torsoPos,
				ref Vector3 leftLegPos,
				ref Vector3 rightLegPos )
			{
				Vector3 dirX = rightLegPos - leftLegPos;
				Vector3 dirY = torsoPos - (rightLegPos + leftLegPos) * 0.5f;
				if( _SafeNormalize( ref dirY ) ) {
					return _ComputeBasisFromXYLockY( out centerLegBasis, ref dirX, ref dirY );
				} else {
					centerLegBasis = Matrix3x3.identity;
					return false;
				}
			}

			//----------------------------------------------------------------------------------------------------------------------------------------

			static Quaternion _GetRotation( ref Vector3 axisDir, float theta, float rate )
			{
				if( (theta >= -IKEpsilon && theta <= IKEpsilon) || (rate >= -IKEpsilon && rate <= IKEpsilon) ) {
					return Quaternion.identity;
				} else {
					return Quaternion.AngleAxis( _SafeAcos( theta ) * rate * Mathf.Rad2Deg, axisDir );
				}
			}

			static void _PullToWeight2( float[] pull, float[] weight )
			{
				Assert( pull != null && pull.Length == 2 );
				Assert( weight != null && weight.Length == 2 );
				if( pull[0] > 0.0f && pull[1] > 0.0f ) {
					float totalPull = pull[0] + pull[1];
					weight[0] = Mathf.Clamp01( pull[0] / totalPull );
					weight[1] = 1.0f - weight[0];
				} else if( pull[0] > 0.0f ) {
					weight[0] = 1.0f;
					weight[1] = 0.0f;
				} else if( pull[1] > 0.0f ) {
					weight[0] = 0.0f;
					weight[1] = 1.0f;
				} else {
					weight[0] = 0.0f;
					weight[1] = 0.0f;
				}
			}
		}
	}
}
