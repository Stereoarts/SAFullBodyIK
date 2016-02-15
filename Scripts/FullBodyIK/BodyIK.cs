// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing

#if SAFULLBODYIK_DEBUG
//#define SAFULLBODYIK_DEBUG_DETAIL_SOLVETORSO
//#define SAFULLBODYIK_DEBUG_DETAIL_SOLVELEGS
#endif

using UnityEngine;

namespace SA
{
	public partial class FullBodyIK
	{
		public class BodyIK
		{
			Bone _hipsBone; // Null accepted.
			Bone[] _spineBones; // Null accepted.
			Bone _spineBone; // Null accepted.
			Bone _spineUBone; // Null accepted.
			Bone _neckBone; // Null accepted.
			Bone[] _legBones; // Null accepted.
			Bone[] _shoulderBones; // Null accepted.
			Bone[] _armBones; // Null accepted.
			Effector _rootEffector;
			Effector _hipsEffector;
			Effector _neckEffector;
			Effector _eyesEffector;
			Effector[] _armEffectors = new Effector[2];
			Effector[] _wristEffectors = new Effector[2];
			Effector[] _footEffectors = new Effector[2];

			Vector3 _defaultCenterLegPos = Vector3.zero;
			Vector3 _defaultCenterFootPos = Vector3.zero;

			Matrix3x3 _centerLegBoneBasis = Matrix3x3.identity;
			Matrix3x3 _centerLegBoneBasisInv = Matrix3x3.identity;

			Matrix3x3 _centerLegToArmBasis = Matrix3x3.identity;        // dirX = legPos[1] - legPos[0], dirY = centerArmPos - centerLegPos
			Matrix3x3 _centerLegToArmBasisInv = Matrix3x3.identity;     // _centerLegToArmBasis.transpose
			Matrix3x3 _centerLegToArmBoneToBaseBasis = Matrix3x3.identity;
			Matrix3x3 _centerLegToArmBaseToBoneBasis = Matrix3x3.identity;

			FastLength[] _shoulderToArmLength = new FastLength[2];
			FastLength[] _legEffectorMaxLength = new FastLength[2];
			FastLength[] _armEffectorMaxLength = new FastLength[2];

			Vector3 _defaultCenterArmPos = Vector3.zero;
			float _defaultCenterLegLen; // LeftLeg to RightLeg Length.
			float _defaultCenterLegHalfLen; // LeftLeg to RightLeg Length / 2.

			Vector3 _defaultCenterEyePos = Vector3.zero;

			SolverInternal _solverInternal;

			Settings _settings;
			InternalValues _internalValues;

			public BodyIK( FullBodyIK fullBodyIK )
			{
				Assert( fullBodyIK != null );

				_settings = fullBodyIK.settings;
				_internalValues = fullBodyIK.internalValues;

				_hipsBone = _PrepareBone( fullBodyIK.bodyBones.hips );
				_neckBone = _PrepareBone( fullBodyIK.headBones.neck );
				_rootEffector = fullBodyIK.rootEffector;
				_hipsEffector = fullBodyIK.bodyEffectors.hips;
				_neckEffector = fullBodyIK.headEffectors.neck;
				_eyesEffector = fullBodyIK.headEffectors.eyes;
				_armEffectors[0] = fullBodyIK.leftArmEffectors.arm;
				_armEffectors[1] = fullBodyIK.rightArmEffectors.arm;
				_wristEffectors[0] = fullBodyIK.leftArmEffectors.wrist;
				_wristEffectors[1] = fullBodyIK.rightArmEffectors.wrist;
				_footEffectors[0] = fullBodyIK.leftLegEffectors.foot;
				_footEffectors[1] = fullBodyIK.rightLegEffectors.foot;

				_spineBones = _PrepareSpineBones( fullBodyIK.bones );
				if( _spineBones != null && _spineBones.Length > 0 ) {
					_spineBone = _spineBones[0];
					_spineUBone = _spineBones[_spineBones.Length - 1];
				}

				// Memo: These should be pair bones.(Necessary each side bones.)
				_legBones = _PrepareBones( fullBodyIK.leftLegBones.leg, fullBodyIK.rightLegBones.leg );
				_armBones = _PrepareBones( fullBodyIK.leftArmBones.arm, fullBodyIK.rightArmBones.arm );
				_shoulderBones = _PrepareBones( fullBodyIK.leftArmBones.shoulder, fullBodyIK.rightArmBones.shoulder );

				_Prepare( fullBodyIK );
			}

			static Bone _PrepareBone( Bone bone )
			{
				return (bone != null && bone.transformIsAlive) ? bone : null;
			}

			static Bone[] _PrepareSpineBones( Bone[] bones )
			{
				if( bones == null || bones.Length != (int)BoneLocation.Max ) {
					Assert( false );
					return null;
				}

				int spineLength = 0;
				for( int i = (int)BoneLocation.Spine; i <= (int)BoneLocation.SpineU; ++i ) {
					if( bones[i] != null && bones[i].transformIsAlive ) {
						++spineLength;
					}
				}

				if( spineLength == 0 ) {
					return null;
				}

				Bone[] spineBones = new Bone[spineLength];
				int index = 0;
				for( int i = (int)BoneLocation.Spine; i <= (int)BoneLocation.SpineU; ++i ) {
					if( bones[i] != null && bones[i].transformIsAlive ) {
						spineBones[index] = bones[i];
						++index;
					}
				}

				return spineBones;
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
						SAFBIKMatMult( out _centerLegToArmBoneToBaseBasis, ref _centerLegToArmBasisInv, ref _internalValues.defaultRootBasis  );
						_centerLegToArmBaseToBoneBasis = _centerLegToArmBoneToBaseBasis.transpose;
					}
				}

				if( _footEffectors != null ) {
					if( _footEffectors[0].bone != null && _footEffectors[1].bone != null ) {
						_defaultCenterFootPos = (_footEffectors[0].bone._defaultPosition + _footEffectors[1].bone._defaultPosition) * 0.5f;
						_defaultCenterLegLen = (_footEffectors[1].bone._defaultPosition - _footEffectors[0].bone._defaultPosition).magnitude;
						_defaultCenterLegHalfLen = _defaultCenterLegLen * 0.5f;
					}
				}

				if( _spineBone != null && _legBones != null ) {
					Assert( _spineBone.transformIsAlive && _legBones[0].transformIsAlive && _legBones[1].transformIsAlive ); // Already checked.
					if( _ComputeCenterLegBasis( out _centerLegBoneBasis,
						ref _spineBone._defaultPosition,
						ref _legBones[0]._defaultPosition,
						ref _legBones[1]._defaultPosition ) ) {
						_centerLegBoneBasisInv = _centerLegBoneBasis.transpose;
					}
				}

				if( _shoulderBones != null && _armBones != null ) {
					for( int i = 0; i < 2; ++i ) {
						Assert( _shoulderBones[i].transformIsAlive && _armBones[i].transformIsAlive ); // Already checked.
						_shoulderToArmLength[i] = FastLength.FromLengthSq( (_shoulderBones[i]._defaultPosition - _armBones[i]._defaultPosition).sqrMagnitude );
					}
				}

				for( int i = 0; i < 2; ++i ) {
					Bone kneeBone = (i == 0) ? fullBodyIK.leftLegBones.knee : fullBodyIK.rightLegBones.knee;
					Bone elbowBone = (i == 0) ? fullBodyIK.leftArmBones.elbow : fullBodyIK.rightArmBones.elbow;
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

			public bool Solve()
			{
				if( !_IsEffectorEnabled() ) {
					return false;
				}

				if( !_PrepareSolverInternal() ) {
					return false;
				}

				var temp = _solverInternal;

				bool neckEnabled = (_neckBone != null && _neckBone.transformIsAlive);

				// Memo: arms / legs don't setup here. (Already prepared in _PrepareSolverInternal())

				if( temp.spinePos != null ) {
					for( int i = 0; i < temp.spinePos.Length; ++i ) {
						temp.spinePos[i] = _spineBones[i].worldPosition;
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
					_PresolveHips();
				}

#if SAFULLBODYIK_DEBUG
				bool _isVisibleWorldTransform = true;
				_internalValues.UpdateDebugValue( "_isVisibleWorldTransform", ref _isVisibleWorldTransform );
#endif

				if( _settings.bodyIK.lowerSolveEnabled ) {
					_LowerSolve( false, true );
				}

				if( _settings.bodyIK.upperSolveEnabled ) {
					_UpperSolve();
				}

				if( _settings.bodyIK.lowerSolveEnabled ) {
					_LowerSolve( false, false );
				}

				if( _settings.bodyIK.computeWorldTransform ) {
					_ComputeWorldTransform();
				}

#if SAFULLBODYIK_DEBUG
				if( _isVisibleWorldTransform ) {
					_internalValues.AddDebugPoint( temp.centerLegPos );
					if( temp.spinePos != null ) {
						for( int i = 0; i < temp.spinePos.Length; ++i ) {
							_internalValues.AddDebugPoint( temp.spinePos[i] );
						}
					}
					_internalValues.AddDebugPoint( temp.neckPos );
					for( int i = 0; i < 2; ++i ) {
						if( temp.shoulderPos != null ) {
							_internalValues.AddDebugPoint( temp.shoulderPos[i] );
						}
						_internalValues.AddDebugPoint( temp.armPos[i] );
						_internalValues.AddDebugPoint( temp.legPos[i] );
					}
				}
#endif

				return true;
			}

			Vector3[] _tempArmPos = new Vector3[2];
			Vector3[] _tempArmPos2 = new Vector3[2];

			bool _UpperSolve()
			{
				var temp = _solverInternal;
				if( temp == null || temp.spinePos == null || temp.spinePos.Length == 0 || _wristEffectors == null ) {
					return false; // No moved.
				}

				float hipsWeight = _hipsEffector.positionEnabled ? _hipsEffector.positionWeight : 0.0f;
				float neckWeight = _neckEffector.positionEnabled ? _neckEffector.positionWeight : 0.0f;
				float eyesWeight = _eyesEffector.positionEnabled ? _eyesEffector.positionWeight : 0.0f;
				float armPull0 = _wristEffectors[0].positionEnabled ? _wristEffectors[0].pull : 0.0f;
				float armPull1 = _wristEffectors[1].positionEnabled ? _wristEffectors[1].pull : 0.0f;
				if( hipsWeight <= IKEpsilon && neckWeight <= IKEpsilon && eyesWeight <= IKEpsilon && armPull0 <= IKEpsilon && armPull1 <= IKEpsilon ) {
					return false; // No moved.
				}

				float upperCenterLegRotateRate = _internalValues.bodyIK.upperCenterLegRotateRate.value;
				float upperSpineRotateRate = _internalValues.bodyIK.upperSpineRotateRate.value;

				Vector3 baseCenterLegPos = Vector3.zero;

				bool continuousSolverEnabled = _internalValues.continuousSolverEnabled;

				// Preprocess for armPos / armPos2
				if( continuousSolverEnabled ) {
					Matrix3x3 centerLegBasis;
					_UpperSolve_PresolveBaseCenterLegTransform( out baseCenterLegPos, out centerLegBasis );

#if SAFULLBODYIK_DEBUG
					_internalValues.AddDebugPoint( baseCenterLegPos, Color.gray, 0.05f );
					_internalValues.AddDebugPoint( baseCenterLegPos + centerLegBasis.column0 * 0.1f, Color.yellow, 0.05f );
#endif

					temp.Backup(); // for Testsolver.

					if( _spineBones != null ) {
						for( int i = 0; i < _spineBones.Length; ++i ) {
							SAFBIKMatMultVecPreSubAdd( out temp.spinePos[i], ref centerLegBasis, ref _spineBones[i]._defaultPosition, ref _defaultCenterLegPos, ref baseCenterLegPos );
						}
					}
					if( _neckBone != null ) {
						SAFBIKMatMultVecPreSubAdd( out temp.neckPos, ref centerLegBasis, ref _neckBone._defaultPosition, ref _defaultCenterLegPos, ref baseCenterLegPos );
					}
					for( int n = 0; n < 2; ++n ) {
						if( _shoulderBones != null ) {
							SAFBIKMatMultVecPreSubAdd( out temp.shoulderPos[n], ref centerLegBasis, ref _shoulderBones[n]._defaultPosition, ref _defaultCenterLegPos, ref baseCenterLegPos );
						}
						if( _armBones != null ) {
							SAFBIKMatMultVecPreSubAdd( out temp.armPos[n], ref centerLegBasis, ref _armBones[n]._defaultPosition, ref _defaultCenterLegPos, ref baseCenterLegPos );
						}
						if( _legBones != null ) {
							SAFBIKMatMultVecPreSubAdd( out temp.legPos[n], ref centerLegBasis, ref _legBones[n]._defaultPosition, ref _defaultCenterLegPos, ref baseCenterLegPos );
						}
					}
					temp.SetDirtyVariables();
				}

				// Solve.
				if( !temp.arms.SolveTargetBeginPos() ) {
					// Nothing.
				}

				// PreTranslate.
				if( _internalValues.bodyIK.upperPreTranslateRate.isGreater0 ) {
					for( int i = 0; i < 2; ++i ) {
						float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
						temp.arms.ResolveTargetBeginPosRated( i, pull ); // Memo: Not contain upperPreTranslateRate

						if( temp.arms.targetBeginPosEnabled[i] ) {
							// for _UpperSolve_Translate()
							_UpperSolve_ShoulderToArm( i ); // Update temp.arms.beginPos(armPos)
						}
					}

					_UpperSolve_Translate(
						ref _internalValues.bodyIK.upperPreTranslateRate,
						ref _internalValues.bodyIK.upperContinuousPreTranslateStableRate,
						ref baseCenterLegPos );

					temp.arms.SolveTargetBeginPos(); // Must call.
				}

				int spineLength = (_spineBones != null) ? (_spineBones.Length) : 0;

				// Resolve for armPos.
				if( _internalValues.bodyIK.upperCenterLegRotateRate.isGreater0 ) {
					for( int i = 0; i < 2; ++i ) {
						float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
						temp.arms.ResolveTargetBeginPosRated( i, pull * upperCenterLegRotateRate );
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
					// for centerLeg / spine solvers.
					_UpperSolve_ShoulderToArm();
				}

				// Collect armPos.
				if( _internalValues.bodyIK.upperCenterLegRotateRate.isGreater0 ) {
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
				if( _internalValues.bodyIK.isFuzzyUpperCenterLegAndSpineRotationRate ) {
					for( int i = 0; i < 2; ++i ) {
						_tempArmPos2[i] = _tempArmPos[i];
					}
				} else if( _internalValues.bodyIK.upperSpineRotateRate.isGreater0 ) {
					// Memo: _upper_limbRotateRate2 should be saved to minimum value to _upper_limbRotateRate1.
					for( int i = 0; i < 2; ++i ) {
						float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
						temp.arms.ResolveTargetBeginPosRated( i, pull * upperSpineRotateRate );
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

				float postTranslateRate = _settings.bodyIK.upperPostTranslateRate;

				// for neck / eyes.
				// presolvedCenterLegPos2 = presolveCenterLegPos + presolved postTranslate.
				Vector3 presolvedCenterLegPos2 = temp.centerLegPos;
				if( neckWeight > IKEpsilon || eyesWeight > IKEpsilon ) {
					Vector3 presolvedTranslate;
					if( _UpperSolve_PreTranslate( out presolvedTranslate,
						ref _internalValues.bodyIK.upperPostTranslateRate,
						ref _internalValues.bodyIK.upperContinuousPostTranslateStableRate,
						ref baseCenterLegPos ) ) {
						presolvedCenterLegPos2 += presolvedTranslate;
                    }
				}

				if( continuousSolverEnabled ) {
					// Salvage bone positions at end of testsolver.
					temp.Restore();

					if( !temp.arms.SolveTargetBeginPos() ) {
						// Nothing.
					}

					if( _internalValues.bodyIK.upperContinuousPreTranslateRate.isGreater0 ||
						_internalValues.bodyIK.upperContinuousPreTranslateStableRate.isGreater0 ) {
						for( int i = 0; i < 2; ++i ) {
							float pull = _wristEffectors[i].positionEnabled ? _wristEffectors[i].pull : 0.0f;
							temp.arms.ResolveTargetBeginPosRated( i, pull ); // Memo: Not contain upperContinuousPreTranslateRate

							if( temp.arms.targetBeginPosEnabled[i] ) { // for _UpperSolve_Translate()
								_UpperSolve_ShoulderToArm( i ); // Update armPos(temp.arms.beginPos)
							}
						}

						_UpperSolve_Translate(
							ref _internalValues.bodyIK.upperContinuousPreTranslateRate,
							ref _internalValues.bodyIK.upperContinuousPreTranslateStableRate,
							ref baseCenterLegPos );

						temp.arms.SolveTargetBeginPos(); // Must call.
					}

					// for centerLeg / spine solvers.
					_UpperSolve_ShoulderToArm();
				}

				// pending: contain neck.
				Vector3 centerArmDirX = _tempArmPos[1] - _tempArmPos[0];
				Vector3 centerArmDirX2 = _tempArmPos2[1] - _tempArmPos2[0];

				if( !SAFBIKVecNormalize2( ref centerArmDirX, ref centerArmDirX2 ) ) {
					return false; // Failsafe.(No moved.)
				}

#if SAFULLBODYIK_DEBUG
				for( int i = 0; i < 2; ++i ) {
					_internalValues.AddDebugPoint( _tempArmPos[i], Color.blue );
					_internalValues.AddDebugPoint( _tempArmPos2[i], Color.blue );
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
					_internalValues.AddDebugPoint( centerArmPos, Color.blue, 0.024f );
					_internalValues.AddDebugPoint( centerArmPos2, Color.blue, 0.024f );
					_internalValues.AddDebugPoint( _tempArmPos[0] + vecX, Color.white );
					_internalValues.AddDebugPoint( _tempArmPos[1] - vecX, Color.black );
#endif
					centerArmDirY = centerArmPos - temp.centerLegPos;
					centerArmDirY2 = centerArmPos2 - temp.centerLegPos;
				}

				if( !SAFBIKVecNormalize2( ref centerArmDirY, ref centerArmDirY2 ) ) {
					return false; // Failsafe.(No moved.)
				}

				// Neck
				if( neckWeight > IKEpsilon ) {
					Vector3 centerLegPos = presolvedCenterLegPos2;
					Vector3 neckPosTo = _neckEffector.worldPosition;

					Matrix3x3 toBasis2, toBasis;
					float theta;
					Vector3 axis;

					float upperNeckRate1 = _settings.bodyIK.upperNeckToCenterLegRate;
					float upperNeckRate2 = _settings.bodyIK.upperNeckToSpineRate;

					if( SAFBIKComputeBasisFromXYLockY( out toBasis2, ref centerArmDirX2, ref centerArmDirY2 ) ) {
						SAFBIKMatMultRet0( ref toBasis2, ref _centerLegToArmBasisInv );
						Vector3 neckPosFrom2;
						SAFBIKMatMultVecPreSubAdd( out neckPosFrom2, ref toBasis2, ref _neckBone._defaultPosition, ref _defaultCenterLegPos, ref centerLegPos );
						if( _ComputeThetaAxis( ref centerLegPos, ref neckPosFrom2, ref neckPosTo, out theta, out axis ) ) {
							Matrix3x3 rotateBasis2;
							_LerpRotateBasis( out rotateBasis2, ref axis, theta, upperNeckRate2 * neckWeight );
							SAFBIKMatMultVec( out centerArmDirX2, ref rotateBasis2, ref centerArmDirX2 );
							SAFBIKMatMultVec( out centerArmDirY2, ref rotateBasis2, ref centerArmDirY2 );
						}
					}

					if( SAFBIKComputeBasisFromXYLockY( out toBasis, ref centerArmDirX, ref centerArmDirY ) ) {
						SAFBIKMatMultRet0( ref toBasis, ref _centerLegToArmBasisInv );
						Vector3 neckPosFrom;
						SAFBIKMatMultVecPreSubAdd( out neckPosFrom, ref toBasis, ref _neckBone._defaultPosition, ref _defaultCenterLegPos, ref centerLegPos );
						if( _ComputeThetaAxis( ref centerLegPos, ref neckPosFrom, ref neckPosTo, out theta, out axis ) ) {
							Matrix3x3 rotateBasis;
							_LerpRotateBasis( out rotateBasis, ref axis, theta, upperNeckRate1 * neckWeight );
							SAFBIKMatMultVec( out centerArmDirX, ref rotateBasis, ref centerArmDirX );
							SAFBIKMatMultVec( out centerArmDirY, ref rotateBasis, ref centerArmDirY );
						}
					}
				}

				// Eyes
				if( eyesWeight > IKEpsilon ) {
					// Based on centerArmDirX2 / centerArmDirY2
					Matrix3x3 toBasis;
					if( SAFBIKComputeBasisFromXYLockY( out toBasis, ref centerArmDirX2, ref centerArmDirY2 ) ) {
						Matrix3x3 toBasisGlobal;
						SAFBIKMatMult( out toBasisGlobal, ref toBasis, ref _centerLegToArmBasisInv );

						Matrix3x3 fromBasis = toBasis;
						SAFBIKMatMultRet0( ref toBasis, ref _centerLegToArmBoneToBaseBasis );

						Vector3 eyePos;
                        SAFBIKMatMultVecPreSubAdd( out eyePos, ref toBasisGlobal, ref _defaultCenterEyePos, ref _defaultCenterLegPos, ref presolvedCenterLegPos2 );

						Vector3 eyeDir = _eyesEffector.worldPosition - eyePos;

						{
							float upperEyesXLimit = _internalValues.bodyIK.upperEyesLimitThetaX.sin;
							float upperEyesYUpLimit = _internalValues.bodyIK.upperEyesLimitThetaYUp.sin;
							float upperEyesYDownLimit = _internalValues.bodyIK.upperEyesLimitThetaYDown.sin;

							SAFBIKMatMultVecInv( out eyeDir, ref toBasis, ref eyeDir ); // to Local

							if( eyeDir.y >= 0.0f ) {
								eyeDir.y *= _settings.bodyIK.upperEyesRateYUp;
							} else {
								eyeDir.y *= _settings.bodyIK.upperEyesRateYDown;
							}

							SAFBIKVecNormalize( ref eyeDir );

							if( eyeDir.z < 0.0f ) {
								float offset = Mathf.Clamp( _settings.bodyIK.upperEyesBackOffsetZ, 0.0f, 0.99f );
								if( offset > IKEpsilon ) {
									if( eyeDir.z > -offset ) {
										eyeDir.z = 0.0f;
									} else {
										eyeDir.z = (eyeDir.z + offset) / (1.0f - offset);
									}
									SAFBIKVecNormalize( ref eyeDir );
								}
							}

							_LimitXY( ref eyeDir, upperEyesXLimit, upperEyesXLimit, upperEyesYDownLimit, upperEyesYUpLimit );

							SAFBIKMatMultVec( out eyeDir, ref toBasis, ref eyeDir ); // to Global

							{
								Vector3 xDir = toBasis.column0;
								Vector3 yDir = toBasis.column1;
								Vector3 zDir = eyeDir;

								if( SAFBIKComputeBasisLockZ( out toBasis, ref xDir, ref yDir, ref zDir ) ) {
									// Nothing.
								}
							}
						}

						SAFBIKMatMultRet0( ref toBasis, ref _centerLegToArmBaseToBoneBasis );

						float upperEyesRate1 = _settings.bodyIK.upperEyesToCenterLegRate;
						float upperEyesRate2 = _settings.bodyIK.upperEyesToSpineRate;

						Matrix3x3 solveBasis;
						if( upperEyesRate2 > IKEpsilon ) {
							SAFBIKMatFastLerp( out solveBasis, ref fromBasis, ref toBasis, upperEyesRate2 );
							centerArmDirX2 = solveBasis.column0;
							centerArmDirY2 = solveBasis.column1;
						}

						if( upperEyesRate1 > IKEpsilon ) {
							if( SAFBIKComputeBasisFromXYLockY( out fromBasis, ref centerArmDirX, ref centerArmDirY ) ) {
								SAFBIKMatFastLerp( out solveBasis, ref fromBasis, ref toBasis, upperEyesRate1 );
								centerArmDirX = solveBasis.column0;
								centerArmDirY = solveBasis.column1;
							}
						}
					}
				}

				float stableCenterLegRate = _settings.bodyIK.upperContinuousCenterLegRotationStableRate;

				if( neckWeight > IKEpsilon || eyesWeight > IKEpsilon ) {
					if( continuousSolverEnabled && stableCenterLegRate > IKEpsilon ) {
						float centerLegToArmLength = (_defaultCenterArmPos - _defaultCenterLegPos).magnitude;

						centerArmPos = presolveCenterLegPos + centerArmDirY * centerLegToArmLength;
                    }
				}

				// centerLeg(Hips)
				{
					Matrix3x3 toBasis;
					if( SAFBIKComputeBasisFromXYLockY( out toBasis, ref centerArmDirX, ref centerArmDirY ) ) {
						Matrix3x3 rotateBasis = Matrix3x3.identity;

						if( _internalValues.animatorEnabled || _internalValues.resetTransforms ) {
							// for animatorEnabled or resetTransform(Base on armPos)
							if( continuousSolverEnabled && stableCenterLegRate > IKEpsilon ) {
								Matrix3x3 presolveCenterLegBasis = Matrix3x3.identity;
								Vector3 solveDirY = centerArmPos - presolveCenterLegPos;
								Vector3 solveDirX = centerArmDirX;
								if( SAFBIKVecNormalize( ref solveDirY ) && SAFBIKComputeBasisFromXYLockY( out presolveCenterLegBasis, ref solveDirX, ref solveDirY ) ) {
									Matrix3x3 tempBasis;
									SAFBIKMatFastLerp( out tempBasis, ref toBasis, ref presolveCenterLegBasis, stableCenterLegRate );
									toBasis = tempBasis;
								}
							}

							Matrix3x3 fromBasis;
							Vector3 currentDirX = temp.armPos[1] - temp.armPos[0];
							Vector3 currentDirY = (temp.armPos[1] + temp.armPos[0]) * 0.5f - temp.centerLegPos;
							if( SAFBIKVecNormalize( ref currentDirY ) && SAFBIKComputeBasisFromXYLockY( out fromBasis, ref currentDirX, ref currentDirY ) ) {
								SAFBIKMatMultInv1( out rotateBasis, ref toBasis, ref fromBasis );
							}
						} else { // for continuousSolverEnabled.(Base on centerLegBasis)
							SAFBIKMatMultRet0( ref toBasis, ref _centerLegToArmBasisInv );

							if( continuousSolverEnabled && stableCenterLegRate > IKEpsilon ) {
								Matrix3x3 presolveCenterLegBasis = Matrix3x3.identity;
								Vector3 solveDirY = centerArmPos - presolveCenterLegPos;
								Vector3 solveDirX = centerArmDirX;
								if( SAFBIKVecNormalize( ref solveDirY ) && SAFBIKComputeBasisFromXYLockY( out presolveCenterLegBasis, ref solveDirX, ref solveDirY ) ) {
									SAFBIKMatMultRet0( ref presolveCenterLegBasis, ref _centerLegToArmBasisInv );
									Matrix3x3 tempBasis;
									SAFBIKMatFastLerp( out tempBasis, ref toBasis, ref presolveCenterLegBasis, stableCenterLegRate );
									toBasis = tempBasis;
								}
							}

							Matrix3x3 centerLegBasis = temp.centerLegBasis;
							SAFBIKMatMultInv1( out rotateBasis, ref toBasis, ref centerLegBasis );
						}

						if( _settings.bodyIK.upperCenterLegLerpRate < 1.0f - IKEpsilon ) {
							SAFBIKMatFastLerpToIdentity( ref rotateBasis, 1.0f - _settings.bodyIK.upperCenterLegLerpRate );
						}

						temp.UpperRotation( -1, ref rotateBasis );
					}
				}

				{
					// Compute spineRate.
					float spineRate = 1.0f;
					for( int i = 0; i < 2; ++i ) {
						if( temp.arms.endPosEnabled[i] ) {
							Vector3 centerLegToArm = temp.arms.beginPos[i] - temp.centerLegPos;
							Vector3 beginToEnd = temp.arms.endPos[i] - temp.arms.beginPos[i];
							if( SAFBIKVecNormalize2( ref centerLegToArm, ref beginToEnd ) ) {
								spineRate *= Mathf.Abs( Vector3.Dot( centerLegToArm, beginToEnd ) );
							}
						}
					}

					spineRate = 1.0f - spineRate;
					if( spineRate > IKEpsilon ) { // Limit centerArmDirX2 / centerArmDirY2 from spineRate
						float spineLimitAngleX = _internalValues.bodyIK.spineLimitAngleX.value;
						float spineLimitAngleY = _internalValues.bodyIK.spineLimitAngleY.value;

						// Recompute centerLegToArmBoneBasisTo2( for Spine )
						{
							float fromToX = Vector3.Dot( centerArmDirX, centerArmDirX2 );
							float fromToXAng = SAFBIKAcos( fromToX );
							fromToXAng *= spineRate;
							if( fromToXAng > spineLimitAngleX ) {
								if( fromToXAng > IKEpsilon ) {
									float balancedRate = spineLimitAngleX / fromToXAng;
									Vector3 dirX2Balanced = Vector3.Lerp( centerArmDirX, centerArmDirX2, balancedRate );
									if( SAFBIKVecNormalize( ref dirX2Balanced ) ) {
										centerArmDirX2 = dirX2Balanced;
									}
								}
							}
						}

						// Pending: spine stiffness.(Sin scale to balanced rate.)
						{
							float fromToY = Vector3.Dot( centerArmDirY, centerArmDirY2 );
							float fromToYAng = SAFBIKAcos( fromToY );
							fromToYAng *= spineRate;
							if( fromToYAng > spineLimitAngleY ) {
								if( fromToYAng > IKEpsilon ) {
									float balancedRate = spineLimitAngleY / fromToYAng;
									Vector3 dirY2Balanced = Vector3.Lerp( centerArmDirY, centerArmDirY2, balancedRate );
									if( SAFBIKVecNormalize( ref dirY2Balanced ) ) {
										centerArmDirY2 = dirY2Balanced;
									}
								}
							}
						}
					}
				}

				if( _settings.bodyIK.upperSolveEnabled ) {

					float centerLegToArmLength = (_defaultCenterArmPos - _defaultCenterLegPos).magnitude;

					int solveLength = Mathf.Min( spineLength, 2 );
					if( !_settings.bodyIK.upperSolveSpine2Enabled ) {
						solveLength = Mathf.Min( spineLength, 1 );
					}

					Matrix3x3 centerLegBasis = temp.centerLegBasis;

					Vector3 centerArmPosY2 = centerArmDirY2 * centerLegToArmLength + temp.centerLegPos;

					float upperSpineLerpRate = _settings.bodyIK.upperSpineLerpRate;

					if( _internalValues.animatorEnabled || _internalValues.resetTransforms ) {
						for( int i = 0; i < solveLength; ++i ) {
                            Vector3 origPos = temp.spinePos[i];

							Vector3 currentDirX = temp.armPos[1] - temp.armPos[0];
							Vector3 currentDirY = (temp.armPos[1] + temp.armPos[0]) * 0.5f - origPos;

							Vector3 targetDirX = centerArmDirX2;
							Vector3 targetDirY = centerArmPosY2 - origPos;

							if( !SAFBIKVecNormalize2( ref currentDirY, ref targetDirY ) ) {
								continue; // Skip.
							}

							Vector3 dirX = targetDirX;
							Vector3 dirY = targetDirY;

							if( i == 0 ) { // Spine
								if( !SAFBIKVecNormalize( ref currentDirX ) ) {
									continue; // Skip.
								}

								dirX = Vector3.Lerp( currentDirX, targetDirX, _settings.bodyIK.spineDirXLegToArmRate );

								dirY = Vector3.Lerp( currentDirY, targetDirY, 0.7f ); // Test Rate!!!
								if( !SAFBIKVecNormalize( ref dirY ) ) { // Failsafe.
									dirY = currentDirY;
								}
							}

							Matrix3x3 toBasis;
							SAFBIKComputeBasisFromXYLockY( out toBasis, ref dirX, ref dirY );
							Matrix3x3 fromBasis;
							SAFBIKComputeBasisFromXYLockY( out fromBasis, ref currentDirX, ref currentDirY );

							Matrix3x3 rotateBasis;
							SAFBIKMatMultInv1( out rotateBasis, ref toBasis, ref fromBasis );

							if( upperSpineLerpRate < 1.0f - IKEpsilon ) {
								SAFBIKMatFastLerpToIdentity( ref rotateBasis, 1.0f - upperSpineLerpRate );
							}

							temp.UpperRotation( i, ref rotateBasis );
                        }
                    } else {
						for( int i = 0; i < solveLength; ++i ) {
							Vector3 origPos = temp.spinePos[i];
							Vector3 dirX = centerArmDirX2;
							Vector3 dirY = centerArmPosY2 - origPos;

							// todo: Preproceess baseY
							// todo: Property Lerp Rate

							if( i == 0 ) { // Spine
								dirX = Vector3.Lerp( centerLegBasis.column0, centerArmDirX2, _settings.bodyIK.spineDirXLegToArmRate );
								if( !SAFBIKVecNormalize( ref dirX ) ) { // Failsafe.
									dirX = centerLegBasis.column0;
								}

								// todo: Prepare _spineDefaultDirY[]
								Vector3 defaultPos = _spineBones[i]._defaultPosition;
								Vector3 defaultChildPod = (i + 1 == spineLength) ? _neckBone._defaultPosition : _spineBones[i + 1]._defaultPosition;
								Vector3 baseY;
								SAFBIKMatMultVecPreSub( out baseY, ref centerLegBasis, ref defaultChildPod, ref defaultPos );
								SAFBIKVecNormalize( ref baseY );

								SAFBIKVecNormalize( ref dirY );
								baseY = Vector3.Lerp( baseY, dirY, 0.5f ); // todo: Test Rate!!!
								SAFBIKVecNormalize( ref baseY );
								dirY = baseY;
							} else { // Spine 2
								dirX = Vector3.Lerp( centerLegBasis.column0, centerArmDirX2, 0.9f ); // todo: Test Rate!!!
								if( !SAFBIKVecNormalize( ref dirX ) ) { // Failsafe.
									dirX = centerLegBasis.column0;
								}
								if( !SAFBIKVecNormalize( ref dirY ) ) { // Failsafe.
									// todo: _spineDefaultDirY[]
									Vector3 childPos = (i + 1 == spineLength) ? temp.neckPos : temp.spinePos[i + 1];
									dirY = childPos - origPos;
									if( !SAFBIKVecNormalize( ref dirY ) ) {
										continue;
									}
								}
							}
						
							Matrix3x3 toBasis;
							if( SAFBIKComputeBasisFromXYLockY( out toBasis, ref dirX, ref dirY ) ) {
								Vector3 childPos = (i + 1 == spineLength) ? temp.neckPos : temp.spinePos[i + 1];

								Vector3 fromY = childPos - origPos;
								Vector3 fromX = temp.nearArmPos[1] - temp.nearArmPos[0];
								if( i == 0 && spineLength >= 2 ) {
									if( SAFBIKVecNormalize( ref fromX ) ) {
										fromX = Vector3.Lerp( centerLegBasis.column0, fromX, _settings.bodyIK.spineDirXLegToArmRate );
									} else { // Failsafe.
										fromX = centerLegBasis.column0;
									}
								}

								Vector3 fromZ = Vector3.Cross( fromX, fromY );
								fromX = Vector3.Cross( fromY, fromZ );
								if( SAFBIKVecNormalize3( ref fromX, ref fromY, ref fromZ ) ) {
									Matrix3x3 fromBasis = Matrix3x3.FromColumn( ref fromX, ref fromY, ref fromZ );
									Matrix3x3 rotateBasis;
									SAFBIKMatMultInv1( out rotateBasis, ref toBasis, ref fromBasis );

									if( upperSpineLerpRate < 1.0f - IKEpsilon ) {
										SAFBIKMatFastLerpToIdentity( ref rotateBasis, 1.0f - upperSpineLerpRate );
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

				_UpperSolve_Translate(
					ref _internalValues.bodyIK.upperPostTranslateRate,
					ref _internalValues.bodyIK.upperContinuousPostTranslateStableRate,
					ref baseCenterLegPos );

				return true;
			}

			void _UpperSolve_PresolveBaseCenterLegTransform( out Vector3 centerLegPos, out Matrix3x3 centerLegBasis )
			{
				Assert( _internalValues != null && _internalValues.continuousSolverEnabled );
				var temp = _solverInternal;
				Assert( temp != null );

				_GetBaseCenterLegTransform( out centerLegPos, out centerLegBasis );

				// Presolve LowerSolver. (for continuousSolverEnabled only)

				if( _footEffectors == null || _legEffectorMaxLength == null ) {
					return;
				}

				if( !_footEffectors[0].bone.transformIsAlive ||
					!_footEffectors[1].bone.transformIsAlive ) {
					return;
				}

				Vector3 footPos0 = _footEffectors[0].positionEnabled ? _footEffectors[0]._hidden_worldPosition : _footEffectors[0].bone.worldPosition;
				Vector3 footPos1 = _footEffectors[1].positionEnabled ? _footEffectors[1]._hidden_worldPosition : _footEffectors[1].bone.worldPosition;

				Vector3 baseLegPos0, baseLegPos1;
				SAFBIKMatMultVecPreSubAdd( out baseLegPos0, ref centerLegBasis, ref _legBones[0]._defaultPosition, ref _defaultCenterLegPos, ref centerLegPos );
				SAFBIKMatMultVecPreSubAdd( out baseLegPos1, ref centerLegBasis, ref _legBones[1]._defaultPosition, ref _defaultCenterLegPos, ref centerLegPos );

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
						Vector3 vecX = centerLegBasis.column0 * _defaultCenterLegHalfLen;
						centerLegPos = Vector3.Lerp( baseLegPos0 + vecX, baseLegPos1 - vecX, temp.legs.lerpRate );
					} else {
						centerLegPos = (baseLegPos0 + baseLegPos1) * 0.5f;
					}
				}
			}

			void _UpperSolve_Transform( int origIndex, ref Matrix3x3 transformBasis )
			{
				var temp = _solverInternal;

				Vector3 origPos = (origIndex == -1) ? temp.centerLegPos : temp.spinePos[origIndex];

				for( int i = 0; i < 2; ++i ) {
					SAFBIKMatMultVecPreSubAdd( out temp.armPos[i], ref transformBasis, ref temp.armPos[i], ref origPos, ref origPos );
					if( _shoulderBones != null ) {
						SAFBIKMatMultVecPreSubAdd( out temp.shoulderPos[i] , ref transformBasis , ref temp.shoulderPos[i] , ref origPos, ref origPos );
					}
				}

				int spineLength = (_spineBones != null) ? (_spineBones.Length) : 0;
				for( int spineIndex = origIndex + 1; spineIndex < spineLength; ++spineIndex ) {
					SAFBIKMatMultVecPreSubAdd( out temp.spinePos[spineIndex], ref transformBasis, ref temp.spinePos[spineIndex], ref origPos, ref origPos );
				}

				if( _neckBone != null ) {
					SAFBIKMatMultVecPreSubAdd( out temp.neckPos, ref transformBasis, ref temp.neckPos, ref origPos, ref origPos );
				}

				if( origIndex == -1 ) {
					if( temp.legPos != null ) {
						for( int i = 0; i < 2; ++i ) {
							SAFBIKMatMultVecPreSubAdd( out temp.legPos[i], ref transformBasis, ref temp.legPos[i], ref origPos, ref origPos );
						}
					}
				}

				temp.SetDirtyVariables();
			}

			Vector3[] _temp_targetTranslate = new Vector3[2]; // Arms(2)

			bool _UpperSolve_PreTranslate( out Vector3 translate, ref CachedRate01 translateRate, ref CachedRate01 stableRate, ref Vector3 stableCenterLegPos )
			{
				// If resetTransform = false, contain targetBeginPos to default transform or modify _UpperSolve_Translate()

				// Memo: Prepare SolveTargetBeginPosRated().

				translate = Vector3.zero;

				var temp = _solverInternal;
				Assert( temp != null );

				bool continuousSolverEnabled = _internalValues.continuousSolverEnabled;

				bool translateEnabled = (continuousSolverEnabled && stableRate.isGreater0);
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
					if( translateRate.isLess1 ) {
						translate *= translateRate.value;
					}

					if( continuousSolverEnabled && stableRate.isGreater0 ) {
						Vector3 extraTranslate = stableCenterLegPos - temp.centerLegPos;
						translate = Vector3.Lerp( translate, extraTranslate, stableRate.value );
					}

					return true;
				}

				return false;
			}

			void _UpperSolve_Translate( ref CachedRate01 translateRate, ref CachedRate01 stableRate, ref Vector3 stableCenterLegPos )
			{
				Vector3 translate;
				if( _UpperSolve_PreTranslate( out translate, ref translateRate, ref stableRate, ref stableCenterLegPos ) ) {
					var temp = _solverInternal;
					Assert( temp != null );
					temp.Translate( ref translate );
				}
			}
			
			void _LowerSolve( bool isBeginOnly, bool firstPass )
			{
				var temp = _solverInternal;
				if( temp == null || temp.spinePos == null || temp.spinePos.Length == 0 ) {
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
							if( SAFBIKVecNormalize( ref legDir ) ) {
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
							temp.LowerRotation( 0, ref origLowerRotation, false );
						}
					}
				}

				if( temp.PrepareLowerTranslate() ) {
					Vector3 origLowerTranslate;
					if( temp.SolveLowerTranslate( out origLowerTranslate ) ) {
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
				if( temp == null || temp.spinePos == null || temp.spinePos.Length == 0 ) {
					return;
				}

				// Compute worldPosition / worldRotation.
				if( _hipsBone != null && _hipsBone.transformIsAlive && temp.spinePos != null && temp.spinePos.Length > 0 && _neckBone != null && _neckBone.transformIsAlive ) {
					Vector3 hipsToSpineDirX = new Vector3( 1.0f, 0.0f, 0.0f );

					Vector3 dirX = temp.legs.beginPos[1] - temp.legs.beginPos[0];
					Vector3 dirY = temp.spinePos[0] - (temp.legs.beginPos[1] + temp.legs.beginPos[0]) * 0.5f;

					Matrix3x3 boneBasis = new Matrix3x3();

					if( SAFBIKVecNormalize( ref dirY ) && SAFBIKComputeBasisFromXYLockY( out boneBasis, ref dirX, ref dirY ) ) {
						Matrix3x3 tempBasis;
						SAFBIKMatMult( out tempBasis, ref boneBasis, ref _centerLegBoneBasisInv );

						hipsToSpineDirX = boneBasis.column0; // Counts as baseBasis.

						Quaternion worldRotation;
						SAFBIKMatMultGetRot( out worldRotation, ref tempBasis, ref _hipsBone._defaultBasis );
                        _hipsBone.worldRotation = worldRotation;

						if( _hipsBone.isWritebackWorldPosition ) {
							Vector3 worldPosition;
							SAFBIKMatMultVecPreSubAdd( out worldPosition, ref tempBasis, ref _hipsBone._defaultPosition, ref _spineBone._defaultPosition, ref temp.spinePos[0] );
							_hipsBone.worldPosition = worldPosition;
						}
					} else { // Failsafe.
						if( SAFBIKVecNormalize( ref dirX ) ) {
							hipsToSpineDirX = dirX;
                        }
					}

					for( int i = 0; i < temp.spinePos.Length; ++i ) {
						if( i + 1 == temp.spinePos.Length ) {
							dirY = temp.neckPos - temp.spinePos[i];
							if( temp.nearArmPos != null ) {
								dirX = temp.nearArmPos[1] - temp.nearArmPos[0];
							} else { // Failsafe.
								dirX = hipsToSpineDirX;
                            }
						} else {
							dirY = temp.spinePos[i + 1] - temp.spinePos[i];
							dirX = hipsToSpineDirX;
							if( temp.nearArmPos != null ) {
								Vector3 dirX0 = temp.nearArmPos[1] - temp.nearArmPos[0];
								if( SAFBIKVecNormalize( ref dirX0 ) ) {
									dirX = Vector3.Lerp( dirX, dirX0, _settings.bodyIK.spineDirXLegToArmRate );
								}
							}
						}

						if( SAFBIKVecNormalize( ref dirY ) && SAFBIKComputeBasisFromXYLockY( out boneBasis, ref dirX, ref dirY ) ) {
							hipsToSpineDirX = boneBasis.column0;
							Quaternion worldRotation;
							SAFBIKMatMultGetRot( out worldRotation, ref boneBasis, ref _spineBones[i]._boneToWorldBasis );
							_spineBones[i].worldRotation = worldRotation;
							if( _spineBones[i].isWritebackWorldPosition ) {
								_spineBones[i].worldPosition = temp.spinePos[i];
							}
						}
					}

					if( _shoulderBones != null ) {
						for( int i = 0; i < 2; ++i ) {
							Vector3 xDir = temp.armPos[i] - temp.shoulderPos[i];
							Vector3 yDir = temp.shoulderPos[i] - temp.spineUPos;
							xDir = (i == 0) ? -xDir : xDir;
							Vector3 zDir = Vector3.Cross( xDir, yDir );
							yDir = Vector3.Cross( zDir, xDir );
							if( SAFBIKVecNormalize3( ref xDir, ref yDir, ref zDir ) ) {
								boneBasis.SetColumn( ref xDir, ref yDir, ref zDir );
								Quaternion worldRotation;
								SAFBIKMatMultGetRot( out worldRotation, ref boneBasis, ref _shoulderBones[i]._boneToWorldBasis );
								_shoulderBones[i].worldRotation = worldRotation;
							}
						}
					}
				}
			}

			bool _IsEffectorEnabled()
			{
				if( _hipsEffector.effectorEnabled ||
					_neckEffector.effectorEnabled ||
					_eyesEffector.effectorEnabled ||
                    _armEffectors[0].effectorEnabled ||
					_armEffectors[1].effectorEnabled ) {
					return true;
				}

				if( _wristEffectors[0].positionEnabled ||
					_wristEffectors[1].positionEnabled ||
					_footEffectors[0].positionEnabled ||
					_footEffectors[1].positionEnabled ) {
					return true;
				}

				return false;
			}

			bool _PrepareSolverInternal()
			{
				if( _armBones == null || _legBones == null ) {
					_solverInternal = null;
					return false;
				}

				if( _solverInternal == null ) {
					_solverInternal = new SolverInternal();
					_solverInternal._centerLegBoneBasisInv = this._centerLegBoneBasisInv;
					if( _spineUBone != null ) {
						if( _shoulderBones != null || _armBones != null ) {
							var nearArmBones = ( _shoulderBones != null ) ? _shoulderBones : _armBones;
							Vector3 dirY = nearArmBones[1]._defaultPosition + nearArmBones[0]._defaultPosition;
							Vector3 dirX = nearArmBones[1]._defaultPosition - nearArmBones[0]._defaultPosition;
							dirY = dirY * 0.5f - _spineUBone._defaultPosition;
							Vector3 dirZ = Vector3.Cross( dirX, dirY );
							dirX = Vector3.Cross( dirY, dirZ );
							if( SAFBIKVecNormalize3( ref dirX, ref dirY, ref dirZ ) ) {
								Matrix3x3 localBasis = Matrix3x3.FromColumn( ref dirX, ref dirY, ref dirZ );
								_solverInternal._spineUBoneLocalAxisBasisInv = localBasis.transpose;
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
				PrepareArray( ref _solverInternal.shoulderPos, _shoulderBones );

				_solverInternal.nearArmPos = (_shoulderBones != null) ? _solverInternal.shoulderPos : _solverInternal.armPos;

				PrepareArray( ref _solverInternal.spinePos, _spineBones );
				return true;
			}

			void _UpperSolve_ShoulderToArm()
			{
				for( int i = 0; i < 2; ++i ) {
					_solverInternal.SolveShoulderToArm(
						i,
						_shoulderBones,
						_shoulderToArmLength,
						_internalValues.bodyIK.shoulderLimitThetaYPlus.sin,
						_internalValues.bodyIK.shoulderLimitThetaYMinus.sin,
						_internalValues.bodyIK.shoulderLimitThetaZ.sin );
				}
			}

			void _UpperSolve_ShoulderToArm( int i )
			{
				_solverInternal.SolveShoulderToArm(
					i,
					_shoulderBones,
					_shoulderToArmLength,
					_internalValues.bodyIK.shoulderLimitThetaYPlus.sin,
					_internalValues.bodyIK.shoulderLimitThetaYMinus.sin,
					_internalValues.bodyIK.shoulderLimitThetaZ.sin );
			}

			void _PresolveHips()
			{
				Assert( _internalValues != null && _internalValues.animatorEnabled );

				var temp = _solverInternal;
				Assert( temp != null );

				if( _hipsEffector == null ) {
					return;
				}

				bool rotationEnabled = _hipsEffector.rotationEnabled && _hipsEffector.rotationWeight > IKEpsilon;
				bool positionEnabled = _hipsEffector.positionEnabled && _hipsEffector.positionWeight > IKEpsilon;

				if( !rotationEnabled && !positionEnabled ) {
					return;
				}

				Matrix3x3 centerLegBasis = temp.centerLegBasis;

				if( rotationEnabled ) {
					Quaternion centerLegRotationTo = _hipsEffector.worldRotation * Inverse( _hipsEffector._defaultRotation );
					Quaternion centerLegRotationFrom;
					SAFBIKMatGetRot( out centerLegRotationFrom, ref centerLegBasis );

					Quaternion centerLegRotation = centerLegRotationTo * Inverse( centerLegRotationFrom );

					if( _hipsEffector.rotationWeight < 1.0f - IKEpsilon ) {
						centerLegRotation = Quaternion.Lerp( Quaternion.identity, centerLegRotation, _hipsEffector.rotationWeight );
					}

					temp.LowerRotation( -1, ref centerLegRotation, true );
					centerLegBasis = temp.centerLegBasis;
				}

				if( positionEnabled ) {
                    Vector3 hipsEffectorWorldPosition = _hipsEffector.worldPosition;
                    Vector3 centerLegPos;
					SAFBIKMatMultVecPreSubAdd( out centerLegPos, ref centerLegBasis, ref _defaultCenterLegPos, ref _hipsEffector._defaultPosition, ref hipsEffectorWorldPosition );

					Vector3 translate = centerLegPos - temp.centerLegPos;
					if( _hipsEffector.positionWeight < 1.0f - IKEpsilon ) {
						translate *= _hipsEffector.positionWeight;
                    }

					temp.Translate( ref translate );
				}
			}

			void _ResetTransforms()
			{
				Assert( _internalValues != null && _internalValues.resetTransforms );
				Matrix3x3 centerLegBasis = Matrix3x3.identity;
				Vector3 centerLegPos = Vector3.zero;
				_GetBaseCenterLegTransform( out centerLegPos, out centerLegBasis );
				_ResetCenterLegTransform( ref centerLegPos, ref centerLegBasis );
			}

			void _GetBaseCenterLegTransform( out Vector3 centerLegPos, out Matrix3x3 centerLegBasis )
			{
				// Use from resetTransforms & continuousSolverEnabled.
				Assert( _internalValues != null );

				if( _hipsEffector != null && _hipsEffector.rotationEnabled && _hipsEffector.rotationWeight > IKEpsilon ) {
					Quaternion centerLegRotation = _hipsEffector.worldRotation * Inverse( _hipsEffector._defaultRotation );
					if( _hipsEffector.rotationWeight < 1.0f - IKEpsilon && _rootEffector != null && _rootEffector.transformIsAlive ) {
						Quaternion rootRotation = _rootEffector.worldRotation * Inverse( _rootEffector._defaultRotation );
						Quaternion tempRotation = Quaternion.Lerp( rootRotation, centerLegRotation, _hipsEffector.rotationWeight );
						SAFBIKMatSetRot( out centerLegBasis, ref tempRotation );
					} else {
						SAFBIKMatSetRot( out centerLegBasis, ref centerLegRotation );
                    }
				} else if( _rootEffector != null && _rootEffector.transformIsAlive ) {
					Quaternion rootEffectorWorldRotation = _rootEffector.worldRotation;
					SAFBIKMatSetRotMultInv1( out centerLegBasis, ref rootEffectorWorldRotation, ref _rootEffector._defaultRotation );
				} else {
					centerLegBasis = Matrix3x3.identity;
				}

				if( _hipsEffector != null && _hipsEffector.positionEnabled && _hipsEffector.positionWeight > IKEpsilon ) {
                    Vector3 hipsEffectorWorldPosition = _hipsEffector.worldPosition;
					SAFBIKMatMultVecPreSubAdd( out centerLegPos, ref centerLegBasis, ref _defaultCenterLegPos, ref _hipsEffector._defaultPosition, ref hipsEffectorWorldPosition );
					if( _hipsEffector.positionWeight < 1.0f - IKEpsilon && _rootEffector != null && _rootEffector.transformIsAlive ) {
						Vector3 rootEffectorWorldPosition = _rootEffector.worldPosition;
						Vector3 rootPosition;
						SAFBIKMatMultVecPreSubAdd( out rootPosition, ref centerLegBasis, ref _defaultCenterLegPos, ref _rootEffector._defaultPosition, ref rootEffectorWorldPosition );
						centerLegPos = Vector3.Lerp( rootPosition, centerLegPos, _hipsEffector.positionWeight );
					}
				} else if( _rootEffector != null && _rootEffector.transformIsAlive ) {
					Vector3 rootEffectorWorldPosition = _rootEffector.worldPosition;
					SAFBIKMatMultVecPreSubAdd( out centerLegPos, ref centerLegBasis, ref _defaultCenterLegPos, ref _rootEffector._defaultPosition, ref rootEffectorWorldPosition );
				} else {
					centerLegPos = Vector3.zero;
				}
			}

			void _ResetCenterLegTransform( ref Vector3 centerLegPos, ref Matrix3x3 centerLegBasis )
			{
				var temp = _solverInternal;
				Assert( temp != null );

				Vector3 defaultCenterLegPos = _defaultCenterLegPos;

				if( _legBones != null ) {
					for( int i = 0; i < 2; ++i ) {
						SAFBIKMatMultVecPreSubAdd( out temp.legPos[i], ref centerLegBasis, ref _legBones[i]._defaultPosition, ref defaultCenterLegPos, ref centerLegPos );
                    }
				}
				if( _spineBones != null ) {
					for( int i = 0; i < _spineBones.Length; ++i ) {
						SAFBIKMatMultVecPreSubAdd( out temp.spinePos[i], ref centerLegBasis, ref _spineBones[i]._defaultPosition, ref defaultCenterLegPos, ref centerLegPos );
					}
				}
				if( _shoulderBones != null ) {
					for( int i = 0; i < 2; ++i ) {
						SAFBIKMatMultVecPreSubAdd( out temp.shoulderPos[i], ref centerLegBasis, ref _shoulderBones[i]._defaultPosition, ref defaultCenterLegPos, ref centerLegPos );
					}
				}
				if( _armBones != null ) {
					for( int i = 0; i < 2; ++i ) {
						SAFBIKMatMultVecPreSubAdd( out temp.armPos[i], ref centerLegBasis, ref _armBones[i]._defaultPosition, ref defaultCenterLegPos, ref centerLegPos );
					}
				}
				if( _neckBone != null ) {
					SAFBIKMatMultVecPreSubAdd( out temp.neckPos, ref centerLegBasis, ref _neckBone._defaultPosition, ref defaultCenterLegPos, ref centerLegPos );
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

				public Vector3[] spinePos;

				public Vector3 neckPos;

				public Vector3[] nearArmPos;
				public Vector3[] shoulderPos;

				public Vector3[] armPos { get { return this.arms.beginPos; } }
				public Vector3[] legPos { get { return this.legs.beginPos; } }

				public Matrix3x3 _centerLegBoneBasisInv = Matrix3x3.identity; // Require setting on initialize.
				public Matrix3x3 _spineUBoneLocalAxisBasisInv = Matrix3x3.identity; // Require setting on initialize.

				public Vector3 _centerArmPos = Vector3.zero;
				public Vector3 _centerLegPos = Vector3.zero;
				public Matrix3x3 _centerLegBasis = Matrix3x3.identity;
				public Matrix3x3 _spineUBasis = Matrix3x3.identity;

				bool _isDirtyCenterArmPos = true;
				bool _isDirtyCenterLegPos = true;
				bool _isDirtyCenterLegBasis = true;
				bool _isDirtySpineUBasis = true;

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

				public Matrix3x3 spineUBasis
				{
					get
					{
						if( _isDirtySpineUBasis ) {
							_UpdateSpineUBasis();
						}

						return _spineUBasis;
					}
				}

				public void _UpdateCenterLegBasis()
				{
					if( _isDirtyCenterLegBasis ) {
						_isDirtyCenterLegBasis = false;
						var legPos = this.legPos;
						_centerLegBasis = Matrix3x3.identity;
						if( this.spinePos != null && this.spinePos.Length > 0 && legPos != null ) {
							Vector3 dirX = legPos[1] - legPos[0];
							Vector3 dirY = this.spinePos[0] - this.centerLegPos;
							Vector3 dirZ = Vector3.Cross( dirX, dirY );
							dirX = Vector3.Cross( dirY, dirZ );
							if( SAFBIKVecNormalize3( ref dirX, ref dirY, ref dirZ ) ) {
								_centerLegBasis.SetColumn( ref dirX, ref dirY, ref dirZ );
								SAFBIKMatMultRet0( ref _centerLegBasis, ref _centerLegBoneBasisInv );
							}
						}
					}
				}

				public void _UpdateSpineUBasis()
				{
					if( _isDirtySpineUBasis ) {
						_isDirtySpineUBasis = false;
						_spineUBasis = Matrix3x3.identity;
						Vector3 dirY = (this.shoulderPos != null) ? (this.shoulderPos[1] + this.shoulderPos[0]) : (this.armPos[1] + this.armPos[0]);
						dirY = dirY * 0.5f - this.spineUPos;
						Vector3 dirX = (this.shoulderPos != null) ? (this.shoulderPos[1] - this.shoulderPos[0]) : (this.armPos[1] - this.armPos[0]);
						Vector3 dirZ = Vector3.Cross( dirX, dirY );
						dirX = Vector3.Cross( dirY, dirZ );
						if( SAFBIKVecNormalize3( ref dirX, ref dirY, ref dirZ ) ) {
							_spineUBasis.SetColumn( ref dirX, ref dirY, ref dirZ );
							SAFBIKMatMultRet0( ref _spineUBasis, ref _spineUBoneLocalAxisBasisInv );
                        }
					}
				}

				public void SetDirtyVariables()
				{
					_isDirtyCenterArmPos = true;
					_isDirtyCenterLegPos = true;
					_isDirtyCenterLegBasis = true;
					_isDirtySpineUBasis = true;
				}

				public Vector3 spineUPos
				{
					get
					{
						if( this.spinePos != null && this.spinePos.Length != 0 ) {
							return this.spinePos[this.spinePos.Length - 1];
						}

						return Vector3.zero;
					}
				}

				public class BackupData
				{
					public Vector3 centerArmPos;
					public Vector3 centerLegPos;
					public Matrix3x3 centerLegBasis;
					public Matrix3x3 spineUBasis;

					public Vector3[] spinePos;
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
					_backupData.spineUBasis = this.spineUBasis;
					CloneArray( ref _backupData.spinePos, this.spinePos );
					_backupData.neckPos = this.neckPos;
					CloneArray( ref _backupData.shoulderPos, this.shoulderPos );
					CloneArray( ref _backupData.armPos, this.arms.beginPos );
					CloneArray( ref _backupData.legPos, this.legs.beginPos );
				}

				public void Restore()
				{
					_isDirtyCenterArmPos = false;
					_isDirtyCenterLegPos = false;
					_isDirtyCenterLegBasis = false;
					_isDirtySpineUBasis = false;
					_centerArmPos = _backupData.centerArmPos;
					_centerLegPos = _backupData.centerLegPos;
					_centerLegBasis = _backupData.centerLegBasis;
					_spineUBasis = _backupData.spineUBasis;
					CloneArray( ref this.spinePos, _backupData.spinePos );
					this.neckPos = _backupData.neckPos;
					CloneArray( ref this.shoulderPos, _backupData.shoulderPos );
					CloneArray( ref this.arms.beginPos, _backupData.armPos );
					CloneArray( ref this.legs.beginPos, _backupData.legPos );
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

					Vector3 origPos = (origIndex == -1) ? this.centerLegPos : this.spinePos[origIndex];

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
						// Fix for rotate 180 degrees or more.( half rotation in GetRotation & double rotation in origRotation * origRotation. )
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
					Vector3 origPos = (origIndex == -1) ? this.centerLegPos : this.spinePos[origIndex];
					Matrix3x3 origBasis = new Matrix3x3( origRotation );

					var beginPos = limb.beginPos;
					if( beginPos != null ) {
						for( int i = 0; i < beginPos.Length; ++i ) {
							SAFBIKMatMultVecPreSubAdd( out beginPos[i], ref origBasis, ref beginPos[i], ref origPos, ref origPos );
						}
					}
				}

				public void UpperRotation( int origIndex, ref Matrix3x3 origBasis )
				{
					Vector3 origPos = (origIndex == -1) ? this.centerLegPos : this.spinePos[origIndex];

					{
						var armPos = this.armPos;
						if( armPos != null ) {
							for( int i = 0; i < armPos.Length; ++i ) {
								SAFBIKMatMultVecPreSubAdd( out armPos[i], ref origBasis, ref armPos[i], ref origPos, ref origPos );
							}
						}
					}

					if( shoulderPos != null ) {
						for( int i = 0; i < this.shoulderPos.Length; ++i ) {
							SAFBIKMatMultVecPreSubAdd( out shoulderPos[i], ref origBasis, ref shoulderPos[i], ref origPos, ref origPos );
						}
					}
					
					SAFBIKMatMultVecPreSubAdd( out neckPos, ref origBasis, ref neckPos, ref origPos, ref origPos );

					// Legs					
					if( origIndex == -1 ) { // Rotation origin is centerLeg
						var legPos = this.legPos;
						if( legPos != null ) {
							for( int i = 0; i < legPos.Length; ++i ) {
								SAFBIKMatMultVecPreSubAdd( out legPos[i], ref origBasis, ref legPos[i], ref origPos, ref origPos );
                            }
						}
						
						_isDirtyCenterLegBasis = true;
					}

					// Spine
					for( int t = (origIndex == -1) ? 0 : origIndex; t < this.spinePos.Length; ++t ) {
						SAFBIKMatMultVecPreSubAdd( out this.spinePos[t], ref origBasis, ref this.spinePos[t], ref origPos, ref origPos );
					}

					_isDirtyCenterArmPos = true;
					_isDirtySpineUBasis = true;
				}

				public void LowerRotation( int origIndex, ref Quaternion origRotation, bool bodyRotation )
				{
					Matrix3x3 origBasis = new Matrix3x3( origRotation );
					LowerRotation( origIndex, ref origBasis, bodyRotation );
				}

				public void LowerRotation( int origIndex, ref Matrix3x3 origBasis, bool bodyRotation )
				{
					Vector3 origPos = (origIndex == -1) ? this.centerLegPos : this.spinePos[origIndex];

					var legPos = this.legPos;
					if( legPos != null ) {
						for( int i = 0; i < 2; ++i ) {
							SAFBIKMatMultVecPreSubAdd( out legPos[i], ref origBasis, ref legPos[i], ref origPos, ref origPos );
                        }
					}

					if( this.spinePos != null ) {
						int length = bodyRotation ? this.spinePos.Length : origIndex;
						for( int n = 0; n < length; ++n ) {
							SAFBIKMatMultVecPreSubAdd( out spinePos[n], ref origBasis, ref spinePos[n], ref origPos, ref origPos );
						}
					}

					_isDirtyCenterArmPos = true;
					_isDirtyCenterLegPos = true;
					_isDirtyCenterLegBasis = true;

					if( bodyRotation || this.spinePos == null || origIndex + 1 == this.spinePos.Length ) {
						SAFBIKMatMultVecPreSubAdd( out neckPos, ref origBasis, ref neckPos, ref origPos, ref origPos );

						var armPos = this.armPos;
						if( armPos != null ) {
							for( int i = 0; i < 2; ++i ) {
								SAFBIKMatMultVecPreSubAdd( out armPos[i], ref origBasis, ref armPos[i], ref origPos, ref origPos );
							}
						}

						if( this.shoulderPos != null ) {
							for( int i = 0; i < 2; ++i ) {
								SAFBIKMatMultVecPreSubAdd( out shoulderPos[i], ref origBasis, ref shoulderPos[i], ref origPos, ref origPos );
							}
						}

						_isDirtySpineUBasis = true;
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

					if( this.spinePos != null ) {
						for( int i = 0; i < this.spinePos.Length; ++i ) {
							this.spinePos[i] += origTranslate;
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
								if( SAFBIKVecNormalize( ref dirX ) ) {
									Matrix3x3 worldBasis = this.spineUBasis;
									SAFBIKMatMultRet0( ref worldBasis, ref shoulderBones[i]._localAxisBasis );

									SAFBIKMatMultVecInv( out dirX, ref worldBasis, ref dirX );
									_LimitYZ( i != 0, ref dirX, limitYMinus, limitYPlus, limitZ, limitZ );
									SAFBIKMatMultVec( out dirX, ref worldBasis, ref dirX );

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
				ref Vector3 spinePos,
				ref Vector3 leftLegPos,
				ref Vector3 rightLegPos )
			{
				Vector3 dirX = rightLegPos - leftLegPos;
				Vector3 dirY = spinePos - (rightLegPos + leftLegPos) * 0.5f;
				if( SAFBIKVecNormalize( ref dirY ) ) {
					return SAFBIKComputeBasisFromXYLockY( out centerLegBasis, ref dirX, ref dirY );
				} else {
					centerLegBasis = Matrix3x3.identity;
					return false;
				}
			}

			//----------------------------------------------------------------------------------------------------------------------------------------

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

			static Quaternion _GetRotation( ref Vector3 axisDir, float theta, float rate )
			{
				if( (theta >= -IKEpsilon && theta <= IKEpsilon) || (rate >= -IKEpsilon && rate <= IKEpsilon) ) {
					return Quaternion.identity;
				} else {
					return Quaternion.AngleAxis( SAFBIKAcos( theta ) * rate * Mathf.Rad2Deg, axisDir );
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
