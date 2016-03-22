// Copyright (c) 2016 Nora
// Released under the "Unity-Chan" license
// http://unity-chan.com/contents/license_en/
// http://unity-chan.com/contents/license_jp/

using UnityEngine;

namespace SA
{
	[System.Serializable]
	public class FullBodyIKUnityChan : FullBodyIK
	{
		Vector3 _headBoneLossyScale = Vector3.one;
		bool _isHeadBoneLossyScaleFuzzyIdentity = true;

		public static Vector3 _unityChan_leftEyeDefaultLocalPosition = new Vector3( -0.042531f + 0.024f, 0.048524f, 0.047682f - 0.02f );
		public static Vector3 _unityChan_rightEyeDefaultLocalPosition = new Vector3( 0.042531f - 0.024f, 0.048524f, 0.047682f - 0.02f );
		Vector3 _unityChan_leftEyeDefaultPosition = Vector3.zero;
		Vector3 _unityChan_rightEyeDefaultPosition = Vector3.zero;

		Quaternion _headToLeftEyeRotation = Quaternion.identity;
		Quaternion _headToRightEyeRotation = Quaternion.identity;

		public override bool _IsHiddenCustomEyes()
		{
			return true;
		}

		public override bool _PrepareCustomEyes( ref Quaternion headToLeftEyeRotation, ref Quaternion headToRightEyeRotation )
		{
			Bone headBone = (headBones != null) ? headBones.head : null;
			Bone leftEyeBone = (headBones != null) ? headBones.leftEye : null;
			Bone rightEyeBone = (headBones != null) ? headBones.rightEye : null;

			if( headBone != null && headBone.transformIsAlive &&
				leftEyeBone != null && leftEyeBone.transformIsAlive &&
				rightEyeBone != null && rightEyeBone.transformIsAlive ) {
				_headToLeftEyeRotation = headToLeftEyeRotation;
				_headToRightEyeRotation = headToRightEyeRotation;

				Vector3 leftPos, rightPos;
				SAFBIKMatMultVec( out leftPos, ref internalValues.defaultRootBasis, ref _unityChan_leftEyeDefaultLocalPosition );
				SAFBIKMatMultVec( out rightPos, ref internalValues.defaultRootBasis, ref _unityChan_rightEyeDefaultLocalPosition );

				_headBoneLossyScale = headBone.transform.lossyScale;
				_isHeadBoneLossyScaleFuzzyIdentity = IsFuzzy( _headBoneLossyScale, Vector3.one );

				if( !_isHeadBoneLossyScaleFuzzyIdentity ) {
					leftPos = Scale( ref leftPos, ref _headBoneLossyScale );
					rightPos = Scale( ref rightPos, ref _headBoneLossyScale );
				}

				_unityChan_leftEyeDefaultPosition = headBone._defaultPosition + leftPos;
				_unityChan_rightEyeDefaultPosition = headBone._defaultPosition + rightPos;
			}

			return true;
		}

		public override void _ResetCustomEyes()
		{
			Bone neckBone = (headBones != null) ? headBones.neck : null;
			Bone headBone = (headBones != null) ? headBones.head : null;
			Bone leftEyeBone = (headBones != null) ? headBones.leftEye : null;
			Bone rightEyeBone = (headBones != null) ? headBones.rightEye : null;

			if( neckBone != null && neckBone.transformIsAlive &&
				headBone != null && headBone.transformIsAlive &&
				leftEyeBone != null && leftEyeBone.transformIsAlive &&
				rightEyeBone != null && rightEyeBone.transformIsAlive ) {

				Quaternion neckWorldRotation = neckBone.worldRotation;
				Vector3 headWorldPosition, neckWorldPosition = neckBone.worldPosition;
				Matrix3x3 neckBasis;
				SAFBIKMatSetRotMultInv1( out neckBasis, ref neckWorldRotation, ref neckBone._defaultRotation );
				SAFBIKMatMultVecPreSubAdd( out headWorldPosition, ref neckBasis, ref headBone._defaultPosition, ref neckBone._defaultPosition, ref neckWorldPosition );

				Quaternion headWorldRotation = headBone.worldRotation;
				Matrix3x3 headBasis;
				SAFBIKMatSetRotMultInv1( out headBasis, ref headWorldRotation, ref headBone._defaultRotation );

				Vector3 worldPotision;

				SAFBIKMatMultVecPreSubAdd( out worldPotision, ref headBasis, ref leftEyeBone._defaultPosition, ref headBone._defaultPosition, ref headWorldPosition );
				leftEyeBone.worldPosition = worldPotision;
				leftEyeBone.worldRotation = headWorldRotation * _headToLeftEyeRotation;

				SAFBIKMatMultVecPreSubAdd( out worldPotision, ref headBasis, ref rightEyeBone._defaultPosition, ref headBone._defaultPosition, ref headWorldPosition );
				rightEyeBone.worldPosition = worldPotision;
				rightEyeBone.worldRotation = headWorldRotation * _headToRightEyeRotation;
			}
		}

		public override void _SolveCustomEyes( ref Matrix3x3 neckBasis, ref Matrix3x3 headBasis, ref Matrix3x3 headBaseBasis )
		{
			Bone neckBone = (headBones != null) ? headBones.neck : null;
			Bone headBone = (headBones != null) ? headBones.head : null;
			Bone leftEyeBone = (headBones != null) ? headBones.leftEye : null;
			Bone rightEyeBone = (headBones != null) ? headBones.rightEye : null;
			Effector eyesEffector = (headEffectors != null) ? headEffectors.eyes : null;

			if( neckBone != null && neckBone.transformIsAlive && 
				headBone != null && headBone.transformIsAlive &&
				leftEyeBone != null && leftEyeBone.transformIsAlive &&
				rightEyeBone != null && rightEyeBone.transformIsAlive &&
				eyesEffector != null ) {

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

				internalValues.UpdateDebugValue( "_eyesHorzLimitAngle", ref _eyesHorzLimitAngle );
				internalValues.UpdateDebugValue( "_eyesVertLimitAngle", ref _eyesVertLimitAngle );
				internalValues.UpdateDebugValue( "_eyesXRate", ref _eyesXRate );
				internalValues.UpdateDebugValue( "_eyesYRate", ref _eyesYRate );
				internalValues.UpdateDebugValue( "_eyesOuterXRotRate", ref _eyesOuterXRotRate );
				internalValues.UpdateDebugValue( "_eyesInnerXRotRate", ref _eyesInnerXRotRate );
				internalValues.UpdateDebugValue( "_eyesXOffset", ref _eyesXOffset );
				internalValues.UpdateDebugValue( "_eyesYOffset", ref _eyesYOffset );
				internalValues.UpdateDebugValue( "_eyesZOffset", ref _eyesZOffset );

				float _innerMoveXRate = 0.063f;
				float _outerMoveXRate = 0.063f;

				internalValues.UpdateDebugValue( "_innerMoveXRate", ref _innerMoveXRate );
				internalValues.UpdateDebugValue( "_outerMoveXRate", ref _outerMoveXRate );

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

				Vector3 headWorldPosition, neckBoneWorldPosition = neckBone.worldPosition;
				SAFBIKMatMultVecPreSubAdd( out headWorldPosition, ref neckBasis, ref headBone._defaultPosition, ref neckBone._defaultPosition, ref neckBoneWorldPosition );
				Vector3 eyesPosition;
				SAFBIKMatMultVecPreSubAdd( out eyesPosition, ref headBasis, ref eyesEffector._defaultPosition, ref headBone._defaultPosition, ref headWorldPosition );

				Vector3 eyesDir = eyesEffector.worldPosition - eyesPosition;

				Matrix3x3 leftEyeBaseBasis = headBaseBasis;
				Matrix3x3 rightEyeBaseBasis = headBaseBasis;

				SAFBIKMatMultVecInv( out eyesDir, ref headBaseBasis, ref eyesDir );

				SAFBIKVecNormalize( ref eyesDir );

				if( eyesEffector.positionWeight < 1.0f - IKEpsilon ) {
					Vector3 tempDir = Vector3.Lerp( new Vector3( 0.0f, 0.0f, 1.0f ), eyesDir, eyesEffector.positionWeight );
					if( SAFBIKVecNormalize( ref tempDir ) ) {
						eyesDir = tempDir;
					}
				}

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
				SAFBIKMatMultVecPreSubAdd( out tempVec, ref headBasis, ref _unityChan_leftEyeDefaultPosition, ref headBone._defaultPosition, ref headWorldPosition );
				leftEyeWorldPosition += tempVec;
				SAFBIKMatMultVecPreSubAdd( out tempVec, ref headBasis, ref _unityChan_rightEyeDefaultPosition, ref headBone._defaultPosition, ref headWorldPosition );
				rightEyeWorldPosition += tempVec;

				Matrix3x3 leftEyeBasis, rightEyeBasis;
				SAFBIKMatMult( out leftEyeBasis, ref leftEyeBaseBasis, ref internalValues.defaultRootBasisInv );
				SAFBIKMatMult( out rightEyeBasis, ref rightEyeBaseBasis, ref internalValues.defaultRootBasisInv );

				Vector3 worldPosition;
				Quaternion worldRotation;

				SAFBIKMatMultVecPreSubAdd( out worldPosition, ref leftEyeBasis, ref leftEyeBone._defaultPosition, ref leftEyeDefaultPosition, ref leftEyeWorldPosition );
				leftEyeBone.worldPosition = worldPosition;
				SAFBIKMatMultGetRot( out worldRotation, ref leftEyeBaseBasis, ref leftEyeBone._baseToWorldBasis );
				leftEyeBone.worldRotation = worldRotation;

				SAFBIKMatMultVecPreSubAdd( out worldPosition, ref rightEyeBasis, ref rightEyeBone._defaultPosition, ref rightEyeDefaultPosition, ref rightEyeWorldPosition );
				rightEyeBone.worldPosition = worldPosition;
				SAFBIKMatMultGetRot( out worldRotation, ref rightEyeBaseBasis, ref rightEyeBone._baseToWorldBasis );
				rightEyeBone.worldRotation = worldRotation;
			}
		}
	}
}
