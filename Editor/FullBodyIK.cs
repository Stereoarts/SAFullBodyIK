// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using EditorUtil = SA.FullBodyIKEditorUtil;
using Util = SA.FullBodyIKUtil;

namespace SA
{
	[CustomEditor( typeof( SA.FullBodyIK ) )]
	public class FullBodyIKInspector : Editor
	{
		bool _initializedGUIStyle;
		GUIStyle _guiStyle_header;
		GUIStyle _guiStyle_boneName_Unselected;
		GUIStyle _guiStyle_boneName_Unselected_Optional;

		Vector2 _scrollViewPos_Bones;
		Vector2 _scrollViewPos_Effectors;

		const float boneNameFieldSize = 100.0f;
		const float effectorNameFieldSize = 80.0f;
		static readonly string[] toolbarContents = new string[] { "Basic", "Bone", "Effector" };

		void _Initialize()
		{
			if( !_initializedGUIStyle || _guiStyle_header == null ) {
				_guiStyle_header = new GUIStyle( EditorStyles.label );
				var styleState = new GUIStyleState();
				styleState.textColor = new Color( 0.7f, 0.7f, 0.0f );
				_guiStyle_header.normal = styleState;
				_guiStyle_header.wordWrap = false;
				_guiStyle_header.fontStyle = FontStyle.Bold;
			}

			if( !_initializedGUIStyle || _guiStyle_boneName_Unselected == null ) {
				_guiStyle_boneName_Unselected = new GUIStyle( EditorStyles.label );
				var styleState = new GUIStyleState();
				styleState.textColor = new Color( 1.0f, 0.4f, 0.4f );
				_guiStyle_boneName_Unselected.normal = styleState;
				_guiStyle_boneName_Unselected.wordWrap = false;
				_guiStyle_boneName_Unselected.alignment = TextAnchor.MiddleLeft;
				_guiStyle_boneName_Unselected.fontStyle = FontStyle.Bold;
			}

			if( !_initializedGUIStyle || _guiStyle_boneName_Unselected_Optional == null ) {
				_guiStyle_boneName_Unselected_Optional = new GUIStyle( EditorStyles.label );
				_guiStyle_boneName_Unselected_Optional.wordWrap = false;
				_guiStyle_boneName_Unselected_Optional.alignment = TextAnchor.MiddleLeft;
				_guiStyle_boneName_Unselected_Optional.fontStyle = FontStyle.Bold;
			}

			_initializedGUIStyle = true;
		}

		void _BoneHeader( string name )
		{
			EditorGUILayout.BeginHorizontal();
			GUILayout.FlexibleSpace();
			GUILayout.Label( name, _guiStyle_header );
			GUILayout.FlexibleSpace();
			EditorGUILayout.EndHorizontal();
		}

		void _BoneField( string boneName, ref FullBodyIK.Bone bone, bool isOptional )
		{
			Util.SafeNew( ref bone );
			var fbik = this.target as FullBodyIK;
			EditorGUILayout.BeginHorizontal();
			if( bone.transform == null ) {
				if( isOptional ) {
					GUILayout.Label( boneName + " *", _guiStyle_boneName_Unselected_Optional, GUILayout.Width( boneNameFieldSize ) );
				} else {
					GUILayout.Label( boneName, _guiStyle_boneName_Unselected, GUILayout.Width( boneNameFieldSize ) );
				}
			} else {
				GUILayout.Label( boneName, GUILayout.Width( boneNameFieldSize ) );
			}
			EditorUtil.GUI.PushEnabled( !Application.isPlaying );
			EditorUtil.GUI.TransformField( fbik, "", ref bone.transform, true );
			EditorUtil.GUI.PopEnabled();
			EditorGUILayout.EndHorizontal();
		}

		void _FingerBoneField( string boneName, ref FullBodyIK.Bone[] bones, bool isOptional )
		{
			if( bones == null || bones.Length != 4 ) {
				bones = new FullBodyIK.Bone[4];
			}

			for( int i = 0; i < bones.Length; ++i ) {
				string name = null;
				if( i + 1 == bones.Length ) {
					name = boneName + " Tip";
				} else {
					name = boneName + " " + (i + 1).ToString();
				}
				_BoneField( name, ref bones[i], isOptional );
			}
		}

		void _EffectorField( string effectorName, ref FullBodyIK.Effector effector )
		{
			var fbik = this.target as FullBodyIK;

			if( effector == null ) {
				return;
			}

			GUILayout.BeginHorizontal();
			GUILayout.Label( effectorName, GUILayout.Width( effectorNameFieldSize ) );
			GUILayout.FlexibleSpace();

			if( fbik._editorSettings.isShowEffectorTransform ) {
				EditorUtil.GUI.ObjectField( fbik, "", ref effector.transform, true, GUILayout.Height( 18.0f ) );
			} else {
				float labelSpace = 24.0f;

				EditorGUILayout.LabelField( "Pos", GUILayout.Width( labelSpace ) );
				EditorUtil.GUI.ToggleLegacy( fbik, "", ref effector.positionEnabled );
				EditorUtil.GUI.PushEnabled( effector.positionEnabled  );
				EditorUtil.GUI.HorizonalSlider( fbik, ref effector.positionWeight, 0.0f, 1.0f, GUILayout.ExpandWidth( false ), GUILayout.Width( 30.0f ) );
				EditorUtil.GUI.PushEnabled( effector.pullContained );
				EditorGUILayout.LabelField( "Pull", GUILayout.Width( labelSpace ) );
				EditorUtil.GUI.HorizonalSlider( fbik, ref effector.pull, 0.0f, 1.0f, GUILayout.ExpandWidth( false ), GUILayout.MinWidth( 30.0f ) );
				EditorUtil.GUI.PopEnabled();
				EditorUtil.GUI.PopEnabled();

				EditorUtil.GUI.PushEnabled( effector.rotationContained  );
				EditorGUILayout.LabelField( "Rot", GUILayout.Width( labelSpace ) );
				EditorUtil.GUI.ToggleLegacy( fbik, "", ref effector.rotationEnabled );
				EditorUtil.GUI.PushEnabled( effector.rotationEnabled  );
				EditorUtil.GUI.HorizonalSlider( fbik, ref effector.rotationWeight, 0.0f, 1.0f, GUILayout.ExpandWidth( false ), GUILayout.Width( 30.0f ) );
				EditorUtil.GUI.PopEnabled();
				EditorUtil.GUI.PopEnabled();
			}
			GUILayout.EndHorizontal();
		}
		
		public override void OnInspectorGUI()
		{
			_Initialize();

			var fbik = this.target as FullBodyIK;

			GUILayout.BeginHorizontal();
			GUILayout.FlexibleSpace();
			EditorUtil.GUI.ToggleLegacy( fbik, "Advanced", ref fbik._editorSettings.isAdvanced );
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			GUILayout.FlexibleSpace();
			EditorUtil.GUI.Toolbar( fbik, ref fbik._editorSettings.toolbarSelected, toolbarContents );
			GUILayout.FlexibleSpace();
			GUILayout.EndHorizontal();
			
			switch( fbik._editorSettings.toolbarSelected ) {
			case 0:
				_OnInspectorGUI_Basic();
				break;
			case 1:
				_OnInspectorGUI_Bones();
				break;
			case 2:
				_OnInspectorGUI_Effectors();
				break;
			}
		}

		bool _isPrefixed;

		void _OnInspectorGUI_Basic()
		{
			var fbik = this.target as FullBodyIK;

			if( !_isPrefixed ) {
				_isPrefixed = true;
				fbik.Prefix();
			}

			if( fbik._settings == null ) {
				fbik._settings = new FullBodyIK.Settings();
			}

			if( fbik._editorSettings.isAdvanced ) {
				fbik._settings.animatorEnabled = (FullBodyIK.AutomaticBool)EditorGUILayout.EnumPopup( "Animator Enabled", fbik._settings.animatorEnabled );
				fbik._settings.resetTransforms = (FullBodyIK.AutomaticBool)EditorGUILayout.EnumPopup( "Reset Transforms", fbik._settings.resetTransforms );
				fbik._settings.automaticConfigureTwistEnabled = EditorGUILayout.Toggle( "Automatic Configure Twist Enabled", fbik._settings.automaticConfigureTwistEnabled );
				fbik._settings.twistEnabled = EditorGUILayout.Toggle( "Twist Enabled", fbik._settings.twistEnabled );
				fbik._settings.modelTemplate = (FullBodyIK.ModelTemplate)EditorGUILayout.EnumPopup( "Model Template", fbik._settings.modelTemplate );

				fbik._settings.limbIK.automaticKneeBaseAngle		= EditorGUILayout.FloatField( "Automatic Knee Base Angle", fbik._settings.limbIK.automaticKneeBaseAngle );
				fbik._settings.limbIK.presolveKneeRate				= EditorGUILayout.FloatField( "Presolve Knee Rate", fbik._settings.limbIK.presolveKneeRate );
				fbik._settings.limbIK.presolveKneeLerpAngle			= EditorGUILayout.FloatField( "Presolve Knee Lerp Angle", fbik._settings.limbIK.presolveKneeLerpAngle );
				fbik._settings.limbIK.presolveKneeLerpLengthRate	= EditorGUILayout.FloatField( "Presolve Knee Lerp Length Rate", fbik._settings.limbIK.presolveKneeLerpLengthRate );
				fbik._settings.limbIK.presolveElbowRate				= EditorGUILayout.FloatField( "Presolve Elbow Rate", fbik._settings.limbIK.presolveElbowRate );
				fbik._settings.limbIK.presolveElbowLerpLengthRate	= EditorGUILayout.FloatField( "Presolve Elbow Lerp Length Rate", fbik._settings.limbIK.presolveElbowLerpLengthRate );
			}

#if SAFULLBODYIK_DEBUG
	EditorGUILayout.Separator();
			if( fbik._debugData.debugValues.Count == 0 ) {
				EditorGUILayout.LabelField( "No debug properties." );
			}

			foreach( var debugValue in fbik._debugData.debugValues ) {
				var v = debugValue.Value;
				switch( debugValue.Value.valueType ) {
				case FullBodyIK.DebugValueType.Int:
					v.intValue = EditorGUILayout.IntField( debugValue.Key, debugValue.Value.intValue );
					break;
				case FullBodyIK.DebugValueType.Float:
					if( debugValue.Key.Contains( "Rate" ) ) {
						v.floatValue = EditorGUILayout.Slider( debugValue.Key, debugValue.Value.floatValue, 0.0f, 1.0f );
					} else {
						v.floatValue = EditorGUILayout.FloatField( debugValue.Key, debugValue.Value.floatValue );
					}
					break;
				case FullBodyIK.DebugValueType.Bool:
					v.boolValue = EditorGUILayout.Toggle( debugValue.Key, debugValue.Value.boolValue );
					break;
				}
			}
#endif
		}

		void _OnInspectorGUI_Bones()
		{
			var fbik = this.target as FullBodyIK;
			
			_scrollViewPos_Bones = EditorGUILayout.BeginScrollView( _scrollViewPos_Bones );

			Animator animator = fbik.gameObject.GetComponent<Animator>();
			bool isAnimatorHumanoid = (animator != null) ? animator.isHuman : false;

			_BoneHeader( "Tool" );
			EditorGUILayout.BeginHorizontal();
			GUILayout.FlexibleSpace();
			EditorUtil.GUI.PushEnabled( !Application.isPlaying && isAnimatorHumanoid );
			if( GUILayout.Button( "Configure from Humanoid" ) ) {
				_ConfigureHumanoidBones();
			}
			EditorUtil.GUI.PopEnabled();
			EditorUtil.GUI.PushEnabled( !Application.isPlaying );
			if( GUILayout.Button( "Reset" ) ) {
				_ResetBones();
			}
			EditorUtil.GUI.PopEnabled();
			EditorGUILayout.EndHorizontal();

			EditorGUILayout.Separator();
			EditorGUILayout.LabelField( "* is Optional." );

			EditorGUILayout.Separator();
			_BoneHeader( "Body" );

			_BoneField( "Pelvis", ref fbik._bodyBones.pelvis, false );
			_BoneField( "Torso", ref fbik._bodyBones.torso, false );
			_BoneField( "Torso2", ref fbik._bodyBones.torso2, true );

			EditorGUILayout.Separator();
			_BoneHeader( "Head" );

			_BoneField( "Neck", ref fbik._headBones.neck, false );
			_BoneField( "Head", ref fbik._headBones.head, true );
			_BoneField( "Left Eye", ref fbik._headBones.leftEye, true );
			_BoneField( "Right Eye", ref fbik._headBones.rightEye, true );

			for( int i = 0; i < 2; ++i ) {
				FullBodyIK.LegBones legBones = (i == 0) ? fbik._leftLegBones : fbik._rightLegBones;

				EditorGUILayout.Separator();
				_BoneHeader( (i == 0) ? "Left Leg" : "Right Leg" );
				string prefix = (i == 0) ? "L " : "R ";

				_BoneField( prefix + "Leg", ref legBones.leg, false );
				_BoneField( prefix + "Knee", ref legBones.knee, false );
				_BoneField( prefix + "Foot", ref legBones.foot, false );
			}

			for( int i = 0; i < 2; ++i ) {
				FullBodyIK.ArmBones armBones = (i == 0) ? fbik._leftArmBones : fbik._rightArmBones;

				EditorGUILayout.Separator();
				_BoneHeader( (i == 0) ? "Left Arm" : "Right Arm" );
				string prefix = (i == 0) ? "L " : "R ";

				_BoneField( prefix + "Shoulder", ref armBones.shoulder, true );
				_BoneField( prefix + "Arm", ref armBones.arm, false );
				if( armBones.armTwist != null ) {
					for( int n = 0; n < armBones.armTwist.Length; ++n ) {
						_BoneField( prefix + "ArmTwist", ref armBones.armTwist[n], true );
					}
				}
				_BoneField( prefix + "Elbow", ref armBones.elbow, false );
				if( armBones.handTwist != null ) {
					for( int n = 0; n < armBones.handTwist.Length; ++n ) {
						_BoneField( prefix + "HandTwist", ref armBones.handTwist[n], true );
					}
				}
				_BoneField( prefix + "Wrist", ref armBones.wrist, false );
			}

			for( int i = 0; i < 2; ++i ) {
				FullBodyIK.FingersBones fingerBones = (i == 0) ? fbik._leftHandFingersBones : fbik._rightHandFingersBones;

				EditorGUILayout.Separator();
				_BoneHeader( (i == 0) ? "Left Fingers" : "Right Fingers" );
				string prefix = (i == 0) ? "L " : "R ";

				_FingerBoneField( prefix + "Thumb", ref fingerBones.thumb, true );
				EditorGUILayout.Separator();
				_FingerBoneField( prefix + "Index", ref fingerBones.index, true );
				EditorGUILayout.Separator();
				_FingerBoneField( prefix + "Middle", ref fingerBones.middle, true );
				EditorGUILayout.Separator();
				_FingerBoneField( prefix + "Ring", ref fingerBones.ring, true );
				EditorGUILayout.Separator();
				_FingerBoneField( prefix + "Little", ref fingerBones.little, true );
				EditorGUILayout.Separator();
			}

			EditorGUILayout.EndScrollView();
		}
		
		void _OnInspectorGUI_Effectors()
		{
			var fbik = this.target as FullBodyIK;

			_BoneHeader( "Tool" );
			GUILayout.BeginHorizontal();
			GUILayout.FlexibleSpace();
			EditorUtil.GUI.PushEnabled( !Application.isPlaying );
			if( GUILayout.Button( "Prepare Transforms" ) ) {
				_PrepareEffectorTransforms();
			}
			if( GUILayout.Button( "Reset" ) ) {
				_ResetEffectorTransforms();
			}
			EditorUtil.GUI.PopEnabled();
			GUILayout.EndHorizontal();

			_BoneHeader( "List" );

			GUILayout.BeginHorizontal();
			GUILayout.FlexibleSpace();
			EditorUtil.GUI.ToggleLegacy( fbik, "Show Transforms", ref fbik._editorSettings.isShowEffectorTransform );
			GUILayout.EndHorizontal();

			EditorGUILayout.Separator();

			_scrollViewPos_Effectors = EditorGUILayout.BeginScrollView( _scrollViewPos_Effectors );
			
			Util.SafeNew( ref fbik._bodyEffectors );
			Util.SafeNew( ref fbik._headEffectors );
			Util.SafeNew( ref fbik._leftArmEffectors );
			Util.SafeNew( ref fbik._rightArmEffectors );
			Util.SafeNew( ref fbik._leftLegEffectors );
			Util.SafeNew( ref fbik._rightLegEffectors );
			Util.SafeNew( ref fbik._leftHandFingersEffectors );
			Util.SafeNew( ref fbik._rightHandFingersEffectors );
			
			EditorGUILayout.Separator();
			_BoneHeader( "Body" );
			_EffectorField( "Pelvis", ref fbik._bodyEffectors.pelvis );

			EditorGUILayout.Separator();
			_BoneHeader( "Head" );
			_EffectorField( "Neck", ref fbik._headEffectors.neck );
			_EffectorField( "Head", ref fbik._headEffectors.head );
			_EffectorField( "Eyes", ref fbik._headEffectors.eyes );

			for( int i = 0; i < 2; ++i ) {
				EditorGUILayout.Separator();
				_BoneHeader( (i == 0) ? "Left Leg" : "Right Leg" );
				var effectors = (i == 0) ? fbik._leftLegEffectors : fbik._rightLegEffectors;
				var prefix = (i == 0) ? "L " : "R ";
				_EffectorField( prefix + "Knee", ref effectors.knee );
				_EffectorField( prefix + "Foot", ref effectors.foot );
			}

			for( int i = 0; i < 2; ++i ) {
				EditorGUILayout.Separator();
				_BoneHeader( (i == 0) ? "Left Arm" : "Right Arm" );
				var effectors = (i == 0) ? fbik._leftArmEffectors : fbik._rightArmEffectors;
				var prefix = (i == 0) ? "L " : "R ";
				_EffectorField( prefix + "Arm", ref effectors.arm );
				_EffectorField( prefix + "Elbow", ref effectors.elbow );
				_EffectorField( prefix + "Wrist", ref effectors.wrist );
			}

			for( int i = 0; i < 2; ++i ) {
				EditorGUILayout.Separator();
				_BoneHeader( (i == 0) ? "Left Wrist Fingers" : "Right Wrist Fingers" );
				var effectors = (i == 0) ? fbik._leftHandFingersEffectors : fbik._rightHandFingersEffectors;
				var prefix = (i == 0) ? "L " : "R ";
				_EffectorField( prefix + "Thumb", ref effectors.thumb );
				_EffectorField( prefix + "Index", ref effectors.index );
				_EffectorField( prefix + "Middle", ref effectors.middle );
				_EffectorField( prefix + "Ring", ref effectors.ring );
				_EffectorField( prefix + "Little", ref effectors.little );
			}

			EditorGUILayout.EndScrollView();
		}
		
		void _ConfigureHumanoidBones()
		{
		}
		
		void _ResetBones()
		{
		}
		
		void _PrepareEffectorTransforms()
		{
		}
		
		void _ResetEffectorTransforms()
		{
		}
	}
}