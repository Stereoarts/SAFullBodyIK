#if SAFULLBODYIK_DEBUG
//#define SAFULLBODYIK_DEBUG_CONSTRUCT_TIME
#define SAFULLBODYIK_DEBUG_LOCKTRANSFORM
#endif

// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing
using UnityEngine;
using System.Collections.Generic;

namespace SA
{
	[ExecuteInEditMode]
	public partial class FullBodyIK : MonoBehaviour
	{
		#if SAFULLBODYIK_DEBUG
		public enum DebugValueType
		{
			Int,
			Float,
			Bool,
		}
		
		public class DebugValue
		{
			public DebugValueType valueType;
			
			public int intValue;
			public float floatValue;
			public bool boolValue;

			public DebugValue( int i )
			{
				this.valueType = DebugValueType.Int;
				this.intValue = i;
				this.floatValue = (float)i;
				this.boolValue = (i != 0);
			}

			public DebugValue( float f )
			{
				this.valueType = DebugValueType.Float;
				this.intValue = (int)f;
				this.floatValue = f;
				this.boolValue = (f != 0.0f);
			}

			public DebugValue( bool b )
			{
				this.valueType = DebugValueType.Bool;
				this.intValue = b ? 1 : 0;
				this.floatValue = b ? 1.0f : 0.0f;
				this.boolValue = b;
			}
			
			public static implicit operator DebugValue( int i ) { return new DebugValue( i ); }
			public static implicit operator int( DebugValue v ) { return v.intValue; }
			public static implicit operator DebugValue( float f ) { return new DebugValue( f ); }
			public static implicit operator float( DebugValue v ) { return v.floatValue; }
			public static implicit operator DebugValue( bool b ) { return new DebugValue( b ); }
			public static implicit operator bool( DebugValue v ) { return v.boolValue; }
		}

		public struct DebugPoint
		{
			public Vector3 pos;
			public Color color;
			public float size;

			public DebugPoint( Vector3 pos )
			{
				this.pos = pos;
				this.color = Color.red;
				this.size = 0.03f;
			}

			public DebugPoint( Vector3 pos, Color color )
			{
				this.pos = pos;
				this.color = color;
				this.size = 0.03f;
			}

			public DebugPoint( Vector3 pos, Color color, float size )
			{
				this.pos = pos;
				this.color = color;
				this.size = size;
			}
		}

		public class DebugData
		{
			public Dictionary<string, DebugValue> debugValues = new Dictionary<string, DebugValue>();
			public List<DebugPoint> debugPoints = new List<DebugPoint>();

			public void AddPoint( Vector3 pos )
			{
				this.debugPoints.Add( new DebugPoint( pos ) );
			}

			public void AddPoint( Vector3 pos, Color color )
			{
				this.debugPoints.Add( new DebugPoint( pos, color ) );
			}

			public void AddPoint( Vector3 pos, Color color, float size )
			{
				this.debugPoints.Add( new DebugPoint( pos, color, size ) );
			}

			public void UpdateValue( string name, ref int v )
			{
				DebugValue debugValue;
				if( this.debugValues.TryGetValue( name, out debugValue ) ) {
					v = debugValue;
					return;
				}

				this.debugValues.Add( name, v );
			}

			public void UpdateValue( string name, ref float v )
			{
				DebugValue debugValue;
				if( this.debugValues.TryGetValue( name, out debugValue ) ) {
					v = debugValue;
					return;
				}

				this.debugValues.Add( name, v );
			}

			public void UpdateValue( string name, ref bool v )
			{
				DebugValue debugValue;
				if( this.debugValues.TryGetValue( name, out debugValue ) ) {
					v = debugValue;
					return;
				}

				this.debugValues.Add( name, v );
			}
		}

		[System.NonSerialized]
		public DebugData _debugData = new DebugData();
		#if SAFULLBODYIK_DEBUG_LOCKTRANSFORM
		public Vector3 _debug_currentPosition = Vector3.zero;
		#endif
		#endif
		
		[System.Serializable]
		public class BodyBones
		{
			public Bone pelvis;
			public Bone torso;
			public Bone torso2;

			public static BodyBones Preset()
			{
				BodyBones bones = new BodyBones();
				bones.pelvis = Bone.Preset( BoneLocation.Pelvis );
				bones.torso = Bone.Preset( BoneLocation.Torso );
				bones.torso2 = Bone.Preset( BoneLocation.Torso2 );
				return bones;
			}
		}

		[System.Serializable]
		public class HeadBones
		{
			public Bone neck;
			public Bone head;
			public Bone leftEye;
			public Bone rightEye;

			public static HeadBones Preset()
			{
				HeadBones bones = new HeadBones();
				bones.neck = Bone.Preset( BoneLocation.Neck );
				bones.head = Bone.Preset( BoneLocation.Head );
				bones.leftEye = Bone.Preset( BoneLocation.LeftEye );
				bones.rightEye = Bone.Preset( BoneLocation.RightEye );
				return bones;
			}
		}

		[System.Serializable]
		public class LegBones
		{
			public Bone leg;
			public Bone knee;
			public Bone foot;

			public static LegBones Preset( Side side )
			{
				LegBones bones = new LegBones();
				bones.leg = Bone.Preset( (side == Side.Left) ? BoneLocation.LeftLeg : BoneLocation.RightLeg );
				bones.knee = Bone.Preset( (side == Side.Left) ? BoneLocation.LeftKnee : BoneLocation.RightKnee );
				bones.foot = Bone.Preset( (side == Side.Left) ? BoneLocation.LeftFoot : BoneLocation.RightFoot );
				return bones;
			}
		}

		[System.Serializable]
		public class ArmBones
		{
			public Bone shoulder;
			public Bone arm;
			public Bone[] armTwist;
			public Bone elbow;
			public Bone[] handTwist;
			public Bone wrist;

			public static ArmBones Preset( Side side )
			{
				ArmBones bones = new ArmBones();
				bones.Repair( side );
                bones.shoulder = Bone.Preset( (side == Side.Left) ? BoneLocation.LeftShoulder : BoneLocation.RightShoulder );
				bones.arm = Bone.Preset( (side == Side.Left) ? BoneLocation.LeftArm : BoneLocation.RightArm );
				bones.elbow = Bone.Preset( (side == Side.Left) ? BoneLocation.LeftElbow : BoneLocation.RightElbow );
				bones.wrist = Bone.Preset( (side == Side.Left) ? BoneLocation.LeftWrist : BoneLocation.RightWrist );

				BoneLocation armTwistLocation = ((side == Side.Left) ? BoneLocation.LeftArmTwist0 : BoneLocation.RightArmTwist0);
				BoneLocation handTwistLocation = ((side == Side.Left) ? BoneLocation.LeftHandTwist0 : BoneLocation.RightHandTwist0);
				for( int i = 0; i < MaxArmTwistLength; ++i ) {
					bones.armTwist[i] = Bone.Preset( (BoneLocation)(armTwistLocation + i) );
				}
				for( int i = 0; i < MaxHandTwistLength; ++i ) {
					bones.handTwist[i] = Bone.Preset( (BoneLocation)(handTwistLocation + i) );
				}

				return bones;
			}

			public void Repair( Side side )
			{
				_Repair( ref armTwist, MaxArmTwistLength );
				_Repair( ref handTwist, MaxHandTwistLength );
			}

			static void _Repair( ref Bone[] bones, int length )
			{
				if( bones == null || bones.Length != length ) {
					System.Array.Resize( ref bones, length );
				}
			}
		}

		[System.Serializable]
		public class FingersBones
		{
			public Bone[] thumb;
			public Bone[] index;
			public Bone[] middle;
			public Bone[] ring;
			public Bone[] little;

			public static FingersBones Preset( Side side )
			{
				FingersBones bones = new FingersBones();
				bones.Repair( side );
				BoneLocation thumbLocation = ((side == Side.Left) ? BoneLocation.LeftHandThumb0 : BoneLocation.RightHandThumb0);
				BoneLocation indexLocation = ((side == Side.Left) ? BoneLocation.LeftHandIndex0 : BoneLocation.RightHandIndex0);
				BoneLocation middleLocation = ((side == Side.Left) ? BoneLocation.LeftHandMiddle0 : BoneLocation.RightHandMiddle0);
				BoneLocation ringLocation = ((side == Side.Left) ? BoneLocation.LeftHandRing0 : BoneLocation.RightHandRing0);
				BoneLocation littleLocation = ((side == Side.Left) ? BoneLocation.LeftHandLittle0 : BoneLocation.RightHandLittle0);
				for( int i = 0; i < MaxHandFingerLength; ++i ) {
					bones.thumb[i] = Bone.Preset( (BoneLocation)(thumbLocation + i) );
					bones.index[i] = Bone.Preset( (BoneLocation)(indexLocation + i) );
					bones.middle[i] = Bone.Preset( (BoneLocation)(middleLocation + i) );
					bones.ring[i] = Bone.Preset( (BoneLocation)(ringLocation + i) );
					bones.little[i] = Bone.Preset( (BoneLocation)(littleLocation + i) );
				}
				return bones;
			}

			public void Repair( Side side )
			{
				_Repair( ref thumb, MaxHandFingerLength );
				_Repair( ref index, MaxHandFingerLength );
				_Repair( ref middle, MaxHandFingerLength );
				_Repair( ref ring, MaxHandFingerLength );
				_Repair( ref little, MaxHandFingerLength );
				// Memo: Don't alloc each bone instances.( Alloc in _Prefix() ).
			}

			static void _Repair( ref Bone[] bones, int length )
			{
				if( bones == null || bones.Length != length ) {
					System.Array.Resize( ref bones, length );
				}
			}
		}
		
		[System.Serializable]
		public class BodyEffectors
		{
			public Effector pelvis;
			
			public static BodyEffectors Preset()
			{
				BodyEffectors effectors = new BodyEffectors();
				effectors.pelvis = Effector.Preset( EffectorLocation.Pelvis );
				return effectors;
			}
		}
		
		[System.Serializable]
		public class HeadEffectors
		{
			public Effector neck;
			public Effector head;
			public Effector eyes;
			
			public static HeadEffectors Preset()
			{
				HeadEffectors effectors = new HeadEffectors();
				effectors.neck = Effector.Preset( EffectorLocation.Neck );
				effectors.head = Effector.Preset( EffectorLocation.Head );
				effectors.eyes = Effector.Preset( EffectorLocation.Eyes );
				return effectors;
			}
		}

		[System.Serializable]
		public class LegEffectors
		{
			public Effector knee;
			public Effector foot;
			
			public static LegEffectors Preset( Side side )
			{
				LegEffectors effectors = new LegEffectors();
				effectors.knee = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftKnee : EffectorLocation.RightKnee );
				effectors.foot = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftFoot : EffectorLocation.RightFoot );
				return effectors;
			}
		}

		[System.Serializable]
		public class ArmEffectors
		{
			public Effector arm;
			public Effector elbow;
			public Effector wrist;
			
			public static ArmEffectors Preset( Side side )
			{
				ArmEffectors effectors = new ArmEffectors();
				effectors.arm = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftArm : EffectorLocation.RightArm );
				effectors.elbow = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftElbow : EffectorLocation.RightElbow );
				effectors.wrist = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftWrist : EffectorLocation.RightWrist );
				return effectors;
			}
		}

		[System.Serializable]
		public class FingersEffectors
		{
			public Effector thumb;
			public Effector index;
			public Effector middle;
			public Effector ring;
			public Effector little;
			
			public static FingersEffectors Preset( Side side )
			{
				FingersEffectors effectors = new FingersEffectors();
				effectors.thumb = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftHandThumb : EffectorLocation.RightHandThumb );
				effectors.index = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftHandIndex : EffectorLocation.RightHandIndex );
				effectors.middle = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftHandMiddle : EffectorLocation.RightHandMiddle );
				effectors.ring = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftHandRing : EffectorLocation.RightHandRing );
				effectors.little = Effector.Preset( (side == Side.Left) ? EffectorLocation.LeftHandLittle : EffectorLocation.RightHandLittle );
				return effectors;
			}
		}

		public enum AutomaticBool
		{
			Auto = -1,
			Disable = 0,
			Enable = 1,
		}

		[System.Serializable]
		public class Settings
		{
			public AutomaticBool animatorEnabled = AutomaticBool.Auto;
			public AutomaticBool resetTransforms = AutomaticBool.Auto;

			public bool automaticConfigureTwistEnabled = true;
			public bool createEffectorTransform = true;

			public bool overrideAutomaticKneePosition = true;
			public ModelTemplate modelTemplate = ModelTemplate.Standard;
			public float automaticKneeBaseAngle = 0.0f;

			public float pelvisEffectorKeepHrizontalRate = 1.0f;
		}

		public class InternalValues
		{
			public bool animatorEnabled;
			public bool resetTransforms;
			public bool continuousSolverEnabled;

			public Vector3 defaultRootPosition = Vector3.zero;
			public Matrix3x3 defaultRootBasis = Matrix3x3.identity;
			public Matrix3x3 defaultRootBasisInv = Matrix3x3.identity;
			public Quaternion defaultRootRotation = Quaternion.identity;

			public bool rootTransformIsAlive = false;
			public Transform rootTransform = null;
		}

		[System.Serializable]
		public class EditorSettings
		{
			public bool isAdvanced;
			public int toolbarSelected;
			public bool isShowEffectorTransform;
		}

		public BodyBones _bodyBones = BodyBones.Preset();
		public HeadBones _headBones = HeadBones.Preset();
		public LegBones _leftLegBones = LegBones.Preset( Side.Left );
		public LegBones _rightLegBones = LegBones.Preset( Side.Right );
		public ArmBones _leftArmBones = ArmBones.Preset( Side.Left );
		public ArmBones _rightArmBones = ArmBones.Preset( Side.Right );
		public FingersBones _leftHandFingersBones = FingersBones.Preset( Side.Left );
		public FingersBones _rightHandFingersBones = FingersBones.Preset( Side.Right );

		public Effector _rootEffector = Effector.Preset( EffectorLocation.Root );
		public BodyEffectors _bodyEffectors = BodyEffectors.Preset();
		public HeadEffectors _headEffectors = HeadEffectors.Preset();
		public LegEffectors _leftLegEffectors = LegEffectors.Preset( Side.Left );
		public LegEffectors _rightLegEffectors = LegEffectors.Preset( Side.Right );
		public ArmEffectors _leftArmEffectors = ArmEffectors.Preset( Side.Left );
		public ArmEffectors _rightArmEffectors = ArmEffectors.Preset( Side.Right );
		public FingersEffectors _leftHandFingersEffectors = FingersEffectors.Preset( Side.Left );
		public FingersEffectors _rightHandFingersEffectors = FingersEffectors.Preset( Side.Right );

		public Settings _settings = new Settings();
		public InternalValues _internalValues = new InternalValues();
		public EditorSettings _editorSettings = new EditorSettings();

		public bool _prepareHumanoid = true; // todo: Move to settings.

		[Range( 0.0f, 1.0f )]
		public float _armToTorsoTwistRate = 0.8f;
		[Range( 0.0f, 1.0f )]
		public float _armToTorsoUTwistRate = 0.9f;
		public bool _shoulderLocked = false;

		public bool bodyIKEnabled = true; // todo: Move to settings.

		Bone[] _bones = new Bone[(int)BoneType.Max];
		Effector[] _effectors = new Effector[(int)EffectorLocation.Max];

		public Bone[] bones { get { return _bones; } }
		public Effector[] effectors { get { return _effectors; } }
		
		BodyIK _bodyIK;
		LimbIK[] _limbIK = new LimbIK[(int)LimbIKLocation.Max];
		HeadIK _headIK;
		FingerIK[] _fingerIK = new FingerIK[(int)FingerIKType.Max];

		bool _isPrefixed;

		void Awake()
		{
			if( !Application.isPlaying ) {
				return;
			}

			#if SAFULLBODYIK_DEBUG_LOCKTRANSFORM
			_debug_currentPosition = this.transform.position;
			#endif

			#if SAFULLBODYIK_DEBUG_CONSTRUCT_TIME
			float constructBeginTime = Time.realtimeSinceStartup;
			#endif
			Prefix();
			ConfigureBoneTransforms();
			Prepare();
			#if SAFULLBODYIK_DEBUG_CONSTRUCT_TIME
			float constructEndTime = Time.realtimeSinceStartup;
			Debug.Log( "Construct time: " + (constructEndTime - constructBeginTime) );
			#endif
		}

		static void _SetBoneTransform( ref Bone bone, Transform transform )
		{
			if( bone == null ) {
				bone = new Bone();
			}

			bone.transform = transform;
		}

		static void _SetFingerBoneTransform( ref Bone[] bones, Transform[,] transforms, int index )
		{
			if( bones == null || bones.Length != MaxHandFingerLength ) {
				bones = new Bone[MaxHandFingerLength];
			}

			for( int i = 0; i != MaxHandFingerLength; ++i ) {
				if( bones[i] == null ) {
					bones[i] = new Bone();
				}
				bones[i].transform = transforms[index, i];
			}
		}

		static bool _IsNeck( Transform trn )
		{
			if( trn != null ) {
				string name = trn.name;
				if( name != null ) {
					if( name.Contains( "Neck" ) || name.Contains( "neck" ) || name.Contains( "NECK" ) ) {
						return true;
					}
					if( name.Contains( "Kubi" ) || name.Contains( "kubi" ) || name.Contains( "KUBI" ) ) {
						return true;
					}
					if( name.Contains( "\u304F\u3073" ) ) { // Kubi(Hira-gana)
						return true;
					}
					if( name.Contains( "\u30AF\u30D3" ) ) { // Kubi(Kana-kana)
						return true;
					}
					if( name.Contains( "\u9996" ) ) { // Kubi(Kanji)
						return true;
					}
				}
			}

			return false;
		}

		// - Call from Awake() or Editor script.
		// - transform is null yet.
		public void Prefix()
		{
			if( _isPrefixed ) {
				return;
			}
			
			_isPrefixed = true;
			
			if( _bodyBones == null ) {
				_bodyBones = BodyBones.Preset();
			}
			if( _headBones == null ) {
				_headBones = HeadBones.Preset();
			}
			if( _leftLegBones == null ) {
				_leftLegBones = LegBones.Preset( Side.Left );
			}
			if( _rightLegBones == null ) {
				_rightLegBones = LegBones.Preset( Side.Right );
			}
			if( _leftArmBones == null ) {
				_leftArmBones = ArmBones.Preset( Side.Left );
			} else {
				_leftArmBones.Repair( Side.Left );
			}
			if( _rightArmBones == null ) {
				_rightArmBones = ArmBones.Preset( Side.Right );
			} else {
				_rightArmBones.Repair( Side.Right );
            }
			if( _leftHandFingersBones == null ) {
				_leftHandFingersBones = FingersBones.Preset( Side.Left );
			} else {
				_leftHandFingersBones.Repair( Side.Left );
			}
			if( _rightHandFingersBones == null ) {
				_rightHandFingersBones = FingersBones.Preset( Side.Right );
			} else {
				_rightHandFingersBones.Repair( Side.Right );
			}

			if( _bodyEffectors == null ) {
				_bodyEffectors = BodyEffectors.Preset();
			}
			if( _headEffectors == null ) {
				_headEffectors = HeadEffectors.Preset();
			}
			if( _leftArmEffectors == null ) {
				_leftArmEffectors = ArmEffectors.Preset( Side.Left );
			}
			if( _rightArmEffectors == null ) {
				_rightArmEffectors = ArmEffectors.Preset( Side.Right );
			}
			if( _leftLegEffectors == null ) {
				_leftLegEffectors = LegEffectors.Preset( Side.Left );
			}
			if( _rightLegEffectors == null ) {
				_rightLegEffectors = LegEffectors.Preset( Side.Right );
			}

			if( _settings == null ) {
				_settings = new Settings();
			}
			if( _bones == null || _bones.Length != (int)BoneLocation.Max ) {
				_bones = new Bone[(int)BoneLocation.Max];
			}
			if( _effectors == null || _effectors.Length != (int)EffectorLocation.Max ) {
				_effectors = new Effector[(int)EffectorLocation.Max];
			}

			_Prefix( ref _bodyBones.pelvis, BoneLocation.Pelvis, null );
			_Prefix( ref _bodyBones.torso, BoneLocation.Torso, _bodyBones.pelvis );
			_Prefix( ref _bodyBones.torso2, BoneLocation.Torso2, _bodyBones.torso );
			_Prefix( ref _headBones.neck, BoneLocation.Neck, _bodyBones.torso2 );
			_Prefix( ref _headBones.head, BoneLocation.Head, _headBones.neck );
			_Prefix( ref _headBones.leftEye, BoneLocation.LeftEye, _headBones.head );
			_Prefix( ref _headBones.rightEye, BoneLocation.RightEye, _headBones.head );
			for( int i = 0; i < 2; ++i ) {
				var legBones = (i == 0) ? _leftLegBones : _rightLegBones;
				_Prefix( ref legBones.leg, (i == 0) ? BoneLocation.LeftLeg : BoneLocation.RightLeg, _bodyBones.pelvis );
				_Prefix( ref legBones.knee, (i == 0) ? BoneLocation.LeftKnee : BoneLocation.RightKnee, legBones.leg );
				_Prefix( ref legBones.foot, (i == 0) ? BoneLocation.LeftFoot : BoneLocation.RightFoot, legBones.knee );

				var armBones = (i == 0) ? _leftArmBones : _rightArmBones;
				_Prefix( ref armBones.shoulder, (i == 0) ? BoneLocation.LeftShoulder : BoneLocation.RightShoulder, _bodyBones.torso2 );
				_Prefix( ref armBones.arm, (i == 0) ? BoneLocation.LeftArm : BoneLocation.RightArm, armBones.shoulder );
				_Prefix( ref armBones.elbow, (i == 0) ? BoneLocation.LeftElbow : BoneLocation.RightElbow, armBones.arm );
				_Prefix( ref armBones.wrist, (i == 0) ? BoneLocation.LeftWrist : BoneLocation.RightWrist, armBones.elbow );

				for( int n = 0; n < MaxArmTwistLength; ++n ) {
					var armTwistLocation = (i == 0) ? BoneLocation.LeftArmTwist0 : BoneLocation.RightArmTwist0;
					_Prefix( ref armBones.armTwist[n], (BoneLocation)((int)armTwistLocation + n), armBones.arm );
				}

				for( int n = 0; n < MaxHandTwistLength; ++n ) {
					var handTwistLocation = (i == 0) ? BoneLocation.LeftHandTwist0 : BoneLocation.RightHandTwist0;
					_Prefix( ref armBones.handTwist[n], (BoneLocation)((int)handTwistLocation + n), armBones.elbow );
				}

				var fingerBones = (i == 0) ? _leftHandFingersBones : _rightHandFingersBones;
				for( int n = 0; n < MaxHandFingerLength; ++n ) {
					var thumbLocation = (i == 0) ? BoneLocation.LeftHandThumb0 : BoneLocation.RightHandThumb0;
					var indexLocation = (i == 0) ? BoneLocation.LeftHandIndex0 : BoneLocation.RightHandIndex0;
					var middleLocation = (i == 0) ? BoneLocation.LeftHandMiddle0 : BoneLocation.RightHandMiddle0;
					var ringLocation = (i == 0) ? BoneLocation.LeftHandRing0 : BoneLocation.RightHandRing0;
					var littleLocation = (i == 0) ? BoneLocation.LeftHandLittle0 : BoneLocation.RightHandLittle0;
					_Prefix( ref fingerBones.thumb[n], (BoneLocation)((int)thumbLocation + n), (n == 0) ? armBones.wrist : fingerBones.thumb[n - 1] );
					_Prefix( ref fingerBones.index[n], (BoneLocation)((int)indexLocation + n), (n == 0) ? armBones.wrist : fingerBones.index[n - 1] );
					_Prefix( ref fingerBones.middle[n], (BoneLocation)((int)middleLocation + n), (n == 0) ? armBones.wrist : fingerBones.middle[n - 1] );
					_Prefix( ref fingerBones.ring[n], (BoneLocation)((int)ringLocation + n), (n == 0) ? armBones.wrist : fingerBones.ring[n - 1] );
					_Prefix( ref fingerBones.little[n], (BoneLocation)((int)littleLocation + n), (n == 0) ? armBones.wrist : fingerBones.little[n - 1] );
				}
			}

			_Prefix( ref _rootEffector, EffectorLocation.Root );
			_Prefix( ref _bodyEffectors.pelvis, EffectorLocation.Pelvis, _rootEffector, _bodyBones.pelvis, _leftLegBones.leg, _rightLegBones.leg );
			_Prefix( ref _headEffectors.neck, EffectorLocation.Neck, _bodyEffectors.pelvis, _headBones.neck );
			_Prefix( ref _headEffectors.head, EffectorLocation.Head, _headEffectors.neck, _headBones.head );
			_Prefix( ref _headEffectors.eyes, EffectorLocation.Eyes, _rootEffector, _headBones.head, _headBones.leftEye, _headBones.rightEye );

			_Prefix( ref _leftLegEffectors.knee, EffectorLocation.LeftKnee, _rootEffector, _leftLegBones.knee );
			_Prefix( ref _leftLegEffectors.foot, EffectorLocation.LeftFoot, _rootEffector, _leftLegBones.foot );
			_Prefix( ref _rightLegEffectors.knee, EffectorLocation.RightKnee, _rootEffector, _rightLegBones.knee );
			_Prefix( ref _rightLegEffectors.foot, EffectorLocation.RightFoot, _rootEffector, _rightLegBones.foot );

			_Prefix( ref _leftArmEffectors.arm, EffectorLocation.LeftArm, _bodyEffectors.pelvis, _leftArmBones.arm );
			_Prefix( ref _leftArmEffectors.elbow, EffectorLocation.LeftElbow, _bodyEffectors.pelvis, _leftArmBones.elbow );
			_Prefix( ref _leftArmEffectors.wrist, EffectorLocation.LeftWrist, _bodyEffectors.pelvis, _leftArmBones.wrist );
			_Prefix( ref _rightArmEffectors.arm, EffectorLocation.RightArm, _bodyEffectors.pelvis, _rightArmBones.arm );
			_Prefix( ref _rightArmEffectors.elbow, EffectorLocation.RightElbow, _bodyEffectors.pelvis, _rightArmBones.elbow );
			_Prefix( ref _rightArmEffectors.wrist, EffectorLocation.RightWrist, _bodyEffectors.pelvis, _rightArmBones.wrist );
		
			_Prefix( ref _leftHandFingersEffectors.thumb, EffectorLocation.LeftHandThumb, _leftArmEffectors.wrist, _leftHandFingersBones.thumb );
			_Prefix( ref _leftHandFingersEffectors.index, EffectorLocation.LeftHandIndex, _leftArmEffectors.wrist, _leftHandFingersBones.index );
			_Prefix( ref _leftHandFingersEffectors.middle, EffectorLocation.LeftHandMiddle, _leftArmEffectors.wrist, _leftHandFingersBones.middle );
			_Prefix( ref _leftHandFingersEffectors.ring, EffectorLocation.LeftHandRing, _leftArmEffectors.wrist, _leftHandFingersBones.ring );
			_Prefix( ref _leftHandFingersEffectors.little, EffectorLocation.LeftHandLittle, _leftArmEffectors.wrist, _leftHandFingersBones.little );

			_Prefix( ref _rightHandFingersEffectors.thumb, EffectorLocation.RightHandThumb, _rightArmEffectors.wrist, _rightHandFingersBones.thumb );
			_Prefix( ref _rightHandFingersEffectors.index, EffectorLocation.RightHandIndex, _rightArmEffectors.wrist, _rightHandFingersBones.index );
			_Prefix( ref _rightHandFingersEffectors.middle, EffectorLocation.RightHandMiddle, _rightArmEffectors.wrist, _rightHandFingersBones.middle );
			_Prefix( ref _rightHandFingersEffectors.ring, EffectorLocation.RightHandRing, _rightArmEffectors.wrist, _rightHandFingersBones.ring );
			_Prefix( ref _rightHandFingersEffectors.little, EffectorLocation.RightHandLittle, _rightArmEffectors.wrist, _rightHandFingersBones.little );

			// Hidden function.
			if( _settings.modelTemplate == ModelTemplate.Standard ) {
				var animator = this.GetComponent<Animator>();
				if( animator != null ) {
					var avatar = animator.avatar;
					if( avatar != null ) {
						if( avatar.name.Contains( "unitychan" ) ) {
							_settings.modelTemplate = ModelTemplate.UnityChan;
                        }
					}
				}
			}
		}
		
		public void CleanupBoneTransforms()
		{
			Prefix();
			
			if( _bones != null ) {
				for( int i = 0; i < _bones.Length; ++i ) {
					Assert( _bones[i] != null );
					if( _bones[i] != null ) {
						_bones[i].transform = null;
					}
				}
			}
		}

		static readonly string[] _LeftKeywords = new string[]
		{
			"left",
			"_l",
		};

		static readonly string[] _RightKeywords = new string[]
		{
			"right",
			"_r",
		};

		static Transform _FindEye( Transform head, bool isRight )
		{
			if( head != null ) {
				string[] keywords = isRight ? _RightKeywords : _LeftKeywords;

				int childCount = head.childCount;
				for( int i = 0; i < childCount; ++i ) {
					Transform child = head.GetChild( i );
					if( child != null ) {
						string name = child.name;
						if( name != null ) {
							name = name.ToLower();
							if( name != null && name.Contains( "eye" ) ) {
								for( int n = 0; n < keywords.Length; ++n ) {
									if( name.Contains( keywords[n] ) ) {
										return child;
									}
								}
							}
						}
					}
				}
            }

			return null;
		}
		
		public void ConfigureBoneTransforms()
		{
			Prefix();
			
			if( _prepareHumanoid ) {
				Animator animator = this.gameObject.GetComponent<Animator>();
				if( animator != null && animator.isHuman ) {
					Transform pelvis = animator.GetBoneTransform( HumanBodyBones.Hips );
					Transform torso = animator.GetBoneTransform( HumanBodyBones.Spine );
					Transform torso2 = animator.GetBoneTransform( HumanBodyBones.Chest );
					Transform neck = animator.GetBoneTransform( HumanBodyBones.Neck );
					Transform head = animator.GetBoneTransform( HumanBodyBones.Head );
					Transform leftEye = animator.GetBoneTransform( HumanBodyBones.LeftEye );
					Transform rightEye = animator.GetBoneTransform( HumanBodyBones.RightEye );
					Transform leftLeg = animator.GetBoneTransform( HumanBodyBones.LeftUpperLeg );
					Transform rightLeg = animator.GetBoneTransform( HumanBodyBones.RightUpperLeg );
					Transform leftKnee = animator.GetBoneTransform( HumanBodyBones.LeftLowerLeg );
					Transform rightKnee = animator.GetBoneTransform( HumanBodyBones.RightLowerLeg );
					Transform leftFoot = animator.GetBoneTransform( HumanBodyBones.LeftFoot );
					Transform rightFoot = animator.GetBoneTransform( HumanBodyBones.RightFoot );
					Transform leftShoulder = animator.GetBoneTransform( HumanBodyBones.LeftShoulder );
					Transform rightShoulder = animator.GetBoneTransform( HumanBodyBones.RightShoulder );
					Transform leftArm = animator.GetBoneTransform( HumanBodyBones.LeftUpperArm );
					Transform rightArm = animator.GetBoneTransform( HumanBodyBones.RightUpperArm );
					Transform leftElbow = animator.GetBoneTransform( HumanBodyBones.LeftLowerArm );
					Transform rightElbow = animator.GetBoneTransform( HumanBodyBones.RightLowerArm );
					Transform leftWrist = animator.GetBoneTransform( HumanBodyBones.LeftHand );
					Transform rightWrist = animator.GetBoneTransform( HumanBodyBones.RightHand );
					Transform[,] leftFingers = new Transform[5, 4];
					Transform[,] rightFingers = new Transform[5, 4];
					for( int n = 0; n < 2; ++n ) {
						int humanBodyBones = ((n == 0) ? (int)HumanBodyBones.LeftThumbProximal : (int)HumanBodyBones.RightThumbProximal);
						Transform[,] fingers = ((n == 0) ? leftFingers : rightFingers);
						for( int i = 0; i < 5; ++i ) {
							for( int j = 0; j < 3; ++j, ++humanBodyBones ) {
								fingers[i, j] = animator.GetBoneTransform( (HumanBodyBones)humanBodyBones );
							}
							// Fix for tips.
							if( fingers[i, 2] != null && fingers[i, 2].childCount != 0 ) {
								fingers[i, 3] = fingers[i, 2].GetChild( 0 );
							}
						}
					}

					if( neck == null ) {
						if( head != null ) {
							Transform t = head.parent;
							if( t != null && _IsNeck( t ) ) {
								neck = t;
							} else {
								neck = head; // Failsafe.
							}
						}
					}

					if( leftEye == null ) {
						leftEye = _FindEye( head, false );
					}
					if( rightEye == null ) {
						rightEye = _FindEye( head, true );
					}

					_SetBoneTransform( ref _bodyBones.pelvis, pelvis );
					_SetBoneTransform( ref _bodyBones.torso, torso );
					_SetBoneTransform( ref _bodyBones.torso2, torso2 );

					_SetBoneTransform( ref _headBones.neck, neck );
					_SetBoneTransform( ref _headBones.head, head );
					_SetBoneTransform( ref _headBones.leftEye, leftEye );
					_SetBoneTransform( ref _headBones.rightEye, rightEye );

					_SetBoneTransform( ref _leftLegBones.leg, leftLeg );
					_SetBoneTransform( ref _leftLegBones.knee, leftKnee );
					_SetBoneTransform( ref _leftLegBones.foot, leftFoot );
					_SetBoneTransform( ref _rightLegBones.leg, rightLeg );
					_SetBoneTransform( ref _rightLegBones.knee, rightKnee );
					_SetBoneTransform( ref _rightLegBones.foot, rightFoot );

					_SetBoneTransform( ref _leftArmBones.shoulder, leftShoulder );
					_SetBoneTransform( ref _leftArmBones.arm, leftArm );
					_SetBoneTransform( ref _leftArmBones.elbow, leftElbow );
					_SetBoneTransform( ref _leftArmBones.wrist, leftWrist );
					_SetBoneTransform( ref _rightArmBones.shoulder, rightShoulder );
					_SetBoneTransform( ref _rightArmBones.arm, rightArm );
					_SetBoneTransform( ref _rightArmBones.elbow, rightElbow );
					_SetBoneTransform( ref _rightArmBones.wrist, rightWrist );

					_SetFingerBoneTransform( ref _leftHandFingersBones.thumb, leftFingers, 0 );
					_SetFingerBoneTransform( ref _leftHandFingersBones.index, leftFingers, 1 );
					_SetFingerBoneTransform( ref _leftHandFingersBones.middle, leftFingers, 2 );
					_SetFingerBoneTransform( ref _leftHandFingersBones.ring, leftFingers, 3 );
					_SetFingerBoneTransform( ref _leftHandFingersBones.little, leftFingers, 4 );

					_SetFingerBoneTransform( ref _rightHandFingersBones.thumb, rightFingers, 0 );
					_SetFingerBoneTransform( ref _rightHandFingersBones.index, rightFingers, 1 );
					_SetFingerBoneTransform( ref _rightHandFingersBones.middle, rightFingers, 2 );
					_SetFingerBoneTransform( ref _rightHandFingersBones.ring, rightFingers, 3 );
					_SetFingerBoneTransform( ref _rightHandFingersBones.little, rightFingers, 4 );
				}
			}

			if( _settings.automaticConfigureTwistEnabled ) {
				var tempBones = new List<Transform>();

				for( int side = 0; side < 2; ++side ) {
					var armBones = (side == 0) ? _leftArmBones : _rightArmBones;
					if( armBones != null &&
						armBones.arm != null && armBones.arm.transform != null &&
						armBones.elbow != null && armBones.elbow.transform != null &&
						armBones.wrist != null && armBones.wrist.transform != null ) {

						armBones.Repair( (Side)side );

						_ConfigureTwistBones( armBones.armTwist, tempBones, armBones.arm.transform, armBones.elbow.transform, (Side)side, true );
						_ConfigureTwistBones( armBones.handTwist, tempBones, armBones.elbow.transform, armBones.wrist.transform, (Side)side, false );
					}
				}
			}
		}

		void _ConfigureTwistBones( Bone[] bones, List<Transform> tempBones, Transform transform, Transform excludeTransform, Side side, bool isArm )
		{
			bool isTwistSpecial = false;
			string twistSpecialName = null;
			if( isArm ) {
				twistSpecialName = (side == Side.Left) ? "LeftArmTwist" : "RightArmTwist";
            } else {
				twistSpecialName = (side == Side.Left) ? "LeftHandTwist" : "RightHandTwist";
			}

			int childCount = transform.childCount;

			for( int i = 0; i < childCount; ++i ) {
				var childTransform = transform.GetChild( i );
				var name = childTransform.name;
				if( name != null && name.Contains( twistSpecialName ) ) {
					isTwistSpecial = true;
					break;
				}
			}

			tempBones.Clear();

			for( int i = 0; i < childCount; ++i ) {
				var childTransform = transform.GetChild( i );
				var name = childTransform.name;
				if( name != null ) {
					if( excludeTransform != childTransform &&
						!excludeTransform.IsChildOf( childTransform ) ) {
						if( isTwistSpecial ) {
							if( name.Contains( twistSpecialName ) ) {
								char nameEnd = name[name.Length - 1];
								if( nameEnd >= '0' && nameEnd <= '9' ) {
									tempBones.Add( childTransform );
								}
							}
						} else {
							tempBones.Add( childTransform );
						}
					}
				}
			}

			childCount = Mathf.Min( tempBones.Count, bones.Length );
			for( int i = 0; i < childCount; ++i ) {
				_SetBoneTransform( ref bones[i], tempBones[i] );
			}
		}

		// - Wakeup for solvers.
		// - Require to setup each transforms.
		public void Prepare()
		{
			Prefix();

			if( this.transform != null ) { // Failsafe.
				_internalValues.defaultRootPosition = this.transform.position;
				_internalValues.defaultRootBasis = Matrix3x3.FromColumn( this.transform.right, this.transform.up, this.transform.forward );
				_internalValues.defaultRootBasisInv = _internalValues.defaultRootBasis.transpose;
				_internalValues.defaultRootRotation = this.transform.rotation;
				_internalValues.rootTransformIsAlive = true;
				_internalValues.rootTransform = this.transform;
			}

			if( _bones != null ) {
				for( int i = 0; i < _bones.Length; ++i ) {
					Assert( _bones[i] != null );
					if( _bones[i] != null ) {
						_bones[i].Prepare( this );
					}
				}
				for( int i = 0; i < _bones.Length; ++i ) {
					Assert( _bones[i] != null );
					if( _bones[i] != null ) {
						_bones[i].PostPrepare();
					}
				}
			}

			if( _effectors != null ) {
				for( int i = 0; i < _effectors.Length; ++i ) {
					Assert( _effectors[i] != null );
					if( _effectors[i] != null ) {
						_effectors[i].Prepare( this );
					}
				}
			}
			
			// Overwrite.
			_bodyIK = new BodyIK( this );

			if( _limbIK == null || _limbIK.Length != (int)LimbIKLocation.Max ) {
				_limbIK = new LimbIK[(int)LimbIKLocation.Max];
			}

			// Overwrite.
			for( int i = 0; i < _limbIK.Length; ++i ) {
				_limbIK[i] = new LimbIK( this, (LimbIKLocation)i );
			}

			_headIK = new HeadIK( this );

			if( _fingerIK == null || _fingerIK.Length != (int)FingerIKType.Max ) {
				_fingerIK = new FingerIK[(int)FingerIKType.Max];
			}

			for( int i = 0; i < _fingerIK.Length; ++i ) {
				_fingerIK[i] = new FingerIK( this, (FingerIKType)i );
            }
		}

		// Caches.
		Animator _animator;
		RuntimeAnimatorController _runtimeAnimatorController;
		Animation _animation; // Legacy support.

		void _UpdateInternalValues()
		{
			// _animatorEnabledImmediately
			if( _settings.animatorEnabled == AutomaticBool.Auto ) {
				_internalValues.animatorEnabled = false;
				if( _animator == null ) {
					_animator = this.GetComponent< Animator >();
				}
				if( _animator != null && _animator.enabled ) {
					_runtimeAnimatorController = _animator.runtimeAnimatorController;
					_internalValues.animatorEnabled = (_runtimeAnimatorController != null);
				}
				if( !_internalValues.animatorEnabled ) { // Legacy support.
					if( _animation == null ) {
						_animation = this.GetComponent< Animation >();
					}
					if( _animation != null && _animation.enabled && _animation.GetClipCount() > 0 ) {
						_internalValues.animatorEnabled = true;
					}
				}
			} else {
				_internalValues.animatorEnabled = (_settings.animatorEnabled != AutomaticBool.Disable);
			}

			if( _settings.resetTransforms == AutomaticBool.Auto ) {
				_internalValues.resetTransforms = !(_internalValues.animatorEnabled);
			} else {
				_internalValues.resetTransforms = (_settings.resetTransforms != AutomaticBool.Disable);
			}

			_internalValues.continuousSolverEnabled = !_internalValues.animatorEnabled && !_internalValues.resetTransforms;
		}

		void LateUpdate()
		{
			if( !Application.isPlaying ) {
				return;
			}

			_internalValues.rootTransformIsAlive = (this.transform != null);

#if SAFULLBODYIK_DEBUG_LOCKTRANSFORM
			this.transform.position = _debug_currentPosition;
#endif

			_UpdateInternalValues();

			if( _effectors != null ) {
				for( int i = 0; i < _effectors.Length; ++i ) {
					if( _effectors[i] != null ) {
						_effectors[i].PrepareUpdate();
					}
				}
			}
			
#if SAFULLBODYIK_DEBUG
			_debugData.debugPoints.Clear();
#endif

			_Bones_PrepareUpdate();

			// Feedback bonePositions to effectorPositions.
			// (for AnimatorEnabled only.)
			if( _effectors != null ) {
				for( int i = 0; i < _effectors.Length; ++i ) {
					if( _effectors[i] != null ) {
						_effectors[i]._hidden_worldPosition = _effectors[i].worldPosition;

						if( _internalValues.animatorEnabled && !_internalValues.resetTransforms ) {
							if( _effectors[i].positionEnabled && _effectors[i].positionWeight < 1.0f - IKEpsilon ) {
								float weight = (_effectors[i].positionWeight > IKEpsilon) ? _effectors[i].positionWeight : 0.0f;
								if( _effectors[i].bone != null && _effectors[i].bone.transformIsAlive ) {
									_effectors[i]._hidden_worldPosition = Vector3.Lerp( _effectors[i].bone.worldPosition, _effectors[i].worldPosition, weight );
								}
							} else if( !_effectors[i].positionEnabled ) {
								if( _effectors[i].bone != null && _effectors[i].bone.transformIsAlive ) {
									_effectors[i]._hidden_worldPosition = _effectors[i].bone.worldPosition;
								}
							}
						}
					}
				}
			}

			// Presolve locations.
			for( int i = 0; i < _limbIK.Length; ++i ) {
				_limbIK[i].PresolveBeinding();
			}

			if( _bodyIK != null ) {
				if( _bodyIK.Solve() ) {
					_Bones_WriteToTransform();
				}
			}

			if( _limbIK != null || _headIK != null ) {
				_Bones_PrepareUpdate();

				bool isSolved = false;
				if( _limbIK != null ) {
					for( int i = 0; i < _limbIK.Length; ++i ) {
						if( _limbIK[i] != null ) {
							if( _limbIK[i].Solve() ) {
								isSolved |= true;
								_limbIK[i].Twist();
							}
						}
					}
				}
				if( _headIK != null ) {
					_headIK.Solve();
					isSolved = true;
                }

				if( isSolved ) {
					_Bones_WriteToTransform();
				}
			}

			if( _fingerIK != null ) {
				_Bones_PrepareUpdate();

				bool isSolved = false;
				for( int i = 0; i < _fingerIK.Length; ++i ) {
					isSolved |= _fingerIK[i].Solve();
				}

				if( isSolved ) {
					_Bones_WriteToTransform();
				}
			}
		}

		void _Bones_PrepareUpdate()
		{
			if( _bones != null ) {
				for( int i = 0; i < _bones.Length; ++i ) {
					if( _bones[i] != null ) {
						_bones[i].PrepareUpdate();
					}
				}
			}
		}

		void _Bones_WriteToTransform()
		{
			if( _bones != null ) {
				for( int i = 0; i < _bones.Length; ++i ) {
					if( _bones[i] != null ) {
						_bones[i].WriteToTransform();
					}
				}
			}
		}

		void OnDestroy()
		{
			if( !Application.isPlaying ) {
				return;
			}
		}

#if UNITY_EDITOR
		void OnDrawGizmos()
		{
			Vector3 cameraForward = Camera.current.transform.forward;

			if( _effectors != null ) {
				for( int i = 0; i != _effectors.Length; ++i ) {
					_DrawEffectorGizmo( _effectors[i] );
				}
			}

			if( _bones != null ) {
				for( int i = 0; i != _bones.Length; ++i ) {
					_DrawBoneGizmo( _bones[i], ref cameraForward );
				}
			}

#if SAFULLBODYIK_DEBUG
			var debugPoints = _debugData.debugPoints;
			for( int i = 0; i < debugPoints.Count; ++i ) {
				Gizmos.color = debugPoints[i].color;
				for( int n = 0; n < 8; ++n ) {
					Gizmos.DrawWireSphere( debugPoints[i].pos, debugPoints[i].size );
				}
			}
#endif
		}

		const float _EffectorGizmoSize = 0.04f;
		const float _FingerEffectorGizmoSize = 0.02f;

		static void _DrawEffectorGizmo( Effector effector )
		{
			if( effector != null ) {
				bool isFinger = (effector.bone != null && effector.bone.boneType == BoneType.HandFinger );
				float effectorSize = (isFinger ? _FingerEffectorGizmoSize : _EffectorGizmoSize);
				Gizmos.color = Color.green;

				Vector3 position = Vector3.zero;
				if( effector.transform != null ) {
					position = effector.transform.position;
				} else {
					position = effector._worldPosition; // Memo: Don't re-write internal flags. (Use _worldPosition directly.)
				}

				Gizmos.DrawWireSphere( position, effectorSize );
				Gizmos.DrawWireSphere( position, effectorSize );
				Gizmos.DrawWireSphere( position, effectorSize );
				Gizmos.DrawWireSphere( position, effectorSize );
			}
		}

		void _DrawBoneGizmo( Bone bone, ref Vector3 cameraForward )
		{
			if( bone == null || bone.transform == null ) {
				return;
			}

			if( bone.boneType == BoneType.Eye ) {
				if( _settings.modelTemplate == ModelTemplate.UnityChan ) {
					return;
				}
			}

			Transform parentTransform = bone.parentTransform;

			Vector3 position = bone.transform.position;
			Matrix3x3 basis = bone.transform.rotation * bone._worldToBoneRotation;

			_DrawTransformGizmo( position, ref basis, ref cameraForward, bone.boneType );

			if( parentTransform != null ) {
				Gizmos.color = Color.white;

				basis = parentTransform.rotation * bone.parentBone._worldToBoneRotation;

				_DrawBoneGizmo( parentTransform.position, position, ref basis, ref cameraForward, bone.boneType );
			}
		}

		static void _DrawTransformGizmo( Vector3 position, ref Matrix3x3 basis, ref Vector3 cameraForward, BoneType boneType )
		{
			Vector3 column0 = basis.column0;
			Vector3 column1 = basis.column1;
			Vector3 column2 = basis.column2;

			// X Axis
			_DrawArrowGizmo( Color.red, ref position, ref column0, ref cameraForward, boneType );
			// Y Axis
			_DrawArrowGizmo( Color.green, ref position, ref column1, ref cameraForward, boneType );
			// Z Axis
			_DrawArrowGizmo( Color.blue, ref position, ref column2, ref cameraForward, boneType );
		}

		const float _ArrowGizmoLowerLength = 0.02f;
		const float _ArrowGizmoLowerWidth = 0.003f;
		const float _ArrowGizmoMiddleLength = 0.0025f;
		const float _ArrowGizmoMiddleWidth = 0.0075f;
		const float _ArrowGizmoUpperLength = 0.05f;
		const float _ArrowGizmoThickness = 0.0002f;
		const int _ArrowGizmoDrawCycles = 8;
		const float _ArrowGizmoEyeScale = 0.5f;
		const float _ArrowGizmoFingerScale = 0.1f;

		static void _DrawArrowGizmo(
			Color color,
			ref Vector3 position,
			ref Vector3 direction,
			ref Vector3 cameraForward,
			BoneType boneType )
		{
			Vector3 nY = Vector3.Cross( cameraForward, direction );
			if( _SafeNormalize( ref nY ) ) {
				Gizmos.color = color;

				float arrowGizmoLowerLength = _ArrowGizmoLowerLength;
				float arrowGizmoLowerWidth = _ArrowGizmoLowerWidth;
				float arrowGizmoMiddleLength = _ArrowGizmoMiddleLength;
				float arrowGizmoMiddleWidth = _ArrowGizmoMiddleWidth;
				float arrowGizmoUpperLength = _ArrowGizmoUpperLength;
				float arrowGizmoThickness = _ArrowGizmoThickness;

				float gizmoScale = 1.0f;
				if( boneType == BoneType.HandFinger ) {
					gizmoScale = _ArrowGizmoFingerScale;
				}
				if( boneType == BoneType.Eye ) {
					gizmoScale = _ArrowGizmoEyeScale;
				}
				if( gizmoScale != 1.0f ) {
					arrowGizmoLowerLength = _ArrowGizmoLowerLength * gizmoScale;
					arrowGizmoLowerWidth = _ArrowGizmoLowerWidth * gizmoScale;
					arrowGizmoMiddleLength = _ArrowGizmoMiddleLength * gizmoScale;
					arrowGizmoMiddleWidth = _ArrowGizmoMiddleWidth * gizmoScale;
					arrowGizmoUpperLength = _ArrowGizmoUpperLength * gizmoScale;
					arrowGizmoThickness = _ArrowGizmoThickness * gizmoScale;
				}

				Vector3 posLower = position + direction * arrowGizmoLowerLength;
				Vector3 posMiddle = position + direction * (arrowGizmoLowerLength + arrowGizmoMiddleLength);
				Vector3 posUpper = position + direction * (arrowGizmoLowerLength + arrowGizmoMiddleLength + arrowGizmoUpperLength);

				Vector3 lowerY = nY * arrowGizmoLowerWidth;
				Vector3 middleY = nY * arrowGizmoMiddleWidth;
				Vector3 thicknessY = nY * arrowGizmoThickness;

				for( int i = 0; i < _ArrowGizmoDrawCycles; ++i ) {
					Gizmos.DrawLine( position, posLower + lowerY );
					Gizmos.DrawLine( position, posLower - lowerY );
					Gizmos.DrawLine( posLower + lowerY, posMiddle + middleY );
					Gizmos.DrawLine( posLower - lowerY, posMiddle - middleY );
					Gizmos.DrawLine( posMiddle + middleY, posUpper );
					Gizmos.DrawLine( posMiddle - middleY, posUpper );
					lowerY -= thicknessY;
					middleY -= thicknessY;
				}
			}
		}

		const float _BoneGizmoOuterLen = 0.015f;
		const float _BoneGizmoThickness = 0.0003f;
		const int _BoneGizmoDrawCycles = 8;
		const float _BoneGizmoFingerScale = 0.25f;

		static void _DrawBoneGizmo( Vector3 fromPosition, Vector3 toPosition, ref Matrix3x3 basis, ref Vector3 cameraForward, BoneType boneType )
		{
			Vector3 dir = toPosition - fromPosition;
			if( _SafeNormalize( ref dir ) ) {
				Vector3 nY = Vector3.Cross( cameraForward, dir );
				if( _SafeNormalize( ref nY ) ) {
					float boneGizmoOuterLen = _BoneGizmoOuterLen;
					float boneGizmoThickness = _BoneGizmoThickness;
					if( boneType == BoneType.HandFinger ) {
						boneGizmoOuterLen = _BoneGizmoOuterLen * _BoneGizmoFingerScale;
						boneGizmoThickness = _BoneGizmoThickness * _BoneGizmoFingerScale;
					}

					Vector3 outerY = nY * boneGizmoOuterLen;
					Vector3 thicknessY = nY * boneGizmoThickness;
					Vector3 interPosition = fromPosition + dir * boneGizmoOuterLen;

					for( int i = 0; i < _BoneGizmoDrawCycles; ++i ) {
						Gizmos.color = (i < _BoneGizmoDrawCycles / 2) ? Color.black : Color.white;
						Gizmos.DrawLine( fromPosition, interPosition + outerY );
						Gizmos.DrawLine( fromPosition, interPosition - outerY );
						Gizmos.DrawLine( interPosition + outerY, toPosition );
						Gizmos.DrawLine( interPosition - outerY, toPosition );
						outerY -= thicknessY;
					}
				}
			}
		}
#endif

		void _Prefix( ref Bone bone, BoneLocation boneLocation, Bone parentBoneLocationBased )
		{
			Assert( _bones != null );
			Bone.Prefix( _bones, ref bone, boneLocation, parentBoneLocationBased );
		}
		
		void _Prefix(
			ref Effector effector,
			EffectorLocation effectorLocation )
		{
			Assert( _effectors != null );
			bool createEffectorTransform = this._settings.createEffectorTransform;
			Effector.Prefix( _effectors, ref effector, effectorLocation, createEffectorTransform, this.transform );
		}
		
		void _Prefix(
			ref Effector effector,
			EffectorLocation effectorLocation,
			Effector parentEffector,
			Bone[] bones )
		{
			_Prefix( ref effector, effectorLocation, parentEffector, (bones != null && bones.Length > 0) ? bones[bones.Length - 1] : null );
		}

		void _Prefix(
			ref Effector effector,
			EffectorLocation effectorLocation,
			Effector parentEffector,
			Bone bone,
			Bone leftBone = null,
			Bone rightBone = null )
		{
			Assert( _effectors != null );
			bool createEffectorTransform = this._settings.createEffectorTransform;
			Effector.Prefix( _effectors, ref effector, effectorLocation, createEffectorTransform, null, parentEffector, bone, leftBone, rightBone );
		}

	}

}