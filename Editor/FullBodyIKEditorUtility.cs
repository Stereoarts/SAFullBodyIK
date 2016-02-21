// Copyright (c) 2016 Nora
// Released under the MIT license
// http://opensource.org/licenses/mit-license.phpusing
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

namespace SA
{
	public static class FullBodyIKEditorUtility
	{
		public static class GUI
		{
			public static void Toolbar( Object target, ref int toolbarSelected, string[] toolbarContents )
			{
				int tempToolbarSelected = GUILayout.Toolbar( toolbarSelected, toolbarContents );
				if( toolbarSelected != tempToolbarSelected ) {
					Undo.RecordObject( target, target.GetType().Name );
					toolbarSelected = tempToolbarSelected;
					EditorUtility.SetDirty( target );
				}
			}

			public static void FloatField( Object target, string name, ref float value, params GUILayoutOption[] options )
			{
				EditorGUI.BeginChangeCheck();
				float tempValue = EditorGUILayout.FloatField( name, value, options );
				if( EditorGUI.EndChangeCheck() ) {
					Undo.RecordObject( target, target.GetType().Name );
					value = tempValue;
					EditorUtility.SetDirty( target );
				}
			}

			public static void FloatField_Clamp01( Object target, string name, ref float value, params GUILayoutOption[] options )
			{
				EditorGUI.BeginChangeCheck();
				float tempValue = EditorGUILayout.FloatField( name, value, options );
				tempValue = Mathf.Clamp01( tempValue );
				if( EditorGUI.EndChangeCheck() ) {
					Undo.RecordObject( target, target.GetType().Name );
					value = tempValue;
					EditorUtility.SetDirty( target );
				}
			}

			public static void Slider( Object target, string name, ref float value, float minValue, float maxValue, params GUILayoutOption[] options )
			{
				EditorGUI.BeginChangeCheck();
				float tempValue = EditorGUILayout.Slider( name, value, minValue, maxValue, options );
				if( EditorGUI.EndChangeCheck() ) {
					Undo.RecordObject( target, target.GetType().Name );
					value = tempValue;
					EditorUtility.SetDirty( target );
				}
			}

			public static void HorizonalSlider( Object target, ref float value, float minValue, float maxValue, params GUILayoutOption[] options )
			{
				EditorGUI.BeginChangeCheck();
				float tempValue = GUILayout.HorizontalSlider( value, minValue, maxValue, options );
				if( EditorGUI.EndChangeCheck() ) {
					Undo.RecordObject( target, target.GetType().Name );
					value = tempValue;
					EditorUtility.SetDirty( target );
				}
			}

			public static void Toggle( Object target, string name, ref bool selected )
			{
				bool tempSelected = EditorGUILayout.Toggle( name, selected );
				if( selected != tempSelected ) {
					Undo.RecordObject( target, target.GetType().Name );
					selected = tempSelected;
					EditorUtility.SetDirty( target );
				}
			}

			public static void ToggleLeft( Object target, string name, ref bool selected )
			{
				bool tempSelected = EditorGUILayout.ToggleLeft( name, selected );
				if( selected != tempSelected ) {
					Undo.RecordObject( target, target.GetType().Name );
					selected = tempSelected;
					EditorUtility.SetDirty( target );
				}
			}

			public static void ToggleLegacy( Object target, string name, ref bool selected )
			{
				bool tempSelected = GUILayout.Toggle( selected, name );
				if( selected != tempSelected ) {
					Undo.RecordObject( target, target.GetType().Name );
					selected = tempSelected;
					EditorUtility.SetDirty( target );
				}
			}

			public static void ToggleLegacy( Object target, string name, ref bool selected, GUIStyle guiStyle, params GUILayoutOption[] options )
			{
				bool tempSelected = GUILayout.Toggle( selected, name, guiStyle, options );
				if( selected != tempSelected ) {
					Undo.RecordObject( target, target.GetType().Name );
					selected = tempSelected;
					EditorUtility.SetDirty( target );
				}
			}

			public static void ObjectField<TYPE_>( Object target, string name, ref TYPE_ obj, bool allowSceneObjects, params GUILayoutOption[] options )
				where TYPE_ : Object
			{
				TYPE_ tempObj = (TYPE_)EditorGUILayout.ObjectField( name, obj, typeof( TYPE_ ), allowSceneObjects, options );
				if( obj != tempObj ) {
					Undo.RecordObject( target, target.GetType().Name );
					obj = tempObj;
					EditorUtility.SetDirty( target );
				}
			}

			public static void TransformField( Object target, string name, ref Transform obj, bool allowSceneObjects, params GUILayoutOption[] options )
			{
				Transform tempObj = (Transform)EditorGUILayout.ObjectField( name, obj, typeof( Transform ), allowSceneObjects, options );
				if( obj != tempObj ) {
					if( tempObj != null && target is Component ) {
						GameObject go = ((Component)target).gameObject;
						if( !tempObj.IsChildOf( go.transform ) ) {
							return;
						}
					}

					Undo.RecordObject( target, target.GetType().Name );
					obj = tempObj;
					EditorUtility.SetDirty( target );
				}
			}

			public static void Field( string name, ref bool value, params GUILayoutOption[] options )
			{
				value = EditorGUILayout.Toggle( name, value, options );
			}

			public static void Field( string name, ref int value, params GUILayoutOption[] options )
			{
				value = EditorGUILayout.IntField( name, value, options );
			}

			public static void Slider( string name, ref int value, int minValue, int maxValue, params GUILayoutOption[] options )
			{
				value = EditorGUILayout.IntSlider( name, value, minValue, maxValue, options );
			}

			public static void Field( string name, ref float value, params GUILayoutOption[] options )
			{
				value = EditorGUILayout.FloatField( name, value, options );
			}

			public static void Slider( string name, ref float value, float minValue, float maxValue, params GUILayoutOption[] options )
			{
				value = EditorGUILayout.Slider( name, value, minValue, maxValue, options );
			}

			public static void Slider01( string name, ref float value, params GUILayoutOption[] options )
			{
				value = EditorGUILayout.Slider( name, value, 0.0f, 1.0f, options );
			}

			static List<bool> _guiEnabledStack = new List<bool>();

			public static void PushEnabled( bool enabled )
			{
				_guiEnabledStack.Add( UnityEngine.GUI.enabled );
				UnityEngine.GUI.enabled = UnityEngine.GUI.enabled && enabled;
			}

			public static void PopEnabled()
			{
				if( _guiEnabledStack.Count > 0 ) {
					bool enabled = _guiEnabledStack[_guiEnabledStack.Count - 1];
					_guiEnabledStack.RemoveAt( _guiEnabledStack.Count - 1 );
					UnityEngine.GUI.enabled = enabled;
				}
			}
		}
	}
}