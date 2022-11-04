using UnityEngine;
using UnityEditor;
using UnityEditor.SettingsManagement;

namespace UnityEditor.ProGrids
{
	static class Preferences
	{
		static Color s_GridColorX;
		static Color s_GridColorY;
		static Color s_GridColorZ;
		static float s_AlphaBump;
		static SnapMethod s_SnapMethod;
		static float s_BracketIncreaseValue;
		static SnapUnit s_GridUnits;
		static bool s_SyncUnitySnap;

		static KeyCode s_IncreaseGridSize = KeyCode.Equals;
		static KeyCode s_DecreaseGridSize = KeyCode.Minus;
		static KeyCode s_NudgePerspectiveBackward = KeyCode.LeftBracket;
		static KeyCode s_NudgePerspectiveForward = KeyCode.RightBracket;
		static KeyCode s_NudgePerspectiveReset = KeyCode.Alpha0;
		static KeyCode s_CyclePerspective = KeyCode.Backslash;

		static GUIContent s_ResetGridModifiers = new GUIContent("Reset Grid Modifiers", "Reset any grid adjustments made by Nudge or Increase / Decrease Size.");

		static bool s_PrefsLoaded = false;

		[UserSettingBlock("ProGrids")]
		public static void PreferencesGUI(string searchContext)
		{
			if (!s_PrefsLoaded)
				s_PrefsLoaded = LoadPreferences();

			EditorGUI.BeginChangeCheck();

			GUILayout.Label("Snap Behavior", EditorStyles.boldLabel);
			s_AlphaBump = EditorGUILayout.Slider(new GUIContent("Tenth Line Alpha", "Every 10th line will have it's alpha value bumped by this amount."), s_AlphaBump, 0f, 1f);
			s_GridUnits = (SnapUnit)EditorGUILayout.EnumPopup("Grid Units", s_GridUnits);
			s_SnapMethod = (SnapMethod) EditorGUILayout.EnumPopup("Snap Method", s_SnapMethod);
			s_SyncUnitySnap = EditorGUILayout.Toggle("Sync w/ Unity Snap", s_SyncUnitySnap);

			GUILayout.Label("Grid Colors", EditorStyles.boldLabel);
			s_GridColorX = EditorGUILayout.ColorField("X Axis", s_GridColorX);
			s_GridColorY = EditorGUILayout.ColorField("Y Axis", s_GridColorY);
			s_GridColorZ = EditorGUILayout.ColorField("Z Axis", s_GridColorZ);

			GUILayout.Label("Shortcuts", EditorStyles.boldLabel);
			s_IncreaseGridSize = (KeyCode)EditorGUILayout.EnumPopup("Increase Grid Size", s_IncreaseGridSize);
			s_DecreaseGridSize = (KeyCode)EditorGUILayout.EnumPopup("Decrease Grid Size", s_DecreaseGridSize);
			s_NudgePerspectiveBackward = (KeyCode)EditorGUILayout.EnumPopup("Nudge Perspective Backward", s_NudgePerspectiveBackward);
			s_NudgePerspectiveForward = (KeyCode)EditorGUILayout.EnumPopup("Nudge Perspective Forward", s_NudgePerspectiveForward);
			s_NudgePerspectiveReset = (KeyCode)EditorGUILayout.EnumPopup(s_ResetGridModifiers, s_NudgePerspectiveReset);
			s_CyclePerspective = (KeyCode)EditorGUILayout.EnumPopup("Cycle Perspective", s_CyclePerspective);

			if (GUILayout.Button("Reset"))
			{
				if (UnityEditor.EditorUtility.DisplayDialog("Delete ProGrids editor preferences?", "Are you sure you want to delete these? This action cannot be undone.", "Yes", "No"))
					ResetPrefs();
			}

			if(EditorGUI.EndChangeCheck())
				SetPreferences();
		}

		public static bool LoadPreferences()
		{
			s_GridColorX = EditorUtility.GetColorFromJson(EditorPrefs.GetString(PreferenceKeys.GridColorX), Defaults.GridColorX);
			s_GridColorY = EditorUtility.GetColorFromJson(EditorPrefs.GetString(PreferenceKeys.GridColorY), Defaults.GridColorY);
			s_GridColorZ = EditorUtility.GetColorFromJson(EditorPrefs.GetString(PreferenceKeys.GridColorZ), Defaults.GridColorZ);
			s_AlphaBump = EditorPrefs.GetFloat(PreferenceKeys.AlphaBump, Defaults.AlphaBump);
			s_BracketIncreaseValue = EditorPrefs.HasKey(PreferenceKeys.BracketIncreaseValue) ? EditorPrefs.GetFloat(PreferenceKeys.BracketIncreaseValue) : .25f;
			s_GridUnits = (SnapUnit) EditorPrefs.GetInt(PreferenceKeys.GridUnit, 0);
			s_SyncUnitySnap = EditorPrefs.GetBool(PreferenceKeys.SyncUnitySnap, true);
			s_SnapMethod = (SnapMethod) EditorPrefs.GetInt(PreferenceKeys.SnapMethod, (int) Defaults.SnapMethod);

			s_IncreaseGridSize = EditorPrefs.HasKey(PreferenceKeys.IncreaseGridSize)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.IncreaseGridSize)
				: KeyCode.Equals;
			s_DecreaseGridSize = EditorPrefs.HasKey(PreferenceKeys.DecreaseGridSize)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.DecreaseGridSize)
				: KeyCode.Minus;
			s_NudgePerspectiveBackward = EditorPrefs.HasKey(PreferenceKeys.NudgePerspectiveBackward)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.NudgePerspectiveBackward)
				: KeyCode.LeftBracket;
			s_NudgePerspectiveForward = EditorPrefs.HasKey(PreferenceKeys.NudgePerspectiveForward)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.NudgePerspectiveForward)
				: KeyCode.RightBracket;
			s_NudgePerspectiveReset = EditorPrefs.HasKey(PreferenceKeys.ResetGridShortcutModifiers)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.ResetGridShortcutModifiers)
				: KeyCode.Alpha0;
			s_CyclePerspective = EditorPrefs.HasKey(PreferenceKeys.CyclePerspective)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.CyclePerspective)
				: KeyCode.Backslash;

			return true;
		}

		public static void SetPreferences()
		{
			EditorPrefs.SetString(PreferenceKeys.GridColorX, JsonUtility.ToJson(s_GridColorX));
			EditorPrefs.SetString(PreferenceKeys.GridColorY, JsonUtility.ToJson(s_GridColorY));
			EditorPrefs.SetString(PreferenceKeys.GridColorZ, JsonUtility.ToJson(s_GridColorZ));
			EditorPrefs.SetFloat(PreferenceKeys.AlphaBump, s_AlphaBump);
			EditorPrefs.SetFloat(PreferenceKeys.BracketIncreaseValue, s_BracketIncreaseValue);
			EditorPrefs.SetInt(PreferenceKeys.GridUnit, (int)s_GridUnits);
			EditorPrefs.SetBool(PreferenceKeys.SyncUnitySnap, s_SyncUnitySnap);
			EditorPrefs.SetInt(PreferenceKeys.IncreaseGridSize, (int)s_IncreaseGridSize);
			EditorPrefs.SetInt(PreferenceKeys.DecreaseGridSize, (int)s_DecreaseGridSize);
			EditorPrefs.SetInt(PreferenceKeys.NudgePerspectiveBackward, (int)s_NudgePerspectiveBackward);
			EditorPrefs.SetInt(PreferenceKeys.NudgePerspectiveForward, (int)s_NudgePerspectiveForward);
			EditorPrefs.SetInt(PreferenceKeys.ResetGridShortcutModifiers, (int)s_NudgePerspectiveReset);
			EditorPrefs.SetInt(PreferenceKeys.CyclePerspective, (int)s_CyclePerspective);
			EditorPrefs.SetInt(PreferenceKeys.SnapMethod, (int) s_SnapMethod);

			if (ProGridsEditor.Instance != null)
			{
				GridRenderer.LoadPreferences();
				ProGridsEditor.Instance.LoadPreferences();
				ProGridsEditor.DoGridRepaint();
			}
		}

		public static void ResetPrefs()
		{
			EditorPrefs.DeleteKey(PreferenceKeys.SnapValue);
			EditorPrefs.DeleteKey(PreferenceKeys.SnapMultiplier);
			EditorPrefs.DeleteKey(PreferenceKeys.SnapEnabled);
			EditorPrefs.DeleteKey(PreferenceKeys.LastOrthoToggledRotation);
			EditorPrefs.DeleteKey(PreferenceKeys.BracketIncreaseValue);
			EditorPrefs.DeleteKey(PreferenceKeys.GridUnit);
			EditorPrefs.DeleteKey(PreferenceKeys.LockGrid);
			EditorPrefs.DeleteKey(PreferenceKeys.LockedGridPivot);
			EditorPrefs.DeleteKey(PreferenceKeys.GridAxis);
			EditorPrefs.DeleteKey(PreferenceKeys.PerspGrid);
			EditorPrefs.DeleteKey(PreferenceKeys.SnapScale);
			EditorPrefs.DeleteKey(PreferenceKeys.PredictiveGrid);
			EditorPrefs.DeleteKey(PreferenceKeys.SnapAsGroup);
			EditorPrefs.DeleteKey(PreferenceKeys.MajorLineIncrement);
			EditorPrefs.DeleteKey(PreferenceKeys.SyncUnitySnap);
			EditorPrefs.DeleteKey(PreferenceKeys.SnapMethod);
			EditorPrefs.DeleteKey(PreferenceKeys.GridColorX);
			EditorPrefs.DeleteKey(PreferenceKeys.GridColorY);
			EditorPrefs.DeleteKey(PreferenceKeys.GridColorZ);
			EditorPrefs.DeleteKey(PreferenceKeys.AlphaBump);
			EditorPrefs.DeleteKey(PreferenceKeys.ShowGrid);
			EditorPrefs.DeleteKey(PreferenceKeys.IncreaseGridSize);
			EditorPrefs.DeleteKey(PreferenceKeys.DecreaseGridSize);
			EditorPrefs.DeleteKey(PreferenceKeys.NudgePerspectiveBackward);
			EditorPrefs.DeleteKey(PreferenceKeys.NudgePerspectiveForward);
			EditorPrefs.DeleteKey(PreferenceKeys.ResetGridShortcutModifiers);
			EditorPrefs.DeleteKey(PreferenceKeys.CyclePerspective);

			LoadPreferences();
		}
	}
}
