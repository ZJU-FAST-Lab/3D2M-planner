namespace UnityEditor.ProGrids
{
	static class PreferenceKeys
	{
		public const string SnapValue = "pgSnapValue";
		public const string SnapMultiplier = "pgSnapMultiplier";
		public const string SnapEnabled = "pgSnapEnabled";
		public const string LastOrthoToggledRotation = "pgLastOrthoToggledRotation";
		public const string BracketIncreaseValue = "pgBracketIncreaseValue";
		public const string GridUnit = "pg_GridUnit";
		public const string LockGrid = "pg_LockGrid";
		public const string LockedGridPivot = "pg_LockedGridPivot";
		public const string GridAxis = "pg_GridAxis";
		public const string PerspGrid = "pg_PerspGrid";
		public const string SnapScale = "pg_SnapOnScale";
		public const string PredictiveGrid = "pg_PredictiveGrid";
		public const string SnapAsGroup = "pg_SnapAsGroup";
		public const string MajorLineIncrement = "pg_MajorLineIncrement";
		public const string SyncUnitySnap = "pg_SyncUnitySnap";
		public const string SnapMethod = "pg_Editor::snapMethod";
		public const string GridColorX = "pg_Editor::gridColorX";
		public const string GridColorY = "pg_Editor::gridColorY";
		public const string GridColorZ = "pg_Editor::gridColorZ";
		public const string AlphaBump = "pg_Editor::AlphaBump";
		public const string ShowGrid = "pg_Editor::ShowGrid";
		public const string IncreaseGridSize = "pg_Editor::IncreaseGridSize";
		public const string DecreaseGridSize = "pg_Editor::DecreaseGridSize";
		public const string NudgePerspectiveBackward = "pg_Editor::NudgePerspectiveBackward";
		public const string NudgePerspectiveForward = "pg_Editor::NudgePerspectiveForward";
		// Previously this only affected the grid offset. 3.0.0 changed it to also affect grid size.
		public const string ResetGridShortcutModifiers = "pg_Editor::NudgePerspectiveReset";
		public const string CyclePerspective = "pg_Editor::CyclePerspective";
		public const string AngleValue = "pg_Editor::AngleValue";
		public const string SnapSettings = "ProGridsEditor::m_SnapSettings";

		public const string ToggleAxisConstraint = "ProGrids::ToggleAxisConstraint";
		public const string ToggleDisable = "ProGrids::ToggleDisable";

		// do not delete these
		public const string UnityMoveSnapX = "MoveSnapX";
		public const string UnityMoveSnapY = "MoveSnapY";
		public const string UnityMoveSnapZ = "MoveSnapZ";
		public const string UnityScaleSnap = "ScaleSnap";

		public const string StoredPreferenceVersion = "pg_Version";
		public const string ProGridsIsEnabled = "pgProGridsIsEnabled";
		public const string ProGridsIsExtended = "pgProGridsIsExtended";
	}
}
