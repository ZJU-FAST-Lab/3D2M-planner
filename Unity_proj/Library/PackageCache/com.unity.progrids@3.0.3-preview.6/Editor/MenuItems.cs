using UnityEditor;
using UnityEditor.ShortcutManagement;
using UnityEngine;

namespace UnityEditor.ProGrids
{
	static class MenuItems
	{
		[MenuItem("Tools/ProGrids/ProGrids Window", false, 15)]
		public static void InitProGrids()
		{
			ProGridsEditor.Init();
			SceneView.RepaintAll();
		}

		[MenuItem("Tools/ProGrids/Close ProGrids", true, 200)]
		public static bool VerifyCloseProGrids()
		{
			return ProGridsEditor.IsEnabled();
		}

		[MenuItem("Tools/ProGrids/Close ProGrids")]
		public static void CloseProGrids()
		{
			ProGridsEditor.Close();
		}

		[MenuItem("Tools/ProGrids/Cycle SceneView Projection", false, 101)]
		public static void CyclePerspective()
		{
			ProGridsEditor.CyclePerspective();
		}

		[MenuItem("Tools/ProGrids/Cycle SceneView Projection", true, 101)]
		[MenuItem("Tools/ProGrids/Increase Grid Size", true, 203)]
		[MenuItem("Tools/ProGrids/Decrease Grid Size", true, 202)]
		public static bool VerifyGridSizeAdjustment()
		{
			return ProGridsEditor.Instance != null;
		}

		[MenuItem("Tools/ProGrids/Decrease Grid Size", false, 202)]
		public static void DecreaseGridSize()
		{
			ProGridsEditor.DecreaseGridSize();
		}

		[MenuItem("Tools/ProGrids/Increase Grid Size", false, 203)]
		public static void IncreaseGridSize()
		{
			ProGridsEditor.IncreaseGridSize();
		}

		[MenuItem("Tools/ProGrids/Nudge Perspective Backward", true, 304)]
		[MenuItem("Tools/ProGrids/Nudge Perspective Forward", true, 305)]
		[MenuItem("Tools/ProGrids/Reset Perspective Nudge", true, 306)]
		public static bool VerifyMenuNudgePerspective()
		{
			return ProGridsEditor.IsEnabled() && !ProGridsEditor.Instance.FullGridEnabled && !ProGridsEditor.Instance.gridIsOrthographic && ProGridsEditor.Instance.GridIsLocked;
		}

		[MenuItem("Tools/ProGrids/Nudge Perspective Backward", false, 304)]
		public static void MenuNudgePerspectiveBackward()
		{
			ProGridsEditor.MenuNudgePerspectiveBackward();
		}

		[MenuItem("Tools/ProGrids/Nudge Perspective Forward", false, 305)]
		public static void MenuNudgePerspectiveForward()
		{
			ProGridsEditor.MenuNudgePerspectiveForward();
		}

		[MenuItem("Tools/ProGrids/Reset Perspective Nudge", false, 306)]
		public static void MenuNudgePerspectiveReset()
		{
			ProGridsEditor.MenuNudgePerspectiveReset();
		}
	}
}
