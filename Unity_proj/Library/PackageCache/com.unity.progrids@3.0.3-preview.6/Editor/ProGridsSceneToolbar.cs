using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace UnityEditor.ProGrids
{
	partial class ProGridsEditor
	{
		static class Styles
		{
			static bool s_IsInitialized;

			public static GUIStyle gridButtonStyle = new GUIStyle();
			public static GUIStyle extendoStyle = new GUIStyle();
			public static GUIStyle gridButtonStyleBlank = new GUIStyle();
			public static GUIStyle backgroundStyle = new GUIStyle();

			public static ToggleContent snapToGridContent = new ToggleContent("Snap", "", "Snaps all selected objects to grid.");
			public static ToggleContent gridEnabledContent = new ToggleContent("Hide", "Show", "Toggles drawing of guide lines on or off.  Note that object snapping is not affected by this setting.");
			public static ToggleContent snapEnabledContent = new ToggleContent("On", "Off", "Toggles snapping on or off.");
			public static ToggleContent lockGridContent = new ToggleContent("Lock", "Unlck", "Lock the perspective grid center in place.");
			public static ToggleContent angleEnabledContent = new ToggleContent("> On", "> Off", "If on, ProGrids will draw angled line guides.  Angle is settable in degrees.");
			public static ToggleContent renderPlaneXContent = new ToggleContent("X", "X", "Renders a grid on the X plane.");
			public static ToggleContent renderPlaneYContent = new ToggleContent("Y", "Y", "Renders a grid on the Y plane.");
			public static ToggleContent renderPlaneZContent = new ToggleContent("Z", "Z", "Renders a grid on the Z plane.");
			public static ToggleContent renderPerspectiveGridContent = new ToggleContent("Full", "Plane", "Renders a 3d grid in perspective mode.");
			public static GUIContent extendMenuContent = new GUIContent("", "Show or hide the scene view menu.");
			public static GUIContent snapIncrementContent = new GUIContent("", "Set the snap increment.");

			public static void Init()
			{
				if (s_IsInitialized)
					return;

				s_IsInitialized = true;

				gridEnabledContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_Vis_On.png");
				gridEnabledContent.imageOff = IconUtility.LoadIcon("ProGrids2_GUI_Vis_Off.png");
				snapEnabledContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_Snap_On.png");
				snapEnabledContent.imageOff = IconUtility.LoadIcon("ProGrids2_GUI_Snap_Off.png");
				snapToGridContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_PushToGrid_Normal.png");
				lockGridContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_Lock_On.png");
				lockGridContent.imageOff = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_Lock_Off.png");
				angleEnabledContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_AngleVis_On.png");
				angleEnabledContent.imageOff = IconUtility.LoadIcon("ProGrids2_GUI_AngleVis_Off.png");
				renderPlaneXContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_X_On.png");
				renderPlaneXContent.imageOff = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_X_Off.png");
				renderPlaneYContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_Y_On.png");
				renderPlaneYContent.imageOff = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_Y_Off.png");
				renderPlaneZContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_Z_On.png");
				renderPlaneZContent.imageOff = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_Z_Off.png");
				renderPerspectiveGridContent.imageOn = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_3D_On.png");
				renderPerspectiveGridContent.imageOff = IconUtility.LoadIcon("ProGrids2_GUI_PGrid_3D_Off.png");

				backgroundStyle.normal.background = EditorGUIUtility.whiteTexture;

				Texture2D iconButtonNormal = IconUtility.LoadIcon("ProGrids2_Button_Normal.png");
				Texture2D iconButtonHover = IconUtility.LoadIcon("ProGrids2_Button_Hover.png");

				extendoOpen = IconUtility.LoadIcon("ProGrids2_MenuExtendo_Open.png");
				extendoClosed = IconUtility.LoadIcon("ProGrids2_MenuExtendo_Close.png");

				extendoStyle.normal.background = extendoOpen;
				extendoStyle.hover.background = extendoOpen;

				if (iconButtonNormal == null)
				{
					gridButtonStyleBlank = new GUIStyle("button");
				}
				else
				{
					gridButtonStyleBlank.normal.background = iconButtonNormal;
					gridButtonStyleBlank.hover.background = iconButtonHover;
					gridButtonStyleBlank.normal.textColor = iconButtonNormal != null ? Color.white : Color.black;
					gridButtonStyleBlank.hover.textColor = new Color(.7f, .7f, .7f, 1f);
				}

				gridButtonStyleBlank.padding = new RectOffset(1, 2, 1, 2);
				gridButtonStyleBlank.alignment = TextAnchor.MiddleCenter;
			}

		}

		const int k_MenuItemPadding = 3;
		static readonly Vector2Int k_MenuPosition = new Vector2Int(8, 8);
		const float k_MenuExtendSpeed = 500f;
		const float k_MenuFadeSpeed = 2.5f;

		static Texture2D extendoClosed;
		static Texture2D extendoOpen;

		Rect menuRect = new Rect(k_MenuPosition.x, k_MenuPosition.y, 42, 16);
		Rect backgroundRect = new Rect(00, 0, 0, 0);
		Rect extendoButtonRect = new Rect(0, 0, 0, 0);
		bool menuOpen = true;
		float menuStart = k_MenuPosition.y;
		float deltaTime = 0f;
		float lastTime = 0f;
		float backgroundFade = 1f;
		bool mouseOverMenu = false;
		Color menuBackgroundColor = new Color(0f, 0f, 0f, .5f);
		Color extendoNormalColor = new Color(.9f, .9f, .9f, .7f);
		Color extendoHoverColor = new Color(0f, .66f, .94f, 1f);
		bool extendoButtonHovering = false;
		bool menuIsOrtho = false;

		int SceneToolbarPositionClosedY
		{
			get { return menuIsOrtho ? -192 : -173; }
		}

		internal void UpdateToolbar()
		{
			deltaTime = Time.realtimeSinceStartup - lastTime;
			lastTime = Time.realtimeSinceStartup;

			if ((menuOpen && menuStart < k_MenuPosition.y) || (!menuOpen && menuStart > SceneToolbarPositionClosedY))
			{
				menuStart += deltaTime * k_MenuExtendSpeed * (menuOpen ? 1f : -1f);
				menuStart = Mathf.Clamp(menuStart, SceneToolbarPositionClosedY, k_MenuPosition.y);
				DoGridRepaint();
			}

			float a = menuBackgroundColor.a;
			backgroundFade = (mouseOverMenu || !menuOpen) ? k_MenuFadeSpeed : -k_MenuFadeSpeed;

			menuBackgroundColor.a = Mathf.Clamp(menuBackgroundColor.a + backgroundFade * deltaTime, 0f, .5f);
			extendoNormalColor.a = menuBackgroundColor.a;
			extendoHoverColor.a = (menuBackgroundColor.a / .5f);

			if (!Mathf.Approximately(menuBackgroundColor.a, a))
				DoGridRepaint();
		}

		void DrawSceneToolbar()
		{
			Styles.Init();
			var currentEvent = Event.current;

			// repaint scene gui if mouse is near controls
			if (currentEvent.type == EventType.MouseMove)
			{
				bool tmp = extendoButtonHovering;
				extendoButtonHovering = extendoButtonRect.Contains(currentEvent.mousePosition);

				if (extendoButtonHovering != tmp)
					DoGridRepaint();

				mouseOverMenu = backgroundRect.Contains(currentEvent.mousePosition);
			}

#if !UNITY_2018_2_OR_NEWER
			bool srgb = GL.sRGBWrite;
			GL.sRGBWrite = false;
#endif

			GUI.backgroundColor = menuBackgroundColor;
			backgroundRect.x = menuRect.x - 4;
			backgroundRect.y = 0;
			backgroundRect.width = menuRect.width + 8;
			backgroundRect.height = menuRect.y + menuRect.height + k_MenuItemPadding;
			GUI.Box(backgroundRect, "", Styles.backgroundStyle);

			// when hit testing mouse for showing the background, add some leeway
			backgroundRect.width += 32f;
			backgroundRect.height += 32f;
			GUI.backgroundColor = Color.white;

			menuRect.y = menuStart;

			Styles.snapIncrementContent.text = (SnapValueInGridUnits * SnapMultiplier).ToString("#.####");

			if (GUI.Button(menuRect, Styles.snapIncrementContent, Styles.gridButtonStyleBlank))
			{
#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
				// On Mac ShowAsDropdown and ShowAuxWindow both throw stack pop exceptions when initialized.
				ScenePreferencesWindow options = EditorWindow.GetWindow<ScenePreferencesWindow>(true, "ProGrids Settings", true);
				Rect screenRect = SceneView.lastActiveSceneView.position;
				options.editor = this;
				options.position = new Rect(screenRect.x + menuRect.x + menuRect.width + k_MenuItemPadding,
					screenRect.y + menuRect.y + 24,
					256,
					140);
#else
				ScenePreferencesWindow options = ScriptableObject.CreateInstance<ScenePreferencesWindow>();
				Rect screenRect = SceneView.lastActiveSceneView.position;
				options.editor = this;
				options.ShowAsDropDown(new Rect(screenRect.x + menuRect.x + menuRect.width + k_MenuItemPadding,
												screenRect.y + menuRect.y + 24,
												0,
												0),
												new Vector2(256, 140));
#endif
			}

			menuRect.y += menuRect.height + k_MenuItemPadding;

			// Draw grid
			if (ToggleContent.ToggleButton(menuRect, Styles.gridEnabledContent, m_DrawGrid, Styles.gridButtonStyle, EditorStyles.miniButton))
				SetGridEnabled(!m_DrawGrid);

			menuRect.y += menuRect.height + k_MenuItemPadding;

			// Snap enabled
			if (ToggleContent.ToggleButton(menuRect, Styles.snapEnabledContent, GetSnapEnabled(), Styles.gridButtonStyle, EditorStyles.miniButton))
				SetSnapEnabled(!GetSnapEnabled());

			menuRect.y += menuRect.height + k_MenuItemPadding;

			// Push to grid
			if (ToggleContent.ToggleButton(menuRect, Styles.snapToGridContent, true, Styles.gridButtonStyle, EditorStyles.miniButton))
				SnapToGrid(Selection.transforms);

			menuRect.y += menuRect.height + k_MenuItemPadding;

			// Lock grid
			if (ToggleContent.ToggleButton(menuRect, Styles.lockGridContent, m_GridIsLocked, Styles.gridButtonStyle, EditorStyles.miniButton))
			{
				m_GridIsLocked = !m_GridIsLocked;
				EditorPrefs.SetBool(PreferenceKeys.LockGrid, m_GridIsLocked);
				EditorPrefs.SetString(PreferenceKeys.LockedGridPivot, m_Pivot.ToString());

				// if we've modified the nudge value, reset the pivot here
				if (!m_GridIsLocked)
					GridRenderOffset = 0f;

				ProGridsEditor.DoGridRepaint();
			}

            if (menuIsOrtho)
            {
                menuRect.y += menuRect.height + k_MenuItemPadding;

                if (ToggleContent.ToggleButton(menuRect, Styles.angleEnabledContent, m_DrawAngles, Styles.gridButtonStyle, EditorStyles.miniButton))
                    SetDrawAngles(!m_DrawAngles);
            }

            else
            {
                // perspective toggles

                menuRect.y += menuRect.height + k_MenuItemPadding + 4;

                if (ToggleContent.ToggleButton(menuRect, Styles.renderPlaneXContent, (m_RenderPlane & Axis.X) == Axis.X && !FullGridEnabled,
                    Styles.gridButtonStyle, EditorStyles.miniButton))
                    SetRenderPlane(Axis.X);

                menuRect.y += menuRect.height + k_MenuItemPadding;

                if (ToggleContent.ToggleButton(menuRect, Styles.renderPlaneYContent, (m_RenderPlane & Axis.Y) == Axis.Y && !FullGridEnabled,
                    Styles.gridButtonStyle, EditorStyles.miniButton))
                    SetRenderPlane(Axis.Y);

                menuRect.y += menuRect.height + k_MenuItemPadding;

                if (ToggleContent.ToggleButton(menuRect, Styles.renderPlaneZContent, (m_RenderPlane & Axis.Z) == Axis.Z && !FullGridEnabled,
                    Styles.gridButtonStyle, EditorStyles.miniButton))
                    SetRenderPlane(Axis.Z);

                menuRect.y += menuRect.height + k_MenuItemPadding;

                if (ToggleContent.ToggleButton(menuRect, Styles.renderPerspectiveGridContent, FullGridEnabled, Styles.gridButtonStyle,
                    EditorStyles.miniButton))
                {
                    FullGridEnabled = !FullGridEnabled;
                    EditorPrefs.SetBool(PreferenceKeys.PerspGrid, FullGridEnabled);
                    ProGridsEditor.DoGridRepaint();
                }
            }

            menuRect.y += menuRect.height + k_MenuItemPadding;

            extendoButtonRect.x = menuRect.x;
			extendoButtonRect.y = menuRect.y;
			extendoButtonRect.width = menuRect.width;
			extendoButtonRect.height = menuRect.height;

			GUI.backgroundColor = extendoButtonHovering ? extendoHoverColor : extendoNormalColor;
			Styles.extendMenuContent.text = extendoOpen == null ? (menuOpen ? "Close" : "Open") : "";
			if (GUI.Button(menuRect, Styles.extendMenuContent, extendoOpen ? Styles.extendoStyle : Styles.gridButtonStyleBlank))
			{
				ToggleMenuVisibility();
				extendoButtonHovering = false;
			}

			GUI.backgroundColor = Color.white;
#if !UNITY_2018_2_OR_NEWER
			GL.sRGBWrite = srgb;
#endif
		}

		void ToggleMenuVisibility()
		{
			menuOpen = !menuOpen;
			EditorPrefs.SetBool(PreferenceKeys.ProGridsIsExtended, menuOpen);

			Styles.extendoStyle.normal.background = menuOpen ? extendoClosed : extendoOpen;
			Styles.extendoStyle.hover.background = menuOpen ? extendoClosed : extendoOpen;

			foreach (System.Action<bool> listener in toolbarEventSubscribers)
				listener(menuOpen);

			DoGridRepaint();
		}

		// skip color fading and stuff
		void SetMenuIsExtended(bool isExtended)
		{
			menuOpen = isExtended;
			menuIsOrtho = gridIsOrthographic;
			menuStart = menuOpen ? k_MenuPosition.y : SceneToolbarPositionClosedY;

			menuBackgroundColor.a = 0f;
			extendoNormalColor.a = menuBackgroundColor.a;
			extendoHoverColor.a = (menuBackgroundColor.a / .5f);

			Styles.extendoStyle.normal.background = menuOpen ? extendoClosed : extendoOpen;
			Styles.extendoStyle.hover.background = menuOpen ? extendoClosed : extendoOpen;

			foreach (System.Action<bool> listener in toolbarEventSubscribers)
				listener(menuOpen);

			EditorPrefs.SetBool(PreferenceKeys.ProGridsIsExtended, menuOpen);
		}
	}
}
