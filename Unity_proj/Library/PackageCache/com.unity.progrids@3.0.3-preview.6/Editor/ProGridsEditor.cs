#if UNITY_2019_1_OR_NEWER
#define SHORTCUT_MANAGER
#endif

using System;
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Reflection;
using System.Linq;
#if SHORTCUT_MANAGER
using UnityEditor.ShortcutManagement;
#endif

namespace UnityEditor.ProGrids
{
	[InitializeOnLoad]
	static class ProGridsInitializer
	{
		/// <summary>
		/// Remembers whether or not ProGrids was open when Unity was shut down last. This happens when Unity opens.
		/// </summary>
		static ProGridsInitializer()
		{
			ProGridsEditor.InitIfEnabled();
			PlayModeStateListener.onEnterEditMode += ProGridsEditor.InitIfEnabled;
			PlayModeStateListener.onEnterPlayMode += ProGridsEditor.DestroyIfEnabled;
		}
	}

	partial class ProGridsEditor
	{
#region Properties

		// used to reset progrids preferences when necessary
		const int k_CurrentPreferencesVersion = 22;

		static ProGridsEditor s_Instance;
		SnapSettings m_SnapSettings;

		bool m_GridIsLocked = false;
		bool m_DrawGrid = true;
		bool m_DrawAngles = false;
		bool m_DoGridRepaint = true;
		Axis m_RenderPlane = Axis.Y;
		float m_AngleValue = 45f;
		bool m_PredictiveGrid = true;
		bool m_GuiResourcesInitialized;


		float m_AlphaBump = .25f;
		Transform m_LastActiveTransform;
		Vector3 m_LastPosition = Vector3.zero;
		Vector3 m_LastScale = Vector3.one;

		bool m_savedFullGrid;
		Axis m_savedAxis;

#if SHORTCUT_MANAGER

		[ClutchShortcut("ProGrids/Toggle Snap on All Axes", typeof(SceneView), KeyCode.S)]
		static void ToggleSnapOnAllAxes(ShortcutArguments args)
		{
			if (!IsEnabled())
				return;

			switch (args.stage)
			{
				case ShortcutStage.Begin:
					s_Instance.m_ToggleAxisConstraint = true;
					break;

				case ShortcutStage.End:
					s_Instance.m_ToggleAxisConstraint = false;
					break;
			}
		}

		[ClutchShortcut("ProGrids/Disable Snap", KeyCode.D)]
		static void TempDisableSnap(ShortcutArguments args)
		{
			if (!IsEnabled())
				return;

			switch (args.stage)
			{
				case ShortcutStage.Begin:
					s_Instance.m_ToggleTempSnap = true;
					break;

				case ShortcutStage.End:
					s_Instance.m_ToggleTempSnap = false;
					break;
			}
		}
#else
		KeyCode m_AxisConstraintKey = KeyCode.S;
		KeyCode m_TempDisableKey = KeyCode.D;

		KeyCode m_IncreaseGridSizeShortcut = KeyCode.Equals;
		KeyCode m_DecreaseGridSizeShortcut = KeyCode.Minus;
		KeyCode m_NudgePerspectiveBackwardShortcut = KeyCode.LeftBracket;
		KeyCode m_NudgePerspectiveForwardShortcut = KeyCode.RightBracket;
		KeyCode m_ResetGridShortcutModifiers = KeyCode.Alpha0;
		KeyCode m_CyclePerspectiveShortcut = KeyCode.Backslash;
#endif

		bool m_ToggleAxisConstraint = false;
		bool m_ToggleTempSnap = false;
		Vector3 m_Pivot = Vector3.zero;
		Vector3 m_CameraDirection = Vector3.zero;
		Vector3 m_PreviousCameraDirection = Vector3.zero;
		// Distance from camera to m_Pivot at the last time the grid mesh was updated.
		float m_LastDistanceCameraToPivot = 0f;
		public float GridRenderOffset { get; set; }
		public bool gridIsOrthographic { get; private set; }
		bool m_IsFirstMove = true;
		float m_PlaneGridDrawDistance = 0f;
		bool m_IsEnabled;
		SnapMethod m_SnapMethod;

		internal static ProGridsEditor Instance
		{
			get { return s_Instance; }
		}

		internal float AlphaBump
		{
			get { return m_AlphaBump; }

			set
			{
				EditorPrefs.SetFloat(PreferenceKeys.AlphaBump, value);
				m_AlphaBump = value;
			}
		}

		internal float AngleValue
		{
			get { return m_AngleValue; }

			set
			{
				EditorPrefs.SetFloat(PreferenceKeys.AngleValue, value);
				m_AngleValue = value;
			}
		}

		internal bool PredictiveGrid
		{
			get { return m_PredictiveGrid; }

			set
			{
				EditorPrefs.SetBool(PreferenceKeys.PredictiveGrid, value);
				m_PredictiveGrid = value;
			}
		}

		internal bool FullGridEnabled { get; private set; }

		internal bool SnapAsGroupEnabled
		{
			get { return m_SnapSettings.SnapAsGroup; }

			set
			{
				m_SnapSettings.SnapAsGroup = value;
				EditorPrefs.SetString(PreferenceKeys.SnapSettings, JsonUtility.ToJson(m_SnapSettings));
			}
		}

		internal bool GetSnapEnabled()
		{
			return m_IsEnabled && (m_ToggleTempSnap ? !m_SnapSettings.SnapEnabled : m_SnapSettings.SnapEnabled);
		}

		internal void SetSnapEnabled(bool isEnabled)
		{
			m_SnapSettings.SnapEnabled = isEnabled;
			EditorPrefs.SetString(PreferenceKeys.SnapSettings, JsonUtility.ToJson(m_SnapSettings));
			if(isEnabled)
				ResetActiveTransformValues();
		}

		internal bool ScaleSnapEnabled
		{
			get { return m_SnapSettings.ScaleSnapEnabled; }

			set
			{
				m_SnapSettings.ScaleSnapEnabled = value;
				EditorPrefs.SetString(PreferenceKeys.SnapSettings, JsonUtility.ToJson(m_SnapSettings));
			}
		}

		internal float SnapMultiplier
		{
			get { return m_SnapSettings.SnapMultiplierFrac(); }
		}

		internal int SnapModifier
		{
			get { return m_SnapSettings.SnapMultiplier; }
			set
			{
				m_SnapSettings.SnapMultiplier = value;
				EditorPrefs.SetString(PreferenceKeys.SnapSettings, JsonUtility.ToJson(m_SnapSettings));
			}
		}

		/// <summary>
		/// Stores the snap value as set by the user interface. This is not multiplied by the grid unit or -+ key modifiers.
		/// </summary>
		/// <remarks>
		/// To get the actual value used to snap objects in the scene, use SnapValueInUnityUnits.
		/// </remarks>
		internal float SnapValueInGridUnits
		{
			get { return m_SnapSettings.SnapValue; }

			set
			{
				m_SnapSettings.SnapValue = value;
				EditorPrefs.SetString(PreferenceKeys.SnapSettings, JsonUtility.ToJson(m_SnapSettings));

				if (EditorPrefs.GetBool(PreferenceKeys.SyncUnitySnap, true))
				{
					float unitySnapUnit = m_SnapSettings.SnapValueInUnityUnits();

					EditorPrefs.SetFloat(PreferenceKeys.UnityMoveSnapX, unitySnapUnit);
					EditorPrefs.SetFloat(PreferenceKeys.UnityMoveSnapY, unitySnapUnit);
					EditorPrefs.SetFloat(PreferenceKeys.UnityMoveSnapZ, unitySnapUnit);

					if (EditorPrefs.GetBool(PreferenceKeys.SnapScale, true))
						EditorPrefs.SetFloat(PreferenceKeys.UnityScaleSnap, unitySnapUnit);

					// If Unity snap sync is enabled, refresh the Snap Settings window if it's open.
					Type snapSettings = typeof(EditorWindow).Assembly.GetType("UnityEditor.SnapSettings");

					if (snapSettings != null)
					{
						FieldInfo snapInitialized = snapSettings.GetField("s_Initialized", BindingFlags.NonPublic | BindingFlags.Static);

						if (snapInitialized != null)
						{
							snapInitialized.SetValue(null, (object) false);

							EditorWindow win = Resources.FindObjectsOfTypeAll<EditorWindow>().FirstOrDefault(x => x.ToString().Contains("SnapSettings"));

							if (win != null)
								win.Repaint();
						}
					}
				}
			}
		}

		/// <summary>
		/// Stores the value that positions are rounded to when snapping in the editor. This is the result of:
		/// `SnapValueInGridUnits * GridUnit * BracketKeyMultiplier`
		/// </summary>
		internal float SnapValueInUnityUnits
		{
			get { return m_SnapSettings.SnapValueInUnityUnits(); }
		}

		internal SnapMethod SnapMethod
		{
			get { return m_SnapMethod; }

			set
			{
				EditorPrefs.SetInt(PreferenceKeys.SnapMethod, (int) value);
				m_SnapMethod = value;
			}
		}

		internal bool GridIsLocked
		{
			get { return m_GridIsLocked; }

			set
			{
				EditorPrefs.SetBool(PreferenceKeys.LockGrid, value);
				m_GridIsLocked = value;
			}
		}

#endregion

#region Menu Actions

#if SHORTCUT_MANAGER
		[Shortcut("ProGrids/Increase Grid Size", typeof(SceneView), KeyCode.Equals)]
#endif
		internal static void IncreaseGridSize()
		{
			if (!IsEnabled())
				return;
			var settings = Instance.m_SnapSettings;
			if (settings.SnapMultiplier < int.MaxValue / 2)
				settings.SnapMultiplier *= 2;
			EditorPrefs.SetString(PreferenceKeys.SnapSettings, JsonUtility.ToJson(settings));
			DoGridRepaint();
		}

#if SHORTCUT_MANAGER
		[Shortcut("ProGrids/Decrease Grid Size", typeof(SceneView), KeyCode.Minus)]
#endif
		internal static void DecreaseGridSize()
		{
			if (!IsEnabled())
				return;
			var settings = Instance.m_SnapSettings;
			if (settings.SnapMultiplier > 1)
				settings.SnapMultiplier /= 2;
			EditorPrefs.SetString(PreferenceKeys.SnapSettings, JsonUtility.ToJson(settings));
			DoGridRepaint();
		}

#if SHORTCUT_MANAGER
		[Shortcut("ProGrids/Reset Grid", typeof(SceneView), KeyCode.Alpha0)]
#endif
		internal static void ResetGrid()
		{
			if (!IsEnabled())
				return;

			MenuNudgePerspectiveReset();
			ResetGridSize();
		}

		internal static void ResetGridSize()
		{
			if (!IsEnabled())
				return;
			var settings = Instance.m_SnapSettings;
			settings.SnapMultiplier = Defaults.DefaultSnapMultiplier;
			EditorPrefs.SetString(PreferenceKeys.SnapSettings, JsonUtility.ToJson(settings));
			DoGridRepaint();
		}

#if SHORTCUT_MANAGER
		[Shortcut("ProGrids/Nudge Perspective Backward", typeof(SceneView), KeyCode.LeftBracket)]
#endif
		internal static void MenuNudgePerspectiveBackward()
		{
			if (!IsEnabled() || !Instance.m_GridIsLocked)
				return;
			Instance.GridRenderOffset -= Instance.m_SnapSettings.SnapValueInUnityUnits();
			DoGridRepaint();
		}

#if SHORTCUT_MANAGER
		[Shortcut("ProGrids/Nudge Perspective Forward", typeof(SceneView), KeyCode.RightBracket)]
#endif
		internal static void MenuNudgePerspectiveForward()
		{
			if (!IsEnabled() || !Instance.m_GridIsLocked)
				return;
			Instance.GridRenderOffset += Instance.m_SnapSettings.SnapValueInUnityUnits();
			DoGridRepaint();
		}

		internal static void MenuNudgePerspectiveReset()
		{
			if (!IsEnabled() || !Instance.m_GridIsLocked)
				return;

			Instance.GridRenderOffset = 0;
			DoGridRepaint();
		}

#if SHORTCUT_MANAGER
		[Shortcut("ProGrids/Cycle Perspective", typeof(SceneView), KeyCode.Backslash)]
#endif
		internal static void CyclePerspective()
		{
			if (!IsEnabled())
				return;

			SceneView scnvw = SceneView.lastActiveSceneView;

			if (scnvw == null)
				return;

			int nextOrtho = EditorPrefs.GetInt(PreferenceKeys.LastOrthoToggledRotation);

			switch (nextOrtho)
			{
				case 0:
					scnvw.orthographic = true;
					scnvw.LookAt(scnvw.pivot, Quaternion.Euler(Vector3.zero));
					nextOrtho++;
					break;

				case 1:
					scnvw.orthographic = true;
					scnvw.LookAt(scnvw.pivot, Quaternion.Euler(Vector3.up * -90f));
					nextOrtho++;
					break;

				case 2:
					scnvw.orthographic = true;
					scnvw.LookAt(scnvw.pivot, Quaternion.Euler(Vector3.right * 90f));
					nextOrtho++;
					break;

				case 3:
					scnvw.orthographic = false;
					scnvw.LookAt(scnvw.pivot, new Quaternion(-0.1f, 0.9f, -0.2f, -0.4f));
					nextOrtho = 0;
					break;
			}

			EditorPrefs.SetInt(PreferenceKeys.LastOrthoToggledRotation, nextOrtho);
		}
#endregion

#region INITIALIZATION / SERIALIZATION

		internal static void InitIfEnabled()
		{
			if (!EditorApplication.isPlayingOrWillChangePlaymode && EditorPrefs.GetBool(PreferenceKeys.ProGridsIsEnabled))
				Init();
		}

		internal static void DestroyIfEnabled()
		{
			if(s_Instance != null)
				s_Instance.Destroy();
		}

		internal static void Init()
		{
			EditorPrefs.SetBool(PreferenceKeys.ProGridsIsEnabled, true);

			if (s_Instance == null)
				new ProGridsEditor().Initialize();
			else
				s_Instance.Initialize();
		}

		~ProGridsEditor()
		{
			EditorApplication.delayCall += Destroy;
		}

		void Initialize()
		{
			s_Instance = this;
			RegisterDelegates();

			// this can fail on the first import to a new project when the toolbar was opened in a previous project.
			// the editor scripts compile and run before unity has a chance to load the resources, resulting in no assets
			// being loaded and the toolbar not rendering properly. don't throw an error because it only occurs in a very
			// specific scenario, and recovers immediately (it will reload correctly after the assets finish importing)
			if (!GridRenderer.Init())
			{
				Destroy();
				return;
			}

			LoadPreferences();
			lastTime = Time.realtimeSinceStartup;
			SetMenuIsExtended(menuOpen);
			OnSelectionChange();

			if (m_DrawGrid)
				EditorUtility.SetUnityGridEnabled(false);

			DoGridRepaint();
		}

		internal static void Close()
		{
			EditorPrefs.SetBool(PreferenceKeys.ProGridsIsEnabled, false);

			if(s_Instance != null)
				s_Instance.Destroy();
		}

		void Destroy()
		{
			GridRenderer.Destroy();
			UnregisterDelegates();
			foreach (Action<bool> listener in toolbarEventSubscribers)
				listener(false);
			EditorUtility.SetUnityGridEnabled(true);
			s_Instance = null;
			SceneView.RepaintAll();
		}

		internal static bool IsEnabled()
		{
			return s_Instance != null && s_Instance.m_IsEnabled;
		}

		void RegisterDelegates()
		{
			UnregisterDelegates();

#if UNITY_2019_1_OR_NEWER
			SceneView.duringSceneGui += OnSceneGUI;
			SceneView.duringSceneGui += DrawSceneGrid;
#else
			SceneView.onSceneGUIDelegate += OnSceneGUI;
			SceneView.onSceneGUIDelegate += DrawSceneGrid;
#endif
			EditorApplication.update += UpdateToolbar;
			Selection.selectionChanged += OnSelectionChange;
			Undo.undoRedoPerformed += ResetActiveTransformValues;

			m_IsEnabled = true;
		}

		void UnregisterDelegates()
		{
			m_IsEnabled = false;

#if UNITY_2019_1_OR_NEWER
			SceneView.duringSceneGui -= OnSceneGUI;
			SceneView.duringSceneGui -= DrawSceneGrid;
#else
			SceneView.onSceneGUIDelegate -= OnSceneGUI;
			SceneView.onSceneGUIDelegate -= DrawSceneGrid;
#endif
			EditorApplication.update -= UpdateToolbar;
			Selection.selectionChanged -= OnSelectionChange;
			Undo.undoRedoPerformed -= ResetActiveTransformValues;
		}

		internal void LoadPreferences()
		{
			if (EditorPrefs.GetInt(PreferenceKeys.StoredPreferenceVersion, k_CurrentPreferencesVersion) != k_CurrentPreferencesVersion)
			{
				EditorPrefs.SetInt(PreferenceKeys.StoredPreferenceVersion, k_CurrentPreferencesVersion);
				Preferences.ResetPrefs();
			}

			string snapSettingsJson = EditorPrefs.GetString(PreferenceKeys.SnapSettings);
			m_SnapSettings = new SnapSettings();
			JsonUtility.FromJsonOverwrite(snapSettingsJson, m_SnapSettings);

			m_SnapMethod = (SnapMethod) EditorPrefs.GetInt(PreferenceKeys.SnapMethod, (int) Defaults.SnapMethod);

#if !SHORTCUT_MANAGER

			m_AxisConstraintKey = EditorPrefs.HasKey(PreferenceKeys.ToggleAxisConstraint)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.ToggleAxisConstraint)
				: KeyCode.S;
			m_TempDisableKey = EditorPrefs.HasKey(PreferenceKeys.ToggleDisable)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.ToggleDisable)
				: KeyCode.D;
			m_IncreaseGridSizeShortcut = EditorPrefs.HasKey(PreferenceKeys.IncreaseGridSize)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.IncreaseGridSize)
				: KeyCode.Equals;
			m_DecreaseGridSizeShortcut = EditorPrefs.HasKey(PreferenceKeys.DecreaseGridSize)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.DecreaseGridSize)
				: KeyCode.Minus;
			m_NudgePerspectiveBackwardShortcut = EditorPrefs.HasKey(PreferenceKeys.NudgePerspectiveBackward)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.NudgePerspectiveBackward)
				: KeyCode.LeftBracket;
			m_NudgePerspectiveForwardShortcut = EditorPrefs.HasKey(PreferenceKeys.NudgePerspectiveForward)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.NudgePerspectiveForward)
				: KeyCode.RightBracket;
			m_ResetGridShortcutModifiers = EditorPrefs.HasKey(PreferenceKeys.ResetGridShortcutModifiers)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.ResetGridShortcutModifiers)
				: KeyCode.Alpha0;
			m_CyclePerspectiveShortcut = EditorPrefs.HasKey(PreferenceKeys.CyclePerspective)
				? (KeyCode)EditorPrefs.GetInt(PreferenceKeys.CyclePerspective)
				: KeyCode.Backslash;
#endif

			m_GridIsLocked = EditorPrefs.GetBool(PreferenceKeys.LockGrid);

			menuOpen = EditorPrefs.GetBool(PreferenceKeys.ProGridsIsExtended, true);

			if (m_GridIsLocked)
			{
				if (EditorPrefs.HasKey(PreferenceKeys.LockedGridPivot))
				{
					string piv = EditorPrefs.GetString(PreferenceKeys.LockedGridPivot);
					string[] pivsplit = piv.Replace("(", "").Replace(")", "").Split(',');

					float x, y, z;

					if (float.TryParse(pivsplit[0], out x) &&
					    float.TryParse(pivsplit[1], out y) &&
					    float.TryParse(pivsplit[2], out z))
					{
						m_Pivot.x = x;
						m_Pivot.y = y;
						m_Pivot.z = z;
					}
				}
			}

			FullGridEnabled = EditorPrefs.GetBool(PreferenceKeys.PerspGrid);

			m_RenderPlane = EditorPrefs.HasKey(PreferenceKeys.GridAxis) ? (Axis) EditorPrefs.GetInt(PreferenceKeys.GridAxis) : Axis.Y;
			m_DrawGrid = EditorPrefs.GetBool(PreferenceKeys.ShowGrid, Defaults.ShowGrid);
			m_PredictiveGrid = EditorPrefs.GetBool(PreferenceKeys.PredictiveGrid, Defaults.PredictiveGrid);
		}
#endregion

#region ONSCENEGUI

		internal static void DoGridRepaint()
		{
			if (Instance != null)
			{
				Instance.m_DoGridRepaint = true;
				SceneView.RepaintAll();
			}
		}

		void OnSceneGUI(SceneView view)
		{
			var currentEvent = Event.current;

#if !SHORTCUT_MANAGER
			HandleSingleKeyShortcuts(currentEvent);
#endif

			if (view == SceneView.lastActiveSceneView)
			{
				Handles.BeginGUI();
				DrawSceneToolbar();
				Handles.EndGUI();
			}

			if (!EditorApplication.isPlayingOrWillChangePlaymode)
				DoTransformSnapping(currentEvent, view.camera);
		}

		void DrawSceneGrid(SceneView view)
		{
			var currentEvent = Event.current;

			if (m_DrawGrid && (currentEvent.type == EventType.Repaint || m_DoGridRepaint))
			{
				Vector3 previousPivot = m_Pivot;
				CalculateGridPlacement(view);

				if (gridIsOrthographic)
				{
                    Axis camAxis = EnumExtension.AxisWithVector(Camera.current.transform.TransformDirection(Vector3.forward).normalized);
                    if(m_RenderPlane != camAxis)
	                    SetRenderPlane(camAxis);
                    GridRenderer.DrawOrthographic(view.camera, camAxis, SnapValueInUnityUnits, m_DrawAngles ? AngleValue : -1f);
				}
				else
				{
					float camDistance = Vector3.Distance(view.camera.transform.position, previousPivot);

					if (m_DoGridRepaint
						|| m_Pivot != previousPivot
						|| Mathf.Abs(camDistance - m_LastDistanceCameraToPivot) > m_LastDistanceCameraToPivot / 2
						|| m_CameraDirection != m_PreviousCameraDirection)
					{
						m_PreviousCameraDirection = m_CameraDirection;
						m_DoGridRepaint = false;
						m_LastDistanceCameraToPivot = camDistance;

						if (FullGridEnabled)
							GridRenderer.SetPerspective3D(view.camera, m_Pivot, m_SnapSettings.SnapValueInUnityUnits());
						else
							m_PlaneGridDrawDistance = GridRenderer.SetPerspective(view.camera, m_RenderPlane, m_SnapSettings.SnapValueInUnityUnits(), m_Pivot, GridRenderOffset);
					}

					GridRenderer.Repaint();
				}
			}
		}

		void CalculateGridPlacement(SceneView view)
		{
			var cam = view.camera;

			bool wasOrtho = gridIsOrthographic;

			gridIsOrthographic = cam.orthographic && Snapping.IsRounded(view.rotation.eulerAngles.normalized);

			m_CameraDirection = Snapping.Sign(m_Pivot - cam.transform.position);

			if (wasOrtho != gridIsOrthographic)
			{
				m_DoGridRepaint = true;

				if (view == SceneView.lastActiveSceneView && gridIsOrthographic != menuIsOrtho)
				{
					if (gridIsOrthographic)
					{
						m_savedAxis = (Axis)EditorPrefs.GetInt(PreferenceKeys.GridAxis);
						m_savedFullGrid = EditorPrefs.GetBool(PreferenceKeys.PerspGrid);
					}
					else
					{
						SetRenderPlane(m_savedAxis);
						FullGridEnabled = m_savedFullGrid;
					}
					SetMenuIsExtended(menuOpen);
				}
			}

			if (gridIsOrthographic)
				return;

			if (FullGridEnabled)
			{
				m_Pivot = m_GridIsLocked || Selection.activeTransform == null ? m_Pivot : Selection.activeTransform.position;
			}
			else
			{
				Vector3 sceneViewPlanePivot = m_Pivot;

				Ray ray = new Ray(cam.transform.position, cam.transform.forward);
				Plane plane = new Plane(Vector3.up, m_Pivot);

				// the only time a locked grid should ever move is if it's m_Pivot is out
				// of the camera's frustum.
				if ((m_GridIsLocked && !cam.InFrustum(m_Pivot)) || !m_GridIsLocked || view != SceneView.lastActiveSceneView)
				{
					float dist;

					if (plane.Raycast(ray, out dist))
						sceneViewPlanePivot = ray.GetPoint(Mathf.Min(dist, m_PlaneGridDrawDistance / 2f));
					else
						sceneViewPlanePivot = ray.GetPoint(Mathf.Min(cam.farClipPlane / 2f, m_PlaneGridDrawDistance / 2f));
				}

				if (m_GridIsLocked)
				{
					m_Pivot = EnumExtension.InverseAxisMask(sceneViewPlanePivot, m_RenderPlane) + EnumExtension.AxisMask(m_Pivot, m_RenderPlane);
				}
				else
				{
					m_Pivot = Selection.activeTransform == null ? m_Pivot : Selection.activeTransform.position;

					if (Selection.activeTransform == null || !cam.InFrustum(m_Pivot))
					{
						m_Pivot = EnumExtension.InverseAxisMask(sceneViewPlanePivot, m_RenderPlane) + EnumExtension.AxisMask(Selection.activeTransform == null ? m_Pivot : Selection.activeTransform.position, m_RenderPlane);
					}
				}
			}
		}

		void DoTransformSnapping(Event currentEvent, Camera camera)
		{
			if (currentEvent.type == EventType.MouseUp)
				m_IsFirstMove = true;

			Transform selected = m_LastActiveTransform;

			if (selected == null)
				return;

			bool positionMoved = !Snapping.Approx(m_LastActiveTransform.position, m_LastPosition);
			bool scaleMoved = !Snapping.Approx(m_LastActiveTransform.localScale, m_LastScale);

			if (!GetSnapEnabled() || !EditorUtility.SnapIsEnabled(selected) || GUIUtility.hotControl < 1)
			{
				m_LastPosition = m_LastActiveTransform.position;
				m_LastScale = m_LastActiveTransform.localScale;
				return;
			}

			if (Tools.current == Tool.Move && positionMoved)
			{
				Vector3 old = selected.position;
				Vector3 mask = old - m_LastPosition;

				bool constraintsOn = SnapMethod == SnapMethod.SnapOnSelectedAxis;

				if (m_ToggleAxisConstraint)
					constraintsOn = !constraintsOn;

				Vector3 snapped;

				if (constraintsOn)
					snapped = Snapping.Round(old, mask, m_SnapSettings.SnapValueInUnityUnits());
				else
					snapped = Snapping.Round(old, m_SnapSettings.SnapValueInUnityUnits());

				Vector3 snapOffset = snapped - old;

				if (m_IsFirstMove)
				{
					Undo.RecordObjects(Selection.transforms, "Move Objects");

					m_IsFirstMove = false;

					if (m_PredictiveGrid && !FullGridEnabled)
					{
						Axis dragAxis = EditorUtility.CalcDragAxis(snapOffset, camera);

						if (dragAxis != Axis.None && dragAxis != m_RenderPlane)
							SetRenderPlane(dragAxis);
					}
				}

				selected.position = snapped;

				if (m_SnapSettings.SnapAsGroup)
				{
					EditorUtility.OffsetTransforms(Selection.transforms, selected, snapOffset);
				}
				else
				{
					foreach (Transform t in Selection.transforms)
						t.position = constraintsOn
							? Snapping.Round(t.position, mask, m_SnapSettings.SnapValueInUnityUnits())
							: Snapping.Round(t.position, m_SnapSettings.SnapValueInUnityUnits());
				}
			}

			if (Tools.current == Tool.Scale && ScaleSnapEnabled && scaleMoved)
			{
				Vector3 old = m_LastActiveTransform.localScale;
				Vector3 mask = old - m_LastScale;

				if (m_PredictiveGrid)
				{
					Axis dragAxis = EditorUtility.CalcDragAxis(Selection.activeTransform.TransformDirection(mask), camera);
					if (dragAxis != Axis.None && dragAxis != m_RenderPlane)
						SetRenderPlane(dragAxis);
				}

				// scale snapping does not respect the SnapMethod by design
				foreach (Transform t in Selection.transforms)
					t.localScale = Snapping.Round(t.localScale, mask, m_SnapSettings.SnapValueInUnityUnits());
			}

			m_LastPosition = m_LastActiveTransform.position;
			m_LastScale = m_LastActiveTransform.localScale;
		}

#if !SHORTCUT_MANAGER
		void HandleSingleKeyShortcuts(Event currentEvent)
		{
			if (!currentEvent.isKey
				|| EditorUtility.SceneViewInUse()
				|| currentEvent.modifiers != EventModifiers.None)
				return;

			var keyCode = currentEvent.keyCode;
			var used = true;

			if (currentEvent.type == EventType.KeyDown)
			{
				if (keyCode == m_TempDisableKey)
					m_ToggleTempSnap = true;
				else if (keyCode == m_AxisConstraintKey)
					m_ToggleAxisConstraint = true;
				else
					used = false;
			}
			else if (currentEvent.type == EventType.KeyUp)
			{
				m_ToggleTempSnap = false;
				m_ToggleAxisConstraint = false;

				if (currentEvent.keyCode == m_IncreaseGridSizeShortcut)
				{
					IncreaseGridSize();
				}
				else if (currentEvent.keyCode == m_DecreaseGridSizeShortcut)
				{
					DecreaseGridSize();
				}
				else if (currentEvent.keyCode == m_NudgePerspectiveBackwardShortcut)
				{
					if (!FullGridEnabled && !gridIsOrthographic && m_GridIsLocked)
						MenuNudgePerspectiveBackward();
				}
				else if (currentEvent.keyCode == m_NudgePerspectiveForwardShortcut)
				{
					if (!FullGridEnabled && !gridIsOrthographic && m_GridIsLocked)
						MenuNudgePerspectiveForward();
				}
				else if (currentEvent.keyCode == m_ResetGridShortcutModifiers)
				{
					if (!FullGridEnabled && !gridIsOrthographic && m_GridIsLocked)
						MenuNudgePerspectiveReset();

					ResetGridSize();
				}
				else if (currentEvent.keyCode == m_CyclePerspectiveShortcut)
				{
					CyclePerspective();
				}
				else
				{
					used = false;
				}
			}
			else
			{
				used = false;
			}

			if (used)
				currentEvent.Use();
		}
#endif

		void OnSelectionChange()
		{
			EditorUtility.ClearSnapEnabledCache();
			ResetActiveTransformValues();
		}

		void ResetActiveTransformValues()
		{
			m_LastActiveTransform = Selection.activeTransform;

			if (m_LastActiveTransform != null)
			{
				m_LastPosition = m_LastActiveTransform.position;
				m_LastScale = m_LastActiveTransform.localScale;
			}
		}
#endregion

#region SETTINGS

		void SetRenderPlane(Axis axis)
		{
			GridRenderOffset = 0f;
			FullGridEnabled = false;
			m_RenderPlane = axis;
			EditorPrefs.SetBool(PreferenceKeys.PerspGrid, FullGridEnabled);
			EditorPrefs.SetInt(PreferenceKeys.GridAxis, (int)m_RenderPlane);
			DoGridRepaint();
		}

		void SetGridEnabled(bool enable)
		{
			m_DrawGrid = enable;

			if (!m_DrawGrid)
				GridRenderer.Destroy();
			else
				EditorUtility.SetUnityGridEnabled(false);

			EditorPrefs.SetBool(PreferenceKeys.ShowGrid, enable);

			DoGridRepaint();
		}

		void SetDrawAngles(bool enable)
		{
			m_DrawAngles = enable;
			DoGridRepaint();
		}

		void SnapToGrid(Transform[] transforms)
		{
			if (transforms != null && transforms.Length > 0)
			{
				Undo.RecordObjects(transforms, "Snap to Grid");

				foreach (Transform t in transforms)
					t.position = Snapping.Round(t.position, SnapValueInUnityUnits);
			}

			PushToGrid(SnapValueInUnityUnits);

			DoGridRepaint();
		}

		bool GetUseAxisConstraints()
		{
			bool useAxisConstraints = Instance.SnapMethod == SnapMethod.SnapOnSelectedAxis;
			return m_ToggleAxisConstraint ? !useAxisConstraints : useAxisConstraints;
		}

		/// <summary>
		/// Retrieves the value used for snapping objects in Unity units. This is equal to `GridUnit * SnapValue * Multiplier`
		/// </summary>
		/// <returns>The value used for snapping objects in Unity units</returns>
		internal float GetSnapValue() { return SnapValueInUnityUnits; }

		/// <summary>
		/// Gets of useAxisConstraints, accounting for the shortcut key toggle.
		/// </summary>
		/// <returns>the value of useAxisConstraints, accounting for the shortcut key toggle.</returns>
		/// <remarks>Used by ProBuilder.</remarks>
		public static bool UseAxisConstraints()
		{
			return Instance != null && Instance.GetUseAxisConstraints();
		}

		/// <summary>
		/// Gets the origin point of the grid. If no instance is active, this method returns zero.
		/// </summary>
		/// <returns>The origin point of the grid or zero if no instance is active.</returns>
		public static Vector3 GetPivot()
		{
			return s_Instance == null ? Vector3.zero : Snapping.Round(s_Instance.m_Pivot, s_Instance.GetSnapValue());
		}

		/// <summary>
		/// Gets the ProGrids snap value in Unity units.
		/// </summary>
		/// <returns>The current snap value.</returns>
		/// <remarks>Used by ProBuilder.</remarks>
		public static float SnapValue()
		{
			return Instance != null ? Instance.GetSnapValue() : 0f;
		}

		/// <summary>
		/// Checks whether snapping is enabled by ProGrids.
		/// </summary>
		/// <returns>True if snapping is enabled; false otherwise.</returns>
		/// <remarks>Used by ProBuilder.</remarks>
		public static bool SnapEnabled()
		{
			return Instance != null && Instance.GetSnapEnabled();
		}

		/// <summary>
		/// Registers a callback to be raised when the ProGrids "Push to Grid" button is pressed.
		/// </summary>
		/// <param name="listener">
		/// A delegate accepting a float value (corresponding to the snap increment).
		/// </param>
		public static void AddPushToGridListener(Action<float> listener)
		{
			pushToGridListeners.Add(listener);
		}

		/// <summary>
		/// Removes a delegate registered via AddPushToGridListener.
		/// </summary>
		/// <param name="listener"></param>
		public static void RemovePushToGridListener(System.Action<float> listener)
		{
			pushToGridListeners.Remove(listener);
		}

		/// <summary>
		/// Registers a delegate to be called when the ProGrids toolbar is opened or closed.
		/// </summary>
		/// <param name="listener">True when toolbar is opening, false if closing.</param>
		/// <remarks>Used by ProBuilder.</remarks>
		public static void AddToolbarEventSubscriber(System.Action<bool> listener)
		{
			toolbarEventSubscribers.Add(listener);
		}

		/// <summary>
		/// Removes a delegate subscribed via AddToolbarEventSubscriber.
		/// </summary>
		/// <param name="listener"></param>
		/// <remarks>Used by ProBuilder.</remarks>
		public static void RemoveToolbarEventSubscriber(System.Action<bool> listener)
		{
			toolbarEventSubscribers.Remove(listener);
		}

		/// <summary>
		/// Tests whether or not ProGrids is open.
		/// </summary>
		/// <returns>True if ProGrids is open in the scene; false if not.</returns>
		/// <remarks>Used by ProBuilder.</remarks>
		public static bool SceneToolbarActive()
		{
			return Instance != null;
		}

		/// <summary>
		/// Checks to see if ProGrids is open and the toolbar is in an extended state.
		/// </summary>
		/// <returns>True if ProGrids is open and the toolbar is in an extended state.</returns>
		/// <remarks>Used by ProBuilder.</remarks>
		public static bool SceneToolbarIsExtended()
		{
			return SceneToolbarActive() && Instance.menuOpen;
		}

		[SerializeField]
		static List<Action<float>> pushToGridListeners = new List<Action<float>>();

		[SerializeField]
		static List<Action<bool>> toolbarEventSubscribers = new List<Action<bool>>();

		static void PushToGrid(float snapValue)
		{
			foreach (Action<float> listener in pushToGridListeners)
				listener(snapValue);
		}

		/// <summary>
		/// Called when the user moves a handle on a ProBuilder object.
		/// </summary>
		/// <remarks>Used by ProBuilder.</remarks>
		static void OnHandleMove(Vector3 worldDirection)
		{
			if (Instance != null)
				Instance.OnHandleMove_Internal(worldDirection);
		}

		void OnHandleMove_Internal(Vector3 worldDirection)
		{
			if (m_PredictiveGrid && m_IsFirstMove && !FullGridEnabled)
			{
				m_IsFirstMove = false;
				Axis dragAxis = EditorUtility.CalcDragAxis(worldDirection, SceneView.lastActiveSceneView.camera);

				if (dragAxis != Axis.None && dragAxis != m_RenderPlane)
					SetRenderPlane(dragAxis);
			}
		}
#endregion
	}
}
