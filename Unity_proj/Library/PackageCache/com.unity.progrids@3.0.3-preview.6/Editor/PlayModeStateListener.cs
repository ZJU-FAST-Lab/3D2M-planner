using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace UnityEditor.ProGrids
{
	[InitializeOnLoad]
	class PlayModeStateListener
	{
#pragma warning disable 649
		public static Action onEnterPlayMode;
		public static Action onExitPlayMode;
		public static Action onEnterEditMode;
		public static Action onExitEditMode;
#pragma warning restore 649

		static PlayModeStateListener()
		{
#if UNITY_2017_2_OR_NEWER
			EditorApplication.playModeStateChanged += (x) =>
			{
				if (x == PlayModeStateChange.EnteredEditMode && onEnterEditMode != null)
					onEnterEditMode();
				else if (x == PlayModeStateChange.ExitingEditMode && onExitEditMode != null)
					onExitEditMode();
				else if (x == PlayModeStateChange.EnteredPlayMode && onEnterPlayMode != null)
					onEnterPlayMode();
				else if (x == PlayModeStateChange.ExitingPlayMode && onExitPlayMode != null)
					onExitPlayMode();
			};
#else
			EditorApplication.playmodeStateChanged += OnPlayModeStateChanged;
#endif
		}

		/// <summary>
		/// Called when the PlayMode State changes
		/// </summary>
		static void OnPlayModeStateChanged()
		{
			bool isPlaying = EditorApplication.isPlaying;
			bool orWillPlay = EditorApplication.isPlayingOrWillChangePlaymode;

			// if these two don't match, that means it's the call prior to actually engaging
			// whatever state. when entering play mode it doesn't make a difference, but on
			// exiting it's the difference between a scene reload and the reloaded scene.
			if (isPlaying != orWillPlay)
			{
				if (isPlaying)
				{
					if (onExitPlayMode != null)
						onExitPlayMode();
				}
				else
				{
					if(onExitEditMode != null)
						onExitEditMode();
				}

				return;
			}
			else
			{
				if (isPlaying)
				{
					if (onEnterPlayMode != null)
						onEnterPlayMode();
				}
				else
				{
					if (onEnterEditMode != null)
						onEnterEditMode();
				}
			}
		}
	}
}
