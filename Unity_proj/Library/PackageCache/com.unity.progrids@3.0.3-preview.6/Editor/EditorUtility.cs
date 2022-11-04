using System;
using System.Globalization;
using System.Reflection;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using System.IO;
using System.Linq;
using UnityEngine.ProGrids;
using Object = UnityEngine.Object;

namespace UnityEditor.ProGrids
{
	static class EditorUtility
	{
		const BindingFlags k_BindingFlagsAll = BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Static;
		const string k_ProGridsDirectory = "Packages/com.unity.progrids/";

		static Dictionary<Transform, SnapEnabledOverride> s_SnapOverrideCache = new Dictionary<Transform, SnapEnabledOverride>();
		static Dictionary<Type, bool> s_NoSnapAttributeTypeCache = new Dictionary<Type, bool>();
		static Dictionary<Type, MethodInfo> s_ConditionalSnapAttributeCache = new Dictionary<Type, MethodInfo>();

		internal static bool SceneViewInUse()
		{
			var e = Event.current;

			return 	e.alt
				|| Tools.current == Tool.View
				|| GUIUtility.hotControl > 0
				|| (e.isMouse && e.button > 0);
		}

#if !UNITY_2019_1_OR_NEWER
		static SceneView.OnSceneFunc onPreSceneGuiDelegate
		{
			get
			{
				var fi = typeof(SceneView).GetField("onPreSceneGUIDelegate", k_BindingFlagsAll);
				return fi != null ? fi.GetValue(null) as SceneView.OnSceneFunc : null;
			}

			set
			{
				var fi = typeof(SceneView).GetField("onPreSceneGUIDelegate", k_BindingFlagsAll);

				if (fi != null)
					fi.SetValue(null, value);
			}
		}

		public static void RegisterOnPreSceneGUIDelegate(SceneView.OnSceneFunc func)
		{
			var del = onPreSceneGuiDelegate;

			if (del == null)
				onPreSceneGuiDelegate = func;
			else
				del += func;
		}

		public static void UnregisterOnPreSceneGUIDelegate(SceneView.OnSceneFunc func)
		{
			var del = onPreSceneGuiDelegate;

			if (del != null)
				del -= func;
		}
#endif

		public static T LoadInternalAsset<T>(string path) where T : Object
		{
			if(path.StartsWith("/"))
				path = path.Substring(1, path.Length - 1);
			return AssetDatabase.LoadAssetAtPath<T>(k_ProGridsDirectory + path);
		}

		internal static Color GetColorFromJson(string json, Color fallback)
		{
			try
			{
				return JsonUtility.FromJson<Color>(json);
			}
			catch
			{
				return fallback;
			}
		}

		static Vector3 VectorToMask(Vector3 vec)
		{
			return new Vector3( Mathf.Abs(vec.x) > Mathf.Epsilon ? 1f : 0f,
								Mathf.Abs(vec.y) > Mathf.Epsilon ? 1f : 0f,
								Mathf.Abs(vec.z) > Mathf.Epsilon ? 1f : 0f );
		}

		static Axis MaskToAxis(Vector3 vec)
		{
			Axis axis = Axis.None;
			if( Mathf.Abs(vec.x) > 0 ) axis |= Axis.X;
			if( Mathf.Abs(vec.y) > 0 ) axis |= Axis.Y;
			if( Mathf.Abs(vec.z) > 0 ) axis |= Axis.Z;
			return axis;
		}

		public static Axis CalcDragAxis(Vector3 movement, Camera cam)
		{
			Vector3 mask = VectorToMask(movement);

			if(mask.x + mask.y + mask.z == 2)
			{
				return MaskToAxis(Vector3.one - mask);
			}
			else
			{
				switch( MaskToAxis(mask) )
				{
					case Axis.X:
						if( Mathf.Abs(Vector3.Dot(cam.transform.forward, Vector3.up)) < Mathf.Abs(Vector3.Dot(cam.transform.forward, Vector3.forward)))
							return Axis.Z;
						else
							return Axis.Y;

					case Axis.Y:
						if( Mathf.Abs(Vector3.Dot(cam.transform.forward, Vector3.right)) < Mathf.Abs(Vector3.Dot(cam.transform.forward, Vector3.forward)))
							return Axis.Z;
						else
							return Axis.X;

					case Axis.Z:
						if( Mathf.Abs(Vector3.Dot(cam.transform.forward, Vector3.right)) < Mathf.Abs(Vector3.Dot(cam.transform.forward, Vector3.up)))
							return Axis.Y;
						else
							return Axis.X;
					default:

						return Axis.None;
				}
			}
		}

		public static float ValueFromMask(Vector3 val, Vector3 mask)
		{
			if(Mathf.Abs(mask.x) > .0001f)
				return val.x;
			else if(Mathf.Abs(mask.y) > .0001f)
				return val.y;
			else
				return val.z;
		}

		public static void SetUnityGridEnabled(bool isEnabled)
		{
			try
			{
				Assembly editorAssembly = Assembly.GetAssembly(typeof(UnityEditor.Editor));
				Type annotationUtility = editorAssembly.GetType("UnityEditor.AnnotationUtility");
				PropertyInfo pi = annotationUtility.GetProperty("showGrid", BindingFlags.NonPublic | BindingFlags.Static);
				pi.SetValue(null, isEnabled, BindingFlags.NonPublic | BindingFlags.Static, null, null, null);
			}
			catch
			{}
		}

		public static bool GetUnityGridEnabled()
		{
			try
			{
				Assembly editorAssembly = Assembly.GetAssembly(typeof(UnityEditor.Editor));
				Type annotationUtility = editorAssembly.GetType("UnityEditor.AnnotationUtility");
				PropertyInfo pi = annotationUtility.GetProperty("showGrid", BindingFlags.NonPublic | BindingFlags.Static);
				return (bool) pi.GetValue(null, null);
			}
			catch
			{}

			return false;
		}

		abstract class SnapEnabledOverride
		{
			public abstract bool IsEnabled();
		}

		class SnapIsEnabledOverride : SnapEnabledOverride
		{
			bool m_SnapIsEnabled;

			public SnapIsEnabledOverride(bool snapIsEnabled)
			{
				m_SnapIsEnabled = snapIsEnabled;
			}

			public override bool IsEnabled() { return m_SnapIsEnabled; }
		}

		class ConditionalSnapOverride : SnapEnabledOverride
		{
			Func<bool> m_IsEnabledDelegate;

			public ConditionalSnapOverride(System.Func<bool> d)
			{
				m_IsEnabledDelegate = d;
			}

			public override bool IsEnabled() { return m_IsEnabledDelegate(); }
		}

		class ConditionalSnapInterfaceOverride : SnapEnabledOverride
		{
			IConditionalSnap m_Target;

			public ConditionalSnapInterfaceOverride(IConditionalSnap target)
			{
				m_Target = target;
			}

			public override bool IsEnabled()
			{
				return m_Target == null || m_Target.snapEnabled;
			}
		}

		public static void ClearSnapEnabledCache()
		{
			s_SnapOverrideCache.Clear();
		}

		public static bool SnapIsEnabled(Transform t)
		{
			SnapEnabledOverride so;

			if (s_SnapOverrideCache.TryGetValue(t, out so))
				return so.IsEnabled();

			object[] attribs = null;

			foreach(var c in t.GetComponents<MonoBehaviour>())
			{
				if(c == null)
					continue;

				Type type = c.GetType();
				bool hasNoSnapAttrib;
				var snapInterface = c as IConditionalSnap;

				if (snapInterface != null)
				{
					s_SnapOverrideCache.Add(t, new ConditionalSnapInterfaceOverride(snapInterface));
					return snapInterface.snapEnabled;
				}
				else if(s_NoSnapAttributeTypeCache.TryGetValue(type, out hasNoSnapAttrib))
				{
					if(hasNoSnapAttrib)
					{
						s_SnapOverrideCache.Add(t, new SnapIsEnabledOverride(false));
						return false;
					}
				}
				else
				{
					// this method is deprecated as of 3.0.0-preview.8
					attribs = type.GetCustomAttributes(true);
					hasNoSnapAttrib = attribs.Any(x => x != null && x.ToString().Contains("ProGridsNoSnap"));
					s_NoSnapAttributeTypeCache.Add(type, hasNoSnapAttrib);

					if(hasNoSnapAttrib)
					{
						s_SnapOverrideCache.Add(t, new SnapIsEnabledOverride(false));
						return false;
					}
				}

				MethodInfo mi;

				if(s_ConditionalSnapAttributeCache.TryGetValue(type, out mi))
				{
					if(mi != null)
					{
						s_SnapOverrideCache.Add(t, new ConditionalSnapOverride(() => { return (bool) mi.Invoke(c, null); }));
						return (bool) mi.Invoke(c, null);
					}
				}
				else
				{
					if( attribs.Any(x => x != null && x.ToString().Contains("ProGridsConditionalSnap")) )
					{
						mi = type.GetMethod("IsSnapEnabled", BindingFlags.Instance | BindingFlags.FlattenHierarchy | BindingFlags.NonPublic | BindingFlags.Public);

						s_ConditionalSnapAttributeCache.Add(type, mi);

						if(mi != null)
						{
							s_SnapOverrideCache.Add(t, new ConditionalSnapOverride(() => { return (bool) mi.Invoke(c, null); }));
							return (bool) mi.Invoke(c, null);
						}
					}
					else
					{
						s_ConditionalSnapAttributeCache.Add(type, null);
					}
				}
			}

			s_SnapOverrideCache.Add(t, new SnapIsEnabledOverride(true));

			return true;
		}

		internal static void OffsetTransforms(Transform[] trsfrms, Transform ignore, Vector3 offset)
		{
			foreach (Transform t in trsfrms)
			{
				if (t != ignore)
					t.position += offset;
			}
		}

		public static bool InFrustum(this Camera cam, Vector3 point)
		{
			Vector3 p = cam.WorldToViewportPoint(point);
			return  (p.x >= 0f && p.x <= 1f) &&
			        (p.y >= 0f && p.y <= 1f) &&
			        p.z >= 0f;
		}
	}
}
