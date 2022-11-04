using System;
using System.Reflection;
using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;
using UnityEngine.ProGrids;
using UObject = UnityEngine.Object;

namespace UnityEditor.ProGrids.Tests
{
	class IgnoreInterfaceIsRespected
	{
		[Test]
		public void GameObjectWithIgnoreInterfaceIsNotSnapped()
		{
			var go = new GameObject();
			Assert.IsTrue(EditorUtility.SnapIsEnabled(go.transform));
			var attrib = go.AddComponent<IgnoreSnapInterface>();
			EditorUtility.ClearSnapEnabledCache();
			attrib.snapEnabled = false;
			Assert.IsFalse(EditorUtility.SnapIsEnabled(go.transform));
			UObject.DestroyImmediate(go);
		}

		[Test]
		public void GameObjectWithIgnoreInterfaceIsSnapped()
		{
			var go = new GameObject();
			Assert.IsTrue(EditorUtility.SnapIsEnabled(go.transform));
			var attrib = go.AddComponent<IgnoreSnapInterface>();
			EditorUtility.ClearSnapEnabledCache();
			attrib.snapEnabled = true;
			Assert.IsTrue(EditorUtility.SnapIsEnabled(go.transform));
			UObject.DestroyImmediate(go);
		}
	}
}
