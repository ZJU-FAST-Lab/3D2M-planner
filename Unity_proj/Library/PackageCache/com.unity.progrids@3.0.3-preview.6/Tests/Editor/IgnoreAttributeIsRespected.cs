using UnityEngine;
using NUnit.Framework;
using UnityEngine.ProGrids;
using UObject = UnityEngine.Object;

namespace UnityEditor.ProGrids.Tests
{
	class IgnoreAttributeIsRespected
	{
		[Test]
		public void GameObjectWithIgnoreAttribIsNotSnapped()
		{
			var go = new GameObject();
			Assert.IsTrue(EditorUtility.SnapIsEnabled(go.transform));
			EditorUtility.ClearSnapEnabledCache();
			go.AddComponent<IgnoreSnap>();
			Assert.IsFalse(EditorUtility.SnapIsEnabled(go.transform));
			UObject.DestroyImmediate(go);
		}

		[Test]
		public void GameObjectWithConditionalIgnoreAttribIsNotSnapped()
		{
			var go = new GameObject();
			Assert.IsTrue(EditorUtility.SnapIsEnabled(go.transform));
			var attrib = go.AddComponent<IgnoreSnapConditionalAttribute>();
			EditorUtility.ClearSnapEnabledCache();
			attrib.m_SnapEnabled = false;
			Assert.IsFalse(EditorUtility.SnapIsEnabled(go.transform));
			UObject.DestroyImmediate(go);
		}

		[Test]
		public void GameObjectWithConditionalIgnoreAttribIsSnapped()
		{
			var go = new GameObject();
			Assert.IsTrue(EditorUtility.SnapIsEnabled(go.transform));
			var attrib = go.AddComponent<IgnoreSnapConditionalAttribute>();
			EditorUtility.ClearSnapEnabledCache();
			attrib.m_SnapEnabled = true;
			Assert.IsTrue(EditorUtility.SnapIsEnabled(go.transform));
			UObject.DestroyImmediate(go);
		}
	}
}
