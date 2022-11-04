using System;
using System.Reflection;
using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;

namespace UnityEditor.ProGrids.Tests
{
	class SnapOnAllAxes
	{
		[Test]
		public void TranslateSnapsAllAxes()
		{
			var cur = new Vector3(.15f, .28f, .15f);
			Assert.AreEqual(Snapping.Round(cur, .5f), new Vector3(0f, .5f, 0f));

			cur = new Vector3(.15f, .15f, .9f);
			Assert.AreEqual(Snapping.Round(cur, .5f), new Vector3(0f, 0f, 1f));

			cur = new Vector3(.15f, .3f, .9f);
			Assert.AreEqual(Snapping.Round(cur, .5f), new Vector3(0f, .5f, 1f));
		}
	}
}
