using UnityEngine;
using NUnit.Framework;

namespace UnityEditor.ProGrids.Tests
{
	class SnapOnSelectedAxisOnly
	{
		[Test]
		public void TranslateOnlySnapsChangedAxis()
		{
			var old = new Vector3(.15f, .15f, .15f);
			var cur = new Vector3(.15f, .28f, .15f);
			Assert.AreEqual(Snapping.Round(cur, cur - old, .5f), new Vector3(.15f, .5f, .15f));

			cur = new Vector3(.15f, .15f, .9f);
			Assert.AreEqual(Snapping.Round(cur, cur - old, .5f), new Vector3(.15f, .15f, 1f));

			cur = new Vector3(.15f, .3f, .9f);
			Assert.AreEqual(Snapping.Round(cur, cur - old, .5f), new Vector3(.15f, .5f, 1f));
		}
	}
}
