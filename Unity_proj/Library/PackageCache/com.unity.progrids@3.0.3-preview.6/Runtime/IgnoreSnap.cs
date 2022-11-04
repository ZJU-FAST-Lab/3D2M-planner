using UnityEngine;

namespace UnityEngine.ProGrids
{
	/// <summary>
	/// Assigns this script to a GameObject to tell ProGrids to ignore snapping on it. Child objects are still subject to snapping.
	/// </summary>
	[ProGridsNoSnap]
	public class IgnoreSnap : MonoBehaviour {}
}
