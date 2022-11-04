using System;

namespace UnityEngine.ProGrids
{
	/// <summary>
	/// Applies this attribute to a MonoBehaviour to disable grid snapping on the parent object.
	/// </summary>
	[AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = true)]
	public class ProGridsNoSnapAttribute : Attribute
	{
	}

	/// <summary>
	/// This class is obsolete. Use the [[IConditionalSnap]] interface instead.
	/// </summary>
	/// <remarks>
	/// Checks for a function named `bool IsSnapEnabled()` on this object so you can
	/// programmatically enable or disable snapping.
	/// </remarks>
	[Obsolete("Use IConditionalSnap interface")]
	[AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = true)]
	public class ProGridsConditionalSnapAttribute : Attribute
	{
	}

	/// <summary>
	/// Implements this interface in a MonoBehaviour to dynamically enable or disable grid snapping on the parent.
	/// GameObject.
	/// </summary>
	public interface IConditionalSnap
	{
		bool snapEnabled { get; }
	}
}
