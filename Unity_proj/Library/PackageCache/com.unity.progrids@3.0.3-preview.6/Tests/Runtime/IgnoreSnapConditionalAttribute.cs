using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.ProGrids;

#pragma warning disable 618
[ProGridsConditionalSnap]
#pragma warning restore 618
public class IgnoreSnapConditionalAttribute : MonoBehaviour
{
	public bool m_SnapEnabled;

	bool IsSnapEnabled()
	{
		return m_SnapEnabled;
	}
}
