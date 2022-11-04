using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.ProGrids;

public class IgnoreSnapInterface : MonoBehaviour, IConditionalSnap
{
	[SerializeField]
	bool m_SnapEnabled;

	public bool snapEnabled
	{
		get { return m_SnapEnabled; }
		set { m_SnapEnabled = value; }
	}
}
