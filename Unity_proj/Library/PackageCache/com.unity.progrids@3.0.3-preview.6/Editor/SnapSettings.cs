using System;
using UnityEngine;

namespace UnityEditor.ProGrids
{
	[Serializable]
	class SnapSettings
	{
		// The snap value in SnapUnit coordinates.
		[SerializeField]
		float m_SnapValue;

		// The unit of measurement in which to apply rounding. Default is meter.
		[SerializeField]
		SnapUnit m_SnapUnit;

		[SerializeField]
		bool m_SnapEnabled;

		[SerializeField]
		bool m_ScaleSnapEnabled;

		[SerializeField]
		bool m_SnapAsGroup;

		// Multiplier is stored as int to avoid rounding errors when halving/doubling the grid resolution.
		[SerializeField]
		int m_SnapMultiplier;

		public SnapSettings()
		{
			m_SnapValue = Defaults.SnapValue;
			m_SnapUnit = Defaults.SnapUnit;
			m_ScaleSnapEnabled = Defaults.SnapOnScale;
			m_SnapAsGroup = Defaults.SnapAsGroup;
			m_SnapMultiplier = Defaults.DefaultSnapMultiplier;
		}

		public float SnapValue
		{
			get { return m_SnapValue; }
			set { m_SnapValue = value; }
		}

		public SnapUnit SnapUnit
		{
			get { return m_SnapUnit; }
			set { m_SnapUnit = value; }
		}

		public bool SnapEnabled
		{
			get { return m_SnapEnabled; }
			set { m_SnapEnabled = value; }
		}

		public bool ScaleSnapEnabled
		{
			get { return m_ScaleSnapEnabled; }
			set { m_ScaleSnapEnabled = value; }
		}

		public bool SnapAsGroup
		{
			get { return m_SnapAsGroup; }
			set { m_SnapAsGroup = value; }
		}

		public int SnapMultiplier
		{
			get { return m_SnapMultiplier; }
			set { m_SnapMultiplier = value; }
		}

		public float SnapMultiplierFrac()
		{
			return m_SnapMultiplier / (float) Defaults.DefaultSnapMultiplier;
		}

		public float SnapValueInUnityUnits()
		{
			return SnapValue * EnumExtension.SnapUnitValue(SnapUnit) * SnapMultiplierFrac();
		}
	}
}
