using UnityEngine;

namespace UnityEditor.ProGrids
{
	[System.Flags]
	enum Axis
	{
		None = 0x0,
		X = 1 << 0,
		Y = 1 << 1,
		Z = 1 << 2
	}

	enum SnapUnit {
		Meter,
		Centimeter,
		Millimeter,
		Inch,
		Foot,
		Yard,
		Parsec
	}

	enum SnapMethod
	{
		SnapOnSelectedAxis,
		SnapOnAllAxes
	}

	static class EnumExtension
	{
		/// <summary>
		/// Multiplies a Vector3 using the inverse value of an axis (for example, Axis.Y becomes Vector3(1, 0, 1) ).
		/// </summary>
		/// <param name="v">The vector to modify</param>
		/// <param name="axis">Which axis to use</param>
		/// <returns>The result as a vector</returns>
		internal static Vector3 InverseAxisMask(Vector3 v, Axis axis)
		{
			switch(axis)
			{
				case Axis.X:
					return Vector3.Scale(v, new Vector3(0f, 1f, 1f));

				case Axis.Y:
					return Vector3.Scale(v, new Vector3(1f, 0f, 1f));

				case Axis.Z:
					return Vector3.Scale(v, new Vector3(1f, 1f, 0f));

				default:
					return v;
			}
		}

		internal static Vector3 AxisMask(Vector3 v, Axis axis)
		{
			switch(axis)
			{
				case Axis.X:
					return Vector3.Scale(v, new Vector3(1f, 0f, 0f));

				case Axis.Y:
					return Vector3.Scale(v, new Vector3(0f, 1f, 0f));

				case Axis.Z:
					return Vector3.Scale(v, new Vector3(0f, 0f, 1f));

				default:
					return v;
			}
		}

		internal static float SnapUnitValue(SnapUnit su)
		{
			switch(su)
			{
				case SnapUnit.Meter:
					return Defaults.Meter;
				case SnapUnit.Centimeter:
					return Defaults.Centimeter;
				case SnapUnit.Millimeter:
					return Defaults.Millimeter;
				case SnapUnit.Inch:
					return Defaults.Inch;
				case SnapUnit.Foot:
					return Defaults.Foot;
				case SnapUnit.Yard:
					return Defaults.Yard;
				case SnapUnit.Parsec:
					return Defaults.Parsec;
				default:
					return Defaults.Meter;
			}
		}

		internal static SnapUnit SnapUnitWithString(string str)
		{
			foreach (SnapUnit su in SnapUnit.GetValues(typeof(SnapUnit)))
			{
				if (su.ToString() == str)
					return su;
			}
			return (SnapUnit)0;
		}

		internal static Axis AxisWithVector(Vector3 val)
		{
			Vector3 v = new Vector3(Mathf.Abs(val.x), Mathf.Abs(val.y), Mathf.Abs(val.z));

			if (v.x > v.y && v.x > v.z)
				return Axis.X;

			if (v.y > v.x && v.y > v.z)
				return Axis.Y;

			return Axis.Z;
		}

		internal static Vector3 VectorWithAxis(Axis axis)
		{
			switch (axis)
			{
				case Axis.X:
					return Vector3.right;
				case Axis.Y:
					return Vector3.up;
				default:
					return Vector3.forward;
			}
		}
	}
}
