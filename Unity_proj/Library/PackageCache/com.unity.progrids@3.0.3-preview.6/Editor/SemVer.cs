using System;
using System.Text;
using System.Text.RegularExpressions;
using UnityEngine;

namespace UnityEditor.ProGrids
{
	enum VersionType
	{
		Development = 0,
		Patch = 1,
		Alpha = 2,
		Beta = 3,
		Final = 4,
	}

	/// <summary>
	/// Represents a version information container that you can compare to another SemVer object.
	/// </summary>
	[Serializable]
	class SemVer : IEquatable<SemVer>, IComparable<SemVer>
	{
		[SerializeField]
		int m_Major = -1;

		[SerializeField]
		int m_Minor = -1;

		[SerializeField]
		int m_Patch = -1;

		[SerializeField]
		int m_Build = 0;

		[SerializeField]
		VersionType m_Type;

		[SerializeField]
		string m_Metadata;

		[SerializeField]
		string m_Date;

		[SerializeField]
		string m_RawString;

		public int major { get { return m_Major; } }
		public int minor { get { return m_Minor; } }
		public int patch { get { return m_Patch; } }
		public int build { get { return m_Build; } }
		public VersionType type { get { return m_Type; } }
		public string metadata { get { return m_Metadata; } }
		public string date { get { return m_Date; } }
		public string rawString { get { return m_RawString; } }

		public const string DefaultStringFormat = "M.m.p-t.b";

		public SemVer()
		{
		}

		public SemVer(string formatted, string date = null)
		{
			SemVer parsed;
			m_RawString = formatted;
			m_Date = date;

			if (TryGetVersionInfo(formatted, out parsed))
			{
				m_Major = parsed.m_Major;
				m_Minor = parsed.m_Minor;
				m_Patch = parsed.m_Patch;
				m_Build = parsed.m_Build;
				m_Type = parsed.m_Type;
				m_Metadata = parsed.metadata;
			}
			else
			{
				Debug.LogError("Failed parsing version info: " + formatted);
			}
		}

		public SemVer(int major, int minor, int patch, int build = 0, VersionType type = VersionType.Development, string date = "", string metadata = "")
		{
			m_Major = major;
			m_Minor = minor;
			m_Patch = patch;
			m_Build = build;
			m_Type = type;
			m_Metadata = metadata;
			m_Date = string.IsNullOrEmpty(date) ? DateTime.Now.ToString("en-US: MM-dd-yyyy") : date;
			m_RawString = ToString();
		}

		public bool IsValid()
		{
			return major != -1 &&
			       minor != -1 &&
			       patch != -1;
		}

		public override bool Equals(object o)
		{
			return o is SemVer && this.Equals((SemVer) o);
		}

		public override int GetHashCode()
		{
			int hash = 13;

			unchecked
			{
				if(IsValid())
				{
					hash = (hash * 7) + major.GetHashCode();
					hash = (hash * 7) + minor.GetHashCode();
					hash = (hash * 7) + patch.GetHashCode();
					hash = (hash * 7) + build.GetHashCode();
					hash = (hash * 7) + type.GetHashCode();
				}
				else
				{
					return string.IsNullOrEmpty(m_Metadata) ? m_Metadata.GetHashCode() : base.GetHashCode();
				}
			}

			return hash;
		}

		public bool Equals(SemVer version)
		{
			if(IsValid() != version.IsValid())
				return false;

			if(IsValid())
			{
				return 	major == version.major &&
						minor == version.minor &&
						patch == version.patch &&
						type == version.type &&
						build == version.build;
			}
			else
			{
				if( string.IsNullOrEmpty(m_Metadata) || string.IsNullOrEmpty(version.m_Metadata) )
					return false;

				return m_Metadata.Equals(version.m_Metadata);
			}
		}

		public int CompareTo(SemVer version)
		{
			const int GREATER = 1;
			const int LESS = -1;

			if(this.Equals(version))
				return 0;
			else if(major > version.major)
				return GREATER;
			else if(major < version.major)
				return LESS;
			else if(minor > version.minor)
				return GREATER;
			else if(minor < version.minor)
				return LESS;
			else if(patch > version.patch)
				return GREATER;
			else if(patch < version.patch)
				return LESS;
			else if((int)type > (int)version.type)
				return GREATER;
			else if((int)type < (int)version.type)
				return LESS;
			else if(build > version.build)
				return GREATER;
			else
				return LESS;
		}

		/// <summary>
		/// Formats a string containing version information using simple formatting. 
		/// The following characters are available:<br/>
		/// 'M' Major<br/>
		/// 'm' Minor<br/>
		/// 'p' Patch<br/>
		/// 'b' Build<br/>
		/// 't' Lowercase single type (f, d, b, or p)<br/>
		/// 'T' Type<br/>
		/// 'd' Date<br/>
		/// 'D' Metadata<br/>
		/// 'R' The input string used to construct this VersionInfo.<br/>
		/// Escape characters with '\\'.
		/// </summary>
		/// <example>
		/// ToString("\buil\d: T:M.m.p") returns "build: Final:2.10.1"
		/// </example>
		/// <param name="format">The string to format</param>
		/// <returns>The formatted string</returns>
		public string ToString(string format)
		{
			var sb = new StringBuilder();
			bool skip = false;

			foreach (char c in format.ToCharArray())
			{
				if (skip)
				{
					sb.Append(c);
					skip = false;
					continue;
				}

				if (c == '\\')
					skip = true;
				else if(c == 'M')
					sb.Append(major);
				else if(c == 'm')
					sb.Append(minor);
				else if(c == 'p')
					sb.Append(patch);
				else if(c == 'b')
					sb.Append(build);
				else if(c == 't')
					sb.Append(char.ToLower(type.ToString()[0]));
				else if(c == 'T')
					sb.Append(type);
				else if (c == 'd')
					sb.Append(date);
				else if (c == 'D')
					sb.Append(metadata);
				else if (c == 'R')
					sb.Append(rawString);
				else
					sb.Append(c);
			}

			return sb.ToString();
		}

		public override string ToString()
		{
			return ToString(DefaultStringFormat);
		}

		/// <summary>
		/// Creates a VersionInfo type from a string formatted in [valid semantic versioning format](https://semver.org/).
		/// For example: TryGetVersionInfo("2.5.3-b.1", out info)
		/// </summary>
		/// <param name="input">String that contains the version information</param>
		/// <param name="version">The variable to write the VersionInfo to. </param>
		/// <returns></returns>
		public static bool TryGetVersionInfo(string input, out SemVer version)
		{
			version = new SemVer();
			version.m_RawString = input;
			bool ret = false;

			const string k_MajorMinorPatchRegex = "([0-9]+\\.[0-9]+\\.[0-9]+)";
			const string k_VersionReleaseRegex = "(?i)(?<=\\-)[a-z0-9\\-\\.]+";
			const string k_VersionReleaseLooseRegex = "(?<=[0-9]+\\.[0-9]+\\.[0-9]+)[a-z0-9\\-\\.\\+]+";
			const string k_MetadataRegex = "(?<=\\+).+";

			try
			{
				var mmp = Regex.Match(input, k_MajorMinorPatchRegex);

				if (!mmp.Success)
					return false;

				string[] mmpSplit = mmp.Value.Split('.');

				int.TryParse(mmpSplit[0], out version.m_Major);
				int.TryParse(mmpSplit[1], out version.m_Minor);
				int.TryParse(mmpSplit[2], out version.m_Patch);

				ret = true;

				// from here down is less rigid
				var preReleaseVersion = Regex.Match(input, k_VersionReleaseRegex);

				// this is technically wrong version formatting, but it's common enough to see so we try our best to
				// parse it
				if (!preReleaseVersion.Success)
					preReleaseVersion = Regex.Match(input, k_VersionReleaseLooseRegex);

				if (preReleaseVersion.Success)
				{
					// If not parsed, "Development" is returned.
					version.m_Type = GetVersionType(preReleaseVersion.Value);

					// If not parsed, "0" is returned.
					version.m_Build = GetBuildNumber(preReleaseVersion.Value);
				}

				var meta = Regex.Match(input, k_MetadataRegex);

				if (meta.Success)
					version.m_Metadata = meta.Value;
			}
			catch
			{
				ret = false;
			}

			return ret;
		}

		static VersionType GetVersionType(string input)
		{
			const string k_AlphaRegex = "(?i)^(alpha|a)(?=[^a-z]|\\Z)";
			const string k_BetaRegex = "(?i)^(beta|b)(?=[^a-z]|\\Z)";
			const string k_PatchRegex = "(?i)^(patch|p)(?=[^a-z]|\\Z)";
			const string k_FinalRegex = "(?i)^(final|f)(?=[^a-z]|\\Z)";

			if (Regex.IsMatch(input, k_AlphaRegex, RegexOptions.Multiline))
				return VersionType.Alpha;

			if (Regex.IsMatch(input, k_BetaRegex, RegexOptions.Multiline))
				return VersionType.Beta;

			if (Regex.IsMatch(input, k_PatchRegex, RegexOptions.Multiline))
				return VersionType.Patch;

			if (Regex.IsMatch(input, k_FinalRegex, RegexOptions.Multiline))
				return VersionType.Final;

			return VersionType.Development;
		}

		static int GetBuildNumber(string input)
		{
			var number = Regex.Match(input, "[0-9]+");

			int buildNo = 0;

			if (number.Success && int.TryParse(number.Value, out buildNo))
				return buildNo;

			return 0;
		}
	}
}
