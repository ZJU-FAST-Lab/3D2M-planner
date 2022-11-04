using UnityEngine;
using System.Collections;

namespace UnityEditor.ProGrids
{
	[System.Serializable]
	class ToggleContent
	{
		public readonly string textOn, textOff;
		public Texture2D imageOn, imageOff;
		public string tooltip;

		GUIContent m_GuiContent = new GUIContent();

		public ToggleContent(string onText, string offText, string tooltip)
		{
			textOn = onText;
			textOff = offText;
			imageOn = null;
			imageOff = null;
			this.tooltip = tooltip;

			m_GuiContent.tooltip = tooltip;
		}

		public ToggleContent(string onText, string offText, Texture2D onImage, Texture2D offImage, string tip)
		{
			textOn = onText;
			textOff = offText;
			imageOn = onImage;
			imageOff = offImage;
			tooltip = tip;

			m_GuiContent.tooltip = tooltip;
		}

		public static bool ToggleButton(Rect r, ToggleContent content, bool enabled, GUIStyle imageStyle, GUIStyle altStyle)
		{
			content.m_GuiContent.image = enabled ? content.imageOn : content.imageOff;
			content.m_GuiContent.text = content.m_GuiContent.image == null ? (enabled ? content.textOn : content.textOff) : "";

			return GUI.Button(r, content.m_GuiContent, content.m_GuiContent.image != null ? imageStyle : altStyle);
		}
	}
}
