using UnityEngine;

namespace Unity.PlasticSCM.Editor.UI
{
    internal class OverlayRect
    {
        internal static Rect GetRightBottonRect(
            Rect selectionRect)
        {
            if (selectionRect.width > selectionRect.height)
                return GetOverlayRectForSmallestSize(
                    selectionRect);

            return GetOverlayRectForOtherSizes(selectionRect);
        }

        internal static Rect GetCenteredRect(
                Rect selectionRect)
        {
            return new Rect(
                selectionRect.x + 3f,
                selectionRect.y + 1f,
                UnityConstants.OVERLAY_STATUS_ICON_SIZE,
                UnityConstants.OVERLAY_STATUS_ICON_SIZE);
        }

        static Rect GetOverlayRectForSmallestSize(
                    Rect selectionRect)
        {
            return new Rect(
                selectionRect.x + 5f,
                selectionRect.y + 4f,
                UnityConstants.OVERLAY_STATUS_ICON_SIZE,
                UnityConstants.OVERLAY_STATUS_ICON_SIZE);
        }

        static Rect GetOverlayRectForOtherSizes(
            Rect selectionRect)
        {
            float widthRatio = selectionRect.width / 
                UNITY_STANDARD_ICON_SIZE;
            float heightRatio = selectionRect.height / 
                UNITY_STANDARD_ICON_SIZE;

            return new Rect(
               selectionRect.x + (OFFSET_STANDARD_ICON * widthRatio) - 1f,
               selectionRect.y + (OFFSET_STANDARD_ICON * heightRatio) - 13f,
               UnityConstants.OVERLAY_STATUS_ICON_SIZE * widthRatio,
               UnityConstants.OVERLAY_STATUS_ICON_SIZE * heightRatio);
        }

        const float OFFSET_STANDARD_ICON = 20f;
        const int UNITY_STANDARD_ICON_SIZE = 32;
    }
}