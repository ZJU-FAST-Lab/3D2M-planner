# Using Visual Scripting with Ahead-of-Time platforms

> [!NOTE]
> For versions 2019/2020 LTS, download the Visual Scripting package from the [Unity Asset Store](https://assetstore.unity.com/packages/tools/visual-bolt-163802).

Visual Scripting supports all Unity build targets, including **ahead-of-time (AOT)** platforms, like mobile devices, consoles, and desktops.

As of version 1.5.1, AOT Pre-build is an automatic pre-build step for all AOT platforms.

## Building for Universal Windows Platform

When building for Universal Windows Platform (UWP, formerly known as Windows Store Apps, WSA, or Metro), Visual Scripting requires the use of Unity's **IL2CPP** scripting backend. For more information on IL2CPP, see the [Unity User Manual section on IL2CPP](https://docs.unity3d.com/Manual/IL2CPP.html).

To change the platform, go to **Edit** &gt; **Project Settings** &gt; **Player**, and select **Other Settings**. In the **Configuration** sub-section, select the correct value in the dropdown. 

![](images/vs-uwp-player-settings.png)
