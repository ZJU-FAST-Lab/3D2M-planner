# Create or restore a Visual Scripting backup 

If you don't use a version control system, such as Unity Collaborate, Git, or Subversion, it's a good practice to create backups of your Visual Scripting assets and settings. You can create a backup at any time from your Project Settings. 

You should also back up your data before updating Visual Scripting to a new version. For more information on the update process, see [Update your version of Visual Scripting](vs-update.md).

## Create a new backup 

To create a new backup of your Visual Scripting assets and settings: 

1. Go to **Edit** &gt; **Project Settings**, then select **Visual Scripting**. 
2. Select **Create Backup**, then select **OK**. 
    Visual Scripting creates a new .zip file, with a name in the format `Assets_YYYY_MM_DD_HH_MM_SS`, in a `Backups` folder inside your Unity Project.

## Restore an existing backup 

To restore an existing backup of your Visual Scripting assets and settings: 

1. Go to **Edit** &gt; **Project Settings**, then select **Visual Scripting**. 
2. Select **Restore Backup**. 
    Visual Scripting opens your `Backups` folder in your operating system's file explorer, where you can extract a .zip back-up file and import your graphs and settings back into Unity. 
    
    For more information on how to import assets into Unity, see the [Importing section in the Unity User Manual](xref:ImportingAssets).