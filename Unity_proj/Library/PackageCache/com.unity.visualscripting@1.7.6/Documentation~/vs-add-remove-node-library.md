# Add or remove nodes in your Node Library

Visual Scripting can use Unity features from packages and third-party assets through their assemblies. Assemblies are special files that contain the code for the feature you want to use in Visual Scripting. 

To use a new package or third-party asset in Visual Scripting, you need to import it into Unity. For more information on adding a new package to Unity, see the [Unity User Manual section on adding and removing packages](https://docs.unity3d.com/2021.2/Documentation/Manual/upm-ui-actions.html). For more information on adding a new third-party asset to Unity, see the [Unity User Manual section on importing assets](https://docs.unity3d.com/2021.2/Documentation/Manual/ImportingAssets.html).
 
 Visual Scripting also has a set of default assemblies and nodes. You can add additional assemblies through your Project Settings. 

## Add nodes to your Node Library

 To add a new assembly and its nodes to your Node Library: 

 1. Go to **Edit** &gt; **Project Settings**, then select **Visual Scripting**. 
 2. Expand **Node Library**. 
 3. At the end of your listed assemblies, select **Add** (+). 
 4. In your new assembly entry, select an available assembly from the **Assembly** menu. 

    Visual Scripting adds the assembly and its nodes to your Node Library. To use the nodes in your project, you must [add the nodes' types to your Type Options](vs-add-remove-type-options.md#add-a-type-to-your-type-options) and [regenerate your Node Library in your Project Settings](vs-configuration.md).

## Remove nodes from your Node Library 

 To remove an assembly and its nodes from your Node Library: 

 1. Go to **Edit** &gt; **Project Settings**, then select **Visual Scripting**. 
 2. Expand **Node Library**. 
 3. In the list of assemblies in your Node Library, locate the entry for the assembly you want to remove. 
 4. Select **Remove** (-). 

    Visual Scripting removes the assembly and its nodes from your node library. To remove the nodes from the fuzzy finder and prevent them from appearing in your project, you must [regenerate your Node Library in your Project Settings](vs-configuration.md). You may also want to [remove the nodes' types from your Type Options](vs-add-remove-type-options.md#remove-a-type-from-your-type-options).
 