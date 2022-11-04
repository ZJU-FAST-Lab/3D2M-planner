# Add the Inspectable attribute to the source code for a custom type

If you have access to the source code for your custom type, you can add the `[Inspectable]` attribute to the relevant fields and classes that should be accessible in the Unity Editor. By adding the attribute, you can avoid creating a custom PropertyDrawer for your custom type. 

To add the `[Inspectable]` attribute to your source code for your custom type: 

1. Open the script file or files where you defined your custom classes and types in an external editor. 
    If you double-click the file or files inside your Unity project, Unity opens the file in the program you specified in your preferences, under **External Script Editor**. For more information on script editors in Unity, see the [Unity User Manual section on Integrated development environment (IDE) support](https://docs.unity3d.com/Manual/ScriptingToolsIDEs.html).

2. In your external editor, on a line above your class definition, and any properties you want to have available in the Unity Inspector, add the `[Inspectable]` attribute, as shown in the example below: 

    ```csharp

    using System;
    using UnityEngine; 
    using Unity.VisualScripting;

    [Inspectable]
    public class MyClass
    {
        [Inspectable]
        public string name;

        [Inspectable]
        public int amount;

        public string dontShowThis;
    }

    ```

3. In Unity, follow the process for regenerating your Node Library, as described in [Configuring your project settings](vs-configuration.md).

Any variables using your custom types should now be modifiable from the Unity Inspector.