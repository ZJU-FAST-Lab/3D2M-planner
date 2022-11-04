# Create a custom PropertyDrawer for a custom variable type

If you want to use a custom type from a custom class in Visual Scripting, and you don't have access to its source code, you must have a custom PropertyDrawer. 

You can't assign a value to a custom type inside the Editor or initialize the value for a variable with a custom type if it doesn't have a PropertyDrawer. 

> [!NOTE]
> The class for your custom type must have the `[Serializable]` tag in its source code to create a custom PropertyDrawer.

To create a custom PropertyDrawer: 

1. (Optional) If your Project window isn't already open, go to **Window** &gt; **General** &gt; **Project**, or press CTRL + 5 (macOS: Cmd + 5).

2. Right-click a folder in the Project window's folder list, or anywhere in the Project window's preview pane, and go to **Create** &gt; **C# Script**. 

4. Enter a name for your new script file and press Enter. <br/>For example, if you want to create a PropertyDrawer for a new custom type called `Counter`, you could call your script file `CounterDrawer`.

5. Double-click your new C# file. Unity opens the file in the program you specified in your preferences, under **External Script Editor**. For more information on script editors in Unity, see the [Unity User Manual section on Integrated development environment (IDE) support](https://docs.unity3d.com/Manual/ScriptingToolsIDEs.html).

6. Remove the `Start` and `Update` functions and their comments from your script file. 

7. Above the line that defines your new `public class`, add the following, replacing `<Counter>` with the name of the type you want to assign to this PropertyDrawer, exactly as it appears in Unity: 

    ```csharp
    [CustomPropertyDrawer](type of(<Counter>))]

    ```

8. At the end of the line that defines your new `public class`, change `Monobehavior` to `PropertyDrawer`. 


The method for continuing to create your custom PropertyDrawer depends on the fields you need to display, and how you want them to display in the Editor's interface. For example, you might want to use the UIElements module to create your PropertyDrawer, or you might need to use Unity's IMGUI module to display your specific fields.

For more information on creating your custom PropertyDrawer, see the [PropertyDrawer class in the main Unity Scripting API](https://docs.unity3d.com/ScriptReference/PropertyDrawer.html) and its related methods.

> [!NOTE]
> Once you have created your custom PropertyDrawer, you must generate the additional required property provider scripts from your Visual Scripting Project Settings. For more information, see [Configuring your project settings](vs-configuration.md).