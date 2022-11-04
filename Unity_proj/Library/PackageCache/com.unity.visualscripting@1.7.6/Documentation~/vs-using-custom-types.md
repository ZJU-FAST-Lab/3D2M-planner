# Using custom types

Visual Scripting supports every class and struct type available in Unity. By default, the most common are available in the Fuzzy Finder. You can add additional Unity assemblies, including custom types and classes, through your [Visual Scripting project settings](vs-configuration.md).

To use your custom types and classes in your graphs, you may need to write some additional code. You can't assign a value to a variable with a custom type from the Inspector, or initialize it from inside the Unity Editor if this additional code isn't available.

Depending on your access to the source code for your custom type, you have two options to enable variable assignment and initialization: 

- If you have access to the source code, you can add the `[Inspectable]` attribute to the classes and fields that you want to display and modify in the Editor. 
- If you don't have access to the source code, you must create a custom PropertyDrawer and generate the required property provider scripts.

## Adding the [Inspectable] attribute

If you add the `[Inspectable]` attribute to the code for your custom class, Unity displays its available properties in the Inspector.

Unity provides a basic UI for your types in the Inspector, which may not provide the aesthetic results you want. If you or your users want to configure a property for a custom type using a slider, for example, you shouldn't use the `[Inspectable]` attribute method.

## Creating a custom PropertyDrawer

If you create a custom PropertyDrawer, you can choose how to display each property in the Inspector. 

If you don't have access to the source code, creating a PropertyDrawer is the only way to interact with custom-typed variables in Visual Scripting. For example, if you see an error in the Unity Editor's Inspector window when attempting to use a type from a third-party package, you need to create a custom PropertyDrawer to resolve the error.

> [!NOTE]
> If you are a package developer, or plan to provide your custom classes and types to other users and want those types to be available in Visual Scripting, you should consider creating a custom PropertyDrawer to get the best results for your users. 

For more information on creating a custom PropertyDrawer, see [Create a custom PropertyDrawer for a custom variable type](vs-create-custom-drawer.md).

Once you have a custom PropertyDrawer available for a custom type, you must go into your Visual Scripting Project Settings to generate the necessary property provider scripts. For more information, see the description of the **Generate** option in [Configuring your project settings](vs-configuration.md).

