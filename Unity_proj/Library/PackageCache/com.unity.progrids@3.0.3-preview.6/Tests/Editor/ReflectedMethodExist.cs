using System;
using System.Reflection;
using UnityEngine;
using UnityEditor;
using UnityEngine.TestTools;
using NUnit.Framework;
using System.Collections;

namespace UnityEditor.ProGrids.Tests
{
	class ReflectedMethodsExist
	{
		[Test]
		public void AnnotationUtility_ShowGrid()
		{
			Assembly editorAssembly = Assembly.GetAssembly(typeof(UnityEditor.Editor));
			Assert.IsNotNull(editorAssembly, "UnityEditor Assembly");
			Type annotationUtility = editorAssembly.GetType("UnityEditor.AnnotationUtility");
			Assert.IsNotNull(annotationUtility, "annotationUtility");
			PropertyInfo pi = annotationUtility.GetProperty("showGrid", BindingFlags.NonPublic | BindingFlags.Static);
			Assert.IsNotNull(pi, "AnnotationUtility.showGrid");
		}
	}
}
