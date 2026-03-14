using System;
using System.Linq;
using System.Reflection;

class Program {
    static void Main() {
        var dll = Assembly.LoadFrom(@"C:\Users\beeny\Desktop\Projects\enord\client\.godot\mono\assemblies\Debug\GodotSharp.dll");
        var rdType = dll.GetType("Godot.RenderingDevice");
        foreach(var m in rdType.GetMethods().Where(m => m.Name.Contains("Shader") || m.Name.Contains("Spir"))) {
            Console.WriteLine(m.Name);
        }
    }
}
