using Godot;
using System;

public partial class IpLabel : Label
{
	private Main _main;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		Node n = this;
		while (n != null && !(n is Main)) n = n.GetParent();
		_main = n as Main;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		if (_main != null)
		{
			if (string.IsNullOrEmpty(_main.PiIp))
			{
				Text = "IP: UNKNOWN";
			}
			else
			{
				Text = "IP: " + _main.PiIp;
			}
		}
	}
}
