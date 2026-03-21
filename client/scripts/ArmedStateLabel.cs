using Godot;
using System;

public partial class ArmedStateLabel : Label
{
	private Main _main;

	public override void _Ready()
	{
		Node n = this;
		while (n != null && !(n is Main)) n = n.GetParent();
		_main = n as Main;
	}

	public override void _Process(double delta)
	{
		if (_main != null)
		{
			if (_main._armState == 1)
			{
				Text = "ARMED";
				Modulate = Colors.Red;
			}
			else if (_main._armState == 0)
			{
				Text = "DISARMED";
				Modulate = Colors.Green;
			}
			else
			{
				Text = "UNKNOWN";
				Modulate = Colors.Gray;
			}
		}
	}
}
