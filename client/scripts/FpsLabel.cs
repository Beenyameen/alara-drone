using Godot;
using System;

public partial class FpsLabel : Label
{
	private Main _main;
	private int _lastFrames = 0;
	private double _timer = 0;

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
		if (_main == null || _main.FeedNode == null) return;

		_timer += delta;
		if (_timer >= 1.0)
		{
			int currentFrames = _main.FeedNode.FramesReceived;
			int fps = Math.Max(0, (int)Math.Round((currentFrames - _lastFrames) / _timer));
			Text = $"FPS: {fps}";
			_lastFrames = currentFrames;
			_timer = 0;
		}
	}
}
