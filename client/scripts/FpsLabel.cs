using Godot;
using System;

public partial class FpsLabel : Label
{
	private Feed _feed;
	private int _lastFrames = 0;
	private double _timer = 0;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		_feed = GetNode<Feed>("../../Feed");
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		_timer += delta;
		if (_timer >= 1.0)
		{
			int currentFrames = _feed.FramesReceived;
			int fps = (int)Math.Round((currentFrames - _lastFrames) / _timer);
			Text = $"FPS: {fps}";
			_lastFrames = currentFrames;
			_timer = 0;
		}
	}
}
