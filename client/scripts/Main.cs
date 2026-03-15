using Godot;
using System;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public partial class Main : Control
{
	public RGBFeed Rgb;
	public DepthFeed Depth;
	public Node3D World;
	public Node Current;
	public byte[] RgbData, DepthData, TrajectoryData;

	public override void _Ready()
	{
		AddChild(Rgb = GD.Load<PackedScene>("res://scenes/rgb_feed.tscn").Instantiate<RGBFeed>());
		AddChild(Depth = GD.Load<PackedScene>("res://scenes/depth_feed.tscn").Instantiate<DepthFeed>());
		AddChild(World = GD.Load<PackedScene>("res://scenes/world.tscn").Instantiate<Node3D>());
		SwitchScene(Rgb);
		new Thread(() => {
			using var sub = new SubscriberSocket();
			sub.Options.ReceiveHighWatermark = 2;
			sub.Connect("tcp://127.0.0.1:12000");
			sub.Subscribe("");
			while (true)
			{
				if (Current != Rgb && Current != Depth)
				{
					Thread.Sleep(50);
					continue;
				}
				NetMQMessage m = new NetMQMessage();
				if (sub.TryReceiveMultipartMessage(TimeSpan.FromMilliseconds(50), ref m) && m.FrameCount >= 3)
				{
					RgbData = m[1].Buffer;
					DepthData = m[2].Buffer;
				}
			}
		}) { IsBackground = true }.Start();
		new Thread(() => {
			using var sub = new SubscriberSocket();
			sub.Options.ReceiveHighWatermark = 2;
			sub.Connect("tcp://127.0.0.1:13000");
			sub.Subscribe("");
			while (true)
			{
				if (Current != World)
				{
					Thread.Sleep(50);
					continue;
				}
				if (sub.TryReceiveFrameBytes(TimeSpan.FromMilliseconds(50), out var m) && m.Length > 0 && m.Length % 64 == 0) TrajectoryData = m;
			}
		}) { IsBackground = true }.Start();
	}

	public override void _Process(double delta)
	{
		if (Input.IsActionJustPressed("select_rgb")) SwitchScene(Rgb);
		if (Input.IsActionJustPressed("select_depth")) SwitchScene(Depth);
		if (Input.IsActionJustPressed("select_world")) SwitchScene(World);
		if (RgbData != null && Current == Rgb) { Rgb.UpdateFeed(RgbData); RgbData = null; }
		if (DepthData != null && Current == Depth) { Depth.UpdateFeed(DepthData); DepthData = null; }
	}

	public void SwitchScene(Node n)
	{
		Current = n;
		Rgb.Visible = Rgb == n;
		Depth.Visible = Depth == n;
		World.Visible = World == n;
		Rgb.ProcessMode = Rgb == n ? ProcessModeEnum.Inherit : ProcessModeEnum.Disabled;
		Depth.ProcessMode = Depth == n ? ProcessModeEnum.Inherit : ProcessModeEnum.Disabled;
		World.ProcessMode = World == n ? ProcessModeEnum.Inherit : ProcessModeEnum.Disabled;
	}
}
