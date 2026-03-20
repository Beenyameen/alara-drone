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
			using var udp = new System.Net.Sockets.UdpClient();
			udp.Client.SetSocketOption(System.Net.Sockets.SocketOptionLevel.Socket, System.Net.Sockets.SocketOptionName.ReuseAddress, true);
			udp.Client.Bind(new System.Net.IPEndPoint(System.Net.IPAddress.Any, 10000));
			udp.Client.ReceiveTimeout = 5000;
			string ip = null;
			while (ip == null)
			{
				try
				{
					var ep = new System.Net.IPEndPoint(System.Net.IPAddress.Any, 0);
					var bytes = udp.Receive(ref ep);
					string msg = System.Text.Encoding.UTF8.GetString(bytes);
					if (msg.StartsWith("PI:")) ip = msg.Split(':')[1];
				}
				catch (System.Net.Sockets.SocketException) { }
			}
			using var sub = new SubscriberSocket();
			sub.Options.ReceiveHighWatermark = 2;
			sub.Connect($"tcp://{ip}:11000");
			sub.Subscribe("");
			
			using var pub = new PublisherSocket();
			pub.Options.SendHighWatermark = 2;
			pub.Bind("tcp://0.0.0.0:12000");

			while (true)
			{
				NetMQMessage m = new NetMQMessage();
				if (sub.TryReceiveMultipartMessage(TimeSpan.FromMilliseconds(50), ref m) && m.FrameCount >= 3)
				{
					short[] decompressed = Rvl.RvlCodec.Decompress(m[2].Buffer);
					byte[] depthBytes = new byte[decompressed.Length * 2];
					Buffer.BlockCopy(decompressed, 0, depthBytes, 0, depthBytes.Length);
					
					pub.SendMoreFrame(m[0].Buffer).SendMoreFrame(m[1].Buffer).SendFrame(depthBytes);

					if (Current == Rgb || Current == Depth)
					{
						RgbData = m[1].Buffer;
						DepthData = depthBytes;
					}
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
		
		if (n != World) Input.MouseMode = Input.MouseModeEnum.Visible;
	}
}
