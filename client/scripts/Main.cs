using Godot;
using System;
using System.Threading;
using NetMQ;
using NetMQ.Sockets;

public partial class Main : Control
{
	public Control FeedLayout;
	public Feed FeedNode;
	public Node3D World;
	public Node Current;
	public byte[] RgbData, DepthData, TrajectoryData;
	public Vector3 ImuData;
	public string PiIp;
	public volatile bool _armToggleRequested = false;
	public volatile int _armState = -1;

	public override void _Ready()
	{
		FeedLayout = GD.Load<PackedScene>("res://scenes/feed.tscn").Instantiate<Control>();
		AddChild(FeedLayout);
		FeedNode = FeedLayout.GetNode<Feed>("Feed");
		var throttleNode = FeedLayout.GetNode<ThrottleVisualiser>("InstrumentsPanel/ThrottleVisualiser");
		
		AddChild(World = GD.Load<PackedScene>("res://scenes/world.tscn").Instantiate<Node3D>());
		SwitchScene(FeedLayout, Feed.Mode.Rgb);
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
					if (msg.StartsWith("PI:")) PiIp = ip = msg.Split(':')[1];
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

					if (Current == FeedLayout)
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
		new Thread(() => {
			while (PiIp == null) Thread.Sleep(100);
			using var pub = new PublisherSocket();
			pub.Connect($"tcp://{PiIp}:15000");
			using var sub = new SubscriberSocket();
			sub.Connect($"tcp://{PiIp}:16000");
			sub.Subscribe("");
			var req = new RequestSocket();
			req.Connect($"tcp://{PiIp}:15001");
			while (true)
			{
				if (_armToggleRequested)
				{
					_armToggleRequested = false;
					req.SendFrame("TOGGLE_ARM");
					if (req.TryReceiveFrameString(TimeSpan.FromMilliseconds(2000), out string reply))
					{
						if (reply == "1") _armState = 1;
						else if (reply == "0") _armState = 0;
					}
					else
					{
						req.Dispose();
						req = new RequestSocket();
						req.Connect($"tcp://{PiIp}:15001");
					}
				}
				else if (_armState == -1)
				{
					req.SendFrame("CHECK_ARM");
					if (req.TryReceiveFrameString(TimeSpan.FromMilliseconds(500), out string reply))
					{
						if (reply == "1") _armState = 1;
						else if (reply == "0") _armState = 0;
					}
					else
					{
						req.Dispose();
						req = new RequestSocket();
						req.Connect($"tcp://{PiIp}:15001");
					}
				}

				var d = new byte[16];
				Buffer.BlockCopy(BitConverter.GetBytes(FeedNode.R), 0, d, 0, 4);
				Buffer.BlockCopy(BitConverter.GetBytes(FeedNode.P), 0, d, 4, 4);
				Buffer.BlockCopy(BitConverter.GetBytes(FeedNode.Y), 0, d, 8, 4);
				Buffer.BlockCopy(BitConverter.GetBytes(throttleNode.Throttle / 100f), 0, d, 12, 4);
				pub.SendFrame(d);
				if (sub.TryReceiveFrameBytes(TimeSpan.FromMilliseconds(100), out var imu) && imu.Length == 12)
					ImuData = new Vector3(BitConverter.ToSingle(imu, 0), BitConverter.ToSingle(imu, 4), BitConverter.ToSingle(imu, 8));
			}
		}) { IsBackground = true }.Start();
	}

	public override void _Process(double delta)
	{
		if (Input.IsActionJustPressed("arm")) _armToggleRequested = true;
		if (Input.IsActionJustPressed("select_rgb")) SwitchScene(FeedLayout, Feed.Mode.Rgb);
		if (Input.IsActionJustPressed("select_depth")) SwitchScene(FeedLayout, Feed.Mode.Depth);
		if (Input.IsActionJustPressed("select_world")) SwitchScene(World);
		
		if (Current == FeedLayout) 
		{
			if (FeedNode.CurrentMode == Feed.Mode.Rgb && RgbData != null) 
			{ 
				FeedNode.UpdateFeedRgb(RgbData); 
				RgbData = null; 
			}
			else if (FeedNode.CurrentMode == Feed.Mode.Depth && DepthData != null) 
			{ 
				FeedNode.UpdateFeedDepth(DepthData); 
				DepthData = null; 
			}
		}
	}

	public void SwitchScene(Node n, Feed.Mode mode = Feed.Mode.Rgb)
	{
		Current = n;
		FeedLayout.Visible = FeedLayout == n;
		World.Visible = World == n;
		FeedLayout.ProcessMode = FeedLayout == n ? ProcessModeEnum.Inherit : ProcessModeEnum.Disabled;
		World.ProcessMode = World == n ? ProcessModeEnum.Inherit : ProcessModeEnum.Disabled;
		
		if (FeedLayout == n)
		{
			FeedNode.CurrentMode = mode;
		}
		
		if (n != World) Input.MouseMode = Input.MouseModeEnum.Visible;
	}
}
