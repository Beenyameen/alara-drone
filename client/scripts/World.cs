using Godot;
using NetMQ;
using NetMQ.Sockets;
using System.Threading;

public partial class World : Node3D
{
	public MultiMeshInstance3D Mm;
	public MultiMeshInstance3D PointsMm;
	public StaticBody3D Sb;
	public Player P;
	public int LastX = int.MinValue, LastZ = int.MinValue;

	private Thread _zmqThread;
	private volatile bool _runZmq = true;
	private byte[] _latestPoints = null;
	private bool _pointsReady = false;
	private System.Threading.Mutex _pointsMutex = new System.Threading.Mutex();

	private RenderingDevice _rd;
	private Rid _shader;
	private Rid _pipeline;

	public override void _Ready()
	{
		Sb = new StaticBody3D();
		Sb.AddChild(new CollisionShape3D { Shape = new WorldBoundaryShape3D() });
		AddChild(Sb);
		var originMarker = new CsgPolygon3D {
			Polygon = new Vector2[] { new Vector2(-0.5f, -0.5f), new Vector2(0.5f, -0.5f), new Vector2(0.5f, 0.5f), new Vector2(-0.5f, 0.5f), new Vector2(-0.5f, -0.5f), new Vector2(-0.4f, -0.4f), new Vector2(-0.4f, 0.4f), new Vector2(0.4f, 0.4f), new Vector2(0.4f, -0.4f), new Vector2(-0.4f, -0.4f) },
			Mode = CsgPolygon3D.ModeEnum.Depth,
			Depth = 0.001f,
			MaterialOverride = new StandardMaterial3D { AlbedoColor = new Color(0, 1, 0), EmissionEnabled = true, Emission = new Color(0, 1, 0), EmissionEnergyMultiplier = 4.0f }
		};
		originMarker.Rotation = new Vector3(-Mathf.Pi / 2, 0, 0);
		originMarker.Position = new Vector3(0, 0.01f, 0);
		AddChild(originMarker);

		Mm = new MultiMeshInstance3D {
			Multimesh = new MultiMesh {
				TransformFormat = MultiMesh.TransformFormatEnum.Transform3D,
				UseColors = true,
				InstanceCount = 31500,
				Mesh = new PlaneMesh { Size = new Vector2(1, 1), Material = new StandardMaterial3D { VertexColorUseAsAlbedo = true } }
			}
		};
		AddChild(Mm);

		PointsMm = new MultiMeshInstance3D {
			ExtraCullMargin = 10000.0f,
			Multimesh = new MultiMesh {
				TransformFormat = MultiMesh.TransformFormatEnum.Transform3D,
				UseColors = true,
				InstanceCount = 0,
				Mesh = new BoxMesh { Size = new Vector3(0.025f, 0.025f, 0.025f), Material = new StandardMaterial3D { VertexColorUseAsAlbedo = true, ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded } }
			}
		};
		AddChild(PointsMm);

		P = GetNode<Player>("Player");

		_rd = RenderingServer.CreateLocalRenderingDevice();
		var shaderSource = new RDShaderSource();
		shaderSource.Language = RenderingDevice.ShaderLanguage.Glsl;
		shaderSource.SourceCompute = FileAccess.GetFileAsString("res://scenes/points.glsl");
		var shaderSpirV = _rd.ShaderCompileSpirVFromSource(shaderSource);
		
		if (!string.IsNullOrEmpty(shaderSpirV.CompileErrorCompute))
		{

		}

		_shader = _rd.ShaderCreateFromSpirV(shaderSpirV);
		_pipeline = _rd.ComputePipelineCreate(_shader);

		_zmqThread = new Thread(ZmqLoop);
		_zmqThread.Start();
	}

	private void ZmqLoop()
	{
		while (_runZmq)
		{
			try
			{
				using var req = new RequestSocket();
				req.Connect("tcp://127.0.0.1:15000");
				
				while (_runZmq)
				{
					req.SendFrameEmpty();
					if (req.TryReceiveFrameBytes(System.TimeSpan.FromSeconds(2), out var bytes))
					{
						_pointsMutex.WaitOne();
						_latestPoints = bytes;
						_pointsReady = true;
						_pointsMutex.ReleaseMutex();
					}
					else
					{
						// Timeout occurred. REQ socket state is now broken, break to recreate it.
						break;
					}
					Thread.Sleep(1000);
				}
			}
			catch (System.Exception e)
			{

				Thread.Sleep(2000);
			}
		}
	}

	public override void _ExitTree()
	{
		_runZmq = false;
		_zmqThread.Join();
		_rd.FreeRid(_pipeline);
		_rd.FreeRid(_shader);
		_rd.Free();
	}

	public override void _Process(double delta)
	{
		if (Input.IsActionJustPressed("floor"))
		{
			Mm.Visible = !Mm.Visible;
			Sb.ProcessMode = Mm.Visible ? ProcessModeEnum.Inherit : ProcessModeEnum.Disabled;
		}

		_pointsMutex.WaitOne();
		if (_pointsReady)
		{
			_pointsReady = false;
			byte[] rawBytes = _latestPoints;
			_pointsMutex.ReleaseMutex();

			if (rawBytes != null && rawBytes.Length > 0)
			{
				int pointCount = rawBytes.Length / 24; // 6 floats * 4 bytes

				if (pointCount > 0) UpdatePointCloud(rawBytes, pointCount);
			}
		}
		else
		{
			_pointsMutex.ReleaseMutex();
		}

		int px = Mathf.FloorToInt(P.Position.X), pz = Mathf.FloorToInt(P.Position.Z);
		if (px == LastX && pz == LastZ) return;
		LastX = px; LastZ = pz;
		int i = 0;
		for (int x = -100; x <= 100; x++)
		for (int z = -100; z <= 100; z++)
			if (x * x + z * z <= 10000)
			{
				int wx = px + x, wz = pz + z;
				Mm.Multimesh.SetInstanceTransform(i, new Transform3D(Basis.Identity, new Vector3(wx + 0.5f, 0, wz + 0.5f)));
				Mm.Multimesh.SetInstanceColor(i++, (wx + wz) % 2 == 0 ? new Color(0.2f, 0.2f, 0.2f) : new Color(0.8f, 0.8f, 0.8f));
			}
		Mm.Multimesh.VisibleInstanceCount = i;
	}

	private void UpdatePointCloud(byte[] inputBytes, int pointCount)
	{
		var inputBuffer = _rd.StorageBufferCreate((uint)inputBytes.Length, inputBytes);
		uint outputSize = (uint)(pointCount * 64);
		var outputBuffer = _rd.StorageBufferCreate(outputSize);

		var uniform1 = new RDUniform { UniformType = RenderingDevice.UniformType.StorageBuffer, Binding = 0 };
		uniform1.AddId(inputBuffer);

		var uniform2 = new RDUniform { UniformType = RenderingDevice.UniformType.StorageBuffer, Binding = 1 };
		uniform2.AddId(outputBuffer);

		var uniformSet = _rd.UniformSetCreate(new Godot.Collections.Array<RDUniform> { uniform1, uniform2 }, _shader, 0);

		var pushConstant = new byte[16];
		System.BitConverter.GetBytes((uint)pointCount).CopyTo(pushConstant, 0);

		long computeList = _rd.ComputeListBegin();
		_rd.ComputeListBindComputePipeline(computeList, _pipeline);
		_rd.ComputeListBindUniformSet(computeList, uniformSet, 0);
		_rd.ComputeListSetPushConstant(computeList, pushConstant, (uint)pushConstant.Length);
		_rd.ComputeListDispatch(computeList, (uint)Mathf.CeilToInt(pointCount / 256.0f), 1, 1);
		_rd.ComputeListEnd();

		_rd.Submit();
		_rd.Sync();

		byte[] outputBytes = _rd.BufferGetData(outputBuffer);

		float[] floats = new float[outputBytes.Length / 4];
		System.Buffer.BlockCopy(outputBytes, 0, floats, 0, outputBytes.Length);
		
		PointsMm.Multimesh.InstanceCount = pointCount;
		PointsMm.Multimesh.Buffer = floats;

		_rd.FreeRid(uniformSet);
		_rd.FreeRid(inputBuffer);
		_rd.FreeRid(outputBuffer);
	}
}
