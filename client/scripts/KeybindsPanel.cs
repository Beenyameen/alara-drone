using Godot;
using System;
using System.Collections.Generic;

public partial class KeybindsPanel : VBoxContainer
{
	public enum BindingDisplayMode
	{
		Both,
		Auto,
		KeyboardMouseOnly,
		ControllerOnly
	}

	private enum InputDeviceKind
	{
		Unknown,
		KeyboardMouse,
		Controller
	}

	[Export] public BindingDisplayMode DisplayMode { get; set; } = BindingDisplayMode.Both;

	private readonly List<Label> _generatedLabels = new();
	private InputDeviceKind _lastInputDevice = InputDeviceKind.Unknown;

	public override void _Ready()
	{
		RebuildLabels();
	}

	public override void _Input(InputEvent @event)
	{
		InputDeviceKind currentDevice = GetInputDeviceKind(@event);
		if (currentDevice == InputDeviceKind.Unknown || currentDevice == _lastInputDevice)
		{
			return;
		}

		_lastInputDevice = currentDevice;
		if (DisplayMode == BindingDisplayMode.Auto)
		{
			RebuildLabels();
		}
	}

	private void RebuildLabels()
	{
		foreach (Label label in _generatedLabels)
		{
			if (IsInstanceValid(label))
			{
				label.QueueFree();
			}
		}
		_generatedLabels.Clear();

		foreach (StringName action in InputMap.GetActions())
		{
			string actionName = action.ToString();
			if (actionName.StartsWith("global_") || actionName.StartsWith("pilot_") || !actionName.StartsWith("ui"))
			{
				string displayName = actionName.Replace("global_", "").Replace("pilot_", "").Replace("_", " ");

				var events = InputMap.ActionGetEvents(action);
				string bindingText = GetBindingText(events);
				if (!string.IsNullOrEmpty(bindingText))
				{
					Label label = new Label
					{
						Text = $"{bindingText} = {displayName}"
					};
					label.AddThemeFontSizeOverride("font_size", 24);

					AddChild(label);
					_generatedLabels.Add(label);
				}
			}
		}
	}

	private string GetBindingText(Godot.Collections.Array<InputEvent> events)
	{
		var nonControllerBindings = new List<string>();
		var controllerBindings = new List<string>();
		var seenNonController = new HashSet<string>();
		var seenController = new HashSet<string>();

		foreach (InputEvent inputEvent in events)
		{
			string eventText = FormatEventText(inputEvent);
			if (string.IsNullOrEmpty(eventText))
			{
				continue;
			}

			bool isControllerEvent = inputEvent is InputEventJoypadButton || inputEvent is InputEventJoypadMotion;
			if (isControllerEvent)
			{
				if (seenController.Add(eventText))
				{
					controllerBindings.Add(eventText);
				}
			}
			else
			{
				if (seenNonController.Add(eventText))
				{
					nonControllerBindings.Add(eventText);
				}
			}
		}

		string keyboardMouseText = nonControllerBindings.Count > 0 ? string.Join(" / ", nonControllerBindings) : string.Empty;
		string controllerText = controllerBindings.Count > 0 ? string.Join(" / ", controllerBindings) : string.Empty;

		switch (DisplayMode)
		{
			case BindingDisplayMode.Both:
				if (!string.IsNullOrEmpty(keyboardMouseText) && !string.IsNullOrEmpty(controllerText))
				{
					return $"{keyboardMouseText} | {controllerText}";
				}
				return !string.IsNullOrEmpty(keyboardMouseText) ? keyboardMouseText : controllerText;

			case BindingDisplayMode.Auto:
				if (_lastInputDevice == InputDeviceKind.Controller)
				{
					return !string.IsNullOrEmpty(controllerText) ? controllerText : keyboardMouseText;
				}
				return !string.IsNullOrEmpty(keyboardMouseText) ? keyboardMouseText : controllerText;

			case BindingDisplayMode.KeyboardMouseOnly:
				return keyboardMouseText;

			case BindingDisplayMode.ControllerOnly:
				return controllerText;

			default:
				return string.Empty;
		}
	}

	private static InputDeviceKind GetInputDeviceKind(InputEvent inputEvent)
	{
		if (inputEvent is InputEventJoypadButton || inputEvent is InputEventJoypadMotion)
		{
			return InputDeviceKind.Controller;
		}

		if (inputEvent is InputEventKey || inputEvent is InputEventMouseButton || inputEvent is InputEventMouseMotion)
		{
			return InputDeviceKind.KeyboardMouse;
		}

		return InputDeviceKind.Unknown;
	}

	private static string FormatEventText(InputEvent inputEvent)
	{
		if (inputEvent is InputEventJoypadButton joypadButtonEvent)
		{
			return FormatJoypadButtonText(joypadButtonEvent);
		}

		if (inputEvent is InputEventJoypadMotion joypadMotionEvent)
		{
			return FormatJoypadMotionText(joypadMotionEvent);
		}

		return inputEvent
			.AsText()
			.Replace(" (Physical)", "")
			.Replace(" - Physical", "")
			.Trim();
	}

	private static string FormatJoypadButtonText(InputEventJoypadButton inputEvent)
	{
		string buttonName = ((int)inputEvent.ButtonIndex) switch
		{
			0 => "A",
			1 => "B",
			2 => "X",
			3 => "Y",
			4 => "Back",
			5 => "Guide",
			6 => "Start",
			7 => "LS",
			8 => "RS",
			9 => "LB",
			10 => "RB",
			11 => "DPad Up",
			12 => "DPad Down",
			13 => "DPad Left",
			14 => "DPad Right",
			_ => $"Btn {(int)inputEvent.ButtonIndex}"
		};

		return $"{buttonName}";
	}

	private static string FormatJoypadMotionText(InputEventJoypadMotion inputEvent)
	{
		string axisName = ((int)inputEvent.Axis) switch
		{
			0 => "LX",
			1 => "LY",
			2 => "RX",
			3 => "RY",
			4 => "LT",
			5 => "RT",
			_ => $"Axis {(int)inputEvent.Axis}"
		};

		string direction = inputEvent.AxisValue >= 0.0f ? "+" : "-";
		return $"{axisName}{direction}";
	}
}
