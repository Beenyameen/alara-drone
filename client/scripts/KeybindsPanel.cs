using Godot;
using System;

public partial class KeybindsPanel : VBoxContainer
{
	public override void _Ready()
	{
		foreach (StringName action in InputMap.GetActions())
		{
			string actionName = action.ToString();
			if (actionName.StartsWith("global_") || actionName.StartsWith("pilot_"))
			{
				string displayName = actionName.Replace("global_", "").Replace("pilot_", "").Replace("_", " ");
				
				var events = InputMap.ActionGetEvents(action);
				if (events.Count > 0)
				{
					InputEvent e = events[0];
					string keyName = e.AsText().Replace(" (Physical)", "").Replace(" - Physical", "").Replace(" (Physical)", "");

					Label label = new Label
					{
						Text = $"{keyName} = {displayName}"
					};
					label.AddThemeFontSizeOverride("font_size", 24);
					
					AddChild(label);
				}
			}
		}
	}
}
