# Controls Sandbox App status msg
# Minimal app-level status. The per-control data is carried by the
# nepi_interfaces/ControlsStatus message published by the ControlsIF
# instance at <node>/controls/status.

string name
string controls_namespace
bool controls_ready
