# nepi_app_controls_sandbox

Controls Sandbox — a demonstration NEPI app that registers exactly one control of
each `nepi_controls` / `ControlsIF` type through a single `ControlsIF` instance and
renders each one in the RUI with the correct widget.

## Purpose

This app is the first consumer of the `nepi_controls` (SDK) / `ControlsStatus` /
`ControlsIF` (API) pipeline. It exercises all ten `CONTROL_TYPES`:

`Menu`, `Selection`, `Selections`, `Trigger`, `Bool`, `String`, `Int`, `Float`,
`FloatSlider`, `FloatsSlider`.

## ROS interface

- Node: `app_controls_sandbox`
- Published: `.../app_controls_sandbox/status` (`ControlsSandboxStatus`, latched)
- Published: `.../app_controls_sandbox/controls/status` (`nepi_interfaces/ControlsStatus`, latched)
  — one `Control` per registered control, published by the `ControlsIF` instance.
- Subscribed (per-control value setters + display management): under
  `.../app_controls_sandbox/controls/` — `set_<type>_control_value`,
  `set_control_hidden`, `set_control_display_name`, `set_control_description`,
  `set_control_move`, `set_control_order`, `set_control_reset`,
  `set_control_factory_reset`.

## RUI

- `NepiAppControlsSandbox.js` — main panel.
- `Nepi_IF_Controls.js` — the reusable Controls box (one widget per control).
  This component lives in the shared `nepi_rui` source tree
  (`src/rui_webserver/rui-app/src/`) alongside the other controls components, and
  is imported by the main panel; it is not shipped in this app package.
- `NepiAppControlsSandbox-Settings.js` — the Controls Settings box (display
  management), a Select-dropdown settings panel (Nepi_IF_Settings pattern) shown
  only in `develop` run mode or when admin mode is set.
