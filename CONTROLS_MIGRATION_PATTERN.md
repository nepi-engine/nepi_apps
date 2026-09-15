# Migrating a NEPI App to ControlsIF

This document records how `nepi_app_fake_gps` was moved off its bespoke
parameter-and-panel implementation and onto the `ControlsIF` control structure,
on both the node side and the RUI side. It is written so the remaining apps can
follow the same route. Everything below describes what was actually done, not
what could be done.

The pattern sources are `ControlsIF` in
`nepi_engine/nepi_api/src/nepi_api/system_if.py`, the dict helpers in
`nepi_engine/nepi_sdk/src/nepi_sdk/nepi_controls.py`, and the two renderers
`Nepi_IF_Controls.js` and `Nepi_IF_Control.js` in `nepi_rui`.


## What moves and what stays

The dividing line is simple. Anything an operator adjusts from the RUI while the
app is running becomes a control. Anything read once at startup, or owned by
another interface, stays where it is.

In practice almost everything moves. In the fake GPS app all seven node
parameters became controls and the node's `PARAMS_DICT` went to `None`. Two
categories stayed behind.

The first is state owned by a sub-interface. `NavPoseIF` keeps its own
namespace, its own configuration tier and its own RUI treatment. Do not pull a
sub-interface's parameters into the app's control set; you would be publishing
the same state from two places.

The second is commands. A command that carries a whole compound value in one
message -- a geopoint, an ENU offset -- cannot be expressed atomically by a
control set, because a control set writes each value independently. The fake GPS
app kept `reset`, `goto_location`, `goto_position` and `go_stop` on the wire as
its programmatic API, and added Button controls that call the same internal
methods. The rule that matters is that there is exactly one implementation per
command. The topic callback and the Button handler both delegate to a private
method (`stopMove`, `gotoEnuPosition`, `gotoGeoLocation`); neither reimplements
the other. What must never survive the migration is a control and a legacy
parameter both writing the same piece of state.

Status messages are left alone. `NepiAppFakeGpsStatus` still reports `enabled`,
`selected_mavros_node`, `satellites_visible` and `gps_pub_rate_hz` as read-only
status. Editing the status message would ripple into the connect API and the
RUI for no gain.


## Building the init dict

The control set is a module-level dictionary literal next to the app's factory
defaults. Key order is display order: `create_controls_dict` iterates the init
dict, Python preserves insertion order, and `update_status_msg` emits
`controls_msg_list` in that order, which is the order the RUI renders. There is
no separate ordering field to set.

Each entry needs a `type`, something to seed a value with, and -- depending on
the type -- bounds or an option list. `display_name` and `description` are what
the operator reads, so write them as UI labels: what the thing does, plainly.

Type choice is where the real decisions are.

Prefer `Selection` over `Menu` whenever the stored value is meaningful text. A
`Menu` value is the **index** into its option list, and that index silently
re-points at a different option if the list is ever reordered or rebuilt. The
fake GPS app's target mavros node is a `Selection` for exactly this reason: its
option list is rebuilt on every discovery cycle.

Group values into one multi-value control only when they genuinely share a
bound. A control carries a single `min_bound`/`max_bound` pair for all of its
entries. The three ENU offset axes share one metre bound, so they are one
`Floats` control with `display_row` set and per-axis `display_labels`. Latitude,
longitude and altitude do not share a bound, so they stay three separate `Float`
controls with correct per-axis bounds.

Buttons need no default. `create_controls_dict` seeds every trigger type with a
"never fired" value of `0`, and the status message reports seconds since the
last press, or `-999` for never. A press arrives from the RUI as the
non-numeric sentinel `TRIGGER`, which the value cleaner stamps with the current
time.

Where the app declares a bound already -- the fake GPS publish rate clamps to
`MIN_GPS_PUB_RATE_HZ`/`MAX_GPS_PUB_RATE_HZ` -- reuse the existing constant so
the control and the node cannot drift apart. Where no bound exists, derive one
from how the value is used and say so in a comment. Do not leave a numeric
control unbounded just because the old parameter was.

Two limits are worth knowing before you write the dict. `round` is clamped to
six decimal places, so a latitude control holds about 0.11 m of resolution; the
seventh decimal of a factory geopoint does not survive the round trip. And a
control name containing the substring `Trigger` is rewritten to `Button` by
`create_controls_dict`, so avoid that substring in names.

Because `create_controls_dict` drops a malformed control with only a log warning
rather than raising, the fake GPS app validates its own dict before mounting:

    def checkControlsInitDict(self):
        controls_dict = nepi_controls.create_controls_dict(CONTROLS_INIT_DICT)
        missing = [n for n in CONTROLS_INIT_DICT.keys() if n not in controls_dict.keys()]
        if len(missing) > 0:
            self.msg_if.pub_warn("controls dropped at registration: " + str(missing))

That costs one extra pass at startup and turns a silently missing widget into a
named warning. Copy it.


## Where ControlsIF is instantiated

`ControlsIF` is mounted after the node's own `NodeClassIF` is ready, and before
anything reads app state. In `nepi_app_fake_gps` the order is:

1. `nepi_sdk.init_node`, `MsgIF`, class variables -- including `self.controls_if = None`.
2. `NodeClassIF(...)`, then `node_if.wait_for_ready()`.
3. `_setupControls()` -- build `ControlsIF`, then `wait_for_controls_ready(timeout=10)`.
4. `initCb(do_updates=True)` -- read the controls into app state and seed the simulation.
5. `_setupNavpose()` -- the other sub-interface.
6. Worker thread, then the discovery and status timers.

Three ordering constraints make that sequence load-bearing.

`self.controls_if` must be `None` before `NodeClassIF` is constructed. With
`init_configs` set, `NodeClassIF` calls the app's `initCb` during construction,
which is before `ControlsIF` exists. `initCb` therefore runs twice at startup:
once falling back to factory values, once after `_setupControls` with whatever
the config manager restored. Every control read goes through a guarded accessor
that returns a fallback, so the early call is harmless.

`wait_for_controls_ready` must complete before step 4. `ControlsIF` sets its
ready flag at the end of its own `__init__`, after its `NodeClassIF` has come up
and its saved parameters have been applied.

Config restore happens in the right order for free. The node's config tier is
restored during step 2 and a saved node-namespace dump includes the controls
subtree; `ControlsIF`'s own config tier is restored during step 3 and its `init`
then applies the stored values to the controls dict. Node first, controls
second.

`_setupControls` follows the degrade-to-`None` contract the app already uses for
`NavPoseIF`: the constructor is wrapped in `try/except`, a failure logs a
warning and leaves `self.controls_if` as `None`, and every read goes through
`getControlValue(name, fallback)`. The app then runs at factory settings rather
than failing to start.


## Why node_if is left as None, and what breaks if it is shared

`ControlsIF` is given no `node_if`. It builds and owns its own `NodeClassIF` and
calls `wait_for_ready()` on it. This matches how `NavPoseIF` is already mounted
in the same node.

Passing the node's `node_if` instead takes the shared branch, where
`register_pubs`, `register_subs` and `add_params` do a keyed `dict.update()` on
the one shared registry. Per the 2026-07 decision in the workspace `CLAUDE.md`,
a generic key -- `status_pub`, `reset`, `enable`, `capabilities_query` -- then
silently overwrites the entry a sibling interface or the node itself registered.
The overwritten publisher stays advertised on the wire and never publishes
again, and a wrong-type publish is swallowed by a throttled `try/except`, so the
failure is invisible in the logs and shows up only as a dead panel.

`ControlsIF` prefixes its own registry keys with the namespace-derived
`node_if_prefix` (`app_fake_gps_controls_`), and `NavPoseIF` does the same with
`app_fake_gps_navpose_`, so those two would not in fact collide today. That is
not a reason to share. The node's own keys are unprefixed, the prefix scheme is
not enforced anywhere, and separate `NodeClassIF` instances make the question
moot. ROS wire names derive from namespace plus topic, never from the registry
key, so nothing on the wire changes either way.

One consequence of the unshared choice: `ControlsIF` gets its own
`NodeConfigsIF`, which advertises `save_config`, `reset_config` and
`factory_reset_config` on the controls namespace. That is why the app's
`resetCb` and `factoryResetCb` explicitly hand the reset down to
`self.controls_if.reset()` / `.factory_reset()`. Without that, an app-level
reset would leave the controls at their current values.

There is also a live trap in the shared branch: `ControlsIF.unregister()`
iterates `self.SUBS_DICT` and `self.PUBS_DICT`, attributes the class never
defines. The unshared branch calls `unregister_class()` and is unaffected.


## How the updated callback applies a change

`ControlsIF` calls `controls_updated_callback(control_name)` after its dict has
been updated and its status published. The fake GPS app registers
`controlsUpdatedCb`, which does three things in order.

First it calls `applyControls()`, unconditionally. That method is the single
point where a control value becomes running app state -- it reads every value
control and assigns the corresponding attribute. Re-running all of it on every
update is cheap, and it keeps the callback from having to know which control
feeds which attribute.

Second it dispatches on the Button controls by name, calling the same private
command methods the topic callbacks call.

Third it persists and republishes: `node_if.save_config()` for non-Button
controls, preserving the auto-save-on-change behaviour the removed `set_*`
callbacks had, then the app's own `publish_status()`. The config interface
debounces the save onto its own timer, so a slider drag does not write a file
per frame.

The node also writes controls, in two places, and both go through the same
callback. `useCurrentLocation` copies the live simulated position into six
controls; the discovery pass auto-selects the first mavros node it finds.
Recursion terminates at depth two because none of the value-control paths writes
another control. Check that property when you add a Button that writes controls.

Discovered option lists are the node's responsibility. `ControlsIF` carries
whatever option list it is given; it does not discover anything. The fake GPS
app calls `set_control_options('mavros_node', ['None'] + discovered)` whenever
the discovered set changes, and only then sets the value. That order is
mandatory: a `Selection` value that is not in the current option list is
rejected outright and falls back to the first option.


## How the RUI mounts the set

The app component drops its hand-written widgets and renders the set through the
shared `Nepi_IF_Controls`, following the two existing mount examples
(`NepiDeviceLSX-Controls.js` and `Nepi_IF_Process.js`):

    <div style={{ borderTop: "1px solid #ffffff", ... }} />
    <Label title={"Fake GPS Controls"} />
    <NepiIFControls
      namespace={controlsNamespace}
      title={null}
      make_section={false}
      allways_show_controls={true}
    />

The `namespace` prop is the **controls** namespace, not the app namespace. The
component appends `/status` itself and the child widgets append
`/update_control`. `ControlsIF` roots itself one level below the node namespace,
so the app namespace with `"/controls"` appended is what you pass. This is the
single most common way to get an empty control box: a component that renders its
heading and then never fills in, because nothing publishes on the namespace it
subscribed to.

`make_section={false}` inlines the set under the divider and label instead of
wrapping it in its own Section. `title={null}` suppresses the component's
default `"CONTROLS"` heading, which would otherwise print a second label under
the one you just wrote. `allways_show_controls={true}` forces the set open and
hides the "Show Controls" toggle -- right when the controls are the page,
wrong when they are a secondary box on a device page.

What the app still renders itself is the read-only half of the old panel: the
values the node reports and the operator cannot set. In the fake GPS app that is
the mavros connection state, the moving flag, and the live position and heading,
all still read from the app's own status message subscription.

`NepiIFConfig` stays mounted on the **app** namespace, not the controls
namespace. A save there dumps the whole node parameter subtree, which includes
the controls namespace, so one config box still persists and resets the control
values.

Two things not to do. Do not hand-edit `Nepi_IF_Apps.js` or `NepiApps.js`; app
registration is generated from the `RUI_DICT` in the app's params yaml by the
RUI build. If the component file name or main class name does not change, the
`RUI_DICT` does not change either -- the fake GPS migration needed no change
there at all. And do not edit anything under `nepi_rui` to make an app's
migration work; if a widget cannot migrate because it lives in a shared
component, leave it and say so.


## The connect API

The app's `api/connect_app_*.py` is part of the migration, not an afterthought.
State setters that lost their topics become control updates: drop the typed
publisher, add one `update_control` publisher on the controls namespace, and
keep the public method signatures unchanged so existing callers do not break.
`ConnectAppFakeGps.set_enabled`, `.select_mavros_node` and `.set_gps_pub_rate`
all kept their signatures and now publish `UpdateControl`. A generic
`set_control_value(name, value, index=None)` was added alongside them, plus a
`ControlsStatus` subscription so a consumer can read the control values back.
Command publishers stay as they are.


## Follow-on backlog

Six apps remain, in roughly increasing order of difficulty.

`nepi_app_nav_sim` and `nepi_app_file_pub_depthmap` also mount `NavPoseIF` and
follow this pattern directly, including the startup ordering and the
degrade-to-`None` contract described above.

`nepi_app_file_pub_img`, `nepi_app_file_pub_vid` and `nepi_app_onvif_mgr` mount
no sub-interface and are simpler: there is no second interface to order against,
so `ControlsIF` slots in immediately after `NodeClassIF`. Note that
`nepi_app_onvif_mgr` must keep using services rather than topic subscribers for
its device control, for the WS-Discovery reason recorded in this repo's
`CLAUDE.md`; that constraint is about its own control path, not about mounting a
control set.

`nepi_app_image_viewer` is the awkward case. It mounts no sub-interface, but its
widgets live in the shared `NepiIFImageViewersSelector` component under
`nepi_rui` rather than in the app. Migrating it requires a decision about that
shared component first -- whether it gains a controls-driven mode, or whether
the app stops using it -- and that decision is out of scope for an app-local
migration pass.


## Upstream defects encountered

Recorded during the fake GPS migration and not fixed by it. All are outside
`nepi_apps`.

- `nepi_sdk/nepi_controls.py:880` -- `get_bounds()` raises `UnboundLocalError`
  for an unknown control name instead of returning its prepared `[-999,-999]`.
- `nepi_api/system_if.py:428` -- `set_control_max_bound()` calls `set_min_bound`
  with a `max_bound` keyword that function does not accept; it always raises
  `TypeError`. Use `set_control_bounds` instead.
- `nepi_api/system_if.py:332` -- `ControlsIF.unregister()` reads `self.SUBS_DICT`
  and `self.PUBS_DICT`, which the class never defines. Only reachable on the
  shared-`node_if` branch.
- `nepi_sdk/nepi_controls.py:238` -- a variable named `control_type` is assigned
  the control's *name* and then used as a type at lines 351 and 371.
- `nepi_rui/.../Nepi_IF_Control.js:855` -- the fallthrough renders a header label
  for every type including `Button`, so a Button shows its label twice.
- `nepi_api/system_if.py:588` -- `reset()` and `factory_reset()` have the same
  in-memory effect, because both start from `nepi_controls.reset_values()`,
  which restores factory defaults.
- `ControlsIF` holds no lock around `controls_dict`, which is now reachable from
  both a subscriber thread and a timer thread.
