# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [3.0.3-preview.6] - 2020-02-05

### Bug Fixes

- Fix a bug where ProBuilder could still snap movement after closing ProGrids.

## [3.0.3-preview.5] - 2020-01-14

### Bug Fixes

- Fix warnings with Unity 2019.3.

## [3.0.3-preview.4] - 2019-01-23

### Bug Fixes

- Fix grid not rendering in SRP projects.
- Fix ProBuilder still applying snap values when ProGrids is closed.
- Fix case where orbiting out of iso view would not reset the render plane.
- Fix bug where ProBuilder would sometimes not snap new objects to the ProGrids grid.
- Fix bug where single-key shortcuts would override Unity shortcuts.
- Remove `About Window`.
- Fix bug where rotating to isographic view would not reset the projection grid.
- Fix X and Z grid colors being swapped.
- Consistently use locale setting aware strings when storing preferences.

## [3.0.2-preview] - 2018-05-08

### Bug Fixes

- Fix single key shortcuts interfering with scene navigation.
- Fix scene interface rendering with a dark tint in Unity 2018.2 beta.

## [3.0.1-preview] - 2018-04-30

### Features

- New About window.
- Now distributed as Package Manager module.
- Project now uses Assembly Definition files to reduce compilation overhead.
- Add a shortcut to reset the snap multiplier (alpha numeric 0 by default).

### Bug Fixes

- Temporary objects are now longer created in scene files.
- Fix grid rendering on top of UI elements.
- Remove duplicate snap on scale pref in Prefences pane.
- Remove duplicate preference fields from in-scene settings window.
- Fix single key shortcuts not working.
- Fix snapping multiple objects not undoing to original state.
- Fix multiple objects snapping to first selected transform instead of the active transform.

### Changes

- Change color of "close" button to light blue.
- Remove `pg_` suffix from class and file names.
- Remove automatic About Window popup on update.
- `Alpha 0` shortcut now resets grid size modifier and offset.
- Fix reset shortcut not affecting the size modifier in some cases.

## [2.5.0-f.0] - 2017-08-03

### Features

- Single key shortcuts now configurable via preferences.

### Bug Fixes

- Don't prevent compiling to Windows Store target.
- Single key shortcuts no long beep on Mac.
- Fix null reference error if GameObject has a null component.

## [2.4.1-f.0] - 2017-04-03

### Bug Fixes

- Prevent About Window from opening other tool changelogs.

## [2.4.0-f.0] - 2017-03-29

### Features

- Add `pg_IgnoreSnapAttribute` and `ProGridsConditionalSnapAttribute` to disable or conditionally disable snapping on objects.
- Increase accessible grid multiplier range.

### Bug Fixes

- Fix sRGB import settings on icons.
- Prevert overflow when increasing grid multiplier.

## [2.3.0-f.0] - 2016-12-30

### Features

- Add option to set major line increment.
- Automatically hide and show the Unity grid when opening / closing ProGrids.

### Bug Fixes

- Fix bug where ProGrids could fail to find icons when root folder is moved.
- Fix bug where ProGrids would not remember it's state between Unity sessions.

### Changes

- Slightly increase opacity of default grid colors.

## [2.2.7-f.0]

### Bug Fixes

- Fix cases where `Snap on Selected Axes` would sometimes be unset.

## [2.2.6-f.0]

### Bug Fixes

- Fix warnings in Unity 5.4 regarding API use during serialization.

## [2.2.5-f.0]

### Bug Fixes

- Fix an issue where ProGrids would not stay open across Unity restarts.

## [2.2.4-f.0]

### Bug Fixes

- Fix issue where adjusting grid offset would not repaint grid.
- Attempt to load GUI resources on deserialization, possibly addressing issues with menu icons not loading.

## [2.2.3-f.0]

### Bug Fixes

- If icons aren't found, search the project for matching images (allows user to rename or move ProGrids folder).
- Make menu usable even if icons aren't found in project.
- Fix bug where grid would fail to render on Mac.
- Improve performance of grid rendering and increase draw distance.

## [2.2.2-f.0]

### Bug Fixes

- Fix possible leak in pg_GridRenderer.
- Fix 10th line highlight being lost on script reload.
- Remember open/closed state between Unity loads.
- Fix bug where multiple ProGrids instances could potentially be instantiated.

## [2.2.1-f.0]

### Features

- New interface jettisons bulky Editor Window in favor of a minimal dropdown in the active sceneview.
- New "Predictive Grid" option will automatically change the grid plane to best match the current movement.
- Add option to snap all selected objects independently of on another (toggle off "Snap as Group").

### Bug Fixes

- Improve support for multiple open scene view windows.
- Respect local rotation when calculating snap value.

## [2.1.7-f.0]

### Features

- Add preference to enabled snapping scale values.

## [2.1.6-p.2]

### Features

- Unity 5 compatibility.
- Add documentation PDF.

### Bug Fixes

- Fix Upgradable API warning.
- Fix version marking in About.

## [2.1.5-f.0]

### Bug Fixes

- Fix crash on OSX in Unity 5.
- Remember grid position when closing and re-opening ProGrids.
- Grid lines no longer render on top of geometry in Deferred Rendering.
- Improve performance of Editor when rendering perspective grids.

## [2.1.4-f.0]

### Bug Fixes

- Remember On/Off state when closing window.
- ProBuilder now respects temporary snapping disable toggle.
- ProBuilder now respects temporary axis constraint toggles.
- Snap value resolution now retained when using -/+ keys to increase or decrease size.

### Changes

- Remove deprecated SixBySeven.dll.
- Remove unused font from Resources folder.

## [2.1.3-f.0]

### Bug Fixes

- Catch instance where GridRenderer would not detect Deferred Rendering path, causing grid to appear black and spotty.
- Remember grid show/hide preferences across Unity launches.

## [2.1.2-f.0]

### Bug Fixes

- Fix missing grid when using Deferred Rendering path.
- Fix conflicting shortcut for toggle axis constraints.

## [2.1.1-f.0]

### Features

- New perspective plane grids.
- New perspective 3d grid.
- Redesigned interface
- New `[` and `]` shortcuts decrease and increase grid resolution.
- New `-` and `+` shortcuts move 3d plane grid forwards and backwards along axis.
- New `\` shortcut key to toggle between orthographic axis and perspective modes.
- Improve orthographic grid rendering performance.
- Highlight every 10th line.
- New preference toggles use of Axis Constraints while dragging objects (use 'C' key to invert preference on the fly).
- Shiny new About window.

### Bug Fixes

- Update grid in real time while modifying preferences.
- Catch a rare divide by zero exception on Unity 3.

### Changes
- Move ProGrids from 6by7 folder to ProCore.
- Use new `ProCore.dll` library instead of `SixBySeven.dll`.
