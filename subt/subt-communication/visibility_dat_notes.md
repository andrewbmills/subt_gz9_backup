# Subt Comms Visibility Table DAT generation

## World Boundary

Before generating the .dat file, make sure that the tiles in the world are
within the bounds defined by the `kMin` and `kMax` constants in:

    <path_to_subt>/subt-communication/subt_gazebo_los_model/include/subt_gazebo_los_model/VisibilityTable.hh

If not, modify these variables so that they are large enough to cover the
entire world and rebuild.

## Generate the DAT file

Check out the `gazebo9` branch in subt. Copy the .dot and .world file into
`<path_to_subt>/subt_gazebo/worlds` then rebuild + install.

Important: Make sure you have all the models referred to by the .world file in
your gazebo models path. If you already have them in your fuel cache
(~/.ignition/fuel), you can convert them to gazebo format by following these
steps:

1. copy the model to a temporary directory, e.g.

    mkdir /tmp/fuel_models
    cp -r  ~/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/* /tmp/fuel_models

1. convert model.sdf resource path and update model directory layout

    cd /tmp/fuel_models
    find . -name "*.sdf" -exec sed -i 's/file:\/\/.*\/models\/\(.*\)\/[0-9]\//model:\/\/\1\//g' {} \;
    for d in */; do v=$(ls "$d"); mv  "$d"/$v/* "$d"; done

1. set GAZEBO_MODEL_PATH to this temporary directory or copy them to ~/.gazebo/models


Launch world for generating the .dat file:

    roslaunch subt_gazebo_los_model visibility.launch scenario:=<world_name>

This should create a .dat file in the `<install_dir>/share/subt_gazebo/worlds/`
directory.

## Visualize comms

To test the world with dat file, launch subt:

    roslaunch subt_gazebo quickstart.launch scenario:=<world_name>

Publish a msg to enable comms visualization:

    ign service -s /subt/comms_model/visualize --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "X1"' --force-version 4.0.0

This enables visualization of comms for X1 and you should see colored point
clouds in the tiles around X1, which represent the regions that X1's comms can
reach. Use the cost caculation method below to verify that comms created from
the dat file is correct.

### Cost caculation

Here are the costs along the edges between dfferent type of nodes (tiles) in
the dot graph:

```
s <-> s:  1
s <-> i:  3
i <-> i:  6
```

where `i` refers to intersection (including bend and vertical shaft) tiles and
`s` refers to straight tiles and everything else. The max cost available is 15
for tunnel circuit and 10 for urban circuit.

### visualize dot file

    xdot <world_name>.dot

