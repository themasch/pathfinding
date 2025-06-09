Experiments in pathfinding build on top of the [bevy](https://bevy.org) game engine.

## TODO

 - [ ] cleanup code, data structures, etc
 - [ ] optimize path
       - use something like that to skip waypoints https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html
 - [ ] debug tool: manually place start & end points, instead of random generation.
 - [ ] make it work in 3D
       A few problems to solve for that: the position of our entities will rarely exactly line up with the navmesh in the Y axis.
       So we need to project the entity position onto it, but deal with overlapping parts of the mesh correctly (e.g. bridges, multi story buildings, ...)
 - [ ] proper cost function. Currently we use the raw distance. Extend this to be able to consider terrain costs, elevation, forbidden areas,...
 - [ ] navmesh generation
 - [ ] make the navmesh its own asset type, independent from renderable 3d meshes. Generating a renderable mesh for debugging should be a special case, not the other way around.
 - [ ] make it generic enough to drop in projects

## Goals for the far future

 - [ ] collision avoidence, multiple agents, maybe swarms/boids? Flow fields?
 - [ ] actually good navmesh generation

## Non-Goals

This does not aim to become a generic plugin for path finding with bevy. Use something based on [pathfinding](https://crates.io/crates/pathfinding) for that.
It also will not provide progress for path finding research. It will not invent something great, it wont be the fasted, best, most correct,... pathfinding solution.
