use bevy::{
    color::palettes::css::{CRIMSON, WHITE},
    gizmos,
    input::common_conditions::*,
    math::sampling::UniformMeshSampler,
    pbr::wireframe::{WireframeConfig, WireframePlugin},
    prelude::*,
    state::commands,
};
use bevy_obj::ObjPlugin;
use rand::distributions::Distribution;
use rand::rngs::ThreadRng;
use std::{
    cell::RefCell,
    cmp::Ordering,
    collections::{BinaryHeap, HashMap},
    f32::{INFINITY, consts::PI},
    rc::Rc,
    time::Instant,
};

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, ObjPlugin, WireframePlugin::default()))
        .insert_resource(WireframeConfig {
            global: true,
            default_color: WHITE.into(),
        })
        .add_event::<PathFindingRequest>()
        .add_event::<PathFound>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (generate_new_nav_graph, find_path, spawn_found_path),
        )
        .add_systems(
            Update,
            regenerate_test_route_parameters.run_if(input_just_pressed(KeyCode::KeyR)),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    ass: Res<AssetServer>,
) {
    let navmesh = ass.load("navmesh.obj");

    let navmesh_material_handle = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        ..default()
    });

    commands.spawn((
        Mesh3d(navmesh),
        NavRoot,
        MeshMaterial3d(navmesh_material_handle.clone()),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.025))),
        MeshMaterial3d(navmesh_material_handle.clone()),
        Start,
    ));

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.0125))),
        MeshMaterial3d(navmesh_material_handle.clone()),
        End,
    ));

    // light
    commands.spawn((
        DirectionalLight {
            illuminance: light_consts::lux::OVERCAST_DAY,
            shadows_enabled: true,
            ..default()
        },
        Transform {
            translation: Vec3::new(0.0, 2.0, 0.0),
            rotation: Quat::from_rotation_x(-PI / 4.),
            ..default()
        },
    ));

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 1.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

#[derive(Component)]
struct NavRoot;

#[derive(Component, Debug)]
struct NavGraph {
    nodes: Vec<NavNode>,
    triangles: Vec<(NodeIndex, NodeIndex, NodeIndex)>,
}

type NodeIndex = usize;
type TravelCost = f32;

#[derive(Debug)]
struct NavNode {
    pos: Vec3,
    connections: Vec<(NodeIndex, TravelCost)>,
    triangles: Vec<NodeIndex>,
}

impl NavNode {
    fn cost_to(&self, other: &NodeIndex) -> Option<TravelCost> {
        self.connections
            .iter()
            .find(|(n, _)| n == other)
            .map(|(_, c)| *c)
    }

    fn path_end(pos: Vec3, tri: (NodeIndex, NodeIndex, NodeIndex), graph: &NavGraph) -> Self {
        NavNode {
            pos: pos,
            connections: vec![
                (tri.0, graph.nodes[tri.0].pos.distance(pos)),
                (tri.1, graph.nodes[tri.1].pos.distance(pos)),
                (tri.2, graph.nodes[tri.2].pos.distance(pos)),
            ],
            triangles: vec![],
        }
    }
}

impl NavNode {
    fn new(pos: Vec3) -> Self {
        NavNode {
            pos,
            connections: Vec::new(),
            triangles: Vec::new(),
        }
    }
}

fn generate_new_nav_graph(
    qry: Query<(&Mesh3d, Entity), (With<NavRoot>, Without<NavGraph>)>,
    meshes: Res<Assets<Mesh>>,
    mut cmds: Commands,
) {
    for (mesh, entity) in qry {
        println!("building new nav graph for {:?} {:?}", mesh.id(), entity);
        let start = Instant::now();
        let cmesh: &Mesh = meshes.get(mesh.id()).unwrap();
        let tris = cmesh.triangles().unwrap();
        let mut node_map = HashMap::<(u32, u32, u32), NodeIndex>::new();
        let mut nodes = Vec::new();
        let mut triangles = Vec::new();
        println!("vetex count: {}", cmesh.count_vertices());
        println!("tri count: {}", cmesh.triangles().unwrap().count());
        for t in tris {
            let v0 = t.vertices[0];
            let v1 = t.vertices[1];
            let v2 = t.vertices[2];

            let v0_idx = register_node(v0, &mut node_map, &mut nodes);
            let v1_idx = register_node(v1, &mut node_map, &mut nodes);
            let v2_idx = register_node(v2, &mut node_map, &mut nodes);

            let t_idx = triangles.len();
            triangles.push((v0_idx, v1_idx, v2_idx));

            let dv0v1 = v0.distance(v1);
            let dv0v2 = v0.distance(v2);
            let dv1v2 = v1.distance(v2);

            nodes[v0_idx].triangles.push(t_idx);
            nodes[v1_idx].triangles.push(t_idx);
            nodes[v2_idx].triangles.push(t_idx);
            // 3x 8b

            nodes[v0_idx].connections.push((v1_idx, dv0v1));
            nodes[v0_idx].connections.push((v2_idx, dv0v2));
            nodes[v1_idx].connections.push((v0_idx, dv0v1));
            nodes[v1_idx].connections.push((v2_idx, dv1v2));
            nodes[v2_idx].connections.push((v0_idx, dv0v2));
            nodes[v2_idx].connections.push((v1_idx, dv1v2));
            // 6x 16b
        }
        println!("nodes: {:?}", nodes);
        let graph = NavGraph { nodes, triangles };

        println!("size: {:?}", size_of_val(&graph));
        cmds.entity(entity).insert(graph);

        println!("took: {:?}", start.elapsed());
    }
}

fn register_node(
    v: Vec3,
    node_map: &mut HashMap<(u32, u32, u32), NodeIndex>,
    nodes: &mut Vec<NavNode>,
) -> NodeIndex {
    *node_map
        .entry((v.x.to_bits(), v.y.to_bits(), v.z.to_bits()))
        .or_insert_with(|| {
            let len = nodes.len();
            nodes.push(NavNode::new(v));
            len
        })
}

#[derive(Component)]
struct Start;
#[derive(Component)]
struct End;

fn regenerate_test_route_parameters(
    nav_mesh: Query<&Mesh3d, (With<NavRoot>, With<NavGraph>)>,
    mut route_start: Query<&mut Transform, (With<Start>, Without<End>)>,
    mut route_end: Query<&mut Transform, (With<End>, Without<Start>)>,
    mut ev_request_path: EventWriter<PathFindingRequest>,
    path_gizmos: Query<Entity, With<Gizmo>>,
    mut commands: Commands,
    meshes: Res<Assets<Mesh>>,
) {
    let mut start = route_start.single_mut().unwrap();
    let mut end = route_end.single_mut().unwrap();

    let mesh = nav_mesh.single().unwrap();
    let cmesh: &Mesh = meshes.get(mesh.id()).unwrap();
    let sampler = UniformMeshSampler::try_new(cmesh.triangles().unwrap()).unwrap();

    let mut rng = ThreadRng::default();

    let a = sampler.sample(&mut rng);
    let b = sampler.sample(&mut rng);

    start.translation = a.clone();
    end.translation = b.clone();

    ev_request_path.send(PathFindingRequest(a, b));

    for entity in path_gizmos.iter() {
        commands.entity(entity).despawn();
    }
}

#[derive(Event)]
struct PathFindingRequest(Vec3, Vec3);

#[derive(Event)]
struct PathFound(Path);

fn find_path(
    mut ev_path_requests: EventReader<PathFindingRequest>,
    mut ev_path_found: EventWriter<PathFound>,
    nav_mesh: Query<&NavGraph, With<NavRoot>>,
) {
    let Ok(nav_graph) = nav_mesh.single() else {
        return;
    };

    for PathFindingRequest(from, to) in ev_path_requests.read() {
        let start = Instant::now();
        let path = astar(&nav_graph, *from, *to);
        if let Some(path) = path {
            println!("found path: {}, took {:?}", path.cost(), start.elapsed());
            ev_path_found.write(PathFound(path));
        } else {
            println!("found no path");
        }
    }
}

fn spawn_found_path(
    mut ev_path_found: EventReader<PathFound>,
    mut commands: Commands,
    mut gizmo_assets: ResMut<Assets<GizmoAsset>>,
) {
    for PathFound(path) in ev_path_found.read() {
        let mut path_gizmo = GizmoAsset::default();
        path_gizmo.linestrip(path.points.iter().map(|n| n.pos), CRIMSON);
        commands.spawn(Gizmo {
            handle: gizmo_assets.add(path_gizmo),
            line_config: GizmoLineConfig {
                width: 5.0,
                ..default()
            },
            ..default()
        });
    }
}

fn find_triangles(
    graph: &NavGraph,
    from: Vec3,
    to: Vec3,
) -> (Option<NodeIndex>, Option<NodeIndex>) {
    let mut start_tri = None;
    let mut end_tri = None;
    for (i, tri) in graph.triangles.iter().enumerate() {
        if start_tri.is_some() && end_tri.is_some() {
            return (start_tri, end_tri);
        }
        if start_tri.is_none()
            && point_in_triangle(
                from,
                graph.nodes[tri.0].pos,
                graph.nodes[tri.1].pos,
                graph.nodes[tri.2].pos,
            )
        {
            start_tri = Some(i);
        }

        if end_tri.is_none()
            && point_in_triangle(
                to,
                graph.nodes[tri.0].pos,
                graph.nodes[tri.1].pos,
                graph.nodes[tri.2].pos,
            )
        {
            end_tri = Some(i);
        }
    }

    return (start_tri, end_tri);
}

struct Waypoint {
    pos: Vec3,
    node: Option<NodeIndex>,
}

impl Waypoint {
    fn for_end(pos: Vec3) -> Self {
        Self { pos, node: None }
    }

    fn for_node(pos: Vec3, node: NodeIndex) -> Self {
        Self {
            pos,
            node: Some(node),
        }
    }
}

struct Path {
    points: Vec<Waypoint>,
}

impl Path {
    fn cost(&self) -> f32 {
        self.points
            .windows(2)
            .fold(0.0, |acc, pair| acc + pair[0].pos.distance(pair[1].pos))
    }
}

impl From<Vec<Waypoint>> for Path {
    fn from(points: Vec<Waypoint>) -> Self {
        Path { points }
    }
}

fn astar(graph: &NavGraph, from: Vec3, to: Vec3) -> Option<Path> {
    let distance = from.distance(to);
    println!(
        "need to find a path from {:?} to {:?}, direct distance: {}",
        from, to, distance
    );
    let start = Instant::now();

    let (start_tri, end_tri) = find_triangles(graph, from, to);

    let start_tri_index = start_tri?;
    let end_tri_index = end_tri?;

    println!(
        "found triangles that contain our start and end points: {:?} {:?} (took: {:?})",
        start_tri,
        end_tri,
        start.elapsed()
    );

    if start_tri_index == end_tri_index {
        println!(
            "we got a simple path, straight line: {:?} {:?} (distance: {})",
            from,
            to,
            from.distance(to)
        );

        return Some(
            vec![
                Waypoint {
                    pos: from,
                    node: None,
                },
                Waypoint {
                    pos: to,
                    node: None,
                },
            ]
            .into(),
        );
    }

    let start_tri = graph.triangles[start_tri_index];
    let end_tri = graph.triangles[end_tri_index];
    let goal_nodes = [end_tri.0, end_tri.1, end_tri.2];

    let start_node = NavNode::path_end(from, start_tri, graph);
    let end_node = NavNode::path_end(to, end_tri, graph);

    let mut pfnodes = HashMap::new();
    let mut open_set = BinaryHeap::new();
    let mut came_from = HashMap::new();
    for (node_index, cost) in start_node.connections {
        let node = &graph.nodes[node_index];
        let g = cost;
        let f = g + node.pos.distance(end_node.pos);
        let node = Rc::new(RefCell::new(AStarNode {
            g,
            f,
            node: NodeKey::GraphNode(node_index),
        }));
        pfnodes.insert(NodeKey::GraphNode(node_index), node.clone());
        open_set.push(node.clone());
        came_from.insert(NodeKey::GraphNode(node_index), NodeKey::Start);
    }

    while !open_set.is_empty() {
        let current_rc = open_set.pop().unwrap();
        let current = current_rc.borrow();

        if current.node == NodeKey::End {
            let p = reconstruct_path(&came_from, current.node);
            return Some(build_waypoints(p, graph, from, to));
        }

        let NodeKey::GraphNode(current_index) = current.node else {
            panic!("expected a graph node, got: {:?}", current.node);
        };

        let node = &graph.nodes[current_index];

        for (n, cost) in &node.connections {
            let neighbor_index = NodeKey::GraphNode(*n);
            let neighbor_g = &graph.nodes[*n];
            let neighbor_a_rc = pfnodes.entry(neighbor_index).or_insert_with(|| {
                Rc::new(RefCell::new(AStarNode {
                    g: INFINITY,
                    f: INFINITY,
                    node: neighbor_index,
                }))
            });
            let mut neighbor_a = neighbor_a_rc.borrow_mut();

            let tg = current.g + cost;

            if tg < neighbor_a.g {
                neighbor_a.g = tg;
                neighbor_a.f = tg + neighbor_g.pos.distance(end_node.pos);
                came_from.insert(neighbor_index, current.node);
                drop(neighbor_a);
                open_set.push(neighbor_a_rc.clone());
            }
        }

        if goal_nodes.contains(&current_index) {
            let neighbor_g = &end_node;
            let neighbor_a_rc = pfnodes.entry(NodeKey::End).or_insert_with(|| {
                Rc::new(RefCell::new(AStarNode {
                    g: INFINITY,
                    f: INFINITY,
                    node: NodeKey::End,
                }))
            });
            let mut neighbor_a = neighbor_a_rc.borrow_mut();

            let tg = current.g + neighbor_g.cost_to(&current_index).unwrap_or(f32::INFINITY);

            if tg < neighbor_a.g {
                neighbor_a.g = tg;
                neighbor_a.f = tg + neighbor_g.pos.distance(end_node.pos);
                came_from.insert(NodeKey::End, current.node);
                drop(neighbor_a);
                open_set.push(neighbor_a_rc.clone());
            }
        }
    }

    None
}

fn build_waypoints(p: Vec<NodeKey>, graph: &NavGraph, from: Vec3, to: Vec3) -> Path {
    p.iter()
        .map(|nidx| match nidx {
            NodeKey::GraphNode(idx) => Waypoint {
                pos: graph.nodes[*idx].pos,
                node: Some(*idx),
            },
            NodeKey::End => Waypoint::for_end(to),
            NodeKey::Start => Waypoint::for_end(from),
        })
        .collect::<Vec<Waypoint>>()
        .into()
}

fn reconstruct_path(came_from: &HashMap<NodeKey, NodeKey>, last_node: NodeKey) -> Vec<NodeKey> {
    let mut path = Vec::new();
    let mut current = last_node;

    while let Some(&prev) = came_from.get(&current) {
        path.push(current);
        current = prev;
    }

    path.push(current);
    path.reverse();
    path
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum NodeKey {
    Start,
    End,
    GraphNode(NodeIndex),
}

#[derive(Debug, Copy, Clone)]
struct AStarNode {
    // curremtly known cheapest cost from start to here
    g: f32,
    // estimates the cheapest complete path start to end
    f: f32,
    node: NodeKey,
}

impl PartialEq for AStarNode {
    fn eq(&self, other: &Self) -> bool {
        self.f.total_cmp(&other.f) == Ordering::Equal
    }
}

impl Eq for AStarNode {}

impl Ord for AStarNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f.total_cmp(&self.f)
    }
}

impl PartialOrd for AStarNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // reveresed because we want a  min-heap
        Some(other.g.total_cmp(&self.g))
    }
}

fn sign(p1: Vec3, p2: Vec3, p3: Vec3) -> f32 {
    return (p1.x - p3.x) * (p2.z - p3.z) - (p2.x - p3.x) * (p1.z - p3.z);
}

fn point_in_triangle(pt: Vec3, v1: Vec3, v2: Vec3, v3: Vec3) -> bool {
    let d1 = sign(pt, v1, v2);
    let d2 = sign(pt, v2, v3);
    let d3 = sign(pt, v3, v1);

    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

    return !(has_neg && has_pos);
}
