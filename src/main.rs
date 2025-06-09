use bevy::{
    color::palettes::css::WHITE,
    input::common_conditions::*,
    math::sampling::UniformMeshSampler,
    pbr::wireframe::{WireframeConfig, WireframePlugin},
    prelude::*,
    state::commands,
};
use bevy_obj::ObjPlugin;
use rand::distributions::Distribution;
use rand::rngs::ThreadRng;
use std::{collections::HashMap, f32::consts::PI, time::Instant};

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, ObjPlugin, WireframePlugin::default()))
        .insert_resource(WireframeConfig {
            global: true,
            default_color: WHITE.into(),
        })
        .add_systems(Startup, setup)
        .add_systems(Update, generate_new_nav_graph)
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
    connections: Vec<(NodeIndex, TravelCost)>,
    triangles: Vec<NodeIndex>,
}

impl NavNode {
    fn new() -> Self {
        NavNode {
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

            let dv0v1 = v0.distance_squared(v1);
            let dv0v2 = v0.distance_squared(v2);
            let dv1v2 = v1.distance_squared(v2);

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
            nodes.push(NavNode::new());
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

    start.translation = a;
    end.translation = b;
}
