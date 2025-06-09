use bevy::{
    color::palettes::css::WHITE,
    pbr::wireframe::{WireframeConfig, WireframePlugin},
    prelude::*,
};
use bevy_obj::ObjPlugin;
use std::f32::consts::PI;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, ObjPlugin, WireframePlugin::default()))
        .insert_resource(WireframeConfig {
            global: true,
            default_color: WHITE.into(),
        })
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
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
        MeshMaterial3d(navmesh_material_handle.clone()),
        Transform::IDENTITY.with_scale(Vec3::new(4.0, 4.0, 4.0)),
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
        Transform::from_xyz(2.0, 10.0, 2.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
