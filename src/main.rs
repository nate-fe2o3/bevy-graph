use avian2d::{math::PI, prelude::*};
use bevy::prelude::*;
use bevy_cursor::{CursorLocation, TrackCursorPlugin};
use bevy_egui::EguiPlugin;
use rand::Rng;
fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            MeshPickingPlugin,
            TrackCursorPlugin,
        ))
        .insert_gizmo_config(
            PhysicsGizmos {
                joint_separation_color: Some(Color::WHITE),
                collider_color: None,
                ..default()
            },
            GizmoConfig::default(),
        )
        .add_plugins(EguiPlugin::default())
        .insert_resource(Gravity(Vec2::splat(0.)))
        .insert_resource(Iterations(0))
        .insert_resource(Config::default())
        .add_systems(Startup, setup)
        .add_systems(Update, update) //.run_if(below_cutoff))
        .add_systems(PostUpdate, process_delta_v) //.run_if(below_cutoff))
        .add_event::<DeltaV>()
        .run();
}

const IDEAL_LENGTH: f32 = 50.;
const COOLING_FACTOR: f32 = 0.2;
const NODE_TOTAL: usize = 50;
const NODE_MASS: f32 = 5.;
const COMPLIANCE: f32 = 0.001;
const COLLIDER_RADIUS: f32 = 49.;

#[derive(Resource)]
struct Config {
    ideal_length: f32,
    cooling_factor: f32,
    node_mass: f32,
    compliance: f32,
    node_total: usize,
    collider_radius: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            ideal_length: IDEAL_LENGTH,
            cooling_factor: COOLING_FACTOR,
            node_mass: NODE_MASS,
            compliance: COMPLIANCE,
            node_total: NODE_TOTAL,
            collider_radius: COLLIDER_RADIUS,
        }
    }
}

#[derive(Event)]
struct DeltaV(Entity, Vec2);

#[derive(Resource)]
struct Iterations(usize);

#[derive(Component)]
struct Node;

fn setup(
    mut cmd: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    config: Res<Config>,
) {
    let damping = 1. / config.cooling_factor;
    cmd.spawn(Camera2d);
    let mut ids = Vec::new();
    let mut observer = Observer::new(move_on_drag);
    for _ in 0..config.node_total {
        let angle = rand::thread_rng().gen_range(-PI..=PI);
        // get screen size, max space btwn
        let distance = rand::thread_rng().gen_range(0. ..500.);
        let pos = Vec2::from_angle(angle) * distance;
        let id = cmd
            .spawn((
                Node,
                Mesh2d(meshes.add(Circle::new(5.))),
                MeshMaterial2d(materials.add(Color::hsl(1., 1., 1.))),
                Transform::from_translation(Vec3::new(pos.x, pos.y, 0.)),
                RigidBody::Dynamic,
                Collider::circle(config.collider_radius),
                Mass(config.node_mass),
                Sensor,
                LinearVelocity::default(),
                LinearDamping(damping),
                CollisionEventsEnabled,
            ))
            .id();
        observer.watch_entity(id);
        ids.push(id);
    }
    cmd.spawn(observer);
    let mut ids_iter = ids.iter();
    while let Some(x) = ids_iter.next() {
        let Some(n1) = ids_iter.next() else {
            return;
        };
        let Some(n2) = ids_iter.next() else {
            return;
        };
        cmd.spawn(
            DistanceJoint::new(*x, *n1)
                .with_rest_length(config.ideal_length)
                .with_compliance(config.compliance),
        );
        cmd.spawn(
            DistanceJoint::new(*x, *n2)
                .with_rest_length(config.ideal_length)
                .with_compliance(config.compliance),
        );
    }
}

fn update(
    mut ev_w: EventWriter<DeltaV>,
    mut coll_reader: EventReader<CollisionStarted>,
    query: Query<&Transform>,
    mut i: ResMut<Iterations>,
) {
    i.0 += 1;
    for &CollisionStarted(a_id, b_id) in coll_reader.read() {
        dbg!(&a_id, &b_id);
        let a = query.get(a_id).expect("entity A to exist").translation.xy();
        let b = query.get(b_id).expect("entity B to exist").translation.xy();
        let (rep_a, rep_b) = repulsive_force(a, b);
        ev_w.write_batch([DeltaV(a_id, rep_a), DeltaV(b_id, rep_b)]);
    }
}

fn process_delta_v(
    mut ev_r: EventReader<DeltaV>,
    mut query: Query<&mut LinearVelocity>,
    mut i: ResMut<Iterations>,
) {
    i.0 += 1;
    for DeltaV(id, dv) in ev_r.read() {
        let mut v = query.get_mut(*id).unwrap();
        v.0 += dv;
    }
}

fn repulsive_force(a: Vec2, b: Vec2) -> (Vec2, Vec2) {
    if a == b {
        let force = IDEAL_LENGTH.powi(2);
        let angle = rand::thread_rng().gen_range(-PI..=PI);
        return (
            Vec2::from_angle(angle) * force,
            Vec2::from_angle(angle + PI) * force,
        );
    }
    let diff = a - b;
    let angle = diff.to_angle();
    let force = IDEAL_LENGTH.powi(2) / diff.length();
    (
        Vec2::from_angle(angle) * force,
        Vec2::from_angle(angle + PI) * force,
    )
}

fn move_on_drag(
    trigger: Trigger<Pointer<Drag>>,
    mut transforms: Query<&mut Transform>,
    cursor: Res<CursorLocation>,
) {
    let mut transform = transforms.get_mut(trigger.target).unwrap();
    let Some(pos) = cursor.world_position() else {
        return;
    };
    transform.translation = Vec3::new(pos.x, pos.y, 0.);
}
