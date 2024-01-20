use cgmath::{Basis3, Deg, EuclideanSpace, InnerSpace, Rad, Rotation, Rotation3};
use scan_fmt::scan_fmt;
use std::{error::Error, marker::PhantomData};
use strum::EnumDiscriminants;
use strum_macros::{IntoStaticStr, EnumString};
use uom::{si::f64, si::{angle, angular_velocity, length}};

pub use cgmath;
pub use uom;

/// Arithmetic mean radius (R1) as per IUGG.
pub const EARTH_RADIUS_M: f64 = 6_371_008.8; // TODO: convert to const `length::meter` once supported

#[derive(Clone, Debug)]
pub struct LatLon {
    pub lat: Deg<f64>,
    pub lon: Deg<f64>
}

impl LatLon {
    pub fn new(lat: Deg<f64>, lon: Deg<f64>) -> LatLon {
        LatLon{ lat, lon }
    }
}

pub struct GeoPos {
    pub lat_lon: LatLon,
    pub elevation: f64::Length
}

pub trait FrameOfReference {}

/// Global frame of reference; origin at Earth's center, X points to lat. 0°/lon. 0°, Y points to lat. 0°/lon. 90°,
/// Z points to the North Pole.
#[derive(Clone, Debug)]
pub struct Global;

/// Observer's local frame of reference; X points north, Y points west, Z points up.
#[derive(Clone, Debug)]
pub struct Local;

impl FrameOfReference for Global {}
impl FrameOfReference for Local {}

#[derive(Clone, Debug)]
pub struct Point3<S, T: FrameOfReference>(pub cgmath::Point3<S>, PhantomData<T>);

impl<S, T: FrameOfReference> Point3<S, T> {
    pub fn from(p: cgmath::Point3<S>) -> Point3<S, T> {
        Point3(p, Default::default())
    }

    pub fn from_xyz(x: S, y: S, z: S) -> Point3<S, T> {
        Point3(cgmath::Point3::new(x, y, z), Default::default())
    }
}

#[derive(Clone, Debug)]
pub struct Vector3<S, T: FrameOfReference>(pub cgmath::Vector3<S>, PhantomData<T>);

impl<S, T: FrameOfReference> Vector3<S, T> {
    pub fn from(p: cgmath::Vector3<S>) -> Vector3<S, T> {
        Vector3(p, Default::default())
    }

    pub fn from_xyz(x: S, y: S, z: S) -> Vector3<S, T> {
        Vector3(cgmath::Vector3::new(x, y, z), Default::default())
    }
}

#[derive(Clone, Debug)]
pub struct TargetInfoMessage {
    pub position: Point3<f64, Local>,
    pub velocity: Vector3<f64, Local>, // m/s
    pub track: Deg<f64>
}

impl std::str::FromStr for TargetInfoMessage {
    type Err = Box<dyn Error>;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (x, y, z, vx, vy, vz, track) = scan_fmt!(s, "{};{};{};{};{};{};{}", f64, f64, f64, f64, f64, f64, f64)?;

        Ok(TargetInfoMessage{
            position: Point3::<f64, Local>::from_xyz(x, y, z),
            velocity: Vector3::<f64, Local>::from_xyz(vx, vy, vz),
            track: Deg(track)
        })
    }
}

impl std::fmt::Display for TargetInfoMessage  {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        write!(
            f, "{:.1};{:.1};{:.1};{:.1};{:.1};{:.1};{:.1}\n",
            self.position.0.x, self.position.0.y, self.position.0.z,
            self.velocity.0.x, self.velocity.0.y, self.velocity.0.z,
            self.track.0
        )
    }
}

#[derive(Debug, EnumDiscriminants)]
#[strum_discriminants(derive(IntoStaticStr, EnumString))]
pub enum MountSimulatorMessage {
    GetPosition,
    Info(String),
    Position(Result<(f64::Angle, f64::Angle), Box<dyn Error>>),
    Reply(Result<(), Box<dyn Error>>),
    Slew(f64::AngularVelocity, f64::AngularVelocity),
    Stop,
}

impl MountSimulatorMessage {
    fn ok() -> &'static str { "OK" }
    fn error() -> &'static str { "Error" }
}

impl std::fmt::Display for MountSimulatorMessage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        type Msg = MountSimulatorMessage;
        let name = Into::<&str>::into(MountSimulatorMessageDiscriminants::from(self));

        match self {
            MountSimulatorMessage::GetPosition => write!(f, "{}\n", name),

            MountSimulatorMessage::Info(s) => write!(f, "{};{}\n", name, s),

            MountSimulatorMessage::Position(result) => match result {
                Ok((axis1_pos, axis2_pos)) => write!(
                    f,
                    "{};{};{};{}\n",
                    name,
                    Msg::ok(),
                    axis1_pos.get::<angle::degree>(),
                    axis2_pos.get::<angle::degree>()
                ),
                Err(e) => write!(f, "{};{};{}\n", name, Msg::error(), e)
            },

            MountSimulatorMessage::Reply(result) => match result {
                Ok(()) => write!(f, "{};{}\n", name, Msg::ok()),
                Err(e) => write!(f, "{};{};{}\n", name, Msg::error(), e)
            },

            MountSimulatorMessage::Slew(axis1, axis2) => write!(
                f,
                "{};{};{}\n",
                name,
                axis1.get::<angular_velocity::degree_per_second>(),
                axis2.get::<angular_velocity::degree_per_second>()
            ),

            MountSimulatorMessage::Stop => write!(f, "{}\n", name),
        }
    }
}

impl std::str::FromStr for MountSimulatorMessage {
    type Err = Box<dyn Error>;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        type Msg = MountSimulatorMessage;
        type Discr = MountSimulatorMessageDiscriminants;

        let parts: Vec<&str> = s.split(';').collect();

        loop { // for early exit from block
            if parts.len() < 1 { break; }

            if parts.len() == 2 && parts[0] == Msg::error() {
                return Ok(Msg::Reply(Err(parts[1].into())))
            };

            if parts.len() == 1 && parts[0] == Msg::ok() {
                return Ok(Msg::Reply(Ok(())));
            }

            if parts.len() == 1 && parts[0] == Into::<&str>::into(Discr::GetPosition) {
                return Ok(Msg::GetPosition);
            };

            if parts.len() == 2 && parts[0] == Into::<&str>::into(Discr::Info) {
                return Ok(Msg::Info(parts[1].into()));
            }

            if parts.len() >= 3 && parts[0] == Into::<&str>::into(Discr::Position) {
                if parts[1] == Msg::error() {
                    return Ok(Msg::Position(Err(parts[2].into())));
                }

                if parts.len() == 4 && parts[1] == Msg::ok() {
                    let (axis1_pos, axis2_pos) = match (parts[2].parse::<f64>(), parts[3].parse::<f64>()) {
                        (Ok(val1), Ok(val2)) => (val1, val2),
                        _ => break
                    };

                    return Ok(Msg::Position(Ok((
                        f64::Angle::new::<angle::degree>(axis1_pos),
                        f64::Angle::new::<angle::degree>(axis2_pos)
                    ))));
                }
            }

            if parts[0] == Into::<&str>::into(Discr::Reply) {
                if parts.len() == 2 && parts[1] == Msg::ok() {
                    return Ok(Msg::Reply(Ok(())));
                } else if parts.len() == 3 && parts[1] == Msg::error() {
                    return Ok(Msg::Reply(Err(parts[2].into())));
                }
            }

            if parts.len() == 3 && parts[0] == Into::<&str>::into(Discr::Slew) {
                let (axis1_spd, axis2_spd) = match (parts[2].parse::<f64>(), parts[3].parse::<f64>()) {
                    (Ok(val1), Ok(val2)) => (val1, val2),
                    _ => break
                };

                return Ok(Msg::Slew(
                    f64::AngularVelocity::new::<angular_velocity::degree_per_second>(axis1_spd),
                    f64::AngularVelocity::new::<angular_velocity::degree_per_second>(axis2_spd)
                ));
            }

            if parts.len() == 1 && parts[0] == Into::<&str>::into(Discr::Stop) {
                return Ok(Msg::Stop);
            }

            break;
        }

        Err(format!("invalid message: {}", s).into())
    }
}

pub fn to_global_unit(lat_lon: &LatLon) -> Point3<f64, Global> {
    Point3::<f64, Global>::from_xyz(
        Rad::from(lat_lon.lon).0.cos() * Rad::from(lat_lon.lat).0.cos(),
        Rad::from(lat_lon.lon).0.sin() * Rad::from(lat_lon.lat).0.cos(),
        Rad::from(lat_lon.lat).0.sin()
    )
}

pub fn to_global(position: &GeoPos) -> Point3<f64, Global> {
    let r = EARTH_RADIUS_M + position.elevation.get::<length::meter>();
    Point3::<f64, Global>::from(r * to_global_unit(&position.lat_lon).0)
}

pub fn to_local_point(observer: &Point3<f64, Global>, target: &Point3<f64, Global>) -> Point3<f64, Local> {
    let local_z_axis = observer.0.to_vec().normalize();
    let to_north_pole = cgmath::Point3::new(0.0, 0.0, EARTH_RADIUS_M) - observer.0;
    let local_y_axis = local_z_axis.cross(to_north_pole).normalize();
    let local_x_axis = local_y_axis.cross(local_z_axis);
    let to_target = target.0 - observer.0;

    let x = local_x_axis.dot(to_target);
    let y = local_y_axis.dot(to_target);
    let z = local_z_axis.dot(to_target);

    Point3::<f64, Local>::from_xyz(x, y, z)
}

pub fn to_local_vec(observer: &Point3<f64, Global>, v: &Vector3<f64, Global>) -> Vector3<f64, Local> {
    let earth_center = to_local_point(observer, &Point3::<f64, Global>::from(cgmath::Point3::origin()));
    let p = to_local_point(observer, &Point3::<f64, Global>::from(cgmath::Point3::from_vec(v.0)));

    Vector3::<f64, Local>::from(p.0 - earth_center.0)
}

/// Assumes level flight; same units as `ground_speed`.
pub fn to_global_velocity(geo_pos: &GeoPos, track: Deg<f64>, ground_speed: f64) -> Vector3<f64, Global> {
    type P3G = Point3<f64, Global>;
    type V3G = Vector3<f64, Global>;

    let pos = to_global(geo_pos);
    let north_pole = P3G::from_xyz(0.0, 0.0, EARTH_RADIUS_M);
    let to_north_pole = V3G::from(north_pole.0 - pos.0);
    let west = V3G::from(pos.0.to_vec().cross(to_north_pole.0));
    let north = V3G::from(west.0.cross(pos.0.to_vec()).normalize());
    let track_dir = V3G::from(Basis3::from_axis_angle(pos.0.to_vec().normalize(), -track).rotate_vector(north.0));
    V3G::from(track_dir.0 * ground_speed)
}
