use pcd_core::pointcloud::point::Point;
use projection_transform::{crs::*, jprect::JPRZone, vshift::Jgd2011ToWgs84};

pub fn transform_point(point: Point, input_epsg: EpsgCode, jgd2wgs: &Jgd2011ToWgs84) -> Point {
    match input_epsg {
        EPSG_JGD2011_JPRECT_I
        | EPSG_JGD2011_JPRECT_II
        | EPSG_JGD2011_JPRECT_III
        | EPSG_JGD2011_JPRECT_IV
        | EPSG_JGD2011_JPRECT_V
        | EPSG_JGD2011_JPRECT_VI
        | EPSG_JGD2011_JPRECT_VII
        | EPSG_JGD2011_JPRECT_VIII
        | EPSG_JGD2011_JPRECT_IX
        | EPSG_JGD2011_JPRECT_X
        | EPSG_JGD2011_JPRECT_XI
        | EPSG_JGD2011_JPRECT_XII
        | EPSG_JGD2011_JPRECT_XIII
        | EPSG_JGD2011_JPRECT_XIV
        | EPSG_JGD2011_JPRECT_XV
        | EPSG_JGD2011_JPRECT_XVI
        | EPSG_JGD2011_JPRECT_XVII
        | EPSG_JGD2011_JPRECT_XVIII
        | EPSG_JGD2011_JPRECT_XIX
        | EPSG_JGD2011_JPRECT_I_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_II_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_III_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_IV_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_V_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_VI_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_VII_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_VIII_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_IX_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_X_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_XI_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_XII_JGD2011_HEIGHT
        | EPSG_JGD2011_JPRECT_XIII_JGD2011_HEIGHT => {
            transform_from_jgd2011(point, Some(input_epsg), jgd2wgs)
        }
        _ => {
            panic!("Unsupported input CRS: {}", input_epsg);
        }
    }
}

fn rectangular_to_lnglat(x: f64, y: f64, height: f64, input_epsg: EpsgCode) -> (f64, f64, f64) {
    let zone = JPRZone::from_epsg(input_epsg).unwrap();
    let proj = zone.projection();
    let (lng, lat, height) = proj.project_inverse(x, y, height).unwrap();
    (lng, lat, height)
}

fn transform_from_jgd2011(
    point: Point,
    rectangular: Option<EpsgCode>,
    jgd2wgs: &Jgd2011ToWgs84,
) -> Point {
    let output_epsg = EPSG_WGS84_GEOGRAPHIC_3D;

    // TODO: 6697のまま（ジオイド高を足さない）の処理に対応する
    match output_epsg {
        EPSG_WGS84_GEOGRAPHIC_3D => {
            let x = point.x;
            let y = point.y;
            let z = point.z;

            let (lng, lat, height) = if let Some(input_epsg) = rectangular {
                rectangular_to_lnglat(x, y, z, input_epsg)
            } else {
                (x, y, z)
            };

            let (lng, lat, height) = jgd2wgs.convert(lng, lat, height);

            Point {
                x: lng,
                y: lat,
                z: height,
                color: point.color.clone(),
                attributes: point.attributes.clone(),
            }
        }
        _ => {
            panic!("Unsupported output CRS: {}", output_epsg);
        }
    }
}
