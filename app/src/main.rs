use std::collections::HashMap;
use std::collections::BinaryHeap;
use std::ffi::OsStr;
use std::fs::File;
use std::io::{BufReader, BufWriter, ErrorKind, Read as _, Write};
use std::sync::mpsc;
use std::thread;
use std::{
    fs,
    path::{Path, PathBuf},
};

use bitcode::{Decode, Encode};
use chrono::Local;
use clap::Parser;
use env_logger::Builder;
use glob::glob;
// use gzp::MgzipSyncReader;
// use gzp::{
//     deflate::Mgzip,
//     par::compress::{ParCompress, ParCompressBuilder},
// };
use coordinate_transformer::{EPSG_WGS84_GEOCENTRIC, EPSG_WGS84_GEOGRAPHIC_3D, PointTransformer};
use log::LevelFilter;
use pcd_exporter::gltf::GlbOptions;
use pcd_parser::reader::PointReader;
use pcd_parser::reader::csv::CsvPointReader;
use pcd_parser::reader::las::LasPointReader;
use rayon::iter::{IntoParallelIterator as _, IntoParallelRefIterator as _, ParallelIterator as _};
use tempfile::tempdir;
use tinymvt::tileid::hilbert;

use pcd_core::pointcloud::point::{Point, PointCloud};
use pcd_exporter::tiling;
use pcd_exporter::{
    cesiumtiles::make_tile_content,
    tiling::{TileContent, TileTree, geometric_error},
};
use pcd_parser::parser::{Extension, get_extension};

#[derive(Parser, Debug, Clone)]
#[command(
    name = "Point Tiler",
    about = "A tool for converting point cloud data into 3D Tiles",
    author = "MIERUNE Inc.",
    version = "0.0.1"
)]
struct Cli {
    #[arg(short, long, required = true, num_args = 1.., value_name = "FILE")]
    input: Vec<String>,

    #[arg(short, long, required = true, value_name = "DIR")]
    output: String,

    #[arg(long, required = true)]
    input_epsg: u16,

    #[arg(long, required = true)]
    output_epsg: u16,

    #[arg(long, default_value_t = 15)]
    min: u8,

    #[arg(long, default_value_t = 18)]
    max: u8,

    #[arg(long, default_value_t = 4 * 1024)]
    max_memory_mb: usize,

    #[arg(long, value_name = "N")]
    threads: Option<usize>,

    #[arg(long)]
    quantize: bool,

    #[arg(long)]
    gzip_compress: bool,

    #[arg(long)]
    meshopt: bool,

    #[arg(long)]
    disable_decimation: bool,
}

const IN_MEMORY_WORKFLOW_MULTIPLIER: u64 = 5;

#[derive(Debug, Clone, Copy, Decode, Encode)]
struct CompactPoint {
    x: f64,
    y: f64,
    z: f64,
    r: u16,
    g: u16,
    b: u16,
}

const RUN_RECORD_BYTES: usize = 8 + (8 * 3) + (2 * 3);

impl From<Point> for CompactPoint {
    fn from(point: Point) -> Self {
        Self {
            x: point.x,
            y: point.y,
            z: point.z,
            r: point.color.r,
            g: point.color.g,
            b: point.color.b,
        }
    }
}

impl From<CompactPoint> for Point {
    fn from(point: CompactPoint) -> Self {
        Self {
            x: point.x,
            y: point.y,
            z: point.z,
            color: pcd_core::pointcloud::point::Color {
                r: point.r,
                g: point.g,
                b: point.b,
            },
            attributes: pcd_core::pointcloud::point::PointAttributes {
                intensity: None,
                return_number: None,
                classification: None,
                scanner_channel: None,
                scan_angle: None,
                user_data: None,
                point_source_id: None,
                gps_time: None,
            },
        }
    }
}

fn write_run_file(path: &Path, records: &[(SortKey, CompactPoint)]) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);

    for (key, point) in records {
        writer.write_all(&key.tile_id.to_le_bytes())?;
        writer.write_all(&point.x.to_le_bytes())?;
        writer.write_all(&point.y.to_le_bytes())?;
        writer.write_all(&point.z.to_le_bytes())?;
        writer.write_all(&point.r.to_le_bytes())?;
        writer.write_all(&point.g.to_le_bytes())?;
        writer.write_all(&point.b.to_le_bytes())?;
    }

    writer.flush()?;
    Ok(())
}

struct RunFileReader {
    path: PathBuf,
    reader: BufReader<File>,
}

impl RunFileReader {
    fn open(path: PathBuf) -> std::io::Result<Self> {
        let file = File::open(&path)?;
        Ok(Self {
            path,
            reader: BufReader::new(file),
        })
    }

    fn next_record(&mut self) -> std::io::Result<Option<(SortKey, CompactPoint)>> {
        let mut record = [0u8; RUN_RECORD_BYTES];
        match self.reader.read_exact(&mut record) {
            Ok(()) => {
                let key = SortKey {
                    tile_id: u64::from_le_bytes(record[0..8].try_into().unwrap()),
                };
                let point = CompactPoint {
                    x: f64::from_le_bytes(record[8..16].try_into().unwrap()),
                    y: f64::from_le_bytes(record[16..24].try_into().unwrap()),
                    z: f64::from_le_bytes(record[24..32].try_into().unwrap()),
                    r: u16::from_le_bytes(record[32..34].try_into().unwrap()),
                    g: u16::from_le_bytes(record[34..36].try_into().unwrap()),
                    b: u16::from_le_bytes(record[36..38].try_into().unwrap()),
                };
                Ok(Some((key, point)))
            }
            Err(error) if error.kind() == ErrorKind::UnexpectedEof => Ok(None),
            Err(error) => Err(error),
        }
    }

    fn into_path(self) -> PathBuf {
        self.path
    }
}

#[derive(Debug, Clone, Copy)]
struct HeapItem {
    key: SortKey,
    point: CompactPoint,
    reader_index: usize,
}

impl Ord for HeapItem {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .key
            .cmp(&self.key)
            .then_with(|| other.reader_index.cmp(&self.reader_index))
    }
}

impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for HeapItem {
    fn eq(&self, other: &Self) -> bool {
        self.key == other.key && self.reader_index == other.reader_index
    }
}

impl Eq for HeapItem {}

fn check_and_get_extension(paths: &[PathBuf]) -> Result<Extension, String> {
    let mut extensions = vec![];
    for path in paths.iter() {
        let extension = path.extension().and_then(OsStr::to_str);
        match extension {
            Some(ext) => extensions.push(ext),
            None => return Err("File extension is not found".to_string()),
        }
    }
    extensions.sort();
    extensions.dedup();

    if extensions.len() > 1 {
        return Err("Multiple extensions are not supported".to_string());
    }

    Ok(get_extension(extensions[0]))
}

fn expand_globs(input_patterns: Vec<String>) -> Vec<PathBuf> {
    let mut paths = Vec::new();
    for pattern in input_patterns {
        if pattern.contains('*') || pattern.contains('?') || pattern.contains('[') {
            for entry in glob(&pattern).expect("Failed to read glob pattern") {
                match entry {
                    Ok(path) => paths.push(path),
                    Err(e) => eprintln!("Error: {:?}", e),
                }
            }
        } else {
            paths.push(PathBuf::from(pattern));
        }
    }
    paths
}

fn write_points_to_tile(
    dir_path: &Path,
    tile: (u8, u32, u32),
    points: &[Point],
) -> std::io::Result<()> {
    let (z, x, y) = tile;
    let tile_path = dir_path.join(format!("{}/{}/{}.bin", z, x, y));

    fs::create_dir_all(tile_path.parent().unwrap())?;

    let file = File::create(tile_path)?;
    let encoded = bitcode::encode(points);
    let mut writer = BufWriter::new(file);
    writer.write_all(&encoded)?;

    Ok(())
}

fn read_points_from_tile(file_path: &Path) -> std::io::Result<Vec<Point>> {
    let file = File::open(file_path)?;
    let mut buf_reader = BufReader::new(file);
    let mut buffer = Vec::new();
    buf_reader.read_to_end(&mut buffer)?;
    let points: Vec<Point> = bitcode::decode(&buffer).map_err(|e| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("bitcode decode failed: {e}"),
        )
    })?;
    Ok(points)
}

fn extract_tile_coords(file_path: &Path) -> (u8, u32, u32) {
    let x_dir = file_path.parent().unwrap();
    let z_dir = x_dir.parent().unwrap();

    let z: u8 = z_dir
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();
    let x: u32 = x_dir
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();
    let y: u32 = file_path
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();

    (z, x, y)
}

fn get_tile_list_for_zoom(base_path: &Path, z: u8) -> Vec<PathBuf> {
    let pattern = base_path.join(format!("{}/**/*.bin", z));
    let mut files = Vec::new();
    files.extend(
        glob::glob(pattern.to_str().unwrap())
            .unwrap()
            .filter_map(Result::ok),
    );
    files.sort();
    files
}

fn extract_shard_coords(file_path: &Path) -> (u8, u32, u32) {
    let y_dir = file_path.parent().unwrap();
    let x_dir = y_dir.parent().unwrap();
    let z_dir = x_dir.parent().unwrap();

    let z: u8 = z_dir
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();
    let x: u32 = x_dir
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();
    let y: u32 = y_dir
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();

    (z, x, y)
}

fn group_run_files_by_shard(base_path: &Path, shard_z: u8) -> Vec<((u8, u32, u32), Vec<PathBuf>)> {
    let pattern = base_path.join(format!("{}/**/*.bin", shard_z));
    let mut grouped = HashMap::<(u8, u32, u32), Vec<PathBuf>>::new();

    for file in glob::glob(pattern.to_str().unwrap())
        .unwrap()
        .filter_map(Result::ok)
    {
        let shard = extract_shard_coords(&file);
        grouped.entry(shard).or_default().push(file);
    }

    let mut entries = grouped.into_iter().collect::<Vec<_>>();
    entries.sort_by_key(|(shard, _)| *shard);
    for (_, files) in &mut entries {
        files.sort();
    }
    entries
}

fn meters_per_degree_latitude(latitude_deg: f64) -> f64 {
    let lat = latitude_deg.to_radians();
    111_132.92 - 559.82 * (2.0 * lat).cos() + 1.175 * (4.0 * lat).cos()
}

fn meters_per_degree_longitude(latitude_deg: f64) -> f64 {
    let lat = latitude_deg.to_radians();
    111_412.84 * lat.cos() - 93.5 * (3.0 * lat).cos()
}

fn maybe_decimate_points(tile: (u8, u32, u32), points: Vec<Point>, disable: bool) -> Vec<Point> {
    if disable || points.is_empty() {
        return points;
    }

    let (z, _, y) = tile;
    let voxel_size = geometric_error(z, y) * 0.1;

    let (min_lon, max_lon, min_lat, max_lat, min_height) = points.iter().fold(
        (f64::MAX, f64::MIN, f64::MAX, f64::MIN, f64::MAX),
        |(min_lon, max_lon, min_lat, max_lat, min_height), point| {
            (
                min_lon.min(point.x),
                max_lon.max(point.x),
                min_lat.min(point.y),
                max_lat.max(point.y),
                min_height.min(point.z),
            )
        },
    );

    let origin_lon = (min_lon + max_lon) * 0.5;
    let origin_lat = (min_lat + max_lat) * 0.5;
    let meters_per_lon = meters_per_degree_longitude(origin_lat).max(1.0);
    let meters_per_lat = meters_per_degree_latitude(origin_lat);

    let mut best_points: HashMap<(i64, i64, i64), (f64, usize)> = HashMap::new();

    for (index, point) in points.iter().enumerate() {
        let local_x = (point.x - origin_lon) * meters_per_lon;
        let local_y = (point.y - origin_lat) * meters_per_lat;
        let local_z = point.z - min_height;

        let voxel_index = (
            (local_x / voxel_size).floor() as i64,
            (local_y / voxel_size).floor() as i64,
            (local_z / voxel_size).floor() as i64,
        );
        let voxel_center = (
            (voxel_index.0 as f64 + 0.5) * voxel_size,
            (voxel_index.1 as f64 + 0.5) * voxel_size,
            (voxel_index.2 as f64 + 0.5) * voxel_size,
        );
        let distance = (local_x - voxel_center.0).powi(2)
            + (local_y - voxel_center.1).powi(2)
            + (local_z - voxel_center.2).powi(2);

        match best_points.get_mut(&voxel_index) {
            Some((best_distance, best_index)) => {
                if distance < *best_distance {
                    *best_distance = distance;
                    *best_index = index;
                }
            }
            None => {
                best_points.insert(voxel_index, (distance, index));
            }
        }
    }

    let mut selected = best_points.into_iter().collect::<Vec<_>>();
    selected.sort_unstable_by_key(|(voxel_index, _)| *voxel_index);

    selected
        .into_iter()
        .map(|(_, (_, point_index))| points[point_index].clone())
        .collect()
}

fn group_child_files_by_parent(
    child_files: Vec<PathBuf>,
    z: u8,
) -> HashMap<(u8, u32, u32), Vec<PathBuf>> {
    let mut parent_files = HashMap::<(u8, u32, u32), Vec<PathBuf>>::new();

    for child_file in child_files {
        let (_, cx, cy) = extract_tile_coords(&child_file);
        parent_files
            .entry((z, cx / 2, cy / 2))
            .or_default()
            .push(child_file);
    }

    for files in parent_files.values_mut() {
        files.sort();
    }

    parent_files
}

fn aggregate_zoom_level(base_path: &Path, z: u8, disable_decimation: bool) -> std::io::Result<()> {
    let child_z = z + 1;
    let child_files = get_tile_list_for_zoom(base_path, child_z);

    group_child_files_by_parent(child_files, z)
        .into_par_iter()
        .try_for_each(|(parent_tile, child_files)| -> std::io::Result<()> {
            let mut points = Vec::new();

            for child_file in child_files {
                let (cz, _, _) = extract_tile_coords(&child_file);
                debug_assert_eq!(cz, child_z);
                let mut child_points = read_points_from_tile(&child_file)?;
                points.append(&mut child_points);
            }

            let points = maybe_decimate_points(parent_tile, points, disable_decimation);
            write_points_to_tile(base_path, parent_tile, &points)?;
            Ok(())
        })?;

    Ok(())
}

fn export_tiles_to_glb(
    base_path: &Path,
    output_path: &Path,
    min_zoom: u8,
    max_zoom: u8,
    glb_options: &GlbOptions,
) -> std::io::Result<Vec<TileContent>> {
    let mut all_tiles = Vec::new();
    for z in min_zoom..=max_zoom {
        let files = get_tile_list_for_zoom(base_path, z);
        all_tiles.extend(files);
    }

    let tile_contents: Vec<TileContent> = all_tiles
        .par_iter()
        .map(|tile_file| -> std::io::Result<TileContent> {
            let (tz, tx, ty) = extract_tile_coords(tile_file);
            let mut points = read_points_from_tile(tile_file)?;
            let epsg = EPSG_WGS84_GEOGRAPHIC_3D;
            let pc = PointCloud::new(points.clone(), epsg);

            let mut tile_content = make_tile_content(&(tz, tx, ty), &pc);

            // EPSG:4979 (Geographic 3D) → EPSG:4978 (Geocentric/ECEF)
            let mut geocentric_transformer =
                PointTransformer::new(EPSG_WGS84_GEOGRAPHIC_3D, EPSG_WGS84_GEOCENTRIC, None)
                    .map_err(|e| {
                        std::io::Error::other(format!(
                            "Failed to create geocentric transformer: {e}"
                        ))
                    })?;
            geocentric_transformer
                .transform_points_in_place(&mut points)
                .map_err(|e| {
                    std::io::Error::other(format!("Failed to transform to geocentric: {e}"))
                })?;

            // Compute ECEF bbox min (before axis swap)
            let ecef_min = points.iter().fold([f64::MAX; 3], |mut acc, p| {
                acc[0] = acc[0].min(p.x);
                acc[1] = acc[1].min(p.y);
                acc[2] = acc[2].min(p.z);
                acc
            });

            // Subtract ECEF offset and swap axes for Cesium
            // Local ECEF (X, Y, Z) → Cesium (X, Z, -Y)
            for p in &mut points {
                let (x, y, z) = (p.x - ecef_min[0], p.y - ecef_min[1], p.z - ecef_min[2]);
                p.x = x;
                p.y = z;
                p.z = -y;
            }

            // Store ECEF offset in TileContent (before axis swap)
            tile_content.translation = ecef_min;

            let glb_path = output_path.join(&tile_content.content_path);
            fs::create_dir_all(glb_path.parent().unwrap())?;

            let glb_point_cloud = PointCloud::new(points, epsg);
            let glb = pcd_exporter::gltf::generate_glb_with_options(glb_point_cloud, glb_options)
                .map_err(|e| std::io::Error::other(format!("glb generation failed: {e}")))?;

            if glb_options.gzip_compress {
                let file = File::create(glb_path)?;
                let writer = flate2::write::GzEncoder::new(
                    BufWriter::new(file),
                    flate2::Compression::default(),
                );
                glb.to_writer_with_alignment(writer, 8)?;
            } else {
                let file = File::create(glb_path)?;
                let writer = BufWriter::new(file);
                glb.to_writer_with_alignment(writer, 8)?;
            }

            Ok(tile_content)
        })
        .collect::<std::io::Result<Vec<_>>>()?;

    Ok(tile_contents)
}

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Encode, Decode)]
#[repr(C)]
pub struct SortKey {
    pub tile_id: u64,
}

#[derive(Clone, Copy, Debug)]
pub enum TileIdMethod {
    Hilbert,
}

impl TileIdMethod {
    pub fn zxy_to_id(&self, z: u8, x: u32, y: u32) -> u64 {
        match self {
            TileIdMethod::Hilbert => hilbert::zxy_to_hilbert(z, x, y),
        }
    }

    pub fn id_to_zxy(&self, tile_id: u64) -> (u8, u32, u32) {
        match self {
            TileIdMethod::Hilbert => hilbert::hilbert_to_zxy(tile_id),
        }
    }
}

fn flush_tile_points(
    base_path: &Path,
    tile_id: u64,
    points: &mut Vec<Point>,
    disable_decimation: bool,
) -> std::io::Result<()> {
    if points.is_empty() {
        return Ok(());
    }

    let tile = TileIdMethod::Hilbert.id_to_zxy(tile_id);
    let tile_points = std::mem::take(points);
    let tile_points = maybe_decimate_points(tile, tile_points, disable_decimation);
    write_points_to_tile(base_path, tile, &tile_points)
}

fn merge_shard_run_files(
    run_files: Vec<PathBuf>,
    output_base_path: &Path,
    disable_decimation: bool,
) -> std::io::Result<()> {
    let mut readers = Vec::<Option<RunFileReader>>::new();
    let mut heap = BinaryHeap::<HeapItem>::new();

    for path in run_files {
        let mut reader = RunFileReader::open(path)?;
        let reader_index = readers.len();
        match reader.next_record()? {
            Some((key, point)) => {
                readers.push(Some(reader));
                heap.push(HeapItem {
                    key,
                    point,
                    reader_index,
                });
            }
            None => {
                let empty_path = reader.into_path();
                fs::remove_file(empty_path)?;
                readers.push(None);
            }
        }
    }

    let mut current_tile_id = None;
    let mut tile_points = Vec::<Point>::new();

    while let Some(item) = heap.pop() {
        if current_tile_id != Some(item.key.tile_id) {
            if let Some(tile_id) = current_tile_id {
                flush_tile_points(output_base_path, tile_id, &mut tile_points, disable_decimation)?;
            }
            current_tile_id = Some(item.key.tile_id);
        }

        tile_points.push(Point::from(item.point));

        if let Some(reader) = readers[item.reader_index].as_mut() {
            match reader.next_record()? {
                Some((key, point)) => heap.push(HeapItem {
                    key,
                    point,
                    reader_index: item.reader_index,
                }),
                None => {
                    let exhausted_reader = readers[item.reader_index].take().unwrap();
                    fs::remove_file(exhausted_reader.into_path())?;
                }
            }
        }
    }

    if let Some(tile_id) = current_tile_id {
        flush_tile_points(output_base_path, tile_id, &mut tile_points, disable_decimation)?;
    }

    Ok(())
}

fn estimate_total_size(paths: &[PathBuf]) -> u64 {
    paths
        .iter()
        .map(|p| p.metadata().map(|m| m.len()).unwrap_or(0))
        .sum()
}

fn estimate_processing_size(paths: &[PathBuf], extension: Extension) -> u64 {
    match extension {
        Extension::Las | Extension::Laz => paths
            .iter()
            .map(LasPointReader::estimate_processing_size)
            .sum(),
        Extension::Csv | Extension::Txt => estimate_total_size(paths),
    }
}

fn estimated_in_memory_requirement_bytes(processing_size: u64) -> u64 {
    processing_size.saturating_mul(IN_MEMORY_WORKFLOW_MULTIPLIER)
}

fn should_use_in_memory(processing_size: u64, max_memory_bytes: u64) -> bool {
    estimated_in_memory_requirement_bytes(processing_size) <= max_memory_bytes
}

fn collect_file_sizes(base_path: &Path, files: &mut Vec<(PathBuf, u64)>) -> std::io::Result<()> {
    for entry in fs::read_dir(base_path)? {
        let entry = entry?;
        let path = entry.path();
        let metadata = entry.metadata()?;
        if metadata.is_dir() {
            collect_file_sizes(&path, files)?;
        } else if metadata.is_file() {
            files.push((path, metadata.len()));
        }
    }
    Ok(())
}

fn format_size(bytes: u64) -> String {
    const KIB: f64 = 1024.0;
    const MIB: f64 = 1024.0 * 1024.0;
    const GIB: f64 = 1024.0 * 1024.0 * 1024.0;

    let bytes = bytes as f64;
    if bytes >= GIB {
        format!("{:.2} GiB", bytes / GIB)
    } else if bytes >= MIB {
        format!("{:.2} MiB", bytes / MIB)
    } else if bytes >= KIB {
        format!("{:.2} KiB", bytes / KIB)
    } else {
        format!("{} B", bytes as u64)
    }
}

fn summarize_directory(base_path: &Path) -> std::io::Result<(usize, u64)> {
    let mut files = Vec::new();
    collect_file_sizes(base_path, &mut files)?;
    let total_bytes = files.iter().map(|(_, size)| *size).sum();
    Ok((files.len(), total_bytes))
}

fn log_directory_summary(label: &str, base_path: &Path) {
    match summarize_directory(base_path) {
        Ok((file_count, total_bytes)) => {
            log::info!(
                "{}: {} across {} files",
                label,
                format_size(total_bytes),
                file_count
            );
        }
        Err(e) => {
            log::warn!("Failed to inspect {} at {:?}: {}", label, base_path, e);
        }
    }
}

fn in_memory_workflow(
    input_files: Vec<PathBuf>,
    args: &Cli,
    output_path: &Path,
) -> std::io::Result<()> {
    let extension = check_and_get_extension(&input_files).unwrap();

    log::info!("start parse and transform and tiling...");
    let start_local = std::time::Instant::now();

    let epsg_in = args.input_epsg;
    let epsg_out = args.output_epsg;

    // Read multiple files in parallel
    let mut all_points: Vec<Point> = input_files
        .par_iter()
        .flat_map(|file| {
            let mut reader: Box<dyn PointReader> = match extension {
                Extension::Las | Extension::Laz => {
                    Box::new(LasPointReader::new(vec![file.clone()]).unwrap())
                }
                Extension::Csv | Extension::Txt => {
                    Box::new(CsvPointReader::new(vec![file.clone()]).unwrap())
                }
            };

            let mut points = Vec::new();
            while let Ok(Some(p)) = reader.next_point() {
                points.push(p);
            }
            points
        })
        .collect();

    // Coordinate transformation
    let mut transformer = PointTransformer::new(epsg_in, epsg_out, None)
        .map_err(|e| std::io::Error::other(format!("Failed to create transformer: {e}")))?;
    transformer
        .transform_points_in_place(&mut all_points)
        .map_err(|e| std::io::Error::other(format!("Failed to transform points: {e}")))?;

    log::info!(
        "Finish transforming and tiling in {:?}",
        start_local.elapsed()
    );

    log::info!("start grouping...");
    let start_local = std::time::Instant::now();
    let max_zoom = args.max;

    let map_init = || HashMap::<u64, Vec<Point>>::new();
    let map_fold = |mut map: HashMap<u64, Vec<Point>>, p: Point| {
        let (z, x, y) = tiling::scheme::zxy_from_lng_lat(max_zoom, p.x, p.y);
        let tile_id = TileIdMethod::Hilbert.zxy_to_id(z, x, y);
        map.entry(tile_id).or_default().push(p);
        map
    };
    let map_reduce = |mut a: HashMap<u64, Vec<Point>>, b: HashMap<u64, Vec<Point>>| {
        for (k, mut v) in b {
            a.entry(k).or_default().append(&mut v);
        }
        a
    };

    let tile_map = all_points
        .into_par_iter()
        .fold(map_init, map_fold)
        .reduce(map_init, map_reduce);

    log::info!(
        "Transformed & grouped into {} tiles in {:?}",
        tile_map.len(),
        start_local.elapsed()
    );

    let tmp_tiled_file_dir_path = tempdir().unwrap();

    log::info!("start writing tile files...");
    let start_local = std::time::Instant::now();
    tile_map
        .into_par_iter()
        .try_for_each(|(tile_id, points)| -> std::io::Result<()> {
            let (z, x, y) = TileIdMethod::Hilbert.id_to_zxy(tile_id);
            let tile = (z, x, y);
            let points = maybe_decimate_points(tile, points, args.disable_decimation);

            let tile_path = tmp_tiled_file_dir_path
                .path()
                .join(format!("{}/{}/{}.bin", z, x, y));
            fs::create_dir_all(tile_path.parent().unwrap())?;
            let file = File::create(tile_path)?;
            let encoded = bitcode::encode(&points);
            let mut writer = BufWriter::new(file);
            writer.write_all(&encoded)?;
            Ok(())
        })?;

    log::info!("Wrote tile files in {:?}", start_local.elapsed());
    log_directory_summary(
        "tile files after leaf write",
        tmp_tiled_file_dir_path.path(),
    );

    log::info!("start zoom aggregation...");
    let start_local = std::time::Instant::now();
    for z in (args.min..max_zoom).rev() {
        log::info!("aggregating zoom level: {}", z);
        aggregate_zoom_level(tmp_tiled_file_dir_path.path(), z, args.disable_decimation)?;
        log_directory_summary(
            &format!("tile files after aggregating z={}", z),
            tmp_tiled_file_dir_path.path(),
        );
    }
    log::info!("Finish zoom aggregation in {:?}", start_local.elapsed());

    log::info!("start exporting tiles (GLB)...");
    let start_local = std::time::Instant::now();

    let glb_options = GlbOptions {
        quantize: args.quantize,
        meshopt: args.meshopt,
        gzip_compress: args.gzip_compress,
    };

    let tile_contents = export_tiles_to_glb(
        tmp_tiled_file_dir_path.path(),
        output_path,
        args.min,
        max_zoom,
        &glb_options,
    )?;

    log::info!("Finish exporting tiles in {:?}", start_local.elapsed());
    log_directory_summary("tile files before cleanup", tmp_tiled_file_dir_path.path());
    log_directory_summary("glb output", output_path);

    drop(tmp_tiled_file_dir_path);

    let mut tree = TileTree::default();
    for content in tile_contents {
        tree.add_content(content);
    }
    let tileset = cesiumtiles::tileset::Tileset {
        asset: cesiumtiles::tileset::Asset {
            version: "1.1".to_string(),
            ..Default::default()
        },
        root: tree.into_tileset_root(),
        geometric_error: 1e+100,
        ..Default::default()
    };
    let root_tileset_path = output_path.join("tileset.json");
    fs::create_dir_all(root_tileset_path.parent().unwrap())?;
    fs::write(
        root_tileset_path,
        serde_json::to_string_pretty(&tileset).unwrap(),
    )?;

    Ok(())
}

fn external_sort_workflow(
    input_files: Vec<PathBuf>,
    args: &Cli,
    output_path: &Path,
) -> std::io::Result<()> {
    log::info!("start parse and transform and tiling...");

    let start_local = std::time::Instant::now();

    let tmp_run_file_dir_path = tempdir().unwrap();
    let mut tile_contents_all = Vec::new();

    {
        let max_memory_mb: usize = args.max_memory_mb;
        let max_memory_mb_bytes = max_memory_mb * 1024 * 1024;
        let point_size = 96;
        let default_chunk_points_len = 10_000_000;
        let one_chunk_mem = default_chunk_points_len * point_size;
        let mut channel_capacity = max_memory_mb_bytes / one_chunk_mem;
        if channel_capacity == 0 {
            channel_capacity = 1;
        }

        let num_cores = args.threads.filter(|&n| n > 0).unwrap_or(num_cpus::get());

        let extension = check_and_get_extension(&input_files).unwrap();
        let epsg_in = args.input_epsg;
        let epsg_out = args.output_epsg;

        log::info!("memory budget: {}", format_size(max_memory_mb_bytes as u64));
        log::info!("reader chunk target: {}", format_size(one_chunk_mem as u64));
        log::info!("channel_capacity: {}", channel_capacity);
        log::info!("num_cores: {}", num_cores);

        let (tx, rx) = mpsc::sync_channel::<Vec<Point>>(channel_capacity);

        // Spawn multiple reader threads
        let chunk_size = input_files.len().div_ceil(num_cores);
        let mut handles = vec![];

        for chunk in input_files.chunks(chunk_size) {
            let chunk = chunk.to_vec();
            let tx = tx.clone();
            let extension_copy = extension;

            let handle = thread::spawn(move || {
                // Create a transformer per thread
                let mut transformer = PointTransformer::new(epsg_in, epsg_out, None)
                    .expect("Failed to create transformer");

                let mut buffer = Vec::with_capacity(default_chunk_points_len);
                let mut reader: Box<dyn PointReader> = match extension_copy {
                    Extension::Las | Extension::Laz => {
                        Box::new(LasPointReader::new(chunk).unwrap())
                    }
                    Extension::Csv | Extension::Txt => {
                        Box::new(CsvPointReader::new(chunk).unwrap())
                    }
                };

                while let Ok(Some(p)) = reader.next_point() {
                    buffer.push(p);
                    if buffer.len() >= default_chunk_points_len {
                        // Transform coordinates in batch
                        transformer
                            .transform_points_in_place(&mut buffer)
                            .expect("Failed to transform points");
                        let to_send = std::mem::replace(
                            &mut buffer,
                            Vec::with_capacity(default_chunk_points_len),
                        );
                        if tx.send(to_send).is_err() {
                            break;
                        }
                    }
                }
                if !buffer.is_empty() {
                    // Transform remaining points
                    transformer
                        .transform_points_in_place(&mut buffer)
                        .expect("Failed to transform points");
                    let _ = tx.send(buffer);
                }
            });
            handles.push(handle);
        }

        // Drop the sender to close the channel
        drop(tx);

        for (current_run_index, chunk) in rx.into_iter().enumerate() {
            let mut shard_points = HashMap::<(u8, u32, u32), Vec<(SortKey, CompactPoint)>>::new();

            for p in chunk {
                let shard = tiling::scheme::zxy_from_lng_lat(args.min, p.x, p.y);
                let tile_coords = tiling::scheme::zxy_from_lng_lat(args.max, p.x, p.y);
                let tile_id =
                    TileIdMethod::Hilbert.zxy_to_id(tile_coords.0, tile_coords.1, tile_coords.2);

                shard_points
                    .entry(shard)
                    .or_default()
                    .push((SortKey { tile_id }, CompactPoint::from(p)));
            }

            for ((shard_z, shard_x, shard_y), mut keyed_points) in shard_points {
                keyed_points.sort_by_key(|(k, _)| k.tile_id);

                let run_file_path = tmp_run_file_dir_path.path().join(format!(
                    "{}/{}/{}/run_{}.bin",
                    shard_z, shard_x, shard_y, current_run_index
                ));
                fs::create_dir_all(run_file_path.parent().unwrap()).unwrap();
                write_run_file(&run_file_path, &keyed_points).unwrap();
            }
        }

        for handle in handles {
            handle.join().expect("Reading thread panicked");
        }

        log::info!(
            "Finish transforming and tiling in {:?}",
            start_local.elapsed()
        );
        log_directory_summary("run files after sharding", tmp_run_file_dir_path.path());
    }

    {
        log::info!("start shard processing...");
        let start_local = std::time::Instant::now();

        let shard_runs = group_run_files_by_shard(tmp_run_file_dir_path.path(), args.min);
        let total_shards = shard_runs.len();
        let mut peak_shard_run_bytes = 0u64;
        let mut peak_shard_tile_bytes = 0u64;
        let mut peak_live_intermediate_bytes = 0u64;
        let mut remaining_run_bytes = shard_runs
            .iter()
            .flat_map(|(_, files)| files.iter())
            .map(|path| path.metadata().map(|m| m.len()).unwrap_or(0))
            .sum::<u64>();
        let glb_options = GlbOptions {
            quantize: args.quantize,
            meshopt: args.meshopt,
            gzip_compress: args.gzip_compress,
        };

        for (index, (shard, run_files)) in shard_runs.into_iter().enumerate() {
            let shard_run_bytes = run_files
                .iter()
                .map(|path| path.metadata().map(|m| m.len()).unwrap_or(0))
                .sum::<u64>();
            let shard_run_dir = run_files
                .first()
                .and_then(|path| path.parent())
                .map(Path::to_path_buf);
            peak_shard_run_bytes = peak_shard_run_bytes.max(shard_run_bytes);
            log::info!(
                "shard {}/{}: z{}/{}/{} with {} run files ({})",
                index + 1,
                total_shards,
                shard.0,
                shard.1,
                shard.2,
                run_files.len(),
                format_size(shard_run_bytes)
            );

            let tmp_tiled_file_dir_path = tempdir().unwrap();
            merge_shard_run_files(run_files, tmp_tiled_file_dir_path.path(), args.disable_decimation)?;
            log_directory_summary(
                "tile files after shard sort",
                tmp_tiled_file_dir_path.path(),
            );

            for z in (args.min..args.max).rev() {
                aggregate_zoom_level(tmp_tiled_file_dir_path.path(), z, args.disable_decimation)
                    .unwrap();
            }

            log_directory_summary(
                "tile files before shard cleanup",
                tmp_tiled_file_dir_path.path(),
            );
            let mut shard_tile_files = Vec::new();
            collect_file_sizes(tmp_tiled_file_dir_path.path(), &mut shard_tile_files)?;
            let shard_tile_bytes = shard_tile_files.iter().map(|(_, size)| *size).sum::<u64>();
            peak_shard_tile_bytes = peak_shard_tile_bytes.max(shard_tile_bytes);
            peak_live_intermediate_bytes =
                peak_live_intermediate_bytes.max(remaining_run_bytes + shard_tile_bytes);

            let tile_contents = export_tiles_to_glb(
                tmp_tiled_file_dir_path.path(),
                output_path,
                args.min,
                args.max,
                &glb_options,
            )
            .unwrap();
            tile_contents_all.extend(tile_contents);

            remaining_run_bytes = remaining_run_bytes.saturating_sub(shard_run_bytes);
            if let Some(shard_run_dir) = shard_run_dir
                && shard_run_dir.exists()
            {
                fs::remove_dir_all(&shard_run_dir)?;
            }
            log::info!(
                "shard {}/{} done: tile bins {}, live intermediates {}",
                index + 1,
                total_shards,
                format_size(shard_tile_bytes),
                format_size(remaining_run_bytes + shard_tile_bytes)
            );
        }

        log::info!("Finish shard processing in {:?}", start_local.elapsed());
        log::info!(
            "peak shard run files: {}",
            format_size(peak_shard_run_bytes)
        );
        log::info!(
            "peak shard tile files: {}",
            format_size(peak_shard_tile_bytes)
        );
        log::info!(
            "peak live intermediate files: {}",
            format_size(peak_live_intermediate_bytes)
        );
        log_directory_summary("run files before cleanup", tmp_run_file_dir_path.path());

        drop(tmp_run_file_dir_path);

        log_directory_summary("glb output", output_path);

        let mut tree = TileTree::default();
        for content in tile_contents_all {
            tree.add_content(content);
        }

        let tileset = cesiumtiles::tileset::Tileset {
            asset: cesiumtiles::tileset::Asset {
                version: "1.1".to_string(),
                ..Default::default()
            },
            root: tree.into_tileset_root(),
            geometric_error: 1e+100,
            ..Default::default()
        };

        let root_tileset_path = output_path.join("tileset.json");
        log::info!("write tileset.json: {:?}", root_tileset_path);
        fs::create_dir_all(root_tileset_path.parent().unwrap()).unwrap();
        fs::write(
            root_tileset_path,
            serde_json::to_string_pretty(&tileset).unwrap(),
        )
        .unwrap();
    }
    Ok(())
}

fn main() -> std::io::Result<()> {
    Builder::new()
        .format(|buf, record| {
            writeln!(
                buf,
                "{} [{}] - {}",
                Local::now().format("%Y-%m-%d %H:%M:%S"),
                record.level(),
                record.args()
            )
        })
        .filter(None, LevelFilter::Info)
        .init();

    let args = Cli::parse();

    let thread_count = args
        .threads
        .filter(|&n| n > 0)
        .unwrap_or(num_cpus::get() * 2);
    rayon::ThreadPoolBuilder::new()
        .num_threads(thread_count)
        .build_global()
        .unwrap();

    log::info!("rayon threads: {}", thread_count);
    log::info!("input files: {:?}", args.input);
    log::info!("output folder: {}", args.output);
    log::info!("input EPSG: {}", args.input_epsg);
    log::info!("output EPSG: {}", args.output_epsg);
    log::info!("min zoom: {}", args.min);
    log::info!("max zoom: {}", args.max);
    log::info!("max memory mb: {}", args.max_memory_mb);
    log::info!("threads: {:?}", args.threads);
    log::info!("quantize: {}", args.quantize);
    log::info!("gzip compress: {}", args.gzip_compress);
    log::info!("meshopt: {}", args.meshopt);
    log::info!("disable decimation: {}", args.disable_decimation);

    let start = std::time::Instant::now();

    log::info!("start processing...");
    let input_files = expand_globs(args.input.clone());
    log::info!("Expanded input files: {:?}", input_files);

    let output_path = PathBuf::from(args.output.clone());
    std::fs::create_dir_all(&output_path).unwrap();

    let extension = check_and_get_extension(&input_files).unwrap();
    let total_size = estimate_total_size(&input_files);
    let processing_size = estimate_processing_size(&input_files, extension);
    let max_memory_bytes = args.max_memory_mb as u64 * 1024 * 1024;
    let estimated_in_memory_requirement = estimated_in_memory_requirement_bytes(processing_size);
    log::info!(
        "input size: {}, estimated processing size: {}, estimated in-memory requirement (x{}): {}, threshold: {}",
        format_size(total_size),
        format_size(processing_size),
        IN_MEMORY_WORKFLOW_MULTIPLIER,
        format_size(estimated_in_memory_requirement),
        format_size(max_memory_bytes)
    );

    if should_use_in_memory(processing_size, max_memory_bytes) {
        log::info!("Using in-memory workflow");
        in_memory_workflow(input_files, &args, &output_path)?;
    } else {
        log::info!("Using external sort workflow");
        external_sort_workflow(input_files, &args, &output_path)?;
    }

    log::info!("Elapsed: {:?}", start.elapsed());
    log::info!("Finish processing");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use pcd_core::pointcloud::point::{Color, PointAttributes};
    use tempfile::tempdir;

    fn point(x: f64, y: f64, z: f64) -> Point {
        Point {
            x,
            y,
            z,
            color: Color::default(),
            attributes: PointAttributes {
                intensity: None,
                return_number: None,
                classification: None,
                scanner_channel: None,
                scan_angle: None,
                user_data: None,
                point_source_id: None,
                gps_time: None,
            },
        }
    }

    #[test]
    fn maybe_decimate_points_can_be_disabled() {
        let tile = (18, 0, 0);
        let points = vec![point(1.0, 2.0, 3.0), point(4.0, 5.0, 6.0)];
        let decimated = maybe_decimate_points(tile, points.clone(), true);
        assert_eq!(decimated.len(), points.len());
        assert_eq!(decimated[0].x, points[0].x);
        assert_eq!(decimated[1].x, points[1].x);
    }

    #[test]
    fn group_child_files_by_parent_groups_four_children() {
        let child_files = vec![
            PathBuf::from("/tmp/18/10/20.bin"),
            PathBuf::from("/tmp/18/10/21.bin"),
            PathBuf::from("/tmp/18/11/20.bin"),
            PathBuf::from("/tmp/18/11/21.bin"),
        ];

        let grouped = group_child_files_by_parent(child_files, 17);
        assert_eq!(grouped.len(), 1);
        let files = grouped.get(&(17, 5, 10)).unwrap();
        assert_eq!(files.len(), 4);
    }

    #[test]
    fn aggregate_zoom_level_without_decimation_preserves_all_points() {
        let dir = tempdir().unwrap();
        write_points_to_tile(dir.path(), (18, 10, 20), &[point(1.0, 1.0, 1.0)]).unwrap();
        write_points_to_tile(dir.path(), (18, 10, 21), &[point(2.0, 2.0, 2.0)]).unwrap();
        write_points_to_tile(dir.path(), (18, 11, 20), &[point(3.0, 3.0, 3.0)]).unwrap();
        write_points_to_tile(dir.path(), (18, 11, 21), &[point(4.0, 4.0, 4.0)]).unwrap();

        aggregate_zoom_level(dir.path(), 17, true).unwrap();

        let parent_points = read_points_from_tile(&dir.path().join("17/5/10.bin")).unwrap();
        assert_eq!(parent_points.len(), 4);
    }

    #[test]
    fn should_use_in_memory_requires_five_times_processing_size() {
        let processing_size = 100;

        assert!(!should_use_in_memory(processing_size, 499));
        assert!(should_use_in_memory(processing_size, 500));
    }

    #[test]
    fn merge_shard_run_files_creates_tiles_and_deletes_runs() {
        let run_dir = tempdir().unwrap();
        let tile_dir = tempdir().unwrap();

        let tile_a = (18, 10, 20);
        let tile_b = (18, 10, 21);
        let tile_a_id = TileIdMethod::Hilbert.zxy_to_id(tile_a.0, tile_a.1, tile_a.2);
        let tile_b_id = TileIdMethod::Hilbert.zxy_to_id(tile_b.0, tile_b.1, tile_b.2);

        let run_a = run_dir.path().join("run_a.bin");
        let run_b = run_dir.path().join("run_b.bin");

        write_run_file(
            &run_a,
            &[
                (
                    SortKey { tile_id: tile_a_id },
                    CompactPoint::from(point(1.0, 1.0, 1.0)),
                ),
                (
                    SortKey { tile_id: tile_b_id },
                    CompactPoint::from(point(4.0, 4.0, 4.0)),
                ),
            ],
        )
        .unwrap();
        write_run_file(
            &run_b,
            &[
                (
                    SortKey { tile_id: tile_a_id },
                    CompactPoint::from(point(2.0, 2.0, 2.0)),
                ),
                (
                    SortKey { tile_id: tile_a_id },
                    CompactPoint::from(point(3.0, 3.0, 3.0)),
                ),
            ],
        )
        .unwrap();

        merge_shard_run_files(vec![run_a.clone(), run_b.clone()], tile_dir.path(), true).unwrap();

        let tile_a_points = read_points_from_tile(
            &tile_dir
                .path()
                .join(format!("{}/{}/{}.bin", tile_a.0, tile_a.1, tile_a.2)),
        )
        .unwrap();
        let tile_b_points = read_points_from_tile(
            &tile_dir
                .path()
                .join(format!("{}/{}/{}.bin", tile_b.0, tile_b.1, tile_b.2)),
        )
        .unwrap();

        assert_eq!(tile_a_points.len(), 3);
        assert_eq!(tile_b_points.len(), 1);
        assert!(!run_a.exists());
        assert!(!run_b.exists());
    }
}
