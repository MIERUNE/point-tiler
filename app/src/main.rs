use std::collections::HashMap;
use std::convert::Infallible;
use std::ffi::OsStr;
use std::fs::File;
use std::io::{BufReader, BufWriter, Read as _, Write};
use std::sync::mpsc;
use std::thread;
use std::{
    fs,
    path::{Path, PathBuf},
};

use bitcode::{Decode, Encode};
use bytemuck::{Pod, Zeroable};
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
use itertools::Itertools as _;
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
    // let mut writer: ParCompress<Mgzip> = ParCompressBuilder::new().from_writer(file);
    let mut writer = BufWriter::new(file);

    let encoded = bitcode::encode(points);
    writer.write_all(&encoded)?;

    Ok(())
}

fn read_points_from_tile(file_path: &Path) -> std::io::Result<Vec<Point>> {
    let file = File::open(file_path)?;
    // let mut buf_reader = MgzipSyncReader::new(file);
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

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Pod, Zeroable, Encode, Decode)]
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

struct RunFileIterator {
    files: std::vec::IntoIter<PathBuf>,
    current: Option<std::vec::IntoIter<(SortKey, Point)>>,
}

impl RunFileIterator {
    fn new(files: Vec<PathBuf>) -> Self {
        RunFileIterator {
            files: files.into_iter(),
            current: None,
        }
    }

    fn read_run_file(path: PathBuf) -> Result<Vec<(SortKey, Point)>, Infallible> {
        let file = File::open(path).unwrap();
        // let mut buf_reader = MgzipSyncReader::new(file);
        let mut buf_reader = file;
        let mut buffer = Vec::new();
        buf_reader.read_to_end(&mut buffer).unwrap();
        let data: Vec<(SortKey, Point)> = bitcode::decode(&buffer[..]).unwrap();
        Ok(data)
    }
}

impl Iterator for RunFileIterator {
    type Item = (SortKey, Point);

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if let Some(ref mut iter) = self.current
                && let Some(item) = iter.next()
            {
                return Some(item);
            }

            match self.files.next() {
                Some(file) => {
                    let data = RunFileIterator::read_run_file(file).unwrap();
                    self.current = Some(data.into_iter());
                }
                None => {
                    return None;
                }
            }
        }
    }
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

fn log_directory_sizes(label: &str, base_path: &Path, max_entries: usize) {
    let mut files = Vec::new();
    match collect_file_sizes(base_path, &mut files) {
        Ok(()) => {
            files.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
            let total_bytes: u64 = files.iter().map(|(_, size)| *size).sum();
            log::info!(
                "{} total size: {} bytes across {} files at {:?}",
                label,
                total_bytes,
                files.len(),
                base_path
            );
            for (path, size) in files.iter().take(max_entries) {
                let relative = path.strip_prefix(base_path).unwrap_or(path);
                log::info!("{} file: {:?} ({} bytes)", label, relative, size);
            }
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
            let mut writer = BufWriter::new(file);
            let encoded = bitcode::encode(&points);
            writer.write_all(&encoded)?;
            Ok(())
        })?;

    log::info!("Wrote tile files in {:?}", start_local.elapsed());
    log_directory_sizes(
        "tile files after leaf write",
        tmp_tiled_file_dir_path.path(),
        32,
    );

    log::info!("start zoom aggregation...");
    let start_local = std::time::Instant::now();
    for z in (args.min..max_zoom).rev() {
        log::info!("aggregating zoom level: {}", z);
        aggregate_zoom_level(tmp_tiled_file_dir_path.path(), z, args.disable_decimation)?;
        log_directory_sizes(
            &format!("tile files after aggregating z={}", z),
            tmp_tiled_file_dir_path.path(),
            32,
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
    log_directory_sizes(
        "tile files before cleanup",
        tmp_tiled_file_dir_path.path(),
        32,
    );
    log_directory_sizes("glb output", output_path, 32);

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

        log::info!("max_memory_mb_bytes: {}", max_memory_mb_bytes);
        log::info!("one_chunk_mem: {}", one_chunk_mem);
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
            let mut keyed_points: Vec<(SortKey, Point)> = chunk
                .into_iter()
                .map(|p| {
                    // let transformed = transform_point(p, args.input_epsg, args.output_epsg, &jgd2wgs);

                    let tile_coords = tiling::scheme::zxy_from_lng_lat(args.max, p.x, p.y);
                    let tile_id = TileIdMethod::Hilbert.zxy_to_id(
                        tile_coords.0,
                        tile_coords.1,
                        tile_coords.2,
                    );

                    (SortKey { tile_id }, p)
                })
                .collect();

            keyed_points.sort_by_key(|(k, _)| k.tile_id);

            let run_file_path = tmp_run_file_dir_path
                .path()
                .join(format!("run_{}.bin", current_run_index));
            let file = fs::File::create(run_file_path).unwrap();
            // let mut writer: ParCompress<Mgzip> = ParCompressBuilder::new().from_writer(file);
            let mut writer = BufWriter::new(file);

            let encoded = bitcode::encode(&keyed_points);
            writer.write_all(&encoded).unwrap();
        }

        for handle in handles {
            handle.join().expect("Reading thread panicked");
        }

        log::info!(
            "Finish transforming and tiling in {:?}",
            start_local.elapsed()
        );
        log_directory_sizes("run files", tmp_run_file_dir_path.path(), 32);
    }

    {
        log::info!("start sorting...");
        let start_local = std::time::Instant::now();

        let pattern = tmp_run_file_dir_path.path().join("run_*.bin");
        let run_files = glob::glob(pattern.to_str().unwrap())
            .unwrap()
            .map(|r| r.unwrap())
            .collect::<Vec<_>>();

        let tile_id_iter = RunFileIterator::new(run_files);

        let config =
            kv_extsort::SortConfig::default().max_chunk_bytes(args.max_memory_mb * 1024 * 1024);
        let sorted_iter = kv_extsort::sort(
            tile_id_iter.map(|(key, point)| {
                let encoded_point = bitcode::encode(&point);
                std::result::Result::<_, Infallible>::Ok((key, encoded_point))
            }),
            config,
        );

        let grouped_iter = sorted_iter.chunk_by(|res| match res {
            Ok((key, _)) => (false, *key),
            Err(_) => (true, SortKey { tile_id: 0 }),
        });

        let tmp_tiled_file_dir_path = tempdir().unwrap();

        for ((_, key), group) in &grouped_iter {
            let points = group
                .into_iter()
                .map(|r| r.map(|(_, p)| bitcode::decode::<Point>(&p).unwrap()))
                .collect::<Result<Vec<_>, _>>()
                .unwrap();

            let tile_id = key.tile_id;
            let tile = TileIdMethod::Hilbert.id_to_zxy(tile_id);
            let points = maybe_decimate_points(tile, points, args.disable_decimation);

            let (z, x, y) = tile;
            let tile_path = tmp_tiled_file_dir_path
                .path()
                .join(format!("{}/{}/{}.bin", z, x, y));

            fs::create_dir_all(tile_path.parent().unwrap()).unwrap();

            let file = fs::File::create(tile_path).unwrap();
            // let mut writer: ParCompress<Mgzip> = ParCompressBuilder::new().from_writer(file);
            let mut writer = BufWriter::new(file);

            let encoded = bitcode::encode(&points);
            writer.write_all(&encoded).unwrap();
        }
        log::info!("Finish sorting in {:?}", start_local.elapsed());
        log_directory_sizes("run files before cleanup", tmp_run_file_dir_path.path(), 32);
        log_directory_sizes("tile files after sort", tmp_tiled_file_dir_path.path(), 32);

        drop(tmp_run_file_dir_path);

        log::info!("start zoom aggregation...");
        let start_local = std::time::Instant::now();

        // The parent tile coordinates are calculated from the file with the maximum zoom level
        for z in (args.min..args.max).rev() {
            log::info!("aggregating zoom level: {}", z);
            aggregate_zoom_level(tmp_tiled_file_dir_path.path(), z, args.disable_decimation)
                .unwrap();
            log_directory_sizes(
                &format!("tile files after aggregating z={}", z),
                tmp_tiled_file_dir_path.path(),
                32,
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
            args.max,
            &glb_options,
        )
        .unwrap();
        log::info!("Finish exporting tiles in {:?}", start_local.elapsed());
        log_directory_sizes(
            "tile files before cleanup",
            tmp_tiled_file_dir_path.path(),
            32,
        );
        log_directory_sizes("glb output", output_path, 32);

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
        "Total input size: {}, estimated processing size: {}, estimated in-memory requirement (x{}): {}, threshold: {}",
        total_size,
        processing_size,
        IN_MEMORY_WORKFLOW_MULTIPLIER,
        estimated_in_memory_requirement,
        max_memory_bytes
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
}
