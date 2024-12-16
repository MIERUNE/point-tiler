use std::collections::HashMap;
use std::convert::Infallible;
use std::ffi::OsStr;
use std::fs::File;
use std::io::{Read as _, Write};
use std::sync::{mpsc, Arc};
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
use gzp::MgzipSyncReader;
use gzp::{deflate::Mgzip, par::compress::{ParCompress, ParCompressBuilder}};
use itertools::Itertools as _;
use log::LevelFilter;
use pcd_parser::reader::csv::CsvPointReader;
use pcd_parser::reader::las::LasPointReader;
use pcd_parser::reader::PointReader;
use pcd_transformer::projection::transform_point;
use projection_transform::vshift::Jgd2011ToWgs84;
use rayon::iter::{IntoParallelRefIterator as _, ParallelIterator as _};
use tempfile::tempdir;

use pcd_core::pointcloud::{
    decimation::decimator::{PointCloudDecimator, VoxelDecimator},
    point::{Point, PointCloud},
};
use pcd_exporter::tiling;
use pcd_exporter::{
    cesiumtiles::make_tile_content,
    gltf::generate_quantized_glb,
    tiling::{geometric_error, TileContent, TileTree},
};
use pcd_parser::parser::{get_extension, Extension};
use projection_transform::cartesian::geodetic_to_geocentric;
use tinymvt::tileid::hilbert;

#[derive(Parser, Debug)]
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

    #[arg(short, long, required = true)]
    epsg: u16,

    #[arg(long, default_value_t = 15)]
    min: u8,

    #[arg(long, default_value_t = 18)]
    max: u8,

    #[arg(long, default_value_t = 4 * 1024)]
    max_memory_mb: usize,
}

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
    let mut writer: ParCompress<Mgzip> = ParCompressBuilder::new().from_writer(file);

    let encoded = bitcode::encode(points);
    writer.write_all(&encoded)?;

    Ok(())
}

fn read_points_from_tile(file_path: &Path) -> std::io::Result<Vec<Point>> {
    let file = File::open(file_path)?;
    let mut buf_reader = MgzipSyncReader::new(file);
    let mut buffer = Vec::new();
    buf_reader.read_to_end(&mut buffer).unwrap();
    let points = bitcode::decode(&buffer).unwrap();
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
    files
}

fn aggregate_zoom_level(base_path: &Path, z: u8) -> std::io::Result<()> {
    let child_z = z + 1;
    let child_files = get_tile_list_for_zoom(base_path, child_z);

    let mut parent_map: HashMap<(u8, u32, u32), Vec<Point>> = HashMap::new();

    for child_file in child_files {
        let (cz, cx, cy) = extract_tile_coords(&child_file);
        assert_eq!(cz, child_z);

        let points = read_points_from_tile(&child_file)?;

        for p in points {
            let parent_x = cx / 2;
            let parent_y = cy / 2;
            let parent_coords = (z, parent_x, parent_y);
            parent_map.entry(parent_coords).or_default().push(p);
        }
    }

    for (parent_tile, pts) in parent_map {
        write_points_to_tile(base_path, parent_tile, &pts)?;
    }

    Ok(())
}

fn export_tiles_to_glb(
    base_path: &Path,
    output_path: &Path,
    min_zoom: u8,
    max_zoom: u8,
) -> std::io::Result<Vec<TileContent>> {
    let mut all_tiles = Vec::new();
    for z in min_zoom..=max_zoom {
        let files = get_tile_list_for_zoom(base_path, z);
        all_tiles.extend(files);
    }

    let tile_contents: Vec<TileContent> = all_tiles
        .par_iter()
        .map(|tile_file| {
            let (tz, tx, ty) = extract_tile_coords(tile_file);
            let points = read_points_from_tile(tile_file).unwrap();
            let epsg = 4979;
            let pc = PointCloud::new(points, epsg);

            let tile_content = make_tile_content(&(tz, tx, ty), &pc);

            let ellipsoid = projection_transform::ellipsoid::wgs84();
            let transformed_points: Vec<Point> = pc
                .points
                .par_iter()
                .map(|orig| {
                    let (lng, lat, height) = (orig.x, orig.y, orig.z);
                    let (xx, yy, zz) = geodetic_to_geocentric(&ellipsoid, lng, lat, height);
                    Point {
                        x: xx,
                        y: zz,
                        z: -yy,
                        color: orig.color.clone(),
                        attributes: orig.attributes.clone(),
                    }
                })
                .collect();

            let transformed_pc = PointCloud::new(transformed_points, epsg);
            let geometric_error_value = geometric_error(tz, ty);
            let voxel_size = geometric_error_value * 0.1;
            let decimator = VoxelDecimator { voxel_size };
            let decimated_points = decimator.decimate(&transformed_pc.points);
            let decimated = PointCloud::new(decimated_points, epsg);

            let glb_path = output_path.join(&tile_content.content_path);
            fs::create_dir_all(glb_path.parent().unwrap()).unwrap();
            let glb = generate_quantized_glb(decimated).unwrap();
            let writer = File::create(glb_path).unwrap();
            glb.to_writer_with_alignment(writer, 8).unwrap();

            tile_content
        })
        .collect();

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
            TileIdMethod::Hilbert => hilbert::zxy_to_id(z, x, y),
        }
    }

    pub fn id_to_zxy(&self, tile_id: u64) -> (u8, u32, u32) {
        match self {
            TileIdMethod::Hilbert => hilbert::id_to_zxy(tile_id),
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
        let mut buf_reader = MgzipSyncReader::new(file);
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
            if let Some(ref mut iter) = self.current {
                if let Some(item) = iter.next() {
                    return Some(item);
                }
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

fn main() {
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

    log::info!("input files: {:?}", args.input);
    log::info!("output folder: {}", args.output);
    log::info!("input EPSG: {}", args.epsg);
    log::info!("min zoom: {}", args.min);
    log::info!("max zoom: {}", args.max);
    log::info!("max memory mb: {}", args.max_memory_mb);

    let start = std::time::Instant::now();

    log::info!("start processing...");
    let input_files = expand_globs(args.input);
    log::info!("Expanded input files: {:?}", input_files);

    let output_path = PathBuf::from(args.output);
    std::fs::create_dir_all(&output_path).unwrap();

    let tmp_run_file_dir_path = tempdir().unwrap();

    let min_zoom = args.min;
    let max_zoom = args.max;

    log::info!(
        "start parse and transform and tiling at max_zoom ({})...",
        max_zoom
    );
    let start_local = std::time::Instant::now();

    let jgd2wgs = Arc::new(Jgd2011ToWgs84::default());

    let max_memory_mb: usize = args.max_memory_mb;
    let max_memory_mb_bytes = max_memory_mb * 1024 * 1024;
    let point_size = 96;
    let default_chunk_points_len = 10_000_000;
    let one_chunk_mem = default_chunk_points_len * point_size;
    let mut channel_capacity = max_memory_mb_bytes / one_chunk_mem;
    if channel_capacity == 0 {
        channel_capacity = 1;
    }

    log::info!("max_memory_mb_bytes: {}", max_memory_mb_bytes);
    log::info!("one_chunk_mem: {}", one_chunk_mem);
    log::info!("channel_capacity: {}", channel_capacity);

    let (tx, rx) = mpsc::sync_channel::<Vec<Point>>(channel_capacity);
    let handle = thread::spawn(move || {
        let mut buffer = Vec::with_capacity(default_chunk_points_len);

        let extension = check_and_get_extension(&input_files).unwrap();
        let mut reader: Box<dyn PointReader> = match extension {
            Extension::Las => Box::new(LasPointReader::new(input_files).unwrap()),
            Extension::Laz => Box::new(LasPointReader::new(input_files).unwrap()),
            Extension::Csv => Box::new(CsvPointReader::new(input_files).unwrap()),
            Extension::Txt => Box::new(CsvPointReader::new(input_files).unwrap()),
        };

        while let Ok(Some(p)) = reader.next_point() {
            buffer.push(p);
            if buffer.len() >= default_chunk_points_len {
                if tx.send(buffer.clone()).is_err() {
                    break;
                }
                buffer = Vec::with_capacity(default_chunk_points_len);
            }
        }
        if !buffer.is_empty() {
            tx.send(buffer.clone()).unwrap();
            buffer.clear();
        }
    });

    for (current_run_index, chunk) in rx.into_iter().enumerate() {
        let mut keyed_points: Vec<(SortKey, Point)> = chunk
            .into_iter()
            .map(|p| {
                let transformed = transform_point(p, args.epsg, &jgd2wgs);

                let tile_coords =
                    tiling::scheme::zxy_from_lng_lat(max_zoom, transformed.x, transformed.y);
                let tile_id =
                    TileIdMethod::Hilbert.zxy_to_id(tile_coords.0, tile_coords.1, tile_coords.2);

                (SortKey { tile_id }, transformed)
            })
            .collect();

        keyed_points.sort_by_key(|(k, _)| k.tile_id);

        let run_file_path = tmp_run_file_dir_path
            .path()
            .join(format!("run_{}.bin", current_run_index));
        let file = fs::File::create(run_file_path).unwrap();
        let mut writer: ParCompress<Mgzip> = ParCompressBuilder::new().from_writer(file);

        let encoded = bitcode::encode(&keyed_points);
        writer.write_all(&encoded).unwrap();
    }

    handle.join().expect("Reading thread panicked");

    log::info!(
        "Finish transforming and tiling in {:?}",
        start_local.elapsed()
    );

    log::info!("start sorting...");
    let start_local = std::time::Instant::now();
    let pattern = tmp_run_file_dir_path.path().join("run_*.bin");
    let run_files = glob::glob(pattern.to_str().unwrap())
        .unwrap()
        .map(|r| r.unwrap())
        .collect::<Vec<_>>();

    let tile_id_iter = RunFileIterator::new(run_files);

    let config = kv_extsort::SortConfig::default().max_chunk_bytes(max_memory_mb_bytes);
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

        let (z, x, y) = tile;
        let tile_path = tmp_tiled_file_dir_path
            .path()
            .join(format!("{}/{}/{}.bin", z, x, y));

        fs::create_dir_all(tile_path.parent().unwrap()).unwrap();

        let file = fs::File::create(tile_path).unwrap();
        let mut writer: ParCompress<Mgzip> = ParCompressBuilder::new().from_writer(file);

        let encoded = bitcode::encode(&points);
        writer.write_all(&encoded).unwrap();
    }
    log::info!("Finish sorting in {:?}", start_local.elapsed());

    drop(tmp_run_file_dir_path);

    log::info!("start zoom aggregation...");
    let start_local = std::time::Instant::now();

    // The parent tile coordinates are calculated from the file with the maximum zoom level
    for z in (min_zoom..max_zoom).rev() {
        log::info!("aggregating zoom level: {}", z);
        aggregate_zoom_level(tmp_tiled_file_dir_path.path(), z).unwrap();
    }
    log::info!("Finish zoom aggregation in {:?}", start_local.elapsed());

    log::info!("start exporting tiles (GLB)...");
    let start_local = std::time::Instant::now();
    let tile_contents = export_tiles_to_glb(
        tmp_tiled_file_dir_path.path(),
        &output_path,
        min_zoom,
        max_zoom,
    )
    .unwrap();
    log::info!("Finish exporting tiles in {:?}", start_local.elapsed());

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

    log::info!("Elapsed: {:?}", start.elapsed());
    log::info!("Finish processing");
}
