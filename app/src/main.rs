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
use itertools::Itertools as _;
use log::LevelFilter;
use pcd_exporter::gltf::generate_glb;
use pcd_parser::reader::csv::CsvPointReader;
use pcd_parser::reader::las::LasPointReader;
use pcd_parser::reader::PointReader;
use coordinate_transformer::{PointTransformer, EPSG_WGS84_GEOCENTRIC, EPSG_WGS84_GEOGRAPHIC_3D};
use rayon::iter::{IntoParallelIterator as _, IntoParallelRefIterator as _, ParallelIterator as _};
use tempfile::tempdir;
use tinymvt::tileid::hilbert;

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
    files
}

fn aggregate_zoom_level(base_path: &Path, z: u8) -> std::io::Result<()> {
    let child_z = z + 1;
    let child_files = get_tile_list_for_zoom(base_path, child_z);

    let parent_map: HashMap<(u8, u32, u32), Vec<Point>> = child_files
        .par_iter()
        .try_fold(
            HashMap::<(u8, u32, u32), Vec<Point>>::new,
            |mut map, child_file| -> std::io::Result<_> {
                let (cz, cx, cy) = extract_tile_coords(child_file);
                debug_assert_eq!(cz, child_z);

                let parent_coords = (z, cx / 2, cy / 2);
                let mut points = read_points_from_tile(child_file)?;
                map.entry(parent_coords).or_default().append(&mut points);

                Ok(map)
            },
        )
        .try_reduce(
            HashMap::<(u8, u32, u32), Vec<Point>>::new,
            |mut a, b| -> std::io::Result<_> {
                for (k, mut v) in b {
                    a.entry(k).or_default().append(&mut v);
                }
                Ok(a)
            },
        )?;

    parent_map
        .into_par_iter()
        .try_for_each(|(parent_tile, pts)| -> std::io::Result<()> {
            write_points_to_tile(base_path, parent_tile, &pts)?;
            Ok(())
        })?;

    Ok(())
}

fn export_tiles_to_glb(
    base_path: &Path,
    output_path: &Path,
    min_zoom: u8,
    max_zoom: u8,
    quantize: bool,
    gzip_compress: bool,
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

            let tile_content = make_tile_content(&(tz, tx, ty), &pc);

            // EPSG:4979 (Geographic 3D) → EPSG:4978 (Geocentric/ECEF) 変換
            let mut geocentric_transformer =
                PointTransformer::new(EPSG_WGS84_GEOGRAPHIC_3D, EPSG_WGS84_GEOCENTRIC, None)
                    .map_err(|e| {
                        std::io::Error::other(format!("Failed to create geocentric transformer: {e}"))
                    })?;
            geocentric_transformer
                .transform_points_in_place(&mut points)
                .map_err(|e| {
                    std::io::Error::other(format!("Failed to transform to geocentric: {e}"))
                })?;

            // Cesium 用の座標軸入れ替え (PROJ変換後に実施)
            // ECEF (X, Y, Z) → Cesium (X, Z, -Y)
            for p in &mut points {
                let (x, y, z) = (p.x, p.y, p.z);
                p.x = x;
                p.y = z;
                p.z = -y;
            }

            let geometric_error_value = geometric_error(tz, ty);
            let voxel_size = geometric_error_value * 0.1;
            let decimator = VoxelDecimator { voxel_size };
            let decimated_points = decimator.decimate(&points);
            let decimated = PointCloud::new(decimated_points, epsg);

            let glb_path = output_path.join(&tile_content.content_path);
            fs::create_dir_all(glb_path.parent().unwrap())?;

            let glb = if quantize {
                generate_quantized_glb(decimated)
                    .map_err(|e| std::io::Error::other(format!("glb generation failed: {e}")))?
            } else {
                generate_glb(decimated)
                    .map_err(|e| std::io::Error::other(format!("glb generation failed: {e}")))?
            };

            if gzip_compress {
                let file = File::create(glb_path)?;
                // let writer: ParCompress<Mgzip> = ParCompressBuilder::new().from_writer(file);
                let writer = BufWriter::new(file);
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

fn estimate_total_size(paths: &[PathBuf]) -> u64 {
    paths
        .iter()
        .map(|p| p.metadata().map(|m| m.len()).unwrap_or(0))
        .sum()
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

    // 複数ファイルを並列に読み込む
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

    // 座標変換
    let mut transformer = PointTransformer::new(epsg_in, epsg_out, None).map_err(|e| {
        std::io::Error::other(format!("Failed to create transformer: {e}"))
    })?;
    transformer.transform_points_in_place(&mut all_points).map_err(|e| {
        std::io::Error::other(format!("Failed to transform points: {e}"))
    })?;

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

    log::info!("start zoom aggregation...");
    let start_local = std::time::Instant::now();
    for z in (args.min..max_zoom).rev() {
        log::info!("aggregating zoom level: {}", z);
        aggregate_zoom_level(tmp_tiled_file_dir_path.path(), z)?;
    }
    log::info!("Finish zoom aggregation in {:?}", start_local.elapsed());

    log::info!("start exporting tiles (GLB)...");
    let start_local = std::time::Instant::now();

    let tile_contents = export_tiles_to_glb(
        tmp_tiled_file_dir_path.path(),
        output_path,
        args.min,
        max_zoom,
        args.quantize,
        args.gzip_compress,
    )?;

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

        // CPUコア数を考慮したチャンネル容量の最適化
        let num_cores = num_cpus::get();
        channel_capacity = std::cmp::max(channel_capacity, num_cores * 2);

        let extension = check_and_get_extension(&input_files).unwrap();
        let epsg_in = args.input_epsg;
        let epsg_out = args.output_epsg;

        log::info!("max_memory_mb_bytes: {}", max_memory_mb_bytes);
        log::info!("one_chunk_mem: {}", one_chunk_mem);
        log::info!("channel_capacity: {}", channel_capacity);
        log::info!("num_cores: {}", num_cores);

        let (tx, rx) = mpsc::sync_channel::<Vec<Point>>(channel_capacity);

        // 複数のリーダースレッドを起動
        let chunk_size = input_files.len().div_ceil(num_cores);
        let mut handles = vec![];

        for chunk in input_files.chunks(chunk_size) {
            let chunk = chunk.to_vec();
            let tx = tx.clone();
            let extension_copy = extension;

            let handle = thread::spawn(move || {
                // 各スレッドで独自のトランスフォーマーを作成
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
                        // バッチで座標変換
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
                    // 残りの点を変換
                    transformer
                        .transform_points_in_place(&mut buffer)
                        .expect("Failed to transform points");
                    let _ = tx.send(buffer);
                }
            });
            handles.push(handle);
        }

        // 送信側のチャンネルを閉じるためにdropする
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

        drop(tmp_run_file_dir_path);

        log::info!("start zoom aggregation...");
        let start_local = std::time::Instant::now();

        // The parent tile coordinates are calculated from the file with the maximum zoom level
        for z in (args.min..args.max).rev() {
            log::info!("aggregating zoom level: {}", z);
            aggregate_zoom_level(tmp_tiled_file_dir_path.path(), z).unwrap();
        }
        log::info!("Finish zoom aggregation in {:?}", start_local.elapsed());

        log::info!("start exporting tiles (GLB)...");
        let start_local = std::time::Instant::now();
        let tile_contents = export_tiles_to_glb(
            tmp_tiled_file_dir_path.path(),
            output_path,
            args.min,
            args.max,
            args.quantize,
            args.gzip_compress,
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

    let start = std::time::Instant::now();

    log::info!("start processing...");
    let input_files = expand_globs(args.input.clone());
    log::info!("Expanded input files: {:?}", input_files);

    let output_path = PathBuf::from(args.output.clone());
    std::fs::create_dir_all(&output_path).unwrap();

    let total_size = estimate_total_size(&input_files);
    let max_memory_bytes = args.max_memory_mb as u64 * 1024 * 1024;
    log::info!(
        "Total input size: {}, threshold: {}",
        total_size,
        max_memory_bytes
    );

    if total_size <= max_memory_bytes {
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
