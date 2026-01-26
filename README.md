# Point Tiler

A tool for converting point cloud data (las/laz and csv) into "3D Tiles v1.1".

![alt text](images/README_image.png)

## Features

- Converts LAS/LAZ/CSV point cloud data to 3D Tiles v1.1 format
- It has the PROJ library built-in and supports basic coordinate systems.
- High-performance parallel processing with Rayon
- Streaming processing for large-scale data
- Voxel-based point decimation for efficient LOD
- GZIP compression support for output tiles
- Quantization support for smaller file sizes

## Project Structure

```
point-tiler/
├── app/                       # Main CLI application
├── pcd-core/                  # Core point cloud data structures
├── pcd-parser/                # LAS/LAZ/CSV file parsers
├── pcd-exporter/              # Tiling and 3D Tiles export
├── coordinate-transformer/    # Coordinate transformation (PROJ-based)
└── cesiumtiles-gltf/          # glTF/GLB I/O for 3D Tiles output
```

## Install

Enter the following command.

```sh
curl -sSf https://raw.githubusercontent.com/MIERUNE/point-tiler/main/install.sh | bash
```

Currently, it is not available on Windows.
However, if Rust is installed and Cargo is available, it is possible to run it using the `cargo` command.

### For Developer

Rust must be installed. You can easily install it from the following page.

[Getting started](https://www.rust-lang.org/learn/get-started)

After installing Rust, download this repository.

## Usage

### Options

| Option            | Description                                                                                      |
| ----------------- | ------------------------------------------------------------------------------------------------ |
| `--input`, `-i`   | Input file path(s). Supports `.las`, `.laz`, `.csv`, `.txt`. Multiple files can be specified.    |
| `--output`, `-o`  | Output folder path. Outputs `tileset.json` and GLB files.                                        |
| `--input-epsg`    | EPSG code of the input coordinate system. Supports any EPSG code via PROJ.                       |
| `--output-epsg`   | EPSG code of the output coordinate system. Typically EPSG:4979 (WGS84 Geographic 3D) for Cesium. |
| `--min`           | Minimum zoom level (default: 15)                                                                 |
| `--max`           | Maximum zoom level (default: 18)                                                                 |
| `--max-memory-mb` | Memory limit in MB for workflow selection (default: 4096)                                        |
| `--threads`       | Number of threads for parallel processing (default: number of CPU cores)                         |
| `--quantize`      | Enable quantization for smaller GLB files                                                        |
| `--gzip-compress` | Enable GZIP compression for output tiles                                                         |

### Example

```sh
point_tiler --input app/examples/data/sample.las \
    --output app/examples/data/output \
    --input-epsg 6677 \
    --output-epsg 4979 \
    --min 15 \
    --max 18 \
    --max-memory-mb 8192 \
    --threads 8 \
    --quantize \
    --gzip-compress
```

### Coordinate Systems

This tool uses the PROJ library for coordinate transformation, supporting any EPSG code.

**Common input coordinate systems:**
- Japan Plane Rectangular CS (EPSG:6669-6687)
- UTM zones (e.g., EPSG:32654 for UTM zone 54N)
- Any other projected or geographic coordinate system

**Recommended output coordinate systems:**
- EPSG:4979 (WGS84 Geographic 3D) - Standard for Cesium
- EPSG:6697 (JGD2011 Geographic 3D) - For Japan-specific applications

### Zoom Levels

This tool has a unique concept called a `zoom level` which represents a planar area roughly the same size as 2D tiles such as raster tiles or Google Photorealistic 3D Tiles.
The geometric error varies depending on the zoom level for example, it is approximately 64.0 at zoom level 15, about 8.0 at zoom level 18, and about 1.0 at zoom level 21.
The output tiles are thinned out so that one point is stored in a voxel grid of `Geometric Error × 0.1`.

### CSV/TXT Format

It supports .csv and .txt file extensions.

In CSV format, any columns other than XYZRGB will be ignored.
Also, the column names must be "x", "y", "z" and "r", "g", "b" or "red", "green", "blue".
(The case of the letters does not matter.)

For example, the following data is valid.

```csv
"X","Y","Z","Intensity","ReturnNumber","NumberOfReturns","ScanDirectionFlag","EdgeOfFlightLine","Classification","Synthetic","KeyPoint","Withheld","Overlap","ScanAngleRank","UserData","PointSourceId","GpsTime","Red","Green","Blue"
-5599.971,-35106.097,3.602,680.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.207,19532.000,20046.000,20560.000
-5599.976,-35111.458,3.440,715.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.287,20560.000,21074.000,21588.000
-5599.978,-35115.695,3.543,311.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.347,19018.000,19275.000,20303.000
-5599.992,-35119.757,3.603,722.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.407,18504.000,20046.000,20817.000
-5599.992,-35129.327,3.431,505.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.547,18504.000,19789.000,21074.000
```

## License

[MIT License](./LICENSE)

## Authors

- Satoru Nishio (@nokonoko1203)
- And all contributors

## Use of Third-Party Source Code

This project incorporates and modifies source code from the following third-party projects:

- [PLATEAU GIS Converter](https://github.com/MIERUNE/plateau-gis-converter)
  - Repository: [PLATEAU GIS Converter](https://github.com/MIERUNE/plateau-gis-converter)
  - Original License: [MIT License](https://github.com/MIERUNE/plateau-gis-converter/blob/main/LICENSE)

### Modifications Made

We have made the following modifications to the source code from [PLATEAU GIS Converter](https://github.com/MIERUNE/plateau-gis-converter):

- [nusamai-gltf](https://github.com/MIERUNE/plateau-gis-converter/tree/main/nusamai-gltf) is included as [cesiumtiles-gltf](./cesiumtiles-gltf/).

The modified code retains its original copyright and is used in accordance with the MIT License. For details, see the LICENSE file.
