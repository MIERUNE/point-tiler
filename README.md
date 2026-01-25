# Point Tiler

A tool for converting point cloud data (las/laz and csv) into "3D Tiles v1.1".

![alt text](images/README_image.png)

## install

Enter the following command.

```sh
curl -sSf https://raw.githubusercontent.com/MIERUNE/point-tiler/main/install.sh | bash
```

Currently, it is not available on Windows.
However, if Rust is installed and Cargo is available, it is possible to run it using the `cargo` command.

### for developer

Rust must be installed. You can easily install it from the following page.

[Getting started](https://www.rust-lang.org/learn/get-started)

After installing Rust, download this repository.

## Usage

### LAS/LAZ

- `input`: Specify the `.las/.laz/.csv/.txt` file. Multiple files can be input separated by spaces.
- `output`: Specify the output folder. Output `tileset.json` and glb.
- `input-epsg`: Input the epsg code of the las file. All point clouds are recognized as being in the same coordinate system.(Currently, only the Japanese plane rectangular coordinate system is supported.)
- `output-epsg`: Supports conversion to WGS84 Geographic 3D (EPSG:4979), which is the standard for Cesium, and JGD2011 Geographic 3D (EPSG:6697), which is frequently used in Japan.
- `min`: Specify the minimum zoom level you want to output.
- `max`: Specify the maximum zoom level you want to output.
- `max-memory-mb`: Specify the number of MB of memory available for conversion.
- `quantize`: Perform quantization.
- `--gzip-compress`: The output 3D Tiles are compressed using gzip. The file extension dose not change.

In the repository root, the following commands can be executed.

```sh
point_tiler --input app/examples/data/sample.las \
    --output app/examples/data/output \
    --input-epsg 6677 \
    --output-epsg 4979 \
    --min 15 \
    --max 18 \
    --max-memory-mb 8192 \
    --quantize \
    --gzip-compress
```

This tool has a unique concept called a `zoom level` which represents a planar area roughly the same size as 2D tiles such as raster tiles or Google Photorealistic 3D Tiles.
The geometric error varies depending on the zoom level for example, it is approximately 64.0 at zoom level 15, about 8.0 at zoom level 18, and about 1.0 at zoom level 21.
The output tiles are thinned out so that one point is stored in a voxel grid of `Geometric Error × 0.1`.

### CSV/TXT

It supports .csv and .txt file extensions.

In CSV format, any columns other than XYZRGB will be ignored.
Also, the column names must be “x”, “y”, “z” and “r”, “g”, “b” or “red”, “green”, “blue”.
(The case of the letters does not matter.)

For example, the following data is valid.

```csv
“X“,”Y“,”Z“,”Intensity“,”ReturnNumber“,”NumberOfReturns“,”ScanDirectionFlag“,”EdgeOfFlightLine“,”Classification“,”Synthetic“,”KeyPoint“,”Withheld“,”Overlap“,”ScanAngleRank“,”UserData“,”PointSourceId“,”GpsTime“,”Red“,”Green“,”Blue”
-5599.971,-35106.097,3.602,680.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.207,19532.000,20046.000,20560.000
-5599.976,-35111.458,3.440,715.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.287,20560.000,21074.000,21588.000
-5599.978,-35115.695,3.543,311.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.347,19018.000,19275.000,20303.000
-5599.992,-35119.757,3.603,722.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.407,18504.000,20046.000,20817.000
-5599.992,-35129.327,3.431,505.000,1.000,1.000,1.000,0.000,1.000,0.000,0.00 0,0.000,0.000,-4.000,0.000,95.000,188552.547,18504.000,19789.000,21074.000
```

## Functions to be implemented

- [X] large-scale processing using streaming
- [X] generation of gzip-compressed tiles
- [ ] assignment of attributes using EXT_mesh_features
- [ ] increasing the number of supported coordinate systems

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

- [nusamai-projection](https://github.com/MIERUNE/plateau-gis-converter/tree/main/nusamai-projection) is imported as [projection-transform](./projection-transform/).
- [nusamai-gltf](https://github.com/MIERUNE/plateau-gis-converter/tree/main/nusamai-gltf) is included as [cesiumtiles-gltf](./cesiumtiles-gltf/).
- [nusamai/src/transformer/transform/projection.rs](https://github.com/MIERUNE/plateau-gis-converter/blob/main/nusamai/src/transformer/transform/projection.rs) is included in part of [pcd-transformer](pcd-transformer/src/transform/projection.rs)

The modified code retains its original copyright and is used in accordance with the MIT License. For details, see the LICENSE file.
