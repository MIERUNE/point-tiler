# Point Tiler

A tool for converting point cloud data（las/laz and csv） into 3D Tiles v1.1.

## Usage

- `input`: Specify the las file. Multiple files can be input separated by spaces.
- `output`: Specify the output folder. Output `tileset.json` and glb.
- `epsg`: Input the epsg code of the las file. All point clouds are recognized as being in the same coordinate system.
- `min`: Specify the minimum zoom level you want to output.
- `max`: Specify the maximum zoom level you want to output.

```sh
cargo run -r -p app -- --input app/examples/data/sample.las \
    --output app/examples/data/output \
    --epsg 6677 \
    --min 15 \
    --max 18
```
