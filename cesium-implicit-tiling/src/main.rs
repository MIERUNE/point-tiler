use cesium_implicit_tiling::{
    morton_order::{deinterleave_bits, interleave_bits},
    subdivide::{BoundingVolume, OctreeNode},
};

fn main() {
    // morton order
    let x: u32 = 5;
    let y: u32 = 3;
    let z: u32 = 7;

    let morton_code = interleave_bits(x, y, z);
    println!("Morton code: {}", morton_code);

    let (dx, dy, dz) = deinterleave_bits(morton_code);
    println!("Decoded: x = {}, y = {}, z = {}", dx, dy, dz);

    assert_eq!((x, y, z), (dx, dy, dz));

    // subdivide
    let bounding_box = BoundingVolume {
        min: [0.0, 0.0, 0.0],
        max: [1.0, 1.0, 1.0],
    };

    let subdivision_count = 2;
    let octree = OctreeNode::build(bounding_box.clone(), (0, 0, 0, 0), subdivision_count);

    println!("{}", octree);
}
