use cesium_implicit_tiling::octree::{BoundingVolume, OctreeNode};

fn main() {
    let bounding_box = BoundingVolume {
        min: [0.0, 0.0, 0.0],
        max: [1.0, 1.0, 1.0],
    };

    let subdivision_count = 2;
    let octree = OctreeNode::build(bounding_box.clone(), (0, 0, 0, 0), subdivision_count);

    println!("{}", octree);
}
