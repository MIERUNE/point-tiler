use std::fmt;

#[derive(Debug, Default)]
pub struct TileCoord {
    pub level: u32,
    pub xyz: [u32; 3],
}

impl TileCoord {
    pub fn interleave_bits(&self) -> u64 {
        fn part1by2(mut n: u64) -> u64 {
            n &= 0x1fffff;
            n = (n | n << 32) & 0x1f00000000ffff;
            n = (n | n << 16) & 0x1f0000ff0000ff;
            n = (n | n << 8) & 0x100f00f00f00f00f;
            n = (n | n << 4) & 0x10c30c30c30c30c3;
            n = (n | n << 2) & 0x1249249249249249;
            n
        }

        let [x, y, z] = &self.xyz;

        let mx = part1by2(*x as u64);
        let my = part1by2(*y as u64);
        let mz = part1by2(*z as u64);

        mx | (my << 1) | (mz << 2)
    }

    pub fn deinterleave_bits(morton_code: u64) -> (u32, u32, u32) {
        fn compact1by2(mut n: u64) -> u32 {
            n &= 0x1249249249249249;
            n = (n ^ (n >> 2)) & 0x10c30c30c30c30c3;
            n = (n ^ (n >> 4)) & 0x100f00f00f00f00f;
            n = (n ^ (n >> 8)) & 0x1f0000ff0000ff;
            n = (n ^ (n >> 16)) & 0x1f00000000ffff;
            n = (n ^ (n >> 32)) & 0x1fffff;
            n as u32
        }

        let x = compact1by2(morton_code);
        let y = compact1by2(morton_code >> 1);
        let z = compact1by2(morton_code >> 2);

        (x, y, z)
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoundingVolume {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[derive(Debug, Default)]
pub struct OctreeNode {
    pub bounding_volume: BoundingVolume,
    pub tile_coord: TileCoord,
    pub children: Option<[Box<OctreeNode>; 8]>,
}

impl OctreeNode {
    pub fn build(
        bounding_box: BoundingVolume,
        tile_coords: (u32, u32, u32, u32),
        depth: u32,
    ) -> Self {
        if depth == 0 {
            let tile_coords_object = TileCoord {
                level: tile_coords.0,
                xyz: [tile_coords.1, tile_coords.2, tile_coords.3],
            };
            return OctreeNode {
                bounding_volume: bounding_box,
                tile_coord: tile_coords_object,
                children: None,
            };
        }

        let min = &bounding_box.min;
        let max = &bounding_box.max;

        let mid = [
            (min[0] + max[0]) / 2.0,
            (min[1] + max[1]) / 2.0,
            (min[2] + max[2]) / 2.0,
        ];

        let mut children: [Box<OctreeNode>; 8] = Default::default();

        for i in 0..8 {
            let dx = (i >> 0) & 1;
            let dy = (i >> 1) & 1;
            let dz = (i >> 2) & 1;

            let child_min = [
                if dx == 0 { min[0] } else { mid[0] },
                if dy == 0 { min[1] } else { mid[1] },
                if dz == 0 { min[2] } else { mid[2] },
            ];
            let child_max = [
                if dx == 0 { mid[0] } else { max[0] },
                if dy == 0 { mid[1] } else { max[1] },
                if dz == 0 { mid[2] } else { max[2] },
            ];

            let child_bounding_box = BoundingVolume {
                min: child_min,
                max: child_max,
            };

            let child_tile_coords = (
                tile_coords.0 + 1,             // level + 1
                tile_coords.1 * 2 + dx as u32, // x * 2 + dx
                tile_coords.2 * 2 + dy as u32, // y * 2 + dy
                tile_coords.3 * 2 + dz as u32, // z * 2 + dz
            );

            children[i] = Box::new(OctreeNode::build(
                child_bounding_box,
                child_tile_coords,
                depth - 1,
            ));
        }

        let tile_coords_object = TileCoord {
            level: tile_coords.0,
            xyz: [tile_coords.1, tile_coords.2, tile_coords.3],
        };

        OctreeNode {
            bounding_volume: bounding_box,
            tile_coord: tile_coords_object,
            children: Some(children),
        }
    }
}

impl fmt::Display for OctreeNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let level = self.tile_coord.level;
        let [x, y, z] = self.tile_coord.xyz;
        writeln!(
            f,
            "Level: {}, x: {}, y: {}, z: {}, Bounding Volume: {:?}",
            level, x, y, z, self.bounding_volume
        )?;
        if let Some(children) = &self.children {
            for child in children.iter() {
                write!(f, "{}", child)?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_octree_construction() {
        let bounding_box = BoundingVolume {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };

        let subdivision_count = 2;
        let octree = OctreeNode::build(bounding_box.clone(), (0, 0, 0, 0), subdivision_count);

        let root_coord_level = 0;
        let root_coord_xyz = [0, 0, 0];
        assert_eq!(octree.tile_coord.level, root_coord_level);
        assert_eq!(octree.tile_coord.xyz, root_coord_xyz);
        assert_eq!(octree.bounding_volume, bounding_box);
        assert!(octree.children.is_some());

        if let Some(children) = octree.children {
            assert_eq!(children.len(), 8);

            let first_child = &children[0];
            let first_child_coord_level = 1;
            let first_child_coord_xyz = [0, 0, 0];
            assert_eq!(first_child.tile_coord.level, first_child_coord_level);
            assert_eq!(first_child.tile_coord.xyz, first_child_coord_xyz);
            assert_eq!(
                first_child.bounding_volume,
                BoundingVolume {
                    min: [0.0, 0.0, 0.0],
                    max: [0.5, 0.5, 0.5],
                }
            );
            assert!(first_child.children.is_some());

            if let Some(grand_children) = &first_child.children {
                for grand_child in grand_children.iter() {
                    assert_eq!(grand_child.tile_coord.level, 2);
                    assert!(grand_child.children.is_none());
                }
            }
        }
    }
}
