use std::fmt;

// 境界ボリュームを表す構造体（軸に揃ったバウンディングボックス）
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BoundingVolume {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

// オクツリーのノードを表す構造体
#[derive(Debug, Default)]
pub struct OctreeNode {
    pub bounding_volume: BoundingVolume,
    pub tile_coords: (u32, u32, u32, u32), // (level, x, y, z)
    pub children: Option<[Box<OctreeNode>; 8]>,
}

impl OctreeNode {
    // オクツリーを構築する関数
    pub fn build(
        bounding_box: BoundingVolume,
        tile_coords: (u32, u32, u32, u32),
        depth: u32,
    ) -> Self {
        if depth == 0 {
            // 再帰の終了条件：指定された深さに達したら子ノードはなし
            return OctreeNode {
                bounding_volume: bounding_box,
                tile_coords,
                children: None,
            };
        }

        let min = &bounding_box.min;
        let max = &bounding_box.max;

        // 各軸の中間点を計算
        let mid = [
            (min[0] + max[0]) / 2.0,
            (min[1] + max[1]) / 2.0,
            (min[2] + max[2]) / 2.0,
        ];

        // 子ノードを格納する配列を初期化
        let mut children: [Box<OctreeNode>; 8] = Default::default();

        // 8つの子ノードを生成
        for i in 0..8 {
            let dx = (i >> 0) & 1;
            let dy = (i >> 1) & 1;
            let dz = (i >> 2) & 1;

            // 子ノードのバウンディングボリュームを計算
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

            // 子ノードのタイル座標を計算
            let child_tile_coords = (
                tile_coords.0 + 1,             // level + 1
                tile_coords.1 * 2 + dx as u32, // x * 2 + dx
                tile_coords.2 * 2 + dy as u32, // y * 2 + dy
                tile_coords.3 * 2 + dz as u32, // z * 2 + dz
            );

            // 子ノードを再帰的に構築
            children[i] = Box::new(OctreeNode::build(
                child_bounding_box,
                child_tile_coords,
                depth - 1,
            ));
        }

        OctreeNode {
            bounding_volume: bounding_box,
            tile_coords,
            children: Some(children),
        }
    }
}

// デバッグ用にタイル座標を表示するための実装
impl fmt::Display for OctreeNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (level, x, y, z) = self.tile_coords;
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

// テストモジュール
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_octree_construction() {
        // バウンディングボリュームを定義
        let bounding_box = BoundingVolume {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };

        // オクツリーを構築
        let subdivision_count = 2;
        let octree = OctreeNode::build(bounding_box.clone(), (0, 0, 0, 0), subdivision_count);

        // ルートノードの検証
        assert_eq!(octree.tile_coords, (0, 0, 0, 0));
        assert_eq!(octree.bounding_volume, bounding_box);
        assert!(octree.children.is_some());

        // 子ノードの検証
        if let Some(children) = octree.children {
            // 子ノードの数を確認
            assert_eq!(children.len(), 8);

            // 最初の子ノードを検証
            let first_child = &children[0];
            assert_eq!(first_child.tile_coords, (1, 0, 0, 0));
            assert_eq!(
                first_child.bounding_volume,
                BoundingVolume {
                    min: [0.0, 0.0, 0.0],
                    max: [0.5, 0.5, 0.5],
                }
            );
            assert!(first_child.children.is_some());

            // 葉ノードの検証（深さが2なので、子ノードの子ノードは葉ノード）
            if let Some(grand_children) = &first_child.children {
                for grand_child in grand_children.iter() {
                    assert_eq!(grand_child.tile_coords.0, 2);
                    assert!(grand_child.children.is_none());
                }
            }
        }
    }
}
