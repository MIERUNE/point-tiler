fn interleave_bits(x: u32, y: u32, z: u32) -> u64 {
    fn part1by2(mut n: u64) -> u64 {
        n &= 0x1fffff;
        n = (n | n << 32) & 0x1f00000000ffff;
        n = (n | n << 16) & 0x1f0000ff0000ff;
        n = (n | n << 8) & 0x100f00f00f00f00f;
        n = (n | n << 4) & 0x10c30c30c30c30c3;
        n = (n | n << 2) & 0x1249249249249249;
        n
    }

    let mx = part1by2(x as u64);
    let my = part1by2(y as u64);
    let mz = part1by2(z as u64);

    mx | (my << 1) | (mz << 2)
}

fn deinterleave_bits(morton_code: u64) -> (u32, u32, u32) {
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

fn main() {
    let x: u32 = 5;
    let y: u32 = 3;
    let z: u32 = 7;

    let morton_code = interleave_bits(x, y, z);
    println!("Morton code: {}", morton_code);

    let (dx, dy, dz) = deinterleave_bits(morton_code);
    println!("Decoded: x = {}, y = {}, z = {}", dx, dy, dz);

    assert_eq!((x, y, z), (dx, dy, dz));
}
