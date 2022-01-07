use crate::datatype::vertex::Vertex;

const ACTION: [(i32, i32); 5] = [
    (0, -1), // Left
    (1, 0),  // Up
    (0, 1),  // Right
    (-1, 0), // Down
    (0, 0),  // Wait
];

/// Size of tuple is (i32, i32) to avoid the edge case where
/// units in the Vertex (u16, u16) is between 2^15 and 2^16-1.
pub fn get_next_loc(loc: Vertex, action: usize) -> Option<Vertex> {
    let dir = ACTION[action];
    let x = loc.1 as i32 + dir.1;
    let y = loc.0 as i32 + dir.0;
    if x < 0 || y < 0 {
        None
    } else {
        Some(Vertex(y as u16, x as u16))
    }
}

pub fn is_invalid_loc(map: &Vec<Vec<u8>>, next_loc: Vertex) -> bool {
    let height = map.len() as u16;
    let width = map[0].len() as u16;
    // Out of bound in vertical or horizontal axis or encounter map obstacle
    next_loc.0 >= height
        || next_loc.1 >= width
        || map[next_loc.0 as usize][next_loc.1 as usize] == 0
}
