use crate::datatype::vertex;

const ACTION: [(i32, i32); 5] = [
    (0, -1), // Left
    (1, 0),  // Up
    (0, 1),  // Right
    (-1, 0), // Down
    (0, 0),  // Wait
];

/// Size of tuple is (i32, i32) to avoid the edge case where
/// units in the vertex::Vertex (u16, u16) is between 2^15 and 2^16-1.
const fn get_next_direction(loc: vertex::Vertex, action: usize) -> Option<vertex::Vertex> {
    let dir = ACTION[action];
    let x = loc.1 as i32 + dir.1;
    let y = loc.0 as i32 + dir.0;
    if x < 0 || y < 0 {
        None
    } else {
        Some(vertex::Vertex(y as u16, x as u16))
    }
}

fn is_invalid_loc(map: &[Vec<u8>], next_loc: vertex::Vertex) -> bool {
    let height = map.len() as u16;
    let width = map[0].len() as u16;
    // Out of bound in vertical or horizontal axis or encounter map obstacle
    next_loc.0 >= height
        || next_loc.1 >= width
        || map[next_loc.0 as usize][next_loc.1 as usize] == 0
}

pub fn get_next_loc(
    map: &[Vec<u8>],
    cur_loc: vertex::Vertex,
    action: usize,
) -> Option<vertex::Vertex> {
    let next_loc = match get_next_direction(cur_loc, action) {
        Some(vertex) => vertex,
        None => return None,
    };
    if is_invalid_loc(map, next_loc) {
        return None;
    }
    Some(next_loc)
}
