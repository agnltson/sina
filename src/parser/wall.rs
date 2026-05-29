use winnow::Parser;
use winnow::error::ModalResult;

use crate::utils::vec3::Vec3;
use crate::room::wall::Wall;
use crate::parser::utils;

pub fn parse_wall(input: &mut &str) -> ModalResult<Wall> {
    "make_wall, ".parse_next(input)?;
    "id=".parse_next(input)?;
    let id = utils::parse_id.parse_next(input)?;

    ", a_x=".parse_next(input)?;
    let a_x = utils::parse_float64.parse_next(input)?;
    ", a_y=".parse_next(input)?;
    let a_y = utils::parse_float64.parse_next(input)?;
    ", a_z=".parse_next(input)?;
    let a_z = utils::parse_float64.parse_next(input)?;
    let a = Vec3::new(a_x, a_y, a_z);

    ", b_x=".parse_next(input)?;
    let b_x = utils::parse_float64.parse_next(input)?;
    ", b_y=".parse_next(input)?;
    let b_y = utils::parse_float64.parse_next(input)?;
    ", b_z=".parse_next(input)?;
    let b_z = utils::parse_float64.parse_next(input)?;
    let b = Vec3::new(b_x, b_y, b_z);

    ", height=".parse_next(input)?;
    let _height = utils::parse_float64.parse_next(input)?;

    ", thickness=".parse_next(input)?;
    let _thickness = utils::parse_float64.parse_next(input)?;

    utils::parse_end_of_line.parse_next(input)?;

    Ok(Wall::new(id, a, b))
}
