use winnow::Parser;
use winnow::ascii::{line_ending, multispace0};
use winnow::error::ModalResult;

use crate::utils::vec3::Vec3;
use crate::room::door::Door;
use crate::parser::utils;

pub fn parse_door(input: &mut &str) -> ModalResult<Door> {
    "make_door".parse_next(input)?;
    ", id=".parse_next(input)?;
    let id = utils::parse_id.parse_next(input)?;

    ", wall0_id=".parse_next(input)?;
    let wall0_id = utils::parse_id.parse_next(input)?;

    ", wall1_id=".parse_next(input)?;
    let wall1_id = utils::parse_id.parse_next(input)?;

    ", position_x=".parse_next(input)?;
    let position_x = utils::parse_float64.parse_next(input)?;
    ", position_y=".parse_next(input)?;
    let position_y = utils::parse_float64.parse_next(input)?;
    ", position_z=".parse_next(input)?;
    let position_z = utils::parse_float64.parse_next(input)?;
    let position = Vec3::new(position_x, position_y, position_z);

    ", width=".parse_next(input)?;
    let width = utils::parse_float64.parse_next(input)?;

    ", height=".parse_next(input)?;
    let height = utils::parse_float64.parse_next(input)?;

    utils::parse_end_of_line.parse_next(input)?;

    Ok(Door::new(id, wall0_id, wall1_id, position, width, height))
}
