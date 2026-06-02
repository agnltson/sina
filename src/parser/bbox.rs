use winnow::Parser;
use winnow::error::ModalResult;

use crate::utils::Vec3;
use crate::raw_data::raw_bbox::RawBBox;
use crate::parser::utils;

pub fn parse_bbox(input: &mut &str) -> ModalResult<RawBBox> {
    "make_bbox".parse_next(input)?;
    ", id=".parse_next(input)?;
    let id = utils::parse_id.parse_next(input)?;

    ", class=".parse_next(input)?;
    let _class = utils::parse_class.parse_next(input)?;

    ", position_x=".parse_next(input)?;
    let position_x = utils::parse_float64.parse_next(input)?;
    ", position_y=".parse_next(input)?;
    let position_y = utils::parse_float64.parse_next(input)?;
    ", position_z=".parse_next(input)?;
    let position_z = utils::parse_float64.parse_next(input)?;
    let position = Vec3::new(position_x, position_y, position_z);

    ", angle_z=".parse_next(input)?;
    let angle = utils::parse_float64.parse_next(input)?;

    ", scale_x=".parse_next(input)?;
    let scale_x = utils::parse_float64.parse_next(input)?;
    ", scale_y=".parse_next(input)?;
    let scale_y = utils::parse_float64.parse_next(input)?;
    ", scale_z=".parse_next(input)?;
    let scale_z = utils::parse_float64.parse_next(input)?;
    let scale = Vec3::new(scale_x, scale_y, scale_z);

    utils::parse_end_of_line.parse_next(input)?;

    Ok(RawBBox::new(id, position, angle, scale))
}
