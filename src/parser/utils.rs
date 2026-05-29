use winnow::Parser;
use winnow::ascii::{digit1, float};
use winnow::token::take_while;
use winnow::error::ModalResult;

pub fn parse_id(input: &mut &str) -> ModalResult<u64> {
    let digits = digit1.parse_next(input)?;
    let id = digits.parse::<u64>().unwrap();
    Ok(id)
}

pub fn parse_float64(input: &mut &str) -> ModalResult<f64> {
    float.parse_next(input)
}
