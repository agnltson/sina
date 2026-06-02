use winnow::Parser;
use winnow::ascii::{digit1, alpha1, float, line_ending};
use winnow::combinator::{alt, eof};
use winnow::error::ModalResult;

pub fn parse_id(input: &mut &str) -> ModalResult<u64> {
    let digits = digit1.parse_next(input)?;
    let id = digits.parse::<u64>().unwrap();
    Ok(id)
}

pub fn parse_float64(input: &mut &str) -> ModalResult<f32> {
    float.parse_next(input)
}

pub fn parse_class<'s>(input: &mut &'s str) -> ModalResult<&'s str> {
    alpha1.parse_next(input)
}

pub fn parse_end_of_line(input: &mut &str) -> ModalResult<()> {
    alt((line_ending.value(()), eof.value(()))).parse_next(input)
}
