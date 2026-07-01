use winnow::Parser;
use winnow::ascii::{digit1, alpha1, float, line_ending};
use winnow::combinator::{alt, opt, eof};
use winnow::token::one_of;
use winnow::error::ModalResult;
use ordered_float::OrderedFloat;

pub fn parse_id(input: &mut &str) -> ModalResult<i64> {
    let (sign, digits) = (opt(one_of(['+', '-'])), digit1).parse_next(input)?;
    let value: i64 = digits.parse::<i64>().unwrap();
    Ok(if sign == Some('-') { -value } else { value })
}

pub fn parse_float(input: &mut &str) -> ModalResult<OrderedFloat<f32>> {
    match float.parse_next(input) {
        Ok(f) => Ok(OrderedFloat(f)),
        Err(e) => Err(e),
    }
}

pub fn parse_class<'s>(input: &mut &'s str) -> ModalResult<&'s str> {
    alpha1.parse_next(input)
}

pub fn parse_end_of_line(input: &mut &str) -> ModalResult<()> {
    alt((line_ending.value(()), eof.value(()))).parse_next(input)
}
