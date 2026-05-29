use winnow::Parser;
use winnow::token::take_until;
use winnow::ascii::line_ending;
use winnow::error::ModalResult;

use crate::parser::utils;

// We skip the windows as it is not usefull for us
pub fn skip_window(input: &mut &str) -> ModalResult<()> {
    "make_window".parse_next(input)?;
    take_until(0.., "\n").parse_next(input)?;
    utils::parse_end_of_line.parse_next(input)?;
    Ok(())
}
