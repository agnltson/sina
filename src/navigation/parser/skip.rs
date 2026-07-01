use winnow::Parser;
use winnow::token::take_till;
use winnow::error::ModalResult;

use super::utils;

// We skip the windows as it is not usefull for us
pub fn skip_window(input: &mut &str) -> ModalResult<()> {
    "make_window".parse_next(input)?;
    take_till(0.., |c| c == '\n').parse_next(input)?;
    utils::parse_end_of_line.parse_next(input)?;
    Ok(())
}
