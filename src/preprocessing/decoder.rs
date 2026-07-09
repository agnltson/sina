use crossbeam_channel::Sender;
use ffmpeg_next as ffmpeg;

pub struct DecodedFrame {
    pub frame_index: usize,
    pub width: usize,
    pub height: usize,
    pub stride: usize,
    pub gray_data: Vec<u8>,
}

pub fn decode_and_send(mp4_path: &str, skip_factor: usize, tx: Sender<DecodedFrame>) -> anyhow::Result<()> {
    ffmpeg::init()?;

    let mut ictx = ffmpeg::format::input(&mp4_path)?;

    let input_stream = ictx
        .streams()
        .best(ffmpeg::media::Type::Video)
        .ok_or_else(|| anyhow::anyhow!("No video stream found in the MP4"))?;
    let video_stream_index = input_stream.index();

    let context_decoder = ffmpeg::codec::context::Context::from_parameters(input_stream.parameters())?;
    let mut decoder = context_decoder.decoder().video()?;

    let mut scaler = ffmpeg::software::scaling::context::Context::get(
        decoder.format(),
        decoder.width(),
        decoder.height(),
        ffmpeg::format::Pixel::GRAY8,
        decoder.width(),
        decoder.height(),
        ffmpeg::software::scaling::flag::Flags::BILINEAR,
    )?;

    let mut frame_index: usize = 0;
    let mut decoded = ffmpeg::frame::Video::empty();
    let mut gray = ffmpeg::frame::Video::empty();

    let mut push_frame = |
        frame_index: usize,
        decoded: &ffmpeg::frame::Video,
        gray: &mut ffmpeg::frame::Video
    | -> anyhow::Result<()> {

            scaler.run(decoded, gray)?;

            if frame_index % skip_factor == 0 {
                let width = gray.width() as usize;
                let height = gray.height() as usize;
                let stride = gray.stride(0);
                let expected_len = height * stride;
                let raw = gray.data(0);
                let gray_data = if raw.len() >= expected_len {
                    raw[..expected_len].to_vec()
                } else {
                    raw.to_vec()
                };

                let frame = DecodedFrame {
                    frame_index,
                    width,
                    height,
                    stride,
                    gray_data,
                };
                tx.send(frame)?;
            }
            Ok(())
        };

    for (stream, packet) in ictx.packets() {
        if stream.index() != video_stream_index {
            continue;
        }
        decoder.send_packet(&packet)?;
        while decoder.receive_frame(&mut decoded).is_ok() {
            push_frame(frame_index, &decoded, &mut gray)?;
            frame_index += 1;
        }
    }

    decoder.send_eof()?;
    while decoder.receive_frame(&mut decoded).is_ok() {
        push_frame(frame_index, &decoded, &mut gray)?;
        frame_index += 1;
    }

    Ok(())
}
