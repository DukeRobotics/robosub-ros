use std::fs::File;
use std::io::{Error, Read, BufReader};
use std::vec;

const IDENTIFIER: &[u8] = b"<SALEAE>";

#[derive(Debug)]
struct AnalogData {
    begin_time: f64,
    sample_rate: u64,
    downsample: u64,
    num_samples: u64,
    samples: Vec<f32>,
}

#[inline(always)]
fn read_i32<R: Read>(reader: &mut BufReader<R>,  buf: &mut [u8; 4]) -> i32 {
    reader.read_exact(buf).ok();
    i32::from_le_bytes(*buf)
}

#[inline(always)]
fn read_u64<R: Read>(reader: &mut BufReader<R>, buf: &mut [u8; 8]) -> u64 {
    reader.read_exact(buf).ok();
    u64::from_le_bytes(*buf)
}

#[inline(always)]
fn read_f64<R: Read>(reader: &mut BufReader<R>, buf: &mut [u8; 8]) -> f64 {
    reader.read_exact(buf).ok();
    f64::from_le_bytes(*buf)
}

fn parse_analog(mut reader: BufReader<File>) -> Result<AnalogData, Error> {
    // Parse header
    let mut identifier = [0u8; 8];
    reader.read_exact(&mut identifier)?;

    if identifier != *IDENTIFIER {
        return Err(Error::new(std::io::ErrorKind::Other, "Not a saleae file"));
    }
    
    let mut buf8 = [0u8; 8];

    reader.read_exact(&mut buf8).ok();
    let begin_time = read_f64(&mut reader, &mut buf8);
    let sample_rate = read_u64(&mut reader, &mut buf8);
    let downsample = read_u64(&mut reader, &mut buf8);
    let num_samples = read_u64(&mut reader, &mut buf8);

    // let mut samples = Vec::with_capacity(num_samples as usize);
    let mut samples = vec![0f32; num_samples as usize];

    // read exact to bytes in samples
    reader.read_exact(unsafe {
        std::slice::from_raw_parts_mut(
            samples.as_mut_ptr() as *mut u8,
            num_samples as usize * std::mem::size_of::<f32>(),
        )
    })?;

    Ok(AnalogData {
        begin_time,
        sample_rate,
        downsample,
        num_samples,
        samples,
    })
}

fn main() {
    let filename = "/Users/ethanhorowitz/Desktop/new acoustics data/3/1bin/analog_0.bin";
    println!("Opening {}", filename);

    let f = File::open(filename).unwrap();
    let start_time = std::time::Instant::now();
    let reader = BufReader::new(f);
    let data = parse_analog(reader);
    println!("Time to read: {:?}", start_time.elapsed());
    println!("done");
    // print first 10 samples
    match data {
        Ok(data) => {
            println!("begin_time: {}", data.begin_time);
            println!("sample_rate: {}", data.sample_rate);
            println!("downsample: {}", data.downsample);
            println!("num_samples: {}", data.num_samples);
            println!("samples: {:?}", &data.samples[0..10]);
        }
        Err(e) => {
            println!("Error: {}", e);
        }
    }
}
