use std::{
    vec, 
    fs::{self, File},
    io::{Error, Read, BufReader}
};
use crate::file_times::AudioSamples;

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

fn read_bin(filename: &str) -> Result<AnalogData, Error> {
    let f = File::open(filename)?;
    let reader = BufReader::new(f);
    parse_analog(reader)
}

fn read_folder(folder: &str) -> Result<(AnalogData, AnalogData, AnalogData), Error> {
    let a0 = format!("{}/analog_0.bin", folder);
    let a1 = format!("{}/analog_1.bin", folder);
    let a2 = format!("{}/analog_2.bin", folder);

    let f1 = read_bin(&a0)?;
    let f2 = read_bin(&a1)?;
    let f3 = read_bin(&a2)?;

    Ok((f1, f2, f3))
}

/// Read binary files from a Saleae Logic Analyzer
/// 
/// # Arguments
///     * `folder` - A string slice that holds the folder path
///         * the folder must contain one or more folders, named "_bin" (where _ can be anything), containing the files analog_0.bin, analog_1.bin, analog_2.bin
/// 
/// # Returns
///    * (times, channel1, channel2, channel3)
pub fn read_binary(folder: &str) -> Vec<AudioSamples> {
    // search for folders ending in bin

    let mut samples = Vec::<AudioSamples>::new();

    for entry in fs::read_dir(folder).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();
        if path.is_dir() {
            let pathname = path.to_str().unwrap().to_string();
            if !pathname.ends_with("bin"){ continue; }
            let (ch1, ch2, ch3) = read_folder(pathname.as_str()).unwrap();
            let times = (0..ch1.num_samples).map(|x| ch1.begin_time + x as f64 / ch1.sample_rate as f64).collect();
            let channel1 = ch1.samples.iter().map(|x| *x as f64).collect();
            let channel2 = ch2.samples.iter().map(|x| *x as f64).collect();
            let channel3 = ch3.samples.iter().map(|x| *x as f64).collect();
            samples.push(AudioSamples {
                times,
                channel1,
                channel2,
                channel3
            });
        }
    }

    samples
}
