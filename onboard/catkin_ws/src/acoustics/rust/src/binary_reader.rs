use std::{
    fs::{self, read, File}, io::{BufReader, Error, Read}, vec
};
use pyo3::pyclass;

use crate::file_times::AudioSamples;

const IDENTIFIER: &[u8] = b"<SALEAE>";

#[derive(Debug)]
#[pyclass]
pub struct AnalogData {
    #[pyo3(get)]
    pub begin_time: f64,
    #[pyo3(get)]
    pub sample_rate: u64,
    #[pyo3(get)]
    pub downsample: u64,
    #[pyo3(get)]
    pub num_samples: u64,
    #[pyo3(get)]
    pub samples: Vec<f32>,
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
        sample_rate: sample_rate/downsample,
        downsample,
        num_samples,
        samples,
    })
}

pub fn read_binary_file(filename: &str) -> Result<AnalogData, Error> {
    let f = File::open(filename)?;
    let reader = BufReader::new(f);
    parse_analog(reader)
}

fn analog_to_samples(analog_samples: &Vec<AnalogData>) -> AudioSamples {
    let mut channels = Vec::<Vec<f32>>::new();
    for sample in analog_samples {
        channels.push(sample.samples.clone());
    }
    let ch1 = &analog_samples[0];
    let times = (0..ch1.num_samples).map(|x| ch1.begin_time as f32 + x as f32 / ch1.sample_rate as f32).collect();

    AudioSamples {
        times,
        channels,
        sample_rate: ch1.sample_rate as f64,
    }
}

pub fn read_binary_folder(folder: &str) -> Result<AudioSamples, Error> {
    let mut channels = Vec::<AnalogData>::new();
    let mut bin_files = Vec::<String>::new();
    for entry in fs::read_dir(folder).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();
        if path.is_dir() {
            continue;
        }
        let pathname = path.to_str().unwrap().to_string();
        if !pathname.ends_with("bin"){ continue; }
        bin_files.push(pathname);
       
    }
    bin_files.sort();
    for file in &bin_files {
        let f = read_binary_file(file)?;
        channels.push(f);
    }

    Ok(analog_to_samples(&channels))
}
