use std::{io::Write, sync::Mutex};
use filtering::{get_sustained_spikes, gen_filter};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use std::{env, fs};
use plotpy::{Plot, Curve};

mod csv_reader;
mod binary_reader;
mod filtering;
mod file_times;
mod measure;
use crate::csv_reader::read_csv;
use crate::binary_reader::read_binary;
use crate::filtering::get_time_difs;

const FS: f64 = 625_000.0;
const LOWCUT: f64 = 50_000.0;
const HIGHCUT: f64 = 58_000.0;

struct PingerMeasurement {
    octant: u8,
    h1_2: f64,
    h1_3: f64,
    h2_3: f64,
}

fn get_lines(filename: &str) -> Vec<String> {
    let file = std::fs::read_to_string(filename).unwrap();
    file.lines().map(|x| x.to_string()).collect()
}


fn main() {
    // get 7
    let samples = read_binary("octants_data/7/1bin");
    let filter = gen_filter(LOWCUT, HIGHCUT, FS);
    let spikes1 = get_sustained_spikes(&samples[0].channel1, FS, &filter);
    let spikes2 = get_sustained_spikes(&samples[0].channel2, FS, &filter);
    let spikes3 = get_sustained_spikes(&samples[0].channel3, FS, &filter);

    // plot the spikes with plotpy
    let mut plot = Plot::new();
    plot.set_title("Sustained Spikes");
    let mut curve1 = Curve::new();
    curve1.set_label("Channel 1");
    curve1.draw()
    // plot.set_xlabel("Time (s)");
    // plot.set_ylabel("Amplitude");
    // plot.add_series("Channel 1", &samples[0].times, &samples[0].channel1);



    
}
